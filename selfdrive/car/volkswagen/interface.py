#!/usr/bin/env python
from cereal import car, log
from common.realtime import sec_since_boot
from selfdrive.config import Conversions as CV
from selfdrive.controls.lib.drive_helpers import create_event, get_events, EventTypes as ET
from selfdrive.controls.lib.vehicle_model import VehicleModel
from selfdrive.car.volkswagen.values import DBC, CAR
from selfdrive.car.volkswagen.carstate import CarState, get_mqb_gateway_can_parser, get_mqb_extended_can_parser
from common.params import Params
from selfdrive.car import STD_CARGO_KG, scale_rot_inertia, scale_tire_stiffness
from common.vin import VIN_UNKNOWN

class CanBus(object):
  def __init__(self):
    self.gateway = 0
    self.extended = 2

class CarInterface(object):
  def __init__(self, CP, CarController):
    self.CP = CP

    self.frame = 0
    self.acc_active_prev = 0
    self.gas_pressed_prev = False
    self.brake_pressed_prev = False
    self.engageable = False
    self.CC = None

    # *** init the major players ***
    canbus = CanBus()
    self.CS = CarState(CP, canbus)
    self.VM = VehicleModel(CP)
    self.gw_cp = get_mqb_gateway_can_parser(CP, canbus)
    self.ex_cp = get_mqb_extended_can_parser(CP, canbus)


    # sending if read only is False
    if CarController is not None:
      self.CC = CarController(canbus, CP.carFingerprint)


  @staticmethod
  def compute_gb(accel, speed):
    return float(accel) / 4.0

  @staticmethod
  def calc_accel_override(a_ego, a_target, v_ego, v_target):
    return 1.0

  @staticmethod
  def get_params(candidate, fingerprint, vin="", is_panda_black=False):
    ret = car.CarParams.new_message()

    ret.carFingerprint = candidate
    ret.isPandaBlack = is_panda_black
    ret.carVin = vin
    tire_stiffness_factor = 1.

    if candidate == CAR.GENERICMQB:
      # Check to make sure we received the VIN; we should have this for all MQBs
      # XXX temp removed
      # assert(ret.carVin != VIN_UNKNOWN), "Fingerprinted as Generic MQB but did not detect VIN"

      # Set common MQB parameters that will apply globally
      ret.carName = "volkswagen"
      ret.safetyModel = car.CarParams.SafetyModel.volkswagen
      ret.enableCruise = True # Stock ACC still controls acceleration and braking
      ret.openpilotLongitudinalControl = False
      ret.steerControlType = car.CarParams.SteerControlType.torque
      ret.steerLimitAlert = True # Enable UI alert when steering torque is maxed out

      # Additional common MQB parameters that may be overridden per-vehicle
      ret.steerRatio = 15
      ret.steerRateCost = 0.6
      ret.steerActuatorDelay = 0.05 # Hopefully all MQB racks are similar here
      ret.steerMaxBP = [0.]  # m/s
      ret.steerMaxV = [1.]

      # As a starting point for speed-adjusted lateral tuning, use the example
      # map speed breakpoints from a VW Tiguan (SSP 399 page 9). It's unclear
      # whether the driver assist map breakpoints have any direct bearing on
      # HCA assist torque, but if they're good breakpoints for the driver,
      # they're probably good breakpoints for HCA as well. OP won't be driving
      # 250kph/155mph but it provides interpolation scaling above 100kmh/62mph.
      ret.lateralTuning.pid.kpBP = [0., 15 * CV.KPH_TO_MS, 50 * CV.KPH_TO_MS, 100 * CV.KPH_TO_MS, 250 * CV.KPH_TO_MS]
      ret.lateralTuning.pid.kiBP = [0., 15 * CV.KPH_TO_MS, 50 * CV.KPH_TO_MS, 100 * CV.KPH_TO_MS, 250 * CV.KPH_TO_MS]

      # Use the VIN to look up specific make and model details
      chassiscode = vin[6:8]
      # XXX temp hack to make everything an Atlas if no VIN detected
      if(chassiscode == "00"):
        chassiscode = "NE"

      # B8 Passat, RoW only (North America Passat of this era is PQ/NMS)
      # TODO: Supportable but untested vehicle, will require test and tune
      if chassiscode == "3G":
        # Mass given is average between the sedan and wagon, and the spread is
        # pretty high, may need more detection here somehow.
        # FIXME: Untested vehicle, placeholder tuning values
        ret.mass = 1554
        ret.wheelbase = 2.79

      # Mk1 Volkswagen Arteon 2018-present
      # TODO: Supportable but untested vehicle, will require test and tune
      elif chassiscode == "3H":
        ret.mass = 1748 # Worst case with 4Motion, otherwise 1658
        ret.wheelbase = 2.837
        # Documented steering ratio: static 13.6:1, 2.8 turns L2L
        # No LiveParams tuned steerRatio yet
        # No LiveParams tuned tirestiffnessfactor yet

      # Mk3 Skoda Octavia, 2013-present
      # TODO: Supportable but untested vehicle, will require test and tune
      elif chassiscode == "5E" or chassiscode == "NE":
        ret.mass = 1360
        ret.wheelbase = 2.69
        # Copied from Golf:
        ret.centerToFront = ret.wheelbase * 0.45 # Estimated from public sources
        ret.lateralTuning.pid.kf = 0.00006
        # 0, 15, 50, 100, 250
        ret.lateralTuning.pid.kpV = [0.05, 0.10, 0.15, 0.40, 0.50]
        # ret.lateralTuning.pid.kpV = [0.05, 0.10, 0.15, 0.20, 0.50]
        # ret.lateralTuning.pid.kpV = [0.10, 0.20, 0.30, 0.40, 0.50]
        ret.lateralTuning.pid.kiV = [0.20, 0.15, 0.10, 0.05, 0.05]
        # Documented steering ratio:
        #   Golf R and GTI: progressive 14.1:1 to 9.5:1, 2.1 turns L2L (less software stop for R)
        #   All other variants: static 13.6:1, 2.76 turns L2L
        ret.steerRatio = 15.6 # LiveParams auto tuned for R
        ret.steerRatio = 14.1
        tire_stiffness_factor = 0.6 # LiveParams auto tuned

      # Mk3 Audi A3, S3, and RS3
      # TODO: Supportable but untested vehicle, will require test and tune
      elif chassiscode == "8V" or chassiscode == "FF":
        # FIXME: Wheelbase will vary between some versions (hatch vs sportback) so we may need more detection here somehow
        ret.mass = 1910
        ret.wheelbase = 2.61

      # Mk7 and Mk7.5 Volkswagen Golf, Alltrack, Sportwagen, GTI, Golf R, and
      # e-Golf, 2013-2020 depending on market.
      # Tested and supported for Golf R
      # TODO: All variants besides Golf R untested, will require test and tune
      elif chassiscode == "AU":
        # FIXME: Golf R and GTI may need different lateral tuning with their progressive variable ratio racks
        # FIXME: ... so we need to detect R and GTI distinct from regular Golf, and can't rely on VIN.. use Motor_Code_01 engine, or maybe look at LWI-to-EPS angle variance? May need to do both.
        # Mass will vary a bit, but wheelbase is identical for all variants.
        ret.mass = 1372
        ret.wheelbase = 2.630
        ret.centerToFront = ret.wheelbase * 0.45 # Estimated from public sources
        ret.lateralTuning.pid.kf = 0.00006
        ret.lateralTuning.pid.kpV = [0.10, 0.20, 0.30, 0.40, 0.50]
        ret.lateralTuning.pid.kiV = [0.20, 0.15, 0.10, 0.05, 0.05]
        # Documented steering ratio:
        #   Golf R and GTI: progressive 14.1:1 to 9.5:1, 2.1 turns L2L (less software stop for R)
        #   All other variants: static 13.6:1, 2.76 turns L2L
        #ret.steerRatio = 15.6 # LiveParams auto tuned for R
        ret.steerRatio = 14.1
        tire_stiffness_factor = 0.6 # LiveParams auto tuned

      # Mk7 Volkswagen Jetta (Sagitar in Chinese market), 2019-present
      # TODO: Supportable but untested vehicle, will require test and tune
      elif chassiscode == "BU":
        ret.mass = 1485
        ret.wheelbase = 2.681
        # Documented steering ratio:
        #  Jetta GLI: static 10.3:1, 2.1 turns L2L

      # Mk1 Volkswagen Atlas (Teramont in some markets), 2018-present
      # Fully tested and supported
      elif chassiscode == "CA":
        ret.mass = 2042
        ret.wheelbase = 2.97
        ret.lateralTuning.pid.kf = 0.00006
        ret.lateralTuning.pid.kpV = [0.05, 0.10, 0.15, 0.20, 0.50]
        ret.lateralTuning.pid.kiV = [0.20, 0.15, 0.10, 0.05, 0.05]
        # Documented steering ratio: static 16.3:1, 2.76 turns L2L
        ret.steerRatio = 15.1 # LiveParams auto tuned
        tire_stiffness_factor = 0.6 # LiveParams auto tuned, but wheel/tire size varies by trim package

      # Mk3 Audi TT/TTS/TTRS, 2014-present
      # TODO: Supportable but untested vehicle, will require test and tune
      elif chassiscode == "FV":
        ret.mass = 1328
        ret.wheelbase = 2.50

      # Mk1 Audi Q2 2017-present
      # TODO: Supportable but untested vehicle, will require test and tune
      elif chassiscode == "GA":
        ret.mass = 1205
        ret.wheelbase = 2.60

      # Additional common MQB parameters that need to be set after VIN parse
      ret.mass += STD_CARGO_KG
      # FIXME: have to set this after VIN parse so we have wheelbase, but need to test and allow overrides at the VIN level...
      ret.centerToFront = ret.wheelbase * 0.5

    # TODO: gate this on detection
    ret.enableCamera = True
    ret.steerRatioRear = 0.

    # No support for OP longitudinal control on Volkswagen at this time.
    ret.gasMaxBP = [0.]
    ret.gasMaxV = [.5]
    ret.brakeMaxBP = [0.]
    ret.brakeMaxV = [1.]
    ret.longitudinalTuning.deadzoneBP = [0.]
    ret.longitudinalTuning.deadzoneV = [0.]
    ret.longitudinalTuning.kpBP = [5., 35.]
    ret.longitudinalTuning.kpV = [2.4, 1.5]
    ret.longitudinalTuning.kiBP = [0.]
    ret.longitudinalTuning.kiV = [0.36]
    ret.stoppingControl = True
    ret.startAccel = 0.8

    # TODO: get actual value, for now starting with reasonable value for
    # civic and scaling by mass and wheelbase
    ret.rotationalInertia = scale_rot_inertia(ret.mass, ret.wheelbase)

    # TODO: start from empirically derived lateral slip stiffness for the civic and scale by
    # mass and CG position, so all cars will have approximately similar dyn behaviors
    ret.tireStiffnessFront, ret.tireStiffnessRear = scale_tire_stiffness(ret.mass, ret.wheelbase, ret.centerToFront,
                                                                         tire_stiffness_factor=tire_stiffness_factor)

    return ret

  # returns a car.CarState
  def update(self, c, can_strings):
    canMonoTimes = []

    params = Params()

    self.gw_cp.update_strings(can_strings)
    self.ex_cp.update_strings(can_strings)

    self.CS.update(self.gw_cp, self.ex_cp)

    # create message
    ret = car.CarState.new_message()

    ret.canValid = self.gw_cp.can_valid and self.ex_cp.can_valid

    # speeds
    ret.vEgo = self.CS.v_ego
    ret.aEgo = self.CS.a_ego
    ret.vEgoRaw = self.CS.v_ego_raw
    ret.yawRate = self.CS.yaw_rate * CV.DEG_TO_RAD
    ret.standstill = self.CS.standstill
    ret.wheelSpeeds.fl = self.CS.v_wheel_fl
    ret.wheelSpeeds.fr = self.CS.v_wheel_fr
    ret.wheelSpeeds.rl = self.CS.v_wheel_rl
    ret.wheelSpeeds.rr = self.CS.v_wheel_rr

    # steering wheel
    ret.steeringAngle = self.CS.angle_steers
    ret.steeringRate = self.CS.angle_steers_rate

    # torque and user override. Driver awareness
    # timer resets when the user uses the steering wheel.
    ret.steeringPressed = self.CS.steer_override
    ret.steeringTorque = self.CS.steer_torque_driver

    # ACC cruise state
    ret.cruiseState.available = self.CS.acc_enabled
    ret.cruiseState.enabled = self.CS.acc_active
    ret.cruiseState.speed = self.CS.cruise_set_speed

    # Blinker updates
    # TODO: We have a signal for actual external blinker state, need to import that in addition to the turnstalk
    ret.leftBlinker = bool(self.CS.left_blinker_on)
    ret.rightBlinker = bool(self.CS.right_blinker_on)

    # Doors open, seatbelt unfastened
    ret.doorOpen = not self.CS.door_all_closed
    ret.seatbeltUnlatched = not self.CS.seatbelt

    # Gas, brakes and shifting
    ret.gas = self.CS.pedal_gas / 100
    ret.gasPressed = self.CS.pedal_gas > 0
    ret.brake = self.CS.user_brake / 250 # FIXME: approximated, verify
    ret.brakePressed = bool(self.CS.brake_pressed)
    ret.brakeLights = bool(self.CS.brake_lights)
    ret.gearShifter = self.CS.gear_shifter

    # Update the EON metric configuration to match the car at first startup,
    # or if there's been a change.
    if self.CS.is_metric != self.CS.is_metric_prev:
      params.put("IsMetric", "1" if self.CS.is_metric == 1 else "0")

    # Update dynamic vehicle mass calculated by the drivetrain coordinator.
    # NOTE: At this time, OP's ParamsLearner probably won't make use of a mass
    # value that gets changed after startup.
    # FIXME: Can't actually do this without adding mass to CarState in capnp
    # ret.mass = self.CS.mass

    buttonEvents = []

    # Process button press or release events from ACC steering wheel or
    # control stalk buttons. We don't have enough room in capnp to capture
    # all seven buttons, even with the alt buttons, so the timegap button
    # is not seen as an event at this time.
    # FIXME: Add real main, set, resume, and timegap buttons to be added to capnp
    if self.CS.gra_acc_buttons != self.CS.gra_acc_buttons_prev:
      if self.CS.gra_acc_buttons["main"] != self.CS.gra_acc_buttons_prev["main"]:
        be = car.CarState.ButtonEvent.new_message()
        be.type = 'altButton3'
        be.pressed = bool(self.CS.gra_acc_buttons["main"])
        buttonEvents.append(be)
      if self.CS.gra_acc_buttons["set"] != self.CS.gra_acc_buttons_prev["set"]:
        be = car.CarState.ButtonEvent.new_message()
        be.type = 'altButton1'
        be.pressed = bool(self.CS.gra_acc_buttons["set"])
        buttonEvents.append(be)
      if self.CS.gra_acc_buttons["resume"] != self.CS.gra_acc_buttons_prev["resume"]:
        be = car.CarState.ButtonEvent.new_message()
        be.type = 'altButton2'
        be.pressed = bool(self.CS.gra_acc_buttons["resume"])
        buttonEvents.append(be)
      if self.CS.gra_acc_buttons["cancel"] != self.CS.gra_acc_buttons_prev["cancel"]:
        be = car.CarState.ButtonEvent.new_message()
        be.type = 'cancel'
        be.pressed = bool(self.CS.gra_acc_buttons["cancel"])
        buttonEvents.append(be)
      if self.CS.gra_acc_buttons["accel"] != self.CS.gra_acc_buttons_prev["accel"]:
        be = car.CarState.ButtonEvent.new_message()
        be.type = 'accelCruise'
        be.pressed = bool(self.CS.gra_acc_buttons["accel"])
        buttonEvents.append(be)
      if self.CS.gra_acc_buttons["decel"] != self.CS.gra_acc_buttons_prev["decel"]:
        be = car.CarState.ButtonEvent.new_message()
        be.type = 'decelCruise'
        be.pressed = bool(self.CS.gra_acc_buttons["decel"])
        buttonEvents.append(be)

    # blinkers
    if self.CS.left_blinker_on != self.CS.prev_left_blinker_on:
      be = car.CarState.ButtonEvent.new_message()
      be.type = 'leftBlinker'
      be.pressed = bool(self.CS.left_blinker_on)
      buttonEvents.append(be)

    if self.CS.right_blinker_on != self.CS.prev_right_blinker_on:
      be = car.CarState.ButtonEvent.new_message()
      be.type = 'rightBlinker'
      be.pressed = bool(self.CS.right_blinker_on)
      buttonEvents.append(be)

    events = []

    # Vehicle operation safety checks and events
    if ret.doorOpen:
      events.append(create_event('doorOpen', [ET.NO_ENTRY, ET.SOFT_DISABLE]))
    if ret.seatbeltUnlatched:
      events.append(create_event('seatbeltNotLatched', [ET.NO_ENTRY, ET.SOFT_DISABLE]))
    if ret.gearShifter == 'reverse':
      events.append(create_event('reverseGear', [ET.NO_ENTRY, ET.IMMEDIATE_DISABLE]))
    if not ret.gearShifter == 'drive':
      events.append(create_event('wrongGear', [ET.NO_ENTRY, ET.SOFT_DISABLE]))
    if self.CS.esp_disabled:
      events.append(create_event('espDisabled', [ET.NO_ENTRY, ET.SOFT_DISABLE]))
    if self.CS.park_brake:
      events.append(create_event('parkBrake', [ET.NO_ENTRY, ET.USER_DISABLE]))
    if not self.CS.acc_enabled:
      events.append(create_event('wrongCarMode', [ET.NO_ENTRY, ET.USER_DISABLE]))

    # Vehicle health safety checks and events
    if self.CS.acc_error:
      events.append(create_event('radarFault', [ET.NO_ENTRY, ET.IMMEDIATE_DISABLE]))
    if self.CS.steer_error:
      events.append(create_event('steerTempUnavailable', [ET.NO_ENTRY, ET.IMMEDIATE_DISABLE]))

    # Per the Comma safety model, disable on pedals rising edge or when brake
    # is pressed and speed isn't zero.
    if (ret.gasPressed and not self.gas_pressed_prev) or \
            (ret.brakePressed and (not self.brake_pressed_prev or not ret.standstill)):
      events.append(create_event('pedalPressed', [ET.NO_ENTRY, ET.USER_DISABLE]))
    if ret.gasPressed:
      events.append(create_event('pedalPressed', [ET.PRE_ENABLE]))

    if self.CS.CP.openpilotLongitudinalControl:
      # Engagement and longitudinal control by openpilot, using vision or radar
      # fusion. Send OP engagement events based on the user ACC set and resume
      # buttons. The ACC gap adjustment increase and decrease buttons should
      # not trigger engagement.
      # TODO: For future visiond or radar fusion use; not currently supported.
      for b in buttonEvents:
        if b.type in ["altButton1", "altButton2"] and b.pressed:
          events.append(create_event('buttonEnable', [ET.ENABLE]))
        if b.type in ["cancel"] and b.pressed:
          events.append(create_event('buttonCancel', [ET.USER_DISABLE]))
    else:
      # Engagement and longitudinal control using stock ACC. Observe the car's
      # ACC engagement events and set OP engagement to match. If an OP safety
      # issue would prevent OP engagement, we prevent stock ACC from engaging
      # by filtering set/resume button presses later in CarController.
      self.engageable = not bool(get_events(events, [ET.NO_ENTRY]))
      if self.CS.acc_active and not self.acc_active_prev:
        events.append(create_event('pcmEnable', [ET.ENABLE]))
      # Make sure we disengage if stock ACC does, just in case stock ACC goes
      # away for a reason not already handled by OP above.
      if not self.CS.acc_active:
        events.append(create_event('pcmDisable', [ET.USER_DISABLE]))
      self.acc_active_prev = self.CS.acc_active

    ret.events = events
    ret.buttonEvents = buttonEvents
    ret.canMonoTimes = canMonoTimes

    # update previous brake/gas pressed
    self.gas_pressed_prev = ret.gasPressed
    self.brake_pressed_prev = ret.brakePressed

    # cast to reader so it can't be modified
    return ret.as_reader()

  def apply(self, c):
    can_sends = self.CC.update(c.enabled, self.CS, self.frame, c.actuators,
                   c.hudControl.visualAlert,
                   c.hudControl.audibleAlert,
                   c.hudControl.leftLaneVisible,
                   c.hudControl.rightLaneVisible,
                   self.engageable)
    self.frame += 1
    return can_sends
