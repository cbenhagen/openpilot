from common.numpy_fast import clip, interp
from common.realtime import sec_since_boot
from selfdrive.config import Conversions as CV
from selfdrive.boardd.boardd import can_list_to_can_capnp
from selfdrive.car.vw.carstate import CarState, get_gateway_can_parser, get_extended_can_parser
from selfdrive.car.vw import vwcan
from selfdrive.car.vw.values import CAR, DBC
from selfdrive.can.packer import CANPacker


class CarControllerParams():
  def __init__(self, car_fingerprint):
    self.STEER_MAX = 300               # max stock steer 300
    self.STEER_STEP = 2             # how often we update the steer cmd
    self.STEER_DELTA_UP = 50           # torque increase per refresh
    self.STEER_DELTA_DOWN = 50         # torque decrease per refresh
    self.STEER_DRIVER_ALLOWANCE = 20   # allowed driver torque before start limiting
    self.STEER_DRIVER_MULTIPLIER = 1
    self.STEER_DRIVER_FACTOR = 1



class CarController(object):
  def __init__(self, canbus, car_fingerprint):
    self.start_time = sec_since_boot()
    self.counter = 0
    self.apply_steer_last = 0
    self.car_fingerprint = car_fingerprint

    # Setup detection helper. Routes commands to
    # an appropriate CAN bus number.
    self.canbus = canbus
    self.params = CarControllerParams(car_fingerprint)
    print(DBC)
    self.packer_gw = CANPacker(DBC[car_fingerprint]['pt'])

  def update(self, sendcan, enabled, CS, frame, actuators):
    """ Controls thread """

    P = self.params

    # Send CAN commands.
    can_sends = []
    canbus = self.canbus

    ### STEER ###

    if (frame % P.STEER_STEP) == 0:
      if enabled:
        #final_steer = actuators.steer if enabled else 0.
        #apply_steer = final_steer * P.STEER_MAX
        ## limits due to driver torque
        #driver_max_torque = P.STEER_MAX + (P.STEER_DRIVER_ALLOWANCE + CS.steer_torque_driver * P.STEER_DRIVER_FACTOR) * P.STEER_DRIVER_MULTIPLIER
        #driver_min_torque = -P.STEER_MAX + (-P.STEER_DRIVER_ALLOWANCE + CS.steer_torque_driver * P.STEER_DRIVER_FACTOR) * P.STEER_DRIVER_MULTIPLIER
        #max_steer_allowed = max(min(P.STEER_MAX, driver_max_torque), 0)
        #min_steer_allowed = min(max(-P.STEER_MAX, driver_min_torque), 0)
        #apply_steer = clip(apply_steer, min_steer_allowed, max_steer_allowed)

        ## slow rate if steer torque increases in magnitude
        #if self.apply_steer_last > 0:
        #  apply_steer = clip(apply_steer, max(self.apply_steer_last - P.STEER_DELTA_DOWN, -P.STEER_DELTA_UP), self.apply_steer_last + P.STEER_DELTA_UP)
        #else:
        #  apply_steer = clip(apply_steer, self.apply_steer_last - P.STEER_DELTA_UP, min(self.apply_steer_last + P.STEER_DELTA_DOWN, P.STEER_DELTA_UP))

        #apply_steer = int(round(apply_steer))
        #self.apply_steer_last = apply_steer

        apply_steer = int(clip(actuators.steer * P.STEER_MAX, -P.STEER_MAX, P.STEER_MAX))

        # steer must be a positive value
        if apply_steer > -1:
          steer = apply_steer
          right = 0
        else:
          steer = abs(apply_steer)
          right = 1
      else:
        steer = 0
        right = 0

      lkas_enabled = 1
      idx = (frame / P.STEER_STEP) % 16

      if CS.standstill:
        steer = 0
        lkas_enabled = 0

      can_sends.append(vwcan.create_steering_control(self.packer_gw, canbus.gateway, CS.CP.carFingerprint, steer, idx, lkas_enabled, right))

    sendcan.send(can_list_to_can_capnp(can_sends, msgtype='sendcan').to_bytes())
