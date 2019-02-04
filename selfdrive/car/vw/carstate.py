import numpy as np
from cereal import car
from common.kalman.simple_kalman import KF1D
from selfdrive.config import Conversions as CV
from selfdrive.can.parser import CANParser
from selfdrive.car.vw.values import DBC, CAR

def get_gateway_can_parser(CP, canbus):
  # this function generates lists for signal, messages and initial values
  signals = [
    # sig_name, sig_address, default
    ("LWI_Lenkradwinkel", "LWI_01", 0),         # Absolute steering angle
    ("LWI_VZ_Lenkradwinkel", "LWI_01", 0),      # Steering angle sign
    ("LWI_Lenkradw_Geschw", "LWI_01", 0),       # Absolute steering rate
    ("LWI_VZ_Lenkradw_Geschw", "LWI_01", 0),    # Steering rate sign
    ("ESP_HL_Radgeschw_02", "ESP_19", 0),       # ABS wheel speed, rear left
    ("ESP_HR_Radgeschw_02", "ESP_19", 0),       # ABS wheel speed, rear right
    ("ESP_VL_Radgeschw_02", "ESP_19", 0),       # ABS wheel speed, front left
    ("ESP_VR_Radgeschw_02", "ESP_19", 0),       # ABS wheel speed, front right
    ("ZV_FT_offen", "Gateway_72", 0),           # Door open, driver
    ("ZV_BT_offen", "Gateway_72", 0),           # Door open, passenger
    ("ZV_HFS_offen", "Gateway_72", 0),          # Door open, rear left
    ("ZV_HBFS_offen", "Gateway_72", 0),         # Door open, rear right
    ("ZV_HD_offen", "Gateway_72", 0),           # Trunk or hatch open
    ("BH_Blinker_li", "Gateway_72", 0),         # Left turn signal on
    ("BH_Blinker_re", "Gateway_72", 0),         # Right turn signal on
    ("GE_Fahrstufe", "Getriebe_11", 0),         # Transmission gear selector position
    ("AB_Gurtwarn_VF", "Airbag_01", 0),         # Seatbelt warning, driver
    ("AB_Gurtwarn_VB", "Airbag_01", 0),         # Seatbelt warning, passenger
    ("ESP_Fahrer_bremst", "ESP_05", 0),         # Brake pedal pressed
    ("MO_Fahrpedalrohwert_01", "Motor_20", 0),  # Accelerator pedal value
    ("ACC_Status_ACC", "ACC_06", 0),  # ACC engagement status
  ]

  checks = [
    # sig_address, frequency
    ("LWI_01", 100),      # From J500 Steering Assist with integrated sensors
    ("ESP_19", 100),      # From J104 ABS/ESP controller
    ("ESP_05", 50),       # From J104 ABS/ESP controller
    ("Motor_20", 50),     # From J623 Engine control module
    ("Gateway_72", 10),   # From J533 CAN gateway (aggregated data)
    ("Getriebe_11", 20),  # From J743 Auto transmission control module
    ("Airbag_01", 20),    # From J234 Airbag control module
    ("ACC_06", 50),  # From JXXX ACC radar control module
  ]

  return CANParser(DBC[CP.carFingerprint]['pt'], signals, checks, canbus.gateway)


def get_extended_can_parser(CP, canbus):

  signals = [
    # sig_name, sig_address, default
  ]

  checks = [
    # sig_address, frequency
  ]

  return CANParser(DBC[CP.carFingerprint]['pt'], signals, checks, canbus.extended)

class CarState(object):
  def __init__(self, CP, canbus):
    # initialize can parser
    self.CP = CP


    self.car_fingerprint = CP.carFingerprint
    self.left_blinker_on = False
    self.prev_left_blinker_on = False
    self.right_blinker_on = False
    self.prev_right_blinker_on = False
    self.steer_torque_driver = 0
    self.steer_not_allowed = False
    self.main_on = False
    self.angle_steers_rate = 0

    # vEgo kalman filter
    dt = 0.01
    self.v_ego_kf = KF1D(x0=np.matrix([[0.], [0.]]),
                         A=np.matrix([[1., dt], [0., 1.]]),
                         C=np.matrix([1., 0.]),
                         K=np.matrix([[0.12287673], [0.29666309]]))
    self.v_ego = 0.

  def update(self, gw_cp, ex_cp):

    self.can_valid = True

    self.v_wheel_fl = gw_cp.vl["ESP_19"]['ESP_HL_Radgeschw_02'] * CV.KPH_TO_MS
    self.v_wheel_fr = gw_cp.vl["ESP_19"]['ESP_HR_Radgeschw_02'] * CV.KPH_TO_MS
    self.v_wheel_rl = gw_cp.vl["ESP_19"]['ESP_VL_Radgeschw_02'] * CV.KPH_TO_MS
    self.v_wheel_rr = gw_cp.vl["ESP_19"]['ESP_VR_Radgeschw_02'] * CV.KPH_TO_MS
    speed_estimate = float(np.mean([self.v_wheel_fl, self.v_wheel_fr, self.v_wheel_rl, self.v_wheel_rr]))

    self.v_ego_raw = speed_estimate
    v_ego_x = self.v_ego_kf.update(speed_estimate)
    self.v_ego = float(v_ego_x[0])
    self.a_ego = float(v_ego_x[1])
    self.standstill = self.v_ego_raw < 0.01

    # Update door and trunk/hatch lid status
    self.door_all_closed = not any([gw_cp.vl["Gateway_72"]['ZV_FT_offen'],
                                    gw_cp.vl["Gateway_72"]['ZV_BT_offen'],
                                    gw_cp.vl["Gateway_72"]['ZV_HFS_offen'],
                                    gw_cp.vl["Gateway_72"]['ZV_HBFS_offen'],
                                    gw_cp.vl["Gateway_72"]['ZV_HD_offen']])

    # Update turn signal status
    self.prev_left_blinker_on = self.left_blinker_on
    self.prev_right_blinker_on = self.right_blinker_on

    self.left_blinker_on = gw_cp.vl["Gateway_72"]['BH_Blinker_li']
    self.right_blinker_on = gw_cp.vl["Gateway_72"]['BH_Blinker_re']

    # Update seatbelt warning status
    self.seatbelt = not gw_cp.vl["Airbag_01"]["AB_Gurtwarn_VF"]

    self.steer_torque_driver = 0 #FIXME
    self.acc_active = 1 if gw_cp.vl["ACC_06"]['ACC_Status_ACC'] > 2 else 0
    self.main_on = self.acc_active

    self.steer_override = abs(self.steer_torque_driver) > 1.0

    if gw_cp.vl["LWI_01"]['LWI_VZ_Lenkradwinkel'] == 1:
      self.angle_steers = gw_cp.vl["LWI_01"]['LWI_Lenkradwinkel'] * -1
    else:
      self.angle_steers = gw_cp.vl["LWI_01"]['LWI_Lenkradwinkel']
    # calculate steer rate
    if gw_cp.vl["LWI_01"]['LWI_VZ_Lenkradw_Geschw'] == 1:
      self.angle_steers_rate = gw_cp.vl["LWI_01"]['LWI_Lenkradw_Geschw'] * -1
    else:
      self.angle_steers_rate = gw_cp.vl["LWI_01"]['LWI_Lenkradw_Geschw']
