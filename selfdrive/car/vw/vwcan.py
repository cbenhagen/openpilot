import crcmod
from selfdrive.car.vw.values import CAR, DBC

# Python crcmod works differently from every other CRC calculator in the planet in some subtle
# way. The implied leading 1 on the polynomial isn't a big deal, but for some reason, we need
# to feed it initCrc 0x00 instead of 0xFF like it should be.
vw_checksum = crcmod.mkCrcFun(0x12F, initCrc=0x00, rev=False, xorOut=0xFF)

def create_steering_control(packer, bus, car_fingerprint, steer, idx, lkas_enabled, right):
  values = {
    "CRC": 0xB5, # Magic value that stands in for the CRC during calculation
    "Counter": idx,
    "3": 3,
    "Steer_Torque": steer,
    "LKAS_ON": lkas_enabled,
    "_LKAS_Boost": right,
    "1": 1,
    "Torque_Not_0": 1 if steer != 0 else 0,
    "254": 254,
    "7": 7,
  }
  dat = packer.make_can_msg("LKAS", 0, values)[2]
  dat = dat + '\0'
  checksum = vw_checksum(dat)
  values["CRC"] = checksum
  return packer.make_can_msg("LKAS", 0, values)