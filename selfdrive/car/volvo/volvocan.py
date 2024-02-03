from selfdrive.car.volvo.values import PLATFORM

def cancelACC(packer):
  # Send cancel button to disengage ACC
  msg = {
    "ACCOnOffBtn" : 1,
    "ACCOnOffBtnInv" : 0,
  }

  return packer.make_can_msg("CCButtons", 0, msg)


def manipulateServo(packer, CS):
  # Manipulate data from servo to FSM
  # Set LKATorque and LKAActive to zero otherwise LKA will be disabled. (Check dbc)
  msg = {
      "LKATorque" : 0,
      "SteeringAngleServo" : CS.out.steeringAngleDeg,
      "byte0" : CS.PSCMInfo["byte0"],
      "byte4" : CS.PSCMInfo["byte4"],
      "byte7" : CS.PSCMInfo["byte7"],
      "LKAActive" : CS.PSCMInfo["LKAActive"] & 0xF5,
      "SteeringWheelRateOfChange" : CS.PSCMInfo["SteeringWheelRateOfChange"],
      "steeringRateDeg" : CS.PSCMInfo["SteeringWheelRateOfChange"],
  }

  return packer.make_can_msg("PSCM1", 2, msg)


def create_chksum(dat):
  # Input: dat byte array, and fingerprint
  # Steering direction = 0 -> 3
  # TrqLim = 0 -> 255
  # Steering angle request = -360 -> 360
    
  # Extract LKAAngleRequest, LKADirection and Unknown
  steer_angle_request = ((dat[3] & 0x3F) << 8) + dat[4]
  steering_direction_request = dat[5] & 0x03  
  trqlim = dat[2]
  
  # Sum of all bytes, carry ignored.
  s = (trqlim + steering_direction_request + steer_angle_request + (steer_angle_request >> 8)) & 0xFF
  # Checksum is inverted sum of all bytes
  return s ^ 0xFF


def create_steering_control(packer, SteerCommand):
 
  # Set common parameters
  values = {
    "LKAAngleReq": SteerCommand.angle_request,
    "LKASteerDirection": SteerCommand.steer_direction,
    "TrqLim": SteerCommand.trqlim,
  }

  # Set car specific parameters
  values_static = {
      "SET_X_22": 0x25, # Test these values: 0x24, 0x22
      "SET_X_02": 0,    # Test 0x00, 0x02
      "SET_X_10": 0x10, # Test 0x10, 0x1c, 0x18, 0x00
      "SET_X_A4": 0xa7, # Test 0xa4, 0xa6, 0xa5, 0xe5, 0xe7
  }

  # Combine common and static parameters
  values.update(values_static)

  # Create can message with "translated" can bytes.
  dat = packer.make_can_msg("FSM2", 0, values)[2]
  values["Checksum"] = create_chksum(dat)

  return packer.make_can_msg("FSM2", 0, values)
