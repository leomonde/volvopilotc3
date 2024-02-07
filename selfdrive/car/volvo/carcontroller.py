from dataclasses import dataclass
from common.numpy_fast import clip, interp
from opendbc.can.packer import CANPacker
from selfdrive.car import apply_std_steer_angle_limits
from selfdrive.car.volvo import volvocan
from selfdrive.car.volvo.values import CAR, PLATFORM, CarControllerParams

# Trqlim have no real effect, unused.
@dataclass
class SteerCommand:
  angle_request = 0
  steer_direction = 0
  trqlim = 0


class CarController():
  def __init__(self, dbc_name, CP, VM):
    # Setup detection helper. Routes commands to
    # an appropriate CAN bus number.
    self.CP = CP
    self.car_fingerprint = CP.carFingerprint
    self.CCP = CarControllerParams(CP)
    self.packer = CANPacker(dbc_name)
    self.frame = 0

    # steering related
    self.angle_request_prev = 0
    
    # SteerCommand
    self.SteerCommand = SteerCommand

    # V60
    self.acc_enabled_prev = 0
    self.des_steer_direction_prev = 0
    self.dir_state = 0
    self.block_steering = 0
    self.steer_direction_bf_block = 0
    self.des_steer_direction_prev = 0
    self.UNBLOCKED = 0
    self.BLOCKED = 1
    self.BLOCK_LEN = 16

    # Diag
    self.doDTCRequests = True  # Turn on and off DTC requests
    self.checkPN = False       # Check partnumbers
    self.clearDtcs = False      # Clear dtc on startup
    self.timeout = 0           # Set to 0 as init
    self.diagRequest = { 
      "byte0": 0x03,
      "byte1": 0x19,
      "byte2": 0x02,
      "byte3": 0x02,
      }
    self.flowControl = { 
      "byte0": 0x30,
      "byte1": 0x00,
      "byte2": 0x00,
      "byte3": 0x00,
      }
    self.clearDTC = {
      "byte0": 0x04,
      "byte1": 0x14,
      "byte2": 0xFF,
      "byte3": 0xFF,
      "byte4": 0xFF,
      }

    # Part number
    self.cnt = 0          # Init at 0 always
    self.sndNxtFrame = 0  # Init at low value 
    self.dictKeys = ["byte"+str(x) for x in range(8)]
    startdid = 0xf1a1     # Start with this DID (Data IDentifier, read UDS Spec for more info)
    self.dids = [x for x in range(startdid, startdid+9)]

  def dir_change(self, steer_direction, error):
    """ Filters out direction changes
    
    Uses a simple state machine to determine if we should 
    block or allow the steer_direction bits to pass thru.

    """
    
    dessd = steer_direction
    dzError = 0 if abs(error) < self.CCP.DEADZONE else error 
    tState = -1 

    # Update prev with desired if just enabled.
    self.des_steer_direction_prev = steer_direction if not self.acc_enabled_prev else self.des_steer_direction_prev
    
    # Check conditions for state change
    if self.dir_state == self.UNBLOCKED:
      tState = self.BLOCKED if (steer_direction != self.des_steer_direction_prev and dzError != 0) else tState
    elif self.dir_state == self.BLOCKED:
      if (steer_direction == self.steer_direction_bf_block) or (self.block_steering <= 0) or (dzError == 0):
        tState = self.UNBLOCKED

    # State transition
    if tState == self.UNBLOCKED:
      self.dir_state = self.UNBLOCKED
    elif tState == self.BLOCKED:
      self.steer_direction_bf_block = self.des_steer_direction_prev  
      self.block_steering = self.BLOCK_LEN
      self.dir_state = self.BLOCKED

    #  Run actions in state
    if self.dir_state == self.UNBLOCKED:
      if dzError == 0:
        steer_direction = self.des_steer_direction_prev # Set old request when inside deadzone
    if self.dir_state == self.BLOCKED:
      self.block_steering -= 1
      steer_direction = self.CCP.STEER_NO

    #print("State:{} Sd:{} Sdp:{} Bs:{} Dz:{:.2f} Err:{:.2f}".format(self.dir_state, steer_direction, self.des_steer_direction_prev, self.block_steering, dzError, error))
    return steer_direction

  def update(self, CC, CS, now_nanos):

    """ Controls thread """
    actuators = CC.actuators
    hud_control = CC.hudControl

    # Send CAN commands.
    can_sends = []

    # run at 50hz
    if (self.frame % 2 == 0):

      if CC.latActive: #and CS.out.vEgo > self.CP.minSteerSpeed:
        current_steer_angle = CS.out.steeringAngleDeg
        self.SteerCommand.angle_request = apply_std_steer_angle_limits(actuators.steeringAngleDeg, self.angle_request_prev, CS.out.vEgoRaw, CarControllerParams)
        self.SteerCommand.steer_direction = self.CCP.STEER_LEFT if self.SteerCommand.angle_request > 0 else self.CCP.STEER_RIGHT
        self.SteerCommand.steer_direction = self.dir_change(self.SteerCommand.steer_direction, current_steer_angle-self.SteerCommand.angle_request) # Filter the direction change 

      else:
        self.SteerCommand.steer_direction = self.CCP.NO_STEER
        self.SteerCommand.angle_request = 0
      
      # Cancel ACC if engaged when OP is not, but only above minimum steering speed.
      if not CC.latActive and CS.out.cruiseState.enabled \
         and CS.out.vEgo > self.CP.minSteerSpeed:
        can_sends.append(volvocan.cancelACC(self.packer))

      # update stored values
      self.acc_enabled_prev = 1
      self.angle_request_prev = self.SteerCommand.angle_request
      if self.SteerCommand.steer_direction == self.CCP.STEER_RIGHT or self.SteerCommand.steer_direction == self.CCP.STEER_LEFT: # TODO: Move this inside dir_change, think it should work?
        self.des_steer_direction_prev = self.SteerCommand.steer_direction  # Used for dir_change function

      # Manipulate data from servo to FSM
      # Avoids faults that will stop servo from accepting steering commands.
      can_sends.append(volvocan.manipulateServo(self.packer, CS))
    
      # send can, add to list.
      can_sends.append(volvocan.create_steering_control(self.packer, self.SteerCommand))


    # Send diagnostic requests
    if(self.doDTCRequests):
      if(self.frame % 100 == 0) and (not self.clearDtcs):
        # Request diagnostic codes, 2 Hz
        can_sends.append(self.packer.make_can_msg("diagFSMReq", 2, self.diagRequest))
        #can_sends.append(self.packer.make_can_msg("diagGlobalReq", 2, self.diagRequest))
        can_sends.append(self.packer.make_can_msg("diagGlobalReq", 0, self.diagRequest))
        #can_sends.append(self.packer.make_can_msg("diagPSCMReq", 0, self.diagRequest))
        #can_sends.append(self.packer.make_can_msg("diagCEMReq", 0, self.diagRequest))
        #can_sends.append(self.packer.make_can_msg("diagCVMReq", 0, self.diagRequest))
        self.timeout = self.frame + 5 # Set wait time 

      # Handle flow control in case of many DTC
      if self.frame > self.timeout and self.timeout > 0: # Wait fix time before sending flow control, otherwise just spamming...
        self.timeout = 0 
        if (CS.diag.diagFSMResp & 0x10000000):
          can_sends.append(self.packer.make_can_msg("diagFSMReq", 2, self.flowControl))
        if (CS.diag.diagCEMResp & 0x10000000):
          can_sends.append(self.packer.make_can_msg("diagCEMReq", 0, self.flowControl))
        if (CS.diag.diagPSCMResp & 0x10000000):
          can_sends.append(self.packer.make_can_msg("diagPSCMReq", 0, self.flowControl))
        if (CS.diag.diagCVMResp & 0x10000000):
          can_sends.append(self.packer.make_can_msg("diagCVMReq", 0, self.flowControl))

      # Check part numbers
      if self.checkPN and self.frame > 100 and self.frame > self.sndNxtFrame:
        if self.cnt < len(self.dids):
          did = [0x03, 0x22, (self.dids[self.cnt] & 0xff00)>>8, self.dids[self.cnt] & 0x00ff] # Create diagnostic command
          did.extend([0]*(8-len(did))) 
          diagReq = dict(zip(self.dictKeys,did))
          #can_sends.append(self.packer.make_can_msg("diagGlobalReq", 2, diagReq))
          #can_sends.append(self.packer.make_can_msg("diagGlobalReq", 0, diagReq))
          can_sends.append(self.packer.make_can_msg("diagFSMReq", 2, diagReq))
          can_sends.append(self.packer.make_can_msg("diagCEMReq", 0, diagReq))
          can_sends.append(self.packer.make_can_msg("diagPSCMReq", 0, diagReq))
          can_sends.append(self.packer.make_can_msg("diagCVMReq", 0, diagReq))
          self.cnt += 1
          self.timeout = self.frame+5             # When to send flowControl
          self.sndNxtFrame = self.timeout+5  # When to send next part number request

        elif True:                           # Stop when list has been looped thru.
          self.checkPN = False

      # Clear DTCs in FSM on start
      # TODO check for engine running before clearing dtc.
      if(self.clearDtcs and (self.frame > 0) and (self.frame % 500 == 0)):
        can_sends.append(self.packer.make_can_msg("diagGlobalReq", 0, self.clearDTC))
        can_sends.append(self.packer.make_can_msg("diagFSMReq", 2, self.clearDTC))
        #can_sends.append(self.packer.make_can_msg("diagPSCMReq", 0, self.clearDTC))
        #can_sends.append(self.packer.make_can_msg("diagCEMReq", 0, self.clearDTC))
        self.clearDtcs = False

    # Store old values
    new_actuators = actuators.copy()
    new_actuators.steeringAngleDeg = self.SteerCommand.angle_request

    self.frame += 1
    return new_actuators, can_sends
