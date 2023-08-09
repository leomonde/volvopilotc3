from common.numpy_fast import clip, interp
from opendbc.can.packer import CANPacker
from selfdrive.car import apply_std_steer_angle_limits
from selfdrive.car.volvo import volvocan
from selfdrive.car.volvo.values import CAR, PLATFORM, CarControllerParams
from collections import deque

class SteerCommand:
  angle_request = 0
  steer_direction = 0
  trqlim = 0


class CarController():
  def __init__(self, dbc_name, CP, VW):
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
    self.trq_fifo = deque([])  
    self.fault_frame = -200

    # Diag
    self.doDTCRequests = True  # Turn on and off DTC requests
    self.checkPN = False       # Check partnumbers
    self.clearDtcs = False     # Set false to stop sending diagnostic requests 
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

  def update(self, CC, CS, now_nanos):
             #actuators, 
             #visualAlert, leftLaneVisible,
             #rightLaneVisible, leadVisible,
             #leftLaneDepart, rightLaneDepart):
    """ Controls thread """
    actuators = CC.actuators
    hud_control = CC.hudControl

    # Send CAN commands.
    can_sends = []

    # run at 50hz
    if (self.frame % 2 == 0):
      fingerprint = self.car_fingerprint
       
      if CC.latActive and CS.out.vEgo > self.CP.minSteerSpeed:
        self.SteerCommand.angle_request = apply_std_steer_angle_limits(actuators.steeringAngleDeg, self.angle_request_prev, CS.out.vEgoRaw, CarControllerParams) 

        # Create trqlim from angle request (before constraints)
        self.SteerCommand.trqlim = 0
        self.SteerCommand.steer_direction = self.CCP.STEER

      else:
        self.SteerCommand.steer_direction = self.CCP.STEER_NO
        self.SteerCommand.trqlim = 0
        self.SteerCommand.angle_request = 0
        #self.SteerCommand.angle_request = clip(CS.out.steeringAngleDeg, -359.95, 359.90)  # Cap values at max min values (Cap 2 steps from max min). Max=359.99445, Min=-360.0384 
      
      # Count no of consequtive samples of zero torque by lka.
      # Try to recover, blocking steering request for 2 seconds.
      if fingerprint in PLATFORM.C1:
        if CC.latActive and CS.out.vEgo > self.CP.minSteerSpeed:
          self.trq_fifo.append(CS.PSCMInfo.LKATorque)
          if len(self.trq_fifo) > self.CCP.N_ZERO_TRQ:
            self.trq_fifo.popleft()
        else:
          self.trq_fifo.clear()
          self.fault_frame = -200

        if (self.trq_fifo.count(0) >= self.CCP.N_ZERO_TRQ) and (self.frame == -200):
          self.frame = self.frame+100

        if CC.latActive and (self.frame < self.fault_frame):
          self.SteerCommand.steer_direction = self.CCP.STEER_NO

        if self.frame > self.fault_frame+8:  # Ignore steerWarning for another 8 samples.
          self.fault_frame = -200     


      # update stored values
      self.angle_request_prev = self.SteerCommand.angle_request
      
      # Manipulate data from servo to FSM
      # Avoid fault codes, that will stop LKA
      can_sends.append(volvocan.manipulateServo(self.packer, self.car_fingerprint, CS))
    
      # send can, add to list.
      can_sends.append(volvocan.create_steering_control(self.packer, self.frame, self.CP.carFingerprint, self.SteerCommand, CS.FSMInfo))
    
    
    # Cancel ACC if engaged when OP is not.
    if not CC.latActive and CS.out.cruiseState.enabled:
      can_sends.append(volvocan.cancelACC(self.packer, self.car_fingerprint, CS))


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
