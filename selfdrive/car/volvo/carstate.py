from cereal import car
from common.kalman.simple_kalman import KF1D
from common.conversions import Conversions as CV
from selfdrive.car.interfaces import CarStateBase
from opendbc.can.parser import CANParser
from opendbc.can.can_define import CANDefine
from selfdrive.car.volvo.values import CAR, DBC, CarControllerParams, PLATFORM
from collections import deque

class diagInfo():
  def __init__(self):
    self.diagFSMResp = 0
    self.diagCEMResp = 0
    self.diagPSCMResp = 0
    self.diagCVMResp = 0

class PSCMInfo():
  def __init__(self):
    # Common
    self.byte0 = 0
    self.byte4 = 0
    self.byte7 = 0
    self.LKAActive = 0
    self.LKATorque = 0
    self.SteeringAngleServo = 0

    # C1
    self.byte3 = 0


class FSMInfo():
  def __init__(self):
    # Common
    self.TrqLim = 0
    self.LKAAngleReq = 0
    self.Checksum = 0
    self.LKASteerDirection = 0
    
    # C1
    self.SET_X_E3 = 0
    self.SET_X_B4 = 0
    self.SET_X_08 = 0
    self.SET_X_02 = 0
    self.SET_X_25 = 0


class CCButtons():
  def __init__(self):
    # Common
    self.ACCOnOffBtn = 0
    self.ACCSetBtn = 0
    self.ACCResumeBtn = 0
    self.ACCMinusBtn = 0
    self.TimeGapIncreaseBtn = 0
    self.TimeGapDecreaseBtn = 0

    # C1
    self.ACCStopBtn = 0
    self.byte0 = 0
    self.byte1 = 0
    self.byte2 = 0
    self.byte3 = 0
    self.byte4 = 0
    self.byte5 = 0
    self.byte6 = 0
    self.B7b0 = 0
    self.B7b1 = 0
    self.B7b3 = 0
    self.B7b6 = 0


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.CCP = CarControllerParams(CP)
    self.button_states = {button.event_type: False for button in self.CCP.BUTTONS}
    self.can_define = CANDefine(DBC[CP.carFingerprint]['pt'])

    self.diag = diagInfo() 
    self.PSCMInfo = PSCMInfo() 
    self.FSMInfo = FSMInfo()
    self.CCBtns = CCButtons()

    self.trq_fifo = deque([])

  def create_button_events(self, cp, buttons):
    button_events = []

    for button in buttons:
      state = cp.vl[button.can_addr][button.can_msg] in button.values
      if self.button_states[button.event_type] != state:
        event = car.CarState.ButtonEvent.new_message()
        event.type = button.event_type
        event.pressed = state
        button_events.append(event)
      self.button_states[button.event_type] = state

    return button_events
     
  def update(self, cp, cp_cam):
    ret = car.CarState.new_message()
    
    # Speeds
    ret.vEgoRaw = cp.vl["VehicleSpeed1"]['VehicleSpeed'] * CV.KPH_TO_MS
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = ret.vEgoRaw < 0.1
    
     # Steering
    ret.steeringAngleDeg = cp.vl["PSCM1"]['SteeringAngleServo']
    ret.steeringTorque = cp.vl["PSCM1"]['LKATorque'] # Needed? No signal to check against yet
    ret.steeringPressed = bool(cp.vl["CCButtons"]['ACCSetBtn'] or \
      cp.vl["CCButtons"]['ACCMinusBtn'] or \
      cp.vl["CCButtons"]['ACCResumeBtn']) 
    
    # Update gas and brake
    ret.gas = cp.vl["PedalandBrake"]['AccPedal'] / 102.3
    ret.gasPressed = ret.gas > 0.05
    ret.brakePressed = False

    # Update gear position
    self.shifter_values = self.can_define.dv["TCM0"]['GearShifter']
    can_gear = int(cp.vl["TCM0"]["GearShifter"])
    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(can_gear, None))

    # Belt and doors
    ret.doorOpen = False

    # Check seatbelts
    ret.seatbeltUnlatched = False # No signal yet.

    # ACC status from camera
    ret.cruiseState.available = bool(cp_cam.vl["FSM0"]['ACCStatusOnOff'])
    ret.cruiseState.enabled = bool(cp_cam.vl["FSM0"]['ACCStatusActive'])
    ret.cruiseState.speed = cp.vl["ACC"]['SpeedTargetACC'] * CV.KPH_TO_MS
    
    # Button and blinkers.
    ret.buttonEvents = self.create_button_events(cp, self.CCP.BUTTONS)
    #self.buttonStates['altButton1'] = bool(cp.vl["CCButtons"]['ACCOnOffBtn'])
    #self.buttonStates['accelCruise'] = bool(cp.vl["CCButtons"]['ACCSetBtn'])
    #self.buttonStates['decelCruise'] = bool(cp.vl["CCButtons"]['ACCMinusBtn'])
    #self.buttonStates['setCruise'] = bool(cp.vl["CCButtons"]['ACCSetBtn'])
    #self.buttonStates['resumeCruise'] = bool(cp.vl["CCButtons"]['ACCResumeBtn'])
    #self.buttonStates['cancel'] = bool(cp.vl["CCButtons"]['ACCStopBtn']) No cancel button in V60.
    #self.buttonStates['gapAdjustCruise'] = bool(cp.vl["CCButtons"]['TimeGapIncreaseBtn']) or bool(cp.vl["CCButtons"]['TimeGapDecreaseBtn'])
    ret.leftBlinker = cp.vl["MiscCarInfo"]['TurnSignal'] == 1
    ret.rightBlinker = cp.vl["MiscCarInfo"]['TurnSignal'] == 3

    # Diagnostics, for debugging
    self.diag.diagFSMResp = int(cp_cam.vl["diagFSMResp"]["byte03"])
    self.diag.diagCEMResp = int(cp.vl["diagCEMResp"]["byte03"])
    self.diag.diagCVMResp = int(cp.vl["diagCVMResp"]["byte03"])
    self.diag.diagPSCMResp = int(cp.vl["diagPSCMResp"]["byte03"])

    # ACC Buttons
    if self.CP.carFingerprint in PLATFORM.C1:
      self.CCBtns.ACCStopBtn = bool(cp.vl["CCButtons"]['ACCStopBtn'])
  
    # PSCMInfo
    # Common
    self.PSCMInfo.byte0 = int(cp.vl['PSCM1']['byte0']) 
    self.PSCMInfo.byte4 = int(cp.vl['PSCM1']['byte4']) 
    self.PSCMInfo.byte7 = int(cp.vl['PSCM1']['byte7']) 
    self.PSCMInfo.LKATorque = int(cp.vl['PSCM1']['LKATorque']) 
    self.PSCMInfo.LKAActive = int(cp.vl['PSCM1']['LKAActive']) 
    self.PSCMInfo.SteeringAngleServo = float(cp.vl['PSCM1']['SteeringAngleServo']) 

    # Platform specific
    if self.CP.carFingerprint in PLATFORM.C1:
      self.PSCMInfo.byte3 = int(cp.vl['PSCM1']['byte3']) 

    # FSMInfo
    # Common both platforms

    if self.CP.carFingerprint in PLATFORM.C1:
      # TODO Why use these? In future shold be ok to delete.
      self.FSMInfo.TrqLim = int(cp_cam.vl['FSM1']['TrqLim']) 
      self.FSMInfo.LKAAngleReq = float(cp_cam.vl['FSM1']['LKAAngleReq']) 
      self.FSMInfo.Checksum = int(cp_cam.vl['FSM1']['Checksum']) 
      self.FSMInfo.LKASteerDirection = int(cp_cam.vl['FSM1']['LKASteerDirection'])
      self.FSMInfo.SET_X_E3 = int(cp_cam.vl['FSM1']['SET_X_E3']) 
      self.FSMInfo.SET_X_B4 = int(cp_cam.vl['FSM1']['SET_X_B4']) 
      self.FSMInfo.SET_X_08 = int(cp_cam.vl['FSM1']['SET_X_08']) 
      self.FSMInfo.SET_X_02 = int(cp_cam.vl['FSM1']['SET_X_02']) 
      self.FSMInfo.SET_X_25 = int(cp_cam.vl['FSM1']['SET_X_25']) 
    
    # Check if servo stops responding when acc is active.
    # If N_ZERO_TRQ 0 torque samples in a row is detected,
    # set steerUnavailable. Same logic in carcontroller to
    # decide when to start to recover steering.
    if ret.cruiseState.enabled and ret.vEgo > self.CP.minSteerSpeed:
      self.trq_fifo.append(self.PSCMInfo.LKATorque)
      ret.steerFaultTemporary= True if (self.trq_fifo.count(0) >= self.CCP.N_ZERO_TRQ*2) else False  # *2, runs at 100hz
      if len(self.trq_fifo) > self.CCP.N_ZERO_TRQ*2:                                           # vs 50hz in CarController
        self.trq_fifo.popleft()
    else:
      self.trq_fifo.clear()
      ret.steerFaultTemporary = False

    return ret

  @staticmethod
  def get_can_parser(CP):
    # ptcan on bus 0
    # this function generates lists for signal, messages and initial values

    # Common signals for both platforms
    signals = [
      # sig_name, sig_address
      ("VehicleSpeed", "VehicleSpeed1"),
      ("TurnSignal", "MiscCarInfo"),
      ("ACCOnOffBtn", "CCButtons"),
      ("ACCResumeBtn", "CCButtons"),
      ("ACCSetBtn", "CCButtons"),
      ("ACCMinusBtn", "CCButtons"),
      ("TimeGapIncreaseBtn", "CCButtons"),
      ("TimeGapDecreaseBtn", "CCButtons"),

      # Common PSCM signals
      ("SteeringAngleServo", "PSCM1"),
      ("LKATorque", "PSCM1"),
      ("LKAActive", "PSCM1"),
      ("byte0", "PSCM1"),
      ("byte4", "PSCM1"),
      ("byte7", "PSCM1"),

      # diagnostic
      ("byte03", "diagCEMResp"),
      ("byte47", "diagCEMResp"),
      ("byte03", "diagPSCMResp"),
      ("byte47", "diagPSCMResp"),
      ("byte03", "diagCVMResp"),
      ("byte47", "diagCVMResp"),
    ]

    checks = [
      # sig_address, frequency
      ("CCButtons", 100),
      ("PSCM1", 50),
      ("VehicleSpeed1", 50),
      ("MiscCarInfo", 25),
      ("diagCEMResp", 0),
      ("diagPSCMResp", 0),
      ("diagCVMResp", 0),

    ]

    # Car specific signals
    if CP.carFingerprint in PLATFORM.C1:
      signals += [
      ("SpeedTargetACC", "ACC"),
      ("BrakePedalActive2", "PedalandBrake"),
      ("AccPedal", "PedalandBrake"),
      ("BrakePress0", "BrakeMessages"),
      ("BrakePress1", "BrakeMessages"),
      ("BrakeStatus", "BrakeMessages"),
      ("GearShifter", "TCM0"),
    ]
    # Servo
      signals += [
        ("byte3", "PSCM1"),
      ]

    # Buttons
      signals += [
        ('ACCStopBtn', "CCButtons"),
      ]

    # Checks
      checks += [
        ("BrakeMessages", 50),
        ("ACC", 17),
        ("PedalandBrake", 100),
        ("TCM0", 10),
      ]
    return CANParser(DBC[CP.carFingerprint]['pt'], signals, checks, 0)

  @staticmethod
  def get_adas_can_parser(CP):
    # radar on bus 1, not decoded yet
    # this function generates lists for signal, messages and initial values
    signals = [
      # sig_name, sig_address, default
    ]

    checks = [
      # sig_address, frequency
    ]

    return CANParser(DBC[CP.carFingerprint]['pt'], signals, checks, 1)

  @staticmethod
  def get_cam_can_parser(CP):
    # camera on bus 2
    # Common signals
    signals = [
      # sig_name, sig_address, default
      ("byte03", "diagFSMResp"),
      ("byte47", "diagFSMResp"),

   ]
    # Common checks
    checks = [
      # sig_address, frequency
      ("diagFSMResp", 0),
    ]

    # Car specific
    if CP.carFingerprint in PLATFORM.C1:
      # LKA Request
      signals.append(("TrqLim", "FSM1"))
      signals.append(("LKAAngleReq", "FSM1"))
      signals.append(("Checksum", "FSM1"))
      signals.append(("LKASteerDirection", "FSM1"))
      signals.append(("SET_X_E3", "FSM1"))
      signals.append(("SET_X_B4", "FSM1"))
      signals.append(("SET_X_08", "FSM1"))
      signals.append(("SET_X_02", "FSM1"))
      signals.append(("SET_X_25", "FSM1"))

      # ACC Status
      signals.append(("ACCStatusOnOff", "FSM0"))
      signals.append(("ACCStatusActive", "FSM0"))

      # Checks
      checks.append(('FSM0', 100))
      checks.append(('FSM1', 50))

    return CANParser(DBC[CP.carFingerprint]['pt'], signals, checks, 2)
