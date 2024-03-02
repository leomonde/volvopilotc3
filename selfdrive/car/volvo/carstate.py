from dataclasses import dataclass
from cereal import car
from common.conversions import Conversions as CV
from selfdrive.car.interfaces import CarStateBase
from opendbc.can.parser import CANParser
from opendbc.can.can_define import CANDefine
from selfdrive.car.volvo.values import CAR, DBC, CarControllerParams, PLATFORM

@dataclass
class diagInfo():
  diagFSMResp = 0
  diagCEMResp = 0
  diagPSCMResp = 0
  diagCVMResp = 0

class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.CCP = CarControllerParams(CP)
    self.button_states = {button.event_type: False for button in self.CCP.BUTTONS}
    self.can_define = CANDefine(DBC[CP.carFingerprint]['pt'])

    self.diag = diagInfo()

    # Relaying to FSM (camera)
    self.PSCMInfo = {
      "byte0" : 0,
      "byte4" : 0,
      "byte7" : 0,
      "LKAActive" : 0,
      "SteeringWheelRateOfChange" : 0,
      "steeringRateDeg" : 0,
    }

    # Detect if servo stop responding to steering command.
    self.count_zero_steeringTorque = 0
    self.cruiseState_enabled_prev = 0

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
    ret.cruiseState.speed = cp.vl["ACC_Speed"]['ACC_Speed'] * CV.KPH_TO_MS
    
     # Steering
    ret.steeringAngleDeg = float(cp.vl["PSCM1"]['SteeringAngleServo'])
    ret.steeringTorque = cp.vl["PSCM1"]['LKATorque'] 
    ret.steeringPressed = bool(cp.vl["CCButtons"]['ACCSetBtn'] or \
      cp.vl["CCButtons"]['ACCMinusBtn'] or \
      cp.vl["CCButtons"]['ACCResumeBtn']) 
    
    # Update gas and brake
    ret.gas = cp.vl["AccPedal"]['AccPedal'] / 102.3
    ret.gasPressed = ret.gas > 0.1
    ret.brakePressed = False

    # Update gear position
    ret.gearShifter = self.parse_gear_shifter('D') # TODO: Gear EUCD

    # Belt and doors
    ret.doorOpen = False

    # Check seatbelts
    ret.seatbeltUnlatched = False # No signal yet.

    # ACC status from camera
    accStatus = cp_cam.vl["FSM0"]['ACCStatus']
      
    if accStatus == 2:
      # Acc in ready mode
      ret.cruiseState.available = True
      ret.cruiseState.enabled = False
    elif accStatus >= 6:
      # Acc active
      ret.cruiseState.available = True
      ret.cruiseState.enabled = True
    else:
      # Acc in a unkown mode
      ret.cruiseState.available = False
      ret.cruiseState.enabled = False

    # SNG
    #ret.cruiseState.standstill = bool(cp_cam.vl["FSM3"]['ACC_Standstill'])
    #ret.cruiseState.accdistance = int(cp_cam.vl["FSM1"]['ACC_Distance'])
    
    # Button and blinkers.
    ret.buttonEvents = self.create_button_events(cp, self.CCP.BUTTONS)
    #self.buttonStates['gapAdjustCruise'] = bool(cp.vl["CCButtons"]['TimeGapIncreaseBtn']) or bool(cp.vl["CCButtons"]['TimeGapDecreaseBtn'])
    ret.leftBlinker = cp.vl["MiscCarInfo"]['TurnSignal'] == 1
    ret.rightBlinker = cp.vl["MiscCarInfo"]['TurnSignal'] == 3

    # Diagnostics, for debugging
    self.diag.diagFSMResp = int(cp_cam.vl["diagFSMResp"]["byte03"])
    self.diag.diagCEMResp = int(cp.vl["diagCEMResp"]["byte03"])
    self.diag.diagCVMResp = int(cp.vl["diagCVMResp"]["byte03"])
    self.diag.diagPSCMResp = int(cp.vl["diagPSCMResp"]["byte03"])

    # Store info from servo message PSCM1
    # FSM (camera) checks if LKAActive & LKATorque
    # active when not requested
    self.PSCMInfo["byte0"] = int(cp.vl['PSCM1']['byte0'])
    self.PSCMInfo["byte4"] = int(cp.vl['PSCM1']['byte4'])
    self.PSCMInfo["byte7"] = int(cp.vl['PSCM1']['byte7'])
    self.PSCMInfo["LKAActive"] = int(cp.vl['PSCM1']['LKAActive'])
    self.PSCMInfo["SteeringWheelRateOfChange"] = float(cp.vl['PSCM1']['SteeringWheelRateOfChange'])
    self.PSCMInfo["steeringRateDeg"] = float(cp.vl['PSCM1']['SteeringWheelRateOfChange'])

    # Check if servo stops responding when acc is active.
    if ret.cruiseState.enabled and ret.vEgo > self.CP.minSteerSpeed:
    
      # Reset counter on entry
      if self.cruiseState_enabled_prev != ret.cruiseState.enabled:
        self.count_zero_steeringTorque = 0
    
      # Count up when no torque from servo detected.
      if ret.steeringTorque == 0:
        self.count_zero_steeringTorque += 1
      else:
        self.count_zero_steeringTorque = 0
    
      # Set fault if above threshold
      if self.count_zero_steeringTorque >= 1000:
        ret.steerFaultTemporary = True
      else:
        ret.steerFaultTemporary = False
    
    self.cruiseState_enabled_prev = ret.cruiseState.enabled

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

      ("AccPedal", "AccPedal"),
      ("BrakePedal", "BrakePedal"),
      ("SteeringWheelRateOfChange", "PSCM1"),
      ("ACC_Speed", "ACC_Speed"),
      
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
      
      ("AccPedal", 100),
      ("BrakePedal", 50),
      ("ACC_Speed", 50),
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
      ("TrqLim", "FSM2"),
      ("LKAAngleReq", "FSM2"),
      ("Checksum", "FSM2"),
      ("LKASteerDirection", "FSM2"),
      ("SET_X_22", "FSM2"),
      ("SET_X_02", "FSM2"),
      ("SET_X_10", "FSM2"),
      ("SET_X_A4", "FSM2"),
      ("ACCStatus", "FSM0"),
      ("ACC_Standstill", "FSM3"),
      ("ACC_Distance", "FSM1"),

    ]
    # Common checks
    checks = [
      # sig_address, frequency
      ('FSM0', 100),
      ('FSM2', 50),
      ('FSM3', 50),
      ('FSM1', 50),
      ("diagFSMResp", 0),
    ]

    return CANParser(DBC[CP.carFingerprint]['pt'], signals, checks, 2)
