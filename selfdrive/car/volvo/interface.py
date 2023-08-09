#!/usr/bin/env python3
from cereal import car
from common.conversions import Conversions as CV
from selfdrive.car import STD_CARGO_KG, get_safety_config 
from selfdrive.car.interfaces import CarInterfaceBase
from selfdrive.car.volvo.values import CAR

ButtonType = car.CarState.ButtonEvent.Type
EventName = car.CarEvent.EventName

class CarInterface(CarInterfaceBase):
  def __init__(self, CP, CarController, CarState):
    super().__init__(CP, CarController, CarState)

    # Create variables
    #self.cruiseState_enabled_prev = False
    #self.buttonStatesPrev = BUTTON_STATES.copy()

  @staticmethod
  def compute_gb(accel, speed):
    return float(accel) / 4.0

  @staticmethod
  def _get_params(ret, candidate, fingerprint, car_fw, experimental_long, docs):
    ret.carName = "volvo"
    ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.volvoC1)]
    ret.radarUnavailable = True
    ret.autoResumeSng = False

    ret.steerControlType = car.CarParams.SteerControlType.angle

    # Steering settings
    ret.steerActuatorDelay = 0.2   # Actuator delay from input to output.
    ret.minSteerSpeed = 1. * CV.KPH_TO_MS
    #ret.steerLimitAlert = True     # Do this do anything?
    #ret.steerRateCost = 1.          # Used in pathplanner for punishing? Steering derivative?

    # Assuming all is automatic
    ret.transmissionType = car.CarParams.TransmissionType.automatic
 
    if candidate == CAR.V40:
      # Technical specifications
      ret.mass = 1610 + STD_CARGO_KG
      ret.wheelbase = 2.647
      ret.centerToFront = ret.wheelbase * 0.44
      ret.steerRatio = 14.7

    # No PID control used. Set to a value, otherwise pid loop crashes.
    #ret.steerMaxBP = [0.] # m/s
    #ret.steerMaxV = [1.]
    #ret.lateralTuning.pid.kpBP = [0.]
    #ret.lateralTuning.pid.kiBP = [0.]
    # Tuning factors
    #ret.lateralTuning.pid.kf = 0.0
    #ret.lateralTuning.pid.kpV  = [0.0]
    #ret.lateralTuning.pid.kiV = [0.0]

    return ret

  # returns a car.CarState
  def _update(self, c):
    ret = self.CS.update(self.cp, self.cp_cam)

    events = self.create_common_events(ret)
                                       #pcm_enable=not self.CS.CP.openpilotLongitudinalControl,
                                       #enable_buttons=(ButtonType.setCruise, ButtonType.resumeCruise))

    ret.events = events.to_msg()

    return ret

  def apply(self, c, now_nanos):
    return self.CC.update(c, self.CS, now_nanos)
  
  #def apply(self, c):
  #  can_sends = self.CC.update(c.enabled, self.CS, self.frame,
  #                             c.actuators,
  #                             c.hudControl.visualAlert, c.hudControl.leftLaneVisible,
  #                             c.hudControl.rightLaneVisible, c.hudControl.leadVisible,
  #                             c.hudControl.leftLaneDepart, c.hudControl.rightLaneDepart)
  #  self.frame += 1
  #  return can_sends
