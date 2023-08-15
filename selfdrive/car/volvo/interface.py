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

  @staticmethod
  def compute_gb(accel, speed):
    return float(accel) / 4.0

  @staticmethod
  def _get_params(ret, candidate, fingerprint, car_fw, experimental_long, docs):
    ret.carName = "volvo"
    ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.volvo)]
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

    return ret

  # returns a car.CarState
  def _update(self, c):
    ret = self.CS.update(self.cp, self.cp_cam)

    events = self.create_common_events(ret, pcm_enable=not self.CS.out.cruiseState.enabled)
                                       #enable_buttons=(ButtonType.setCruise, ButtonType.resumeCruise))

    ret.events = events.to_msg()

    return ret

  def apply(self, c, now_nanos):
    return self.CC.update(c, self.CS, now_nanos)
  