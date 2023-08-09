from collections import namedtuple
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Union

from cereal import car
from panda.python import uds
from selfdrive.car import AngleRateLimit, dbc_dict
from selfdrive.car.docs_definitions import CarInfo, CarHarness, CarParts
from selfdrive.car.fw_query_definitions import FwQueryConfig, Request, p16

Ecu = car.CarParams.Ecu
Button = namedtuple('Button', ['event_type', 'can_addr', 'can_msg', 'values'])

"""
Volvo Electronic Control Units abbreviations and network topology
Platforms C1/EUCD

Three main CAN network buses
  1. Powertrain
  2. Chassis (also called MS* CAN) *MS=Medium Speed
  3. Extended
Only mentioning control units of interest on the network buses.

Powertrain CAN
  BCM - Brake Control Module
  CEM - Central Electronic Module
  CVM - Closing Velocity Module (low speed auto emergency braking <30kph)
  FSM - Forward Sensing Module (camera mounted in windscreen)
  PPM - Pedestrian Protection Module (controls pedestrian airbag under the engine hood)
  PSCM - Power Steering Control Module (EPS - Electronic Power Steering)
  SAS - Steering Angle Sensor Module
  SRS - Supplemental Restraint System Module (seatbelts, airbags...)
  TCM - Transmission Control Module

Chassis CAN
  CEM - Central Electronic Module
  DIM - Driver Information Module (the instrument cluster with odo and speedometer, relayed thru CEM)
  PAM - Parking Assistance Module (automatic parking, relayed thru CEM)

Extended CAN
  CEM - Central Electronic Module
  SODL - Side Object Detection Left (relayed thru CEM)
  SODR - Side Object Detection Right (relayed thru CEM)
"""

class CarControllerParams:
  ANGLE_RATE_LIMIT_UP = AngleRateLimit(speed_bp=[0., 8.33, 13.89, 19.44, 25., 30.55, 36.1], angle_v=[2., 1.2, .25, .20, .15, .10, .10])
  ANGLE_RATE_LIMIT_DOWN = AngleRateLimit(speed_bp=[0., 8.33, 13.89, 19.44, 25., 30.55, 36.1], angle_v=[2., 1.2, .25, .20, .15, .10, .10])

  # constants, collected from v40 dbc lka_direction.
  # Normally the car uses STEER_RIGHT and STEER_LEFT.
  # However it's possible to use STEER in C1 platforms.
  # Servo then accepts steering in both directions.
  STEER_NO = 0
  STEER = 3

  # number of 0 torque samples in a row before trying to restore steering.
  # Got one false trigger on motorway with 10. 
  # Increase to 12 is probably a good tradeoff between false triggers 
  # and detecting fault.
  N_ZERO_TRQ = 12
  
  def __init__(self, CP):
    self.BUTTONS = [
      Button(car.CarState.ButtonEvent.Type.altButton1, "CCButtons", "ACCOnOffBtn", [1]),
      Button(car.CarState.ButtonEvent.Type.setCruise, "CCButtons", "ACCSetBtn", [1]),
      Button(car.CarState.ButtonEvent.Type.resumeCruise, "CCButtons", "ACCResumeBtn", [1]),
      Button(car.CarState.ButtonEvent.Type.accelCruise, "CCButtons", "ACCSetBtn", [1]),
      Button(car.CarState.ButtonEvent.Type.decelCruise, "CCButtons", "ACCMinusBtn", [1]),
      Button(car.CarState.ButtonEvent.Type.cancel, "CCButtons", "ACCStopBtn", [1]),
    ]
    #pass

#BUTTON_STATES = {
  #"altButton1": False, # On/Off button
  #"cancel": False,
  #"setCruise": False,
  #"resumeCruise": False,
  #"accelCruise": False,
  #"decelCruise": False,
  #"gapAdjustCruise": False,
#}   

class CAR:
  V40 = "VOLVO V40 2017"

class PLATFORM:
  C1 = [CAR.V40]

@dataclass
class VolvoCarInfo(CarInfo):
  package: str = "Adaptive Cruise Control & Lane Keeping Aid"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.custom]))

CAR_INFO: Dict[str, Optional[Union[VolvoCarInfo, List[VolvoCarInfo]]]] = {
  CAR.V40: VolvoCarInfo("Volvo V40"),
}

ECU_ADDRESS = { 
  CAR.V40: {"BCM": 0x760, "ECM": 0x7E0, "DIM": 0x720, "CEM": 0x726, "FSM": 0x764, "PSCM": 0x730, "TCM": 0x7E1, "CVM": 0x793},
  }

VOLVO_VERSION_REQUEST = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER]) + \
  p16(0xf1a2)
VOLVO_VERSION_RESPONSE = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER + 0x40]) + \
  p16(0xf1a2)

FW_QUERY_CONFIG = FwQueryConfig(
  requests=[
    Request(
      [VOLVO_VERSION_REQUEST],
      [VOLVO_VERSION_RESPONSE],
      bus=0,
      #whitelist_ecus=[Ecu.unkown, Ecu.eps, Ecu.fwdCamera],
    ),
  ],
)

FW_VERSIONS = {
  CAR.V40: {
  (Ecu.unknown, ECU_ADDRESS[CAR.V40]["CEM"], None): [b'31453061 AA\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00'],  # 0xf1a2
  (Ecu.eps, ECU_ADDRESS[CAR.V40]["PSCM"], None): [b'31288595 AE\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00'],            # 0xf1a2
  (Ecu.fwdCamera, ECU_ADDRESS[CAR.V40]["FSM"], None): [b'31400454 AA\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00'],  # 0xf1a2
  }
}

FINGERPRINTS = {
  CAR.V40: [
    # V40 2017
    {8: 8, 16: 8, 48: 8, 64: 8, 85: 8, 101: 8, 112: 8, 114: 8, 117: 8, 128: 8, 176: 8, 192: 8, 208: 8, 224: 8, 240: 8, 245: 8, 256: 8, 272: 8, 288: 8, 291: 8, 293: 8, 304: 8, 325: 8, 336: 8, 352: 8, 424: 8, 432: 8, 437: 8, 464: 8, 472: 8, 480: 8, 528: 8, 608: 8, 624: 8, 640: 8, 648: 8, 652: 8, 656: 8, 657: 8, 681: 8, 693: 8, 704: 8, 707: 8, 709: 8, 816: 8, 832: 8, 848: 8, 853: 8, 864: 8, 880: 8, 912: 8, 928: 8, 943: 8, 944: 8, 968: 8, 970: 8, 976: 8, 992: 8, 997: 8, 1024: 8, 1029: 8, 1061: 8, 1072: 8, 1409: 8},
    # V40 2015
    {8: 8, 16: 8, 64: 8, 85: 8, 101: 8, 112: 8, 114: 8, 117: 8, 128: 8, 176: 8, 192: 8, 224: 8, 240: 8, 245: 8, 256: 8, 272: 8, 288: 8, 291: 8, 293: 8, 304: 8, 325: 8, 336: 8, 424: 8, 432: 8, 437: 8, 464: 8, 472: 8, 480: 8, 528: 8, 608: 8, 648: 8, 652: 8, 656: 8, 657: 8, 681: 8, 693: 8, 704: 8, 707: 8, 709: 8, 816: 8, 832: 8, 864: 8, 880: 8, 912: 8, 928: 8, 943: 8, 944: 8, 968: 8, 970: 8, 976: 8, 992: 8, 997: 8, 1024: 8, 1029: 8, 1061: 8, 1072: 8, 1409: 8},
    # V40 2014
    {8: 8, 16: 8, 64: 8, 85: 8, 101: 8, 112: 8, 114: 8, 117: 8, 128: 8, 176: 8, 192: 8, 224: 8, 240: 8, 245: 8, 256: 8, 272: 8, 288: 8, 291: 8, 293: 8, 304: 8, 325: 8, 336: 8, 424: 8, 432: 8, 437: 8, 464: 8, 472: 8, 480: 8, 528: 8, 608: 8, 648: 8, 652: 8, 657: 8, 681: 8, 693: 8, 704: 8, 707: 8, 709: 8, 816: 8, 864: 8, 880: 8, 912: 8, 928: 8, 943: 8, 944: 8, 968: 8, 970: 8, 976: 8, 992: 8, 997: 8, 1024: 8, 1029: 8, 1072: 8, 1409: 8},
  ],
}

DBC = {
  CAR.V40: dbc_dict('volvo_v40_2017_pt', None),
}
