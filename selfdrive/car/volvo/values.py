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
  NO_STEER = 0
  STEER_NO = 0
  STEER_RIGHT = 1
  STEER_LEFT = 2
  STEER = 3

  # number of 0 torque samples allowed in a row.
  # Got one false trigger on motorway with 10. 
  # Increase to 12 is probably a good tradeoff between false triggers 
  # and detecting fault.
  # Going above this threshold triggers steerFaultTemporary.
  # *2 becuase carState runs in 100Hz.
  N_ZERO_TRQ = 12*2

  # EUCD
  # When changing steer direction steering request need to be blocked. 
  # Otherwise servo wont "listen" to the request.
  # This calibration sets the number of samples to block steering request.
  BLOCK_LEN = 8
  # When close to desired steering angle, don't change steer direction inside deadzone.
  # Since we need to release control of the steering wheel for a brief moment, steering wheel will 
  # unwind by itself. 
  DEADZONE = 0.3

  def __init__(self, CP):
    self.BUTTONS = [
      Button(car.CarState.ButtonEvent.Type.altButton1, "CCButtons", "ACCOnOffBtn", [1]),
      Button(car.CarState.ButtonEvent.Type.setCruise, "CCButtons", "ACCSetBtn", [1]),
      Button(car.CarState.ButtonEvent.Type.resumeCruise, "CCButtons", "ACCResumeBtn", [1]),
      Button(car.CarState.ButtonEvent.Type.accelCruise, "CCButtons", "ACCSetBtn", [1]),
      Button(car.CarState.ButtonEvent.Type.decelCruise, "CCButtons", "ACCMinusBtn", [1]),
      #Button(car.CarState.ButtonEvent.Type.cancel, "CCButtons", "ACCStopBtn", [1]),
    ]

class CAR:
  V60 = "VOLVO V60 2015"

class PLATFORM:
  EUCD = [CAR.V60]

@dataclass
class VolvoCarInfo(CarInfo):
  package: str = "Adaptive Cruise Control & Lane Keeping Aid"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.custom]))

CAR_INFO: Dict[str, Optional[Union[VolvoCarInfo, List[VolvoCarInfo]]]] = {
  CAR.V60: VolvoCarInfo("Volvo V60"),
}

ECU_ADDRESS = { 
  CAR.V60: {"BCM": 0x760, "ECM": 0x7E0, "DIM": 0x720, "CEM": 0x726, "FSM": 0x764, "PSCM": 0x730, "TCM": 0x7E1, "CVM": 0x793},
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
  CAR.V60: {
  (Ecu.transmission, 0x7e1, None): [b'\xf1\xa0YV1FS49CDF2360777'],
  (Ecu.engine, 0x7e0, None): [b'\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00'],
  (Ecu.unknown, 0x726, None): [b'30786853 BK\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00'],
  (Ecu.eps, 0x730, None): [b'31340673 AD\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00'],
  (Ecu.fwdCamera, 0x764, None): [b'31400454 AA\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00'],
  }
}

FINGERPRINTS = {
  CAR.V60: [
   {0: 8, 16: 8, 32: 8, 81: 8, 99: 8, 104: 8, 112: 8, 144: 8, 277: 8, 295: 8, 298: 8, 307: 8, 320: 8, 328: 8, 336: 8, 343: 8, 352: 8, 359: 8, 384: 8, 465: 8, 511: 8, 522: 8, 544: 8, 565: 8, 582: 8, 608: 8, 609: 8, 610: 8, 612: 8, 613: 8, 624: 8, 626: 8, 635: 8, 648: 8, 665: 8, 673: 8, 704: 8, 706: 8, 708: 8, 750: 8, 751: 8, 778: 8, 788: 8, 794: 8, 797: 8, 802: 8, 803: 8, 805: 8, 807: 8, 819: 8, 820: 8, 821: 8, 913: 8, 923: 8, 978: 8, 979: 8, 1006: 8, 1021: 8, 1024: 8, 1029: 8, 1039: 8, 1042: 8, 1045: 8, 1137: 8, 1141: 8, 1152: 8, 1174: 8, 1187: 8, 1198: 8, 1214: 8, 1217: 8, 1226: 8, 1240: 8, 1409: 8},
   {0: 8, 16: 8, 32: 8, 81: 8, 99: 8, 104: 8, 112: 8, 144: 8, 277: 8, 295: 8, 298: 8, 307: 8, 308: 8, 320: 8, 328: 8, 336: 8, 343: 8, 352: 8, 359: 8, 384: 8, 465: 8, 511: 8, 522: 8, 544: 8, 565: 8, 582: 8, 608: 8, 609: 8, 610: 8, 612: 8, 613: 8, 624: 8, 626: 8, 635: 8, 648: 8, 665: 8, 673: 8, 704: 8, 706: 8, 708: 8, 750: 8, 751: 8, 778: 8, 788: 8, 794: 8, 797: 8, 802: 8, 803: 8, 805: 8, 807: 8, 819: 8, 820: 8, 821: 8, 913: 8, 923: 8, 978: 8, 979: 8, 1006: 8, 1021: 8, 1024: 8, 1029: 8, 1039: 8, 1042: 8, 1045: 8, 1137: 8, 1141: 8, 1152: 8, 1174: 8, 1187: 8, 1198: 8, 1214: 8, 1217: 8, 1226: 8, 1240: 8, 1409: 8},
   {0: 8, 16: 8, 32: 8, 81: 8, 99: 8, 104: 8, 112: 8, 144: 8, 277: 8, 295: 8, 298: 8, 307: 8, 308: 8, 320: 8, 328: 8, 336: 8, 343: 8, 352: 8, 359: 8, 384: 8, 465: 8, 511: 8, 522: 8, 544: 8, 565: 8, 582: 8, 608: 8, 609: 8, 610: 8, 612: 8, 613: 8, 624: 8, 626: 8, 635: 8, 648: 8, 665: 8, 673: 8, 704: 8, 706: 8, 708: 8, 750: 8, 751: 8, 778: 8, 788: 8, 794: 8, 797: 8, 802: 8, 803: 8, 805: 8, 807: 8, 819: 8, 820: 8, 821: 8, 913: 8, 923: 8, 978: 8, 979: 8, 1006: 8, 1021: 8, 1024: 8, 1029: 8, 1039: 8, 1042: 8, 1045: 8, 1137: 8, 1141: 8, 1152: 8, 1174: 8, 1187: 8, 1198: 8, 1214: 8, 1217: 8, 1226: 8, 1240: 8, 1409: 8, 1609: 8, 1613: 8, 1649: 8, 1792: 8, 1793: 8, 1798: 8, 1799: 8, 1810: 8, 1813: 8, 1824: 8, 1825: 8, 1832: 8, 1838: 8, 1840: 8, 1842: 8, 1848: 8, 1854: 8, 1855: 8, 1856: 8, 1858: 8, 1859: 8, 1860: 8, 1862: 8, 1863: 8, 1872: 8, 1875: 8, 1879: 8, 1882: 8, 1886: 8, 1888: 8, 1889: 8, 1892: 8, 1896: 8, 1900: 8, 1920: 8, 1924: 8, 1927: 8, 1937: 8, 1953: 8, 1954: 8, 1955: 8, 1968: 8, 1969: 8, 1971: 8, 1975: 8, 1984: 8, 1988: 8, 2000: 8, 2001: 8, 2002: 8, 2004: 8, 2017: 8, 2018: 8, 2020: 8, 2025: 8},
   {16: 8, 32: 8, 81: 8, 99: 8, 104: 8, 112: 8, 144: 8, 277: 8, 295: 8, 298: 8, 307: 8, 308: 8, 320: 8, 328: 8, 336: 8, 343: 8, 352: 8, 359: 8, 384: 8, 465: 8, 511: 8, 522: 8, 544: 8, 565: 8, 582: 8, 608: 8, 609: 8, 610: 8, 612: 8, 613: 8, 624: 8, 626: 8, 635: 8, 648: 8, 665: 8, 673: 8, 704: 8, 706: 8, 708: 8, 750: 8, 751: 8, 778: 8, 788: 8, 794: 8, 797: 8, 802: 8, 803: 8, 805: 8, 807: 8, 819: 8, 820: 8, 821: 8, 913: 8, 923: 8, 978: 8, 979: 8, 1006: 8, 1021: 8, 1024: 8, 1029: 8, 1039: 8, 1042: 8, 1045: 8, 1137: 8, 1141: 8, 1152: 8, 1174: 8, 1187: 8, 1198: 8, 1214: 8, 1217: 8, 1226: 8, 1240: 8, 1409: 8},
   ],
}

DBC = {
  CAR.V60: dbc_dict('volvo_v60_2015_pt', None),
}
