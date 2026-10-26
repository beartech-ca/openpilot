from types import SimpleNamespace

from cereal import car
from opendbc.car import gen_empty_fingerprint
from opendbc.car.chrysler.interface import CarInterface
from opendbc.car.chrysler.values import CAR
from openpilot.common.params import Params
from openpilot.selfdrive.selfdrived.events import EventName
from openpilot.selfdrive.selfdrived.tests.test_tesla_coop_saturation_warning import configure_selfdrived

ABOVE_MIN = 25.0  # m/s, well above the CUSW minimum steering speed (18.5 m/s)
BELOW_MIN = 15.0  # m/s


def make_cusw_params():
  return CarInterface.get_params(CAR.JEEP_CHEROKEE_5TH_GEN, gen_empty_fingerprint(), [], False, False, False, SimpleNamespace())


def make_car_state(speed):
  state = car.CarState.new_message()
  state.canValid = True
  state.vEgo = speed
  state.vEgoRaw = speed
  state.gearShifter = car.CarState.GearShifter.drive
  state.cruiseState.available = True
  state.cruiseState.enabled = True
  return state.as_reader()


def below_steer_alerts_per_dip(always_alert):
  CP = make_cusw_params()
  assert CP.minSteerSpeed > BELOW_MIN
  selfdrived = configure_selfdrived(CP)
  selfdrived.starpilot_toggles.cusw_steer_speed_alert = always_alert
  selfdrived.sm.data["carControl"] = car.CarControl.new_message().as_reader()

  phases = [ABOVE_MIN, BELOW_MIN, ABOVE_MIN, BELOW_MIN, ABOVE_MIN, BELOW_MIN]
  seen = []
  prev = make_car_state(ABOVE_MIN)
  for speed in phases:
    alerted = False
    for _ in range(5):
      state = make_car_state(speed)
      selfdrived.CS_prev = prev
      selfdrived.update_events(state)
      alerted |= EventName.belowSteerSpeed in selfdrived.events.names
      prev = state
    seen.append(alerted)
  # one entry per dip below the minimum steering speed
  return seen[1::2], seen[0::2]


def test_cusw_alerts_on_every_dip_by_default():
  dips, above = below_steer_alerts_per_dip(always_alert=True)
  assert dips == [True, True, True]
  assert above == [False, False, False]


def test_cusw_toggle_off_keeps_starpilot_once_per_drive_alert():
  dips, above = below_steer_alerts_per_dip(always_alert=False)
  assert dips == [True, False, False]
  assert above == [False, False, False]


def test_always_alert_defaults_on():
  params = Params()
  assert params.get_default_value("CUSWSteerSpeedAlert") is True
  assert params.get_stock_value("CUSWSteerSpeedAlert") is True
