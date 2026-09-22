import pytest

from openpilot.common.params import Params


def test_transit_and_lane_centering_params_have_their_defaults():
  params = Params()
  assert params.get_int("TransitLkaIntervention", return_default=True) == 0
  assert params.get_int("TransitLkaRamp", return_default=True) == 0
  assert params.get_bool("TransitLkaContinuation") is False
  assert params.get_float("LaneCenteringIntegralGain", return_default=True) == pytest.approx(0.03)
  assert params.get_bool("DisableDriverMonitoring") is False
