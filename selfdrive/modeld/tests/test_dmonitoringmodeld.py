import re
from pathlib import Path

import numpy as np
import pytest

from cereal import log
from openpilot.common.realtime import DT_DMON
from openpilot.selfdrive.modeld.dmonitoringmodeld import get_attentive_packet
from openpilot.selfdrive.monitoring.policy import DriverMonitoring

PARAM_KEYS_PATH = Path(__file__).resolve().parents[3] / "common/params_keys.h"


@pytest.mark.parametrize("rhd", [False, True])
@pytest.mark.parametrize("calib", [np.zeros(3), np.array([0.01, -0.03, 0.04])])
def test_attentive_packet_stays_alert_free(rhd: bool, calib: np.ndarray) -> None:
  state = get_attentive_packet(123, calib, rhd).driverStateV2
  dm = DriverMonitoring(rhd_saved=rhd)

  for _ in range(int(120 / DT_DMON)):
    dm._update_states(state, calib, 15.0, True, False)
    dm._update_events(False, True, False, False)
    assert dm.alert_level == log.DriverMonitoringState.AlertLevel.none

  assert dm.face_detected
  assert not dm.driver_distracted
  assert dm.active_policy == log.DriverMonitoringState.MonitoringPolicy.vision


@pytest.mark.parametrize("wheel_on_right", [False, True])
def test_attentive_packet_publishes_the_saved_side_not_a_knife_edge(wheel_on_right: bool) -> None:
  """wheelOnRightProb feeds a learner whose decision threshold is exactly 0.5 (policy.py's
  wheelpos_offsetter / _WHEELPOS_THRESHOLD); a constant 0.5 is not neutral to it, it converges
  the filtered mean onto its own threshold and silently forces IsRhdDetected to False. The
  bypass must publish the side already saved instead, so the learner is self-consistent."""
  state = get_attentive_packet(123, np.zeros(3), wheel_on_right).driverStateV2
  assert state.wheelOnRightProb == (1.0 if wheel_on_right else 0.0)


def test_is_rhd_detected_is_a_registered_persistent_bool_param() -> None:
  """This fork's Params wrapper swallows UnknownKeyName on get_bool (reads back False), so a
  passing get_bool("IsRhdDetected") call proves nothing on its own - check the registration
  itself."""
  params_source = PARAM_KEYS_PATH.read_text(encoding="utf-8")
  match = re.search(r'\{"IsRhdDetected",\s*\{([^}]*)\}\}', params_source)
  assert match is not None, "IsRhdDetected is not declared in common/params_keys.h"
  assert "PERSISTENT" in match.group(1)
  assert "BOOL" in match.group(1)
