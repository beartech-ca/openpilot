import numpy as np
import pytest

from cereal import log
from openpilot.common.realtime import DT_DMON
from openpilot.selfdrive.modeld.dmonitoringmodeld import get_attentive_packet
from openpilot.selfdrive.monitoring.policy import DriverMonitoring


@pytest.mark.parametrize("rhd", [False, True])
@pytest.mark.parametrize("calib", [np.zeros(3), np.array([0.01, -0.03, 0.04])])
def test_attentive_packet_stays_alert_free(rhd: bool, calib: np.ndarray) -> None:
  state = get_attentive_packet(123, calib).driverStateV2
  dm = DriverMonitoring(rhd_saved=rhd)

  for _ in range(int(120 / DT_DMON)):
    dm._update_states(state, calib, 15.0, True, False)
    dm._update_events(False, True, False, False)
    assert dm.alert_level == log.DriverMonitoringState.AlertLevel.none

  assert dm.face_detected
  assert not dm.driver_distracted
  assert dm.active_policy == log.DriverMonitoringState.MonitoringPolicy.vision
