import pickle
import re
from pathlib import Path

import numpy as np
import pytest

from cereal import log
from openpilot.common.realtime import DT_DMON
from openpilot.selfdrive.modeld.dmonitoringmodeld import METADATA_PATH, get_attentive_packet, run_frame
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


class _ExplodingModel:
  """Stands in for ModelState: any call to .run() fails the test, proving the tinygrad forward
  pass is skipped rather than merely computed-and-discarded while the bypass is active."""

  def run(self, buf, calib, transform):
    raise AssertionError("model.run() must not be called while the bypass gate is active")


class _FakeModel:
  """A model whose .run() is cheap and counted, wired to the real output_slices layout
  (selfdrive/modeld/models/dmonitoring_model_metadata.pkl) so its output flows through the
  real parse_model_output()/get_driverstate_packet() path unmodified."""

  def __init__(self, output_slices, output):
    self.output_slices = output_slices
    self._output = output
    self.run_calls = 0

  def run(self, buf, calib, transform):
    self.run_calls += 1
    return self._output, 0.01


class _FakePubMaster:
  def __init__(self):
    self.sent = []

  def send(self, name, msg):
    self.sent.append((name, msg))


def _real_output_slices():
  # Trusted first-party repo file: ModelState.__init__ pickle.loads() this same path in
  # production (selfdrive/modeld/dmonitoringmodeld.py), not data from an untrusted source.
  with open(METADATA_PATH, "rb") as f:
    return pickle.load(f)["output_slices"]


def test_run_frame_gate_skips_the_model_and_publishes_synthetic_state_when_disabled() -> None:
  """Exercises the actual dispatch in main()'s loop (run_frame is the extracted per-frame
  gate main() calls), not just the packet builder: with dm_disabled=True, model.run() must
  never be reached and the published packet must be the synthetic attentive one."""
  model = _ExplodingModel()
  pm = _FakePubMaster()

  run_frame(True, model=model, pm=pm, frame_id=99, calib=np.zeros(3), wheel_on_right_saved=True, buf=None, model_transform=None)

  assert len(pm.sent) == 1
  name, msg = pm.sent[0]
  assert name == "driverStateV2"
  assert msg.driverStateV2.frameId == 99
  assert msg.driverStateV2.wheelOnRightProb == 1.0


def test_run_frame_gate_runs_the_model_and_publishes_its_output_when_enabled() -> None:
  """The other side of the same gate: with dm_disabled=False, the model must actually run and
  its own output must be what gets published, not the synthetic packet."""
  output_slices = _real_output_slices()
  size = max(s.stop for s in output_slices.values() if isinstance(s, slice))
  model = _FakeModel(output_slices, np.zeros(size, dtype=np.float32))
  pm = _FakePubMaster()

  run_frame(False, model=model, pm=pm, frame_id=7, calib=np.zeros(3), wheel_on_right_saved=True, buf=object(), model_transform=np.eye(3, dtype=np.float32))

  assert model.run_calls == 1
  assert len(pm.sent) == 1
  name, msg = pm.sent[0]
  assert name == "driverStateV2"
  assert msg.driverStateV2.frameId == 7
  # sigmoid(0.0) == 0.5: distinct from the bypass's discrete 0.0/1.0, so this also proves the
  # real model path produced the published value rather than the synthetic packet.
  assert msg.driverStateV2.wheelOnRightProb == pytest.approx(0.5)
