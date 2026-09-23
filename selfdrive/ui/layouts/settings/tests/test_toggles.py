from openpilot.selfdrive.ui.layouts.settings import toggles


class FakeParams:
  """Mirrors the real Params.put_bool signature: no `block` keyword.

  common/params_pyx.pyx:208 is `def put_bool(self, key, bool val)` and the pure-python
  fallback in common/params.py mirrors it (`def put_bool(self, key, val: bool)`); neither
  accepts `block=`. A caller that passes it raises TypeError before the write happens.
  """

  def __init__(self):
    self.writes = []

  def put_bool(self, key, value):
    self.writes.append((key, value))


def _toggles_layout():
  layout = toggles.TogglesLayout.__new__(toggles.TogglesLayout)
  layout._params = FakeParams()
  return layout


def test_set_driver_monitoring_disabled_true_writes_both_params():
  layout = _toggles_layout()

  layout._set_driver_monitoring_disabled(True)

  assert layout._params.writes == [
    ("DisableDriverMonitoring", True),
    ("OnroadCycleRequested", True),
  ]


def test_set_driver_monitoring_disabled_false_writes_both_params():
  layout = _toggles_layout()

  layout._set_driver_monitoring_disabled(False)

  assert layout._params.writes == [
    ("DisableDriverMonitoring", False),
    ("OnroadCycleRequested", True),
  ]
