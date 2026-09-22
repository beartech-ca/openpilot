from openpilot.selfdrive.ui.layouts.settings.toggles import TogglesLayout
from openpilot.system.ui.widgets import DialogResult


class FakeParams:
  def __init__(self, initial=None):
    self.values = dict(initial or {})

  def get_bool(self, key):
    return bool(self.values.get(key, False))

  def put_bool(self, key, value, block=False):
    self.values[key] = bool(value)


class FakeAction:
  def __init__(self, state=False):
    self.state = state
    self.enabled = True

  def get_state(self):
    return self.state

  def set_state(self, state):
    self.state = state

  def set_enabled(self, enabled):
    self.enabled = enabled


class FakeToggle:
  def __init__(self, state=False):
    self.title = "Disable Driver Monitoring"
    self.action_item = FakeAction(state)


class FakeDialog:
  def __init__(self, text, confirm_text, cancel_text=None, rich=False, callback=None):
    self.text = text
    self.confirm_text = confirm_text
    self.cancel_text = cancel_text
    self.rich = rich
    self.callback = callback


def make_layout(disable_dm=False):
  layout = object.__new__(TogglesLayout)
  layout._params = FakeParams({"DisableDriverMonitoring": disable_dm})
  layout._toggle_defs = {
    "DisableDriverMonitoring": ("title", "description", "monitoring.png", True),
    "RecordFront": ("title", "description", "monitoring.png", True),
  }
  layout._locked_toggles = set()
  layout._toggles = {
    "DisableDriverMonitoring": FakeToggle(disable_dm),
    "RecordFront": FakeToggle(True),
  }
  return layout


def test_enable_dm_disable_requires_confirmation(mocker):
  layout = make_layout()
  push = mocker.patch("openpilot.selfdrive.ui.layouts.settings.toggles.gui_app.push_widget")
  mocker.patch("openpilot.selfdrive.ui.layouts.settings.toggles.ConfirmDialog", FakeDialog)

  layout._toggle_callback(True, "DisableDriverMonitoring")

  assert not layout._params.get_bool("DisableDriverMonitoring")
  dialog = push.call_args.args[0]
  dialog.callback(DialogResult.CONFIRM)
  assert layout._params.get_bool("DisableDriverMonitoring")
  assert layout._params.get_bool("OnroadCycleRequested")


def test_cancel_dm_disable_restores_switch(mocker):
  layout = make_layout()
  push = mocker.patch("openpilot.selfdrive.ui.layouts.settings.toggles.gui_app.push_widget")
  mocker.patch("openpilot.selfdrive.ui.layouts.settings.toggles.ConfirmDialog", FakeDialog)

  layout._toggle_callback(True, "DisableDriverMonitoring")
  push.call_args.args[0].callback(DialogResult.CANCEL)

  assert not layout._params.get_bool("DisableDriverMonitoring")
  assert not layout._toggles["DisableDriverMonitoring"].action_item.get_state()


def test_disabling_dm_bypass_requests_cycle():
  layout = make_layout(disable_dm=True)

  layout._toggle_callback(False, "DisableDriverMonitoring")

  assert not layout._params.get_bool("DisableDriverMonitoring")
  assert layout._params.get_bool("OnroadCycleRequested")


def test_record_front_toggle_disabled_with_dm_bypass(mocker):
  layout = make_layout(disable_dm=True)
  mocker.patch("openpilot.selfdrive.ui.layouts.settings.toggles.ui_state", mocker.Mock(engaged=False))

  layout._update_record_front_toggle()

  assert not layout._toggles["RecordFront"].action_item.enabled
