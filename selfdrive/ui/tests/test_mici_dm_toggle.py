import openpilot.selfdrive.ui.mici.layouts.settings.toggles as mici_toggles


class FakeParams:
  def __init__(self, initial=None):
    self.values = dict(initial or {})

  def get_bool(self, key, default=False):
    return bool(self.values.get(key, default))

  def put_bool(self, key, value, block=False):
    self.values[key] = bool(value)

  def remove(self, key):
    self.values.pop(key, None)


class FakeControl:
  def __init__(self, text, *args, initial_state=False, **kwargs):
    self.text = text
    self.checked = initial_state
    self.enabled = True
    self._options = args[-1] if args and isinstance(args[-1], list) else []

  def set_checked(self, checked):
    self.checked = checked

  def set_enabled(self, enabled):
    self.enabled = enabled

  def set_visible(self, visible):
    pass

  def set_value(self, value):
    pass


class FakeScroller:
  def __init__(self):
    self.items = []

  def add_widgets(self, items):
    self.items.extend(items)


class FakeUiState:
  def __init__(self, initial=None):
    self.params = FakeParams(initial)
    self.engaged = False

  def add_engaged_transition_callback(self, callback):
    pass


def test_c3x_toggle_layout_exposes_dm_switch(mocker):
  mocker.patch.object(mici_toggles.NavScroller, "__init__", lambda self: setattr(self, "_scroller", FakeScroller()))
  mocker.patch.object(mici_toggles, "BigParamControl", FakeControl)
  mocker.patch.object(mici_toggles, "BigMultiParamToggle", FakeControl)
  mocker.patch.object(mici_toggles, "BigToggle", FakeControl, create=True)
  mocker.patch.object(mici_toggles, "ui_state", FakeUiState())

  layout = mici_toggles.TogglesLayoutMici()

  assert layout._disable_dm_toggle in layout._scroller.items
  assert layout._disable_dm_toggle.text == "disable driver monitoring"


def test_c3x_dm_disable_requires_confirmation(mocker):
  state = FakeUiState()
  mocker.patch.object(mici_toggles, "ui_state", state)
  pushed = mocker.patch.object(mici_toggles.gui_app, "push_widget")

  class FakeConfirmPage:
    def __init__(self, on_confirm):
      self.on_confirm = on_confirm

  mocker.patch.object(mici_toggles, "DisableDMConfirmPage", FakeConfirmPage, create=True)
  layout = object.__new__(mici_toggles.TogglesLayoutMici)
  layout._disable_dm_toggle = FakeControl("disable driver monitoring", initial_state=True)
  layout._update_toggles = mocker.Mock()

  layout._on_disable_driver_monitoring(True)

  assert not state.params.get_bool("DisableDriverMonitoring")
  assert not layout._disable_dm_toggle.checked
  pushed.call_args.args[0].on_confirm()
  assert state.params.get_bool("DisableDriverMonitoring")
  assert state.params.get_bool("OnroadCycleRequested")


def test_c3x_dm_restore_requests_onroad_cycle(mocker):
  state = FakeUiState({"DisableDriverMonitoring": True})
  mocker.patch.object(mici_toggles, "ui_state", state)
  layout = object.__new__(mici_toggles.TogglesLayoutMici)
  layout._disable_dm_toggle = FakeControl("disable driver monitoring")
  layout._update_toggles = mocker.Mock()

  layout._on_disable_driver_monitoring(False)

  assert not state.params.get_bool("DisableDriverMonitoring")
  assert state.params.get_bool("OnroadCycleRequested")
