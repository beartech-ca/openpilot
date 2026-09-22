from collections.abc import Callable

from cereal import log

from openpilot.system.ui.widgets.scroller import NavScroller
from openpilot.selfdrive.ui.mici.widgets.button import BigParamControl, BigMultiParamToggle, BigToggle, GreyBigButton
from openpilot.selfdrive.ui.mici.widgets.dialog import BigConfirmationCircleButton
from openpilot.system.ui.lib.application import gui_app
from openpilot.selfdrive.ui.layouts.settings.common import restart_needed_callback
from openpilot.selfdrive.ui.ui_state import ui_state

PERSONALITY_TO_INT = log.LongitudinalPersonality.schema.enumerants


class DisableDMConfirmPage(NavScroller):
  def __init__(self, on_confirm: Callable[[], None]):
    super().__init__()

    warning = gui_app.texture("icons_mici/setup/warning.png", 64, 58)
    confirm = BigConfirmationCircleButton("disable driver\nmonitoring", warning, lambda: self.dismiss(on_confirm), red=True)
    self._scroller.add_widgets([
      GreyBigButton("disable driver monitoring", "scroll to continue", warning),
      GreyBigButton("", "driver monitoring and driver camera recording will be disabled"),
      GreyBigButton("", "the camera stays powered, but its frames will not be analyzed or saved"),
      GreyBigButton("", "you must stay attentive and remain responsible for safe operation"),
      GreyBigButton("", "changing this setting restarts the onroad processes"),
      confirm,
    ])


class TogglesLayoutMici(NavScroller):
  def __init__(self):
    super().__init__()

    self._personality_toggle = BigMultiParamToggle("driving personality", "LongitudinalPersonality", ["aggressive", "standard", "relaxed"])
    self._experimental_btn = BigParamControl("experimental mode", "ExperimentalMode")
    is_metric_toggle = BigParamControl("use metric units", "IsMetric")
    ldw_toggle = BigParamControl("lane departure warnings", "IsLdwEnabled")
    always_on_dm_toggle = BigParamControl("always-on driver monitor", "AlwaysOnDM")
    self._disable_dm_toggle = BigToggle("disable driver monitoring",
                                        initial_state=ui_state.params.get_bool("DisableDriverMonitoring"),
                                        toggle_callback=self._on_disable_driver_monitoring)
    self._record_front = BigParamControl("record & upload driver camera", "RecordFront", toggle_callback=restart_needed_callback)
    record_mic = BigParamControl("record & upload mic audio", "RecordAudio", toggle_callback=restart_needed_callback)
    enable_openpilot = BigParamControl("enable openpilot", "OpenpilotEnabledToggle", toggle_callback=restart_needed_callback)

    self._scroller.add_widgets([
      self._personality_toggle,
      self._experimental_btn,
      is_metric_toggle,
      ldw_toggle,
      always_on_dm_toggle,
      self._disable_dm_toggle,
      self._record_front,
      record_mic,
      enable_openpilot,
    ])

    # Toggle lists
    self._refresh_toggles = (
      ("ExperimentalMode", self._experimental_btn),
      ("IsMetric", is_metric_toggle),
      ("IsLdwEnabled", ldw_toggle),
      ("AlwaysOnDM", always_on_dm_toggle),
      ("DisableDriverMonitoring", self._disable_dm_toggle),
      ("RecordFront", self._record_front),
      ("RecordAudio", record_mic),
      ("OpenpilotEnabledToggle", enable_openpilot),
    )

    enable_openpilot.set_enabled(lambda: not ui_state.engaged)
    self._disable_dm_toggle.set_enabled(lambda: not ui_state.engaged)
    self._record_front.set_enabled(False if ui_state.params.get_bool("RecordFrontLock") else
                                   (lambda: not ui_state.engaged and not ui_state.params.get_bool("DisableDriverMonitoring")))
    record_mic.set_enabled(lambda: not ui_state.engaged)

    if ui_state.params.get_bool("ShowDebugInfo"):
      gui_app.set_show_touches(True)
      gui_app.set_show_fps(True)

    ui_state.add_engaged_transition_callback(self._update_toggles)

  def _update_state(self):
    super()._update_state()

    if ui_state.sm.updated["selfdriveState"]:
      personality = PERSONALITY_TO_INT[ui_state.sm["selfdriveState"].personality]
      if personality != ui_state.personality and ui_state.started:
        self._personality_toggle.set_value(self._personality_toggle._options[personality])
      ui_state.personality = personality

  def show_event(self):
    super().show_event()
    self._update_toggles()

  def _update_toggles(self):
    ui_state.update_params()

    # CP gating for experimental mode
    if ui_state.CP is not None:
      if ui_state.has_longitudinal_control:
        self._experimental_btn.set_visible(True)
        self._personality_toggle.set_visible(True)
      else:
        # no long for now
        self._experimental_btn.set_visible(False)
        self._experimental_btn.set_checked(False)
        self._personality_toggle.set_visible(False)
        ui_state.params.remove("ExperimentalMode")

    # Refresh toggles from params to mirror external changes
    for key, item in self._refresh_toggles:
      item.set_checked(ui_state.params.get_bool(key))

  def _on_disable_driver_monitoring(self, state: bool):
    def set_disabled(disabled: bool):
      ui_state.params.put_bool("DisableDriverMonitoring", disabled, block=True)
      ui_state.params.put_bool("OnroadCycleRequested", True, block=True)
      self._update_toggles()

    if state:
      # Don't show the enabled state until the warning is confirmed.
      self._disable_dm_toggle.set_checked(False)
      gui_app.push_widget(DisableDMConfirmPage(lambda: set_disabled(True)))
    else:
      set_disabled(False)
