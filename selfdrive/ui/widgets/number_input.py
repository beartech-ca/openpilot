"""A settings row that takes a number from the on-screen keyboard.

The Transit lane-centering trim has two values a driver needs to try against the road -
how far off centre to sit, and how hard to pull - and neither is a choice between
presets. This list item shows the current value and opens the keyboard to change it.

The value is clamped to its range on the way in and stored as a FLOAT param, so a typo
cannot reach the controller: openpilot reads these while driving.
"""
import pyray as rl
from collections.abc import Callable

from openpilot.common.params import Params, UnknownKeyName
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.widgets import DialogResult
from openpilot.system.ui.widgets.button import Button, ButtonStyle
from openpilot.system.ui.widgets.keyboard import Keyboard
from openpilot.system.ui.widgets.list_view import (
  BUTTON_BORDER_RADIUS,
  BUTTON_FONT_SIZE,
  BUTTON_HEIGHT,
  BUTTON_WIDTH,
  ItemAction,
  ListItem,
)


class NumberAction(ItemAction):
  def __init__(self, param: str, minimum: float, maximum: float, default: float,
               decimals: int = 2, suffix: str = ""):
    super().__init__(BUTTON_WIDTH, True)
    self._param = param
    self._min = minimum
    self._max = maximum
    self._default = default
    self._decimals = decimals
    self._suffix = suffix
    self._params = Params()
    # min_text_size has to be 0: the keyboard disables its confirm key while the text is
    # shorter than it, so any other value makes the empty entry that restores the default
    # impossible to submit.
    self._keyboard = Keyboard(min_text_size=0, max_text_size=8)
    # _render runs every frame, and Params.get is a file read. Read once and keep it.
    self._cached = self._read()
    self._button = Button("", click_callback=self._open, button_style=ButtonStyle.LIST_ACTION,
                          border_radius=BUTTON_BORDER_RADIUS, font_size=BUTTON_FONT_SIZE)

  def _read(self) -> float:
    # UnknownKeyName belongs here with the rest: params_keys.h is compiled into
    # params_pyx, so a source tree whose build has not caught up yet has the row but not
    # the key. This runs while MainLayout is building every panel, so letting it out takes
    # the whole UI down at startup rather than showing one row with a stale number.
    try:
      raw = self._params.get(self._param, return_default=True)
      return float(raw.decode() if isinstance(raw, bytes) else raw)
    except (TypeError, ValueError, AttributeError, UnknownKeyName):
      return self._default

  def _value(self) -> float:
    return self._cached

  def _label(self) -> str:
    return f"{self._value():.{self._decimals}f}{self._suffix}"

  def set_touch_valid_callback(self, touch_callback: Callable[[], bool]) -> None:
    super().set_touch_valid_callback(touch_callback)
    self._button.set_touch_valid_callback(touch_callback)

  def _open(self):
    self._keyboard.reset()
    # reset() drops the keyboard back to letters, and every value here is a number. There is
    # no public way to pick the page; a rename upstream leaves a dead attribute and the
    # letters page, which is what it does today anyway.
    self._keyboard._layout_name = "numbers"
    self._keyboard.set_text(f"{self._value():.{self._decimals}f}")
    # The driver has to be able to get back to the shipped value after trying things on the
    # road, and there is no reset button on a list row: an empty entry restores it.
    d = self._decimals
    subtitle = (f"{self._min:.{d}f} to {self._max:.{d}f}{self._suffix}" +
                f"  ·  {tr('default')} {self._default:.{d}f}{self._suffix}" +
                f"  ·  {tr('blank restores it')}")
    self._keyboard.set_title(tr("Enter a value"), subtitle)
    self._keyboard.set_callback(self._submit)
    gui_app.push_widget(self._keyboard)

  def _submit(self, result: DialogResult):
    if result != DialogResult.CONFIRM:
      return
    text = self._keyboard.text.strip()
    if not text:
      self._store(self._default)
      return
    try:
      value = float(text)
    except ValueError:
      return  # not a number: leave the stored value alone
    self._store(max(self._min, min(self._max, value)))

  def _store(self, value: float) -> None:
    """Write the value as a float, and wait for it.

    These are FLOAT params and Params.put type-checks: handing it the formatted string
    raises TypeError, which propagates out of the keyboard's callback and closes the
    dialog without saving - the value simply never took. Blocking matters too, because
    put is non-blocking by default and the row re-reads the param to redraw itself; an
    async write leaves the row showing the previous number for a moment after a change
    the driver just made.
    """
    value = round(float(value), self._decimals)
    try:
      self._params.put(self._param, value, block=True)
    except UnknownKeyName:
      return  # see _read: the build has no such key, so there is nowhere to put it
    self._cached = value

  def _render(self, rect: rl.Rectangle) -> bool:
    button_rect = rl.Rectangle(rect.x + rect.width - BUTTON_WIDTH, rect.y + (rect.height - BUTTON_HEIGHT) / 2,
                               BUTTON_WIDTH, BUTTON_HEIGHT)
    self._button.set_rect(button_rect)
    self._button.set_text(self._label())
    self._button.render(button_rect)
    return False


def number_item(title, description, param: str, minimum: float, maximum: float, default: float,
                decimals: int = 2, suffix: str = "") -> ListItem:
  return ListItem(title=title, description=description,
                  action_item=NumberAction(param, minimum, maximum, default, decimals, suffix))
