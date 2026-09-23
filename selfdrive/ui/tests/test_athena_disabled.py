import openpilot.selfdrive.ui.layouts.sidebar as sidebar
from openpilot.selfdrive.ui.lib.prime_state import PrimeState, PrimeType


def test_prime_worker_does_not_start_when_athena_disabled(mocker):
  state = PrimeState()
  thread = mocker.patch("openpilot.selfdrive.ui.lib.prime_state.threading.Thread")
  state.start()
  thread.assert_not_called()


def test_prime_fetch_does_not_call_comma_api_when_athena_disabled(mocker):
  state = PrimeState()
  state._params = mocker.MagicMock()
  state._params.get.return_value = "test-dongle"
  mocker.patch("openpilot.selfdrive.ui.lib.prime_state.get_token", return_value="token")
  api_get = mocker.patch("openpilot.selfdrive.ui.lib.prime_state.api_get")
  state._fetch_prime_status()
  api_get.assert_not_called()


def test_sidebar_reports_connect_disabled():
  label, value, color = sidebar.connection_status(last_ping=0, now_ns=0, athena_enabled=False)
  assert (label, value) == ("CONNECT", "DISABLED")
  assert (color.r, color.g, color.b, color.a) == (sidebar.Colors.GRAY.r, sidebar.Colors.GRAY.g, sidebar.Colors.GRAY.b, sidebar.Colors.GRAY.a)


def test_sidebar_reports_disabled_by_default():
  # No athena_enabled override: exercises this fork's actual binding, the default argument
  # at selfdrive/ui/layouts/sidebar.py:55 (connection_status(..., athena_enabled=ATHENA_ENABLED)),
  # not just the function's branch. test_sidebar_reports_connect_disabled above passes
  # athena_enabled=False explicitly, so it stays green even if ATHENA_ENABLED were flipped
  # back to True; this one does not.
  label, value, color = sidebar.connection_status(last_ping=0, now_ns=0)
  assert (label, value) == ("CONNECT", "DISABLED")
  assert (color.r, color.g, color.b, color.a) == (sidebar.Colors.GRAY.r, sidebar.Colors.GRAY.g, sidebar.Colors.GRAY.b, sidebar.Colors.GRAY.a)


def test_prime_home_screen_shows_neither_pairing_prompt_nor_subscription_claim():
  # With ATHENA_ENABLED False, prime_type is frozen wherever _load_initial_state() found
  # it and can never advance -- so both branches the UI actually reads (is_paired() in
  # selfdrive/ui/widgets/setup.py, is_prime() in selfdrive/ui/widgets/prime.py) must give a
  # single, permanent answer that shows neither an unclosable pairing flow nor a
  # subscription that can no longer be verified or revoked, whatever state was persisted
  # by a previous, Athena-enabled build.
  for frozen_prime_type in (PrimeType.UNKNOWN, PrimeType.UNPAIRED, PrimeType.NONE, PrimeType.MAGENTA):
    state = PrimeState()
    state.prime_type = frozen_prime_type
    assert state.is_paired() is True  # no "Finish Setup / Pair your device" prompt
    assert state.is_prime() is False  # no "SUBSCRIBED / comma prime" claim
