import openpilot.selfdrive.ui.layouts.sidebar as sidebar
from openpilot.selfdrive.ui.lib.prime_state import PrimeState


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
