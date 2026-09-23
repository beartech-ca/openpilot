import random
from collections.abc import Iterable
from types import SimpleNamespace

from hypothesis import settings, given, strategies as st
from parameterized import parameterized
import pytest

from opendbc.car import Bus, gen_empty_fingerprint
from opendbc.can import CANPacker
from opendbc.car.ford import fordcan
from opendbc.car.ford.carcontroller import FordStockCruiseButton
from opendbc.car.gps import FORD_MACH_E_GPS_MESSAGES, get_car_gps_config, parse_ford_can_gps
from opendbc.car.structs import CarParams
from opendbc.car.fw_versions import build_fw_dict
from opendbc.car.ford.interface import CarInterface
from opendbc.car.ford.values import CAR, FW_QUERY_CONFIG, FW_PATTERN, FordSafetyFlags, get_platform_codes, match_vin_to_car
from opendbc.car.ford.fingerprints import FW_VERSIONS

Ecu = CarParams.Ecu


def test_stock_cruise_button_latches_context_until_release():
  button = FordStockCruiseButton()

  assert button.update(True, cruise_available=True, cruise_enabled=True) == (True, False)
  assert button.update(True, cruise_available=True, cruise_enabled=False) == (True, False)
  assert button.update(False, cruise_available=True, cruise_enabled=False) == (False, False)

  assert button.update(True, cruise_available=True, cruise_enabled=False) == (False, True)
  assert button.update(True, cruise_available=True, cruise_enabled=True) == (False, True)
  assert button.update(False, cruise_available=True, cruise_enabled=True) == (False, False)


def test_stock_cruise_button_ignores_press_with_cruise_master_off():
  button = FordStockCruiseButton()

  assert button.update(True, cruise_available=False, cruise_enabled=False) == (False, False)


ECU_ADDRESSES = {
  Ecu.eps: 0x730,          # Power Steering Control Module (PSCM)
  Ecu.abs: 0x760,          # Anti-Lock Brake System (ABS)
  Ecu.fwdRadar: 0x764,     # Cruise Control Module (CCM)
  Ecu.fwdCamera: 0x706,    # Image Processing Module A (IPMA)
  Ecu.engine: 0x7E0,       # Powertrain Control Module (PCM)
  Ecu.shiftByWire: 0x732,  # Gear Shift Module (GSM)
  Ecu.debug: 0x7D0,        # Accessory Protocol Interface Module (APIM)
  Ecu.hud: 0x720,          # Instrument Cluster Module (ICM)
}


ECU_PART_NUMBER = {
  Ecu.eps: [
    b"14D003",
  ],
  Ecu.abs: [
    b"2D053",
  ],
  Ecu.fwdRadar: [
    b"14D049",
  ],
  Ecu.fwdCamera: [
    b"14F397",  # Ford Q3
    b"14H102",  # Ford Q4
  ],
}


class TestFordFW:
  def test_vin_fallback(self):
    def vin(wmi, vds, powertrain, year):
      return f"{wmi}{vds}{powertrain}0{year}1234567"

    assert match_vin_to_car(vin("2FM", "PK4A", "A", "N")) == {str(CAR.FORD_EDGE_MK2)}
    assert match_vin_to_car(vin("3FM", "K1RA", "A", "M")) == {str(CAR.FORD_MUSTANG_MACH_E_MK1)}
    assert match_vin_to_car(vin("1FT", "F1CA", "A", "M")) == {str(CAR.FORD_F_150_MK14)}
    assert match_vin_to_car(vin("1FT", "F1CA", "L", "N")) == {str(CAR.FORD_F_150_LIGHTNING_MK1)}
    assert match_vin_to_car("0" * 17) == set()

  def test_fw_query_config(self):
    for (ecu, addr, subaddr) in FW_QUERY_CONFIG.extra_ecus:
      assert ecu in ECU_ADDRESSES, "Unknown ECU"
      assert addr == ECU_ADDRESSES[ecu], "ECU address mismatch"
      assert subaddr is None, "Unexpected ECU subaddress"

  @parameterized.expand(FW_VERSIONS.items())
  def test_fw_versions(self, car_model: str, fw_versions: dict[tuple[int, int, int | None], Iterable[bytes]]):
    for (ecu, addr, subaddr), fws in fw_versions.items():
      assert ecu in ECU_ADDRESSES, "Unknown ECU"
      assert addr == ECU_ADDRESSES[ecu], "ECU address mismatch"
      assert subaddr is None, "Unexpected ECU subaddress"

      if ecu not in ECU_PART_NUMBER:
        continue

      for fw in fws:
        assert len(fw) == 24, "Expected ECU response to be 24 bytes"

        match = FW_PATTERN.match(fw)
        assert match is not None, f"Unable to parse FW: {fw!r}"
        if match:
          part_number = match.group("part_number")
          assert part_number in ECU_PART_NUMBER[ecu], f"Unexpected part number for {fw!r}"

        codes = get_platform_codes([fw])
        assert 1 == len(codes), f"Unable to parse FW: {fw!r}"

  @settings(max_examples=100)
  @given(data=st.data())
  def test_platform_codes_fuzzy_fw(self, data):
    """Ensure function doesn't raise an exception"""
    fw_strategy = st.lists(st.binary())
    fws = data.draw(fw_strategy)
    get_platform_codes(fws)

  def test_platform_codes_spot_check(self):
    # Asserts basic platform code parsing behavior for a few cases
    results = get_platform_codes([
      b"JX6A-14C204-BPL\x00\x00\x00\x00\x00\x00\x00\x00\x00",
      b"NZ6T-14F397-AC\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00",
      b"PJ6T-14H102-ABJ\x00\x00\x00\x00\x00\x00\x00\x00\x00",
      b"LB5A-14C204-EAC\x00\x00\x00\x00\x00\x00\x00\x00\x00",
    ])
    assert results == {(b"X6A", b"J"), (b"Z6T", b"N"), (b"J6T", b"P"), (b"B5A", b"L")}

  def test_fuzzy_match(self):
    for platform, fw_by_addr in FW_VERSIONS.items():
      # Ensure there's no overlaps in platform codes
      for _ in range(20):
        car_fw = []
        for ecu, fw_versions in fw_by_addr.items():
          ecu_name, addr, sub_addr = ecu
          fw = random.choice(fw_versions)
          car_fw.append(CarParams.CarFw(ecu=ecu_name, fwVersion=fw, address=addr,
                                        subAddress=0 if sub_addr is None else sub_addr))

        CP = CarParams(carFw=car_fw)
        matches = FW_QUERY_CONFIG.match_fw_to_car_fuzzy(build_fw_dict(CP.carFw), CP.carVin, FW_VERSIONS)
        assert matches == {platform}

  def test_match_fw_fuzzy(self):
    offline_fw = {
      (Ecu.eps, 0x730, None): [
        b"L1MC-14D003-AJ\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00",
        b"L1MC-14D003-AL\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00",
      ],
      (Ecu.abs, 0x760, None): [
        b"L1MC-2D053-BA\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00",
        b"L1MC-2D053-BD\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00",
      ],
      (Ecu.fwdRadar, 0x764, None): [
        b"LB5T-14D049-AB\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00",
        b"LB5T-14D049-AD\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00",
      ],
      # We consider all model year hints for ECU, even with different platform codes
      (Ecu.fwdCamera, 0x706, None): [
        b"LB5T-14F397-AD\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00",
        b"NC5T-14F397-AF\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00",
      ],
    }
    expected_fingerprint = CAR.FORD_EXPLORER_MK6

    # ensure that we fuzzy match on all non-exact FW with changed revisions
    live_fw = {
      (0x730, None): {b"L1MC-14D003-XX\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00"},
      (0x760, None): {b"L1MC-2D053-XX\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00"},
      (0x764, None): {b"LB5T-14D049-XX\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00"},
      (0x706, None): {b"LB5T-14F397-XX\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00"},
    }
    candidates = FW_QUERY_CONFIG.match_fw_to_car_fuzzy(live_fw, '', {expected_fingerprint: offline_fw})
    assert candidates == {expected_fingerprint}

    # model year hint in between the range should match
    live_fw[(0x706, None)] = {b"MB5T-14F397-XX\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00"}
    candidates = FW_QUERY_CONFIG.match_fw_to_car_fuzzy(live_fw, '', {expected_fingerprint: offline_fw,})
    assert candidates == {expected_fingerprint}

    # unseen model year hint should not match
    live_fw[(0x760, None)] = {b"M1MC-2D053-XX\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00"}
    candidates = FW_QUERY_CONFIG.match_fw_to_car_fuzzy(live_fw, '', {expected_fingerprint: offline_fw})
    assert len(candidates) == 0, "Should not match new model year hint"


def test_mach_e_longitudinal_toggle_controls_stock_acc_selection():
  stock = CarInterface.get_params(
    CAR.FORD_MUSTANG_MACH_E_MK1, gen_empty_fingerprint(), [], False, False, False, None)
  enhanced = CarInterface.get_params(
    CAR.FORD_MUSTANG_MACH_E_MK1, gen_empty_fingerprint(), [], True, False, False, None)

  assert stock.alphaLongitudinalAvailable
  assert not stock.openpilotLongitudinalControl
  assert stock.pcmCruise
  assert not (stock.safetyConfigs[-1].safetyParam & FordSafetyFlags.LONG_CONTROL)

  assert enhanced.alphaLongitudinalAvailable
  assert enhanced.openpilotLongitudinalControl
  assert enhanced.safetyConfigs[-1].safetyParam & FordSafetyFlags.LONG_CONTROL


def test_mach_e_can_gps_decode():
  nav1 = {
    "GpsHsphLattSth_D_Actl": 2,
    "GpsHsphLongEast_D_Actl": 2,
    "GPS_Latitude_Degrees": 37,
    "GPS_Latitude_Minutes": 57,
    "GPS_Latitude_Min_dec": 0.8864,
    "GPS_Longitude_Degrees": -121,
    "GPS_Longitude_Minutes": 44,
    "GPS_Longitude_Min_dec": 0.22,
  }
  nav2 = {
    "GpsUtcYr_No_Actl": 2026,
    "GpsUtcMnth_No_Actl": 8,
    "GpsUtcDay_No_Actl": 26,
    "GPS_UTC_hours": 0,
    "GPS_UTC_minutes": 24,
    "GPS_UTC_seconds": 38,
    "Gps_B_Falt": 0,
  }
  nav3 = {
    "GPS_dimension": 2,
    "GPS_Hdop": 0.6,
    "GPS_Vdop": 0.8,
    "GPS_Sat_num_in_view": 31,
    "GPS_MSL_altitude": 90,
    "GPS_Speed": 10,
    "GPS_Heading": 180,
  }

  gps = parse_ford_can_gps(nav1, nav2, nav3)

  assert gps is not None
  assert gps["latitude"] == 37.96477333333333
  assert gps["longitude"] == -121.737
  assert abs(gps["altitude"] - 27.432) < 1e-9
  assert abs(gps["speed"] - 10 * 0.44704) < 1e-9
  assert gps["hasFix"]
  assert gps["satelliteCount"] == 0  # 31 is Ford's invalid sentinel.


def test_mach_e_can_gps_fault_invalidates_fix():
  nav1 = {
    "GpsHsphLattSth_D_Actl": 2,
    "GpsHsphLongEast_D_Actl": 2,
    "GPS_Latitude_Degrees": 37,
    "GPS_Latitude_Minutes": 57,
    "GPS_Latitude_Min_dec": 0.8864,
    "GPS_Longitude_Degrees": -121,
    "GPS_Longitude_Minutes": 44,
    "GPS_Longitude_Min_dec": 0.22,
  }
  nav2 = {
    "GpsUtcYr_No_Actl": 2026,
    "GpsUtcMnth_No_Actl": 8,
    "GpsUtcDay_No_Actl": 26,
    "GPS_UTC_hours": 0,
    "GPS_UTC_minutes": 24,
    "GPS_UTC_seconds": 38,
    "Gps_B_Falt": 1,
  }
  nav3 = {
    "GPS_dimension": 2,
    "GPS_Hdop": 0.6,
    "GPS_Vdop": 0.8,
    "GPS_Sat_num_in_view": 31,
    "GPS_MSL_altitude": 90,
    "GPS_Speed": 0,
    "GPS_Heading": 180,
  }

  gps = parse_ford_can_gps(nav1, nav2, nav3)

  assert gps is not None
  assert not gps["hasFix"]
  assert gps["latitude"] == 37.96477333333333
  assert gps["altitude"] == 0.0


def test_mach_e_can_gps_messages_are_optional_main_bus_inputs():
  cp = CarInterface.get_params(CAR.FORD_MUSTANG_MACH_E_MK1, gen_empty_fingerprint(), [], False, False, False, None)
  parser = CarInterface.CarState.get_can_parsers(cp)[Bus.pt]
  gps_config = get_car_gps_config(cp)

  assert gps_config is not None
  assert gps_config.messages == FORD_MACH_E_GPS_MESSAGES
  assert gps_config.decoder is parse_ford_can_gps
  assert set(parser.addresses) >= {0x462, 0x463, 0x464}
  assert set(FORD_MACH_E_GPS_MESSAGES) == set(gps_config.messages) == {
    parser.dbc.addr_to_msg[0x462].name,
    parser.dbc.addr_to_msg[0x463].name,
    parser.dbc.addr_to_msg[0x464].name,
  }
  assert all(parser.message_states[address].ignore_alive for address in (0x462, 0x463, 0x464))


def test_lightning_low_rate_camera_messages_use_declared_frequencies():
  cp = CarInterface.get_params(CAR.FORD_F_150_LIGHTNING_MK1, gen_empty_fingerprint(), [], True, False, False, None)
  cp.enableBsm = True
  parser = CarInterface.CarState.get_can_parsers(cp)[Bus.cam]

  expected_frequencies = {
    "IPMA_Data": 1,
    "Traffic_RecognitnData": 1,
    "Side_Detect_L_Stat": 5,
    "Side_Detect_R_Stat": 5,
  }
  for message, frequency in expected_frequencies.items():
    state = parser.message_states[parser.dbc.name_to_msg[message].address]
    assert state.frequency == frequency
    assert state.timeout_threshold == pytest.approx(10e9 / frequency)


def test_hands_free_cluster_status_is_opt_in():
  packer = CANPacker("ford_lincoln_base_pt")
  CAN = SimpleNamespace(main=0)
  CP = SimpleNamespace(openpilotLongitudinalControl=False)
  hud = SimpleNamespace(leftLaneDepart=False, rightLaneDepart=False)
  stock_values = dict.fromkeys([
    "HaDsply_No_Cs", "HaDsply_No_Cnt", "AccStopStat_D_Dsply", "AccTrgDist2_D_Dsply",
    "AccStopRes_B_Dsply", "TjaWarn_D_Rq", "TjaMsgTxt_D_Dsply", "IaccLamp_D_Rq",
    "AccMsgTxt_D2_Rq", "FcwDeny_B_Dsply", "FcwMemStat_B_Actl", "AccTGap_B_Dsply",
    "CadsAlignIncplt_B_Actl", "AccFllwMde_B_Dsply", "CadsRadrBlck_B_Actl",
    "CmbbPostEvnt_B_Dsply", "AccStopMde_B_Dsply", "FcwMemSens_D_Actl",
    "FcwMsgTxt_D_Rq", "AccWarn_D_Dsply", "FcwVisblWarn_B_Rq", "FcwAudioWarn_B_Rq",
    "AccTGap_D_Dsply", "AccMemEnbl_B_RqDrv", "FdaMem_B_Stat",
  ], 0)

  regular = fordcan.create_acc_ui_msg(
    packer, CAN, CP, True, True, False, False, False, hud, stock_values)
  hands_free = fordcan.create_acc_ui_msg(
    packer, CAN, CP, True, True, False, False, False, hud, stock_values, True)
  expected_regular = packer.make_can_msg("ACCDATA_3", 0, {"Tja_D_Stat": 2})
  expected_hands_free = packer.make_can_msg("ACCDATA_3", 0, {"Tja_D_Stat": 7})

  assert regular == expected_regular
  assert hands_free == expected_hands_free


from opendbc.car.ford.values import (FordFlags, TRANSIT_LKA_AVAIL_VALUES, TransitLkaContinuation,
                                     TransitLkaIntervention, TransitLkaRamp,
                                     transit_lka_continuation_from_toggles, transit_lka_settings_from_toggles)
from opendbc.car.tests.test_car_interfaces import get_test_starpilot_toggles

TransmissionType = CarParams.TransmissionType


def _transit_toggles(**overrides):
  toggles = get_test_starpilot_toggles()
  toggles.transit_lka_intervention = 0
  toggles.transit_lka_ramp = 0
  toggles.transit_lka_continuation = False
  for key, value in overrides.items():
    setattr(toggles, key, value)
  return toggles


def _transit_params(fingerprint_main=None, car_fw=None, candidate=CAR.FORD_TRANSIT_MK5, alpha_long=False, toggles=None):
  fingerprint = {0: {0x176: 8} if fingerprint_main is None else fingerprint_main, 1: {}, 2: {}}
  return CarInterface.get_params(candidate, fingerprint, car_fw or [], alpha_long=alpha_long, is_release=True,
                                 docs=False, starpilot_toggles=toggles or _transit_toggles())


class TestTransitFingerprint:
  # Recorded on the owner's van on 2026-09-16. The ADAS, parkingAdas and engine ECUs also
  # answered, but their responses are logging-only and never take part in matching.
  RECORDED = {
    0x730: b'KK21-14D003-AJ\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
    0x760: b'NK41-2D053-AF\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
    0x706: b'NK3T-14F397-AA\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
    0x764: b'LB5T-14D049-AB\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
  }

  def test_platform_steers_through_lka(self):
    assert CAR.FORD_TRANSIT_MK5.config.flags & FordFlags.LKA_STEERING

  def test_specs_are_the_owners_van(self):
    specs = CAR.FORD_TRANSIT_MK5.config.specs
    assert (specs.mass, specs.wheelbase, specs.steerRatio) == (2864, 3.750, 20.9)

  def test_recorded_firmware_is_listed(self):
    flat = {}
    for (_ecu, addr, _sub), versions in FW_VERSIONS[CAR.FORD_TRANSIT_MK5].items():
      flat.setdefault(addr, []).extend(versions)
    assert set(flat) == set(self.RECORDED)
    for addr, fw in self.RECORDED.items():
      assert fw in flat[addr], f"firmware {fw!r} missing for {hex(addr)}"
      corrupted = bytes([fw[0] ^ 0xFF]) + fw[1:]
      assert corrupted not in flat[addr]


class TestTransitInterface:
  @staticmethod
  def _pscm_asbuilt_fw(tja, lca):
    fw = bytearray(24)
    fw[7] = tja
    fw[8] = lca
    return CarParams.CarFw(ecu=Ecu.eps, address=ECU_ADDRESSES[Ecu.eps], request=[b'\x22\xDE\x01'], fwVersion=bytes(fw))

  def test_0x176_means_automatic(self):
    ret = _transit_params({0x176: 8})
    assert ret.transmissionType == TransmissionType.automatic
    assert ret.minEnableSpeed == -1

  def test_no_0x176_stays_manual(self):
    assert _transit_params({}).transmissionType == TransmissionType.manual

  def test_pscm_tja_lca_bytes_do_not_dashcam_an_lka_platform(self):
    ret = _transit_params(car_fw=[self._pscm_asbuilt_fw(0x01, 0x01)])
    assert not ret.dashcamOnly

  def test_the_same_bytes_still_dashcam_a_curvature_platform(self):
    ret = _transit_params({}, car_fw=[self._pscm_asbuilt_fw(0x01, 0x01)], candidate=CAR.FORD_ESCAPE_MK4)
    assert ret.dashcamOnly

  def test_lka_safety_flag_set(self):
    assert _transit_params().safetyConfigs[-1].safetyParam & FordSafetyFlags.LKA_STEERING

  def test_steer_actuator_delay_is_blues(self):
    assert _transit_params().steerActuatorDelay == pytest.approx(0.2)

  @pytest.mark.parametrize("on", [False, True])
  def test_continuation_toggle_reaches_safety_param(self, on):
    ret = _transit_params(toggles=_transit_toggles(transit_lka_continuation=on))
    assert bool(ret.safetyConfigs[-1].safetyParam & FordSafetyFlags.LKA_CONTINUATION) is on

  def test_continuation_never_lands_on_a_curvature_platform(self):
    ret = _transit_params({}, candidate=CAR.FORD_ESCAPE_MK4, toggles=_transit_toggles(transit_lka_continuation=True))
    assert not ret.safetyConfigs[-1].safetyParam & FordSafetyFlags.LKA_CONTINUATION

  def test_continuation_flag_value_is_pinned(self):
    # Mirrored as FORD_PARAM_LKA_CONTINUATION in opendbc/safety/modes/ford.h; the two must
    # stay equal or panda latches on a different bit than openpilot sets.
    assert FordSafetyFlags.LKA_CONTINUATION == 8

  def test_toggles_missing_entirely_read_as_defaults(self):
    # DummyCarController passes starpilot_toggles=None; nothing here may assume the attributes exist
    assert transit_lka_settings_from_toggles(None) == (TransitLkaIntervention.STANDARD, TransitLkaRamp.SLOW)
    assert transit_lka_continuation_from_toggles(None) is False

  def test_out_of_range_switch_values_fall_back_to_the_default(self):
    toggles = _transit_toggles(transit_lka_intervention=7, transit_lka_ramp=-1)
    assert transit_lka_settings_from_toggles(toggles) == (TransitLkaIntervention.STANDARD, TransitLkaRamp.SLOW)

  def test_in_range_values_are_honoured(self):
    toggles = _transit_toggles(transit_lka_intervention=2, transit_lka_ramp=1)
    assert transit_lka_settings_from_toggles(toggles) == (TransitLkaIntervention.PRESET, TransitLkaRamp.FAST)

  def test_only_the_reports_that_offer_lka_are_accepted(self):
    assert TRANSIT_LKA_AVAIL_VALUES == (2, 3)
    assert int(TransitLkaContinuation.ON) == 1


import itertools
import math

from opendbc.can import CANParser
from opendbc.car import structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.ford.carcontroller import TransitLkaState
from opendbc.car.ford.carstate import CarState
from opendbc.car.ford.values import DBC


def _transit_interface(toggles=None, alpha_long=False):
  toggles = toggles or _transit_toggles()
  fingerprint = {0: {0x176: 8}, 1: {}, 2: {}}
  CP = CarInterface.get_params(CAR.FORD_TRANSIT_MK5, fingerprint, [], alpha_long=alpha_long, is_release=True,
                               docs=False, starpilot_toggles=toggles)
  FPCP = CarInterface.get_starpilot_params(CAR.FORD_TRANSIT_MK5, fingerprint, [], CP, toggles)
  car_interface = CarInterface(CP, FPCP)
  car_interface.update([], toggles)
  return car_interface, toggles


def _decode(message, addr, dat):
  # Register through the constructor: CANParser registers lazily on first vl access and
  # would otherwise skip the very frame being decoded.
  parser = CANParser("ford_lincoln_base_pt", [(message, 0)], 0)
  parser.update([(0, [(addr, dat, 0)])])
  return dict(parser.vl[message])


def _frames(car_interface, toggles, CC, addr, frames=20):
  seen = []
  for i in range(frames):
    _, can_sends = car_interface.apply(CC, i, toggles)
    seen += [dat for a, dat, _bus in can_sends if a == addr]
  assert seen, f"{hex(addr)} was never sent"
  return seen


class TestTransitLkaMessage:
  def setup_method(self):
    self.packer = CANPacker("ford_lincoln_base_pt")
    self.CAN = fordcan.CanBus(None, {0: {}})

  def test_inactive_sends_zero_action_and_a_true_zero_angle(self):
    addr, dat, _bus = fordcan.create_transit_lka_msg(self.packer, self.CAN, False, 3.0, 4, 1)
    assert addr == 0x3CA
    assert (dat[0] >> 5) == 0
    # LaRefAng_No_Req has a -102.4 mrad DBC offset; an all-zero payload would decode to -102.4
    assert _decode("Lane_Assist_Data1", addr, dat)["LaRefAng_No_Req"] == 0.0

  def test_active_sends_requested_action(self):
    addr, dat, _bus = fordcan.create_transit_lka_msg(self.packer, self.CAN, True, 3.0, 4, 1)
    assert _decode("Lane_Assist_Data1", addr, dat)["LkaActvStats_D2_Req"] == 4

  def test_angle_is_clipped_to_the_wire_limit(self):
    addr, hi, _b = fordcan.create_transit_lka_msg(self.packer, self.CAN, True, 99.0, 2, 0)
    _a, cap, _b = fordcan.create_transit_lka_msg(self.packer, self.CAN, True, 5.8, 2, 0)
    assert hi == cap
    assert math.isclose(_decode("Lane_Assist_Data1", addr, hi)["LaRefAng_No_Req"], math.radians(5.8) * 1000.0, abs_tol=0.05)

  def test_frames_are_byte_identical_to_blue(self):
    inactive = bytes(fordcan.create_transit_lka_msg(self.packer, self.CAN)[1])
    active = bytes(fordcan.create_transit_lka_msg(self.packer, self.CAN, True, 5.0, 2, 1)[1])
    assert inactive == bytes.fromhex("0380080000000000")
    assert active == bytes.fromhex("43800ed180000000")


class TestTransitLateralMotionControlHeartbeat:
  """0x3D3 shares limiter state with the 0x3CA angle check in panda, so on the Transit it may
  only ever carry an inactive heartbeat."""

  def test_lka_platform_never_requests_lateral_on_0x3d3(self):
    car_interface, toggles = _transit_interface()
    CC = structs.CarControl()
    CC.enabled = True
    CC.latActive = True
    CC.actuators.steeringAngleDeg = 10.0
    CC.actuators.curvature = 0.01
    seen = [_decode("LateralMotionControl", 0x3D3, dat)["LatCtl_D_Rq"] for dat in _frames(car_interface, toggles, CC.as_reader(), 0x3D3)]
    assert all(v == 0 for v in seen), seen


class TestTransitLkaIntervention:
  def test_standard_position_never_escalates(self):
    s = TransitLkaState(TransitLkaIntervention.STANDARD, TransitLkaRamp.SLOW)
    assert s.update(9.0, 12.0, 0.0)[0] == 4

  def test_increasing_position_always_escalates(self):
    s = TransitLkaState(TransitLkaIntervention.INCREASING, TransitLkaRamp.SLOW)
    assert s.update(0.5, 0.5, 0.0)[0] == 6

  def test_preset_latches_on_both_conditions(self):
    s = TransitLkaState(TransitLkaIntervention.PRESET, TransitLkaRamp.SLOW)
    assert s.update(5.5, 4.0, 0.0)[0] == 4
    assert s.update(4.0, 6.0, 0.0)[0] == 4
    assert s.update(5.5, 6.0, 0.0)[0] == 6
    assert s.update(4.8, 5.0, 0.0)[0] == 6
    assert s.update(4.5, 4.7, 0.0)[0] == 4


class TestTransitLkaHysteresisResetsOnSwitchChange:
  """set_selection() is how the controller now applies a live switch change (carcontroller.py's
  LKA_STEERING block calls it once per LKA frame). PRESET's own if/elif only change state on
  crossing a threshold, so flipping into PRESET from a position that forced increasing/fast
  True must not leave that True latched through the neutral band."""

  def test_switching_into_preset_does_not_inherit_increasing(self):
    s = TransitLkaState(TransitLkaIntervention.INCREASING, TransitLkaRamp.SLOW)
    s.update(0.5, 0.5, 0.0)  # INCREASING forces self.increasing True every frame
    s.set_selection(TransitLkaIntervention.PRESET, TransitLkaRamp.SLOW)
    # Inside the neutral band: not >= the entry pair (req>5.0 and desired>=5.2), not the
    # exit pair either (req<4.6 and desired<4.8), so a latched increasing would persist.
    action, _ = s.update(4.8, 4.9, 0.0)
    assert action == 4, "PRESET inherited INCREASING's escalated action across the switch flip"

  def test_switching_into_preset_does_not_inherit_fast(self):
    s = TransitLkaState(TransitLkaIntervention.STANDARD, TransitLkaRamp.FAST)
    s.update(0.5, 0.5, 0.0)  # FAST forces self.fast True every frame
    s.set_selection(TransitLkaIntervention.STANDARD, TransitLkaRamp.PRESET)
    # Inside the neutral band: not >= RAMP_ENTER_REQ (1.8), not < RAMP_EXIT_REQ (1.5), so a
    # latched fast would persist.
    _, ramp_type = s.update(1.6, 1.6, 0.0)
    assert ramp_type == 0, "PRESET inherited FAST's ramp across the switch flip"

  def test_unchanged_selection_does_not_reset_mid_latch(self):
    # set_selection is called every LKA frame even when nothing changed; it must not
    # clobber a PRESET latch that is legitimately holding mid-cycle.
    s = TransitLkaState(TransitLkaIntervention.PRESET, TransitLkaRamp.SLOW)
    s.update(5.5, 6.0, 0.0)
    assert s.increasing
    s.set_selection(TransitLkaIntervention.PRESET, TransitLkaRamp.SLOW)
    action, _ = s.update(4.8, 4.9, 0.0)  # neutral band: still escalated if truly unchanged
    assert action == 6


class TestTransitLkaRamp:
  def test_preset_enters_on_angle(self):
    s = TransitLkaState(TransitLkaIntervention.STANDARD, TransitLkaRamp.PRESET)
    assert s.update(1.0, 1.0, 0.0)[1] == 0
    assert s.update(1.9, 1.9, 0.0)[1] == 1

  def test_preset_enters_on_demand_rate_after_the_filter_settles(self):
    s = TransitLkaState(TransitLkaIntervention.STANDARD, TransitLkaRamp.PRESET)
    for _ in range(40):
      out = s.update(0.2, 0.2, 40.0)
    assert out[1] == 1

  def test_filter_rejects_a_single_rate_spike(self):
    s = TransitLkaState(TransitLkaIntervention.STANDARD, TransitLkaRamp.PRESET)
    assert s.update(0.2, 0.2, 60.0)[1] == 0

  def test_preset_holds_inside_the_band(self):
    s = TransitLkaState(TransitLkaIntervention.STANDARD, TransitLkaRamp.PRESET)
    s.update(2.0, 2.0, 0.0)
    assert s.update(1.6, 1.6, 0.0)[1] == 1
    assert s.update(1.4, 1.4, 0.0)[1] == 0


class TestTransitLkaActionPairing:
  """The code names the side the van is departing toward, so a positive (leftward) request pairs
  with a Right code, which is what the stock camera does."""

  def test_a_leftward_request_is_a_rightward_departure(self):
    s = TransitLkaState(TransitLkaIntervention.STANDARD, TransitLkaRamp.SLOW)
    assert s.update(1.0, 1.0, 0.0)[0] == 4
    assert s.update(-1.0, -1.0, 0.0)[0] == 2

  def test_escalated_pairing_keeps_the_same_sides(self):
    s = TransitLkaState(TransitLkaIntervention.INCREASING, TransitLkaRamp.SLOW)
    assert s.update(1.0, 1.0, 0.0)[0] == 6
    assert s.update(-1.0, -1.0, 0.0)[0] == 1

  def test_deadband_gives_no_intervention(self):
    s = TransitLkaState(TransitLkaIntervention.STANDARD, TransitLkaRamp.SLOW)
    assert s.update(0.05, 0.05, 0.0)[0] == 0


class TestTransitLkaSwitchesAreLive:
  """Intervention and ramp are read from starpilot_toggles on every LKA frame, so a change
  made while driving takes effect without a restart."""

  @staticmethod
  def _actions(toggles):
    car_interface, _ = _transit_interface(toggles)
    car_interface.CS.lkas_available = True
    CC = structs.CarControl()
    CC.enabled = True
    CC.latActive = True
    CC.actuators.steeringAngleDeg = 4.0
    return [_decode("Lane_Assist_Data1", 0x3CA, dat)["LkaActvStats_D2_Req"] for dat in _frames(car_interface, toggles, CC.as_reader(), 0x3CA)]

  def test_default_reproduces_the_shipped_behaviour(self):
    assert set(self._actions(_transit_toggles())) <= {2, 4}
    assert any(a != 0 for a in self._actions(_transit_toggles()))

  def test_increasing_flips_the_action_codes(self):
    assert set(self._actions(_transit_toggles(transit_lka_intervention=int(TransitLkaIntervention.INCREASING)))) <= {1, 6}

  def test_a_change_between_frames_is_picked_up(self):
    toggles = _transit_toggles()
    car_interface, _ = _transit_interface(toggles)
    car_interface.CS.lkas_available = True
    CC = structs.CarControl()
    CC.enabled = True
    CC.latActive = True
    CC.actuators.steeringAngleDeg = 4.0
    CC = CC.as_reader()
    before = [_decode("Lane_Assist_Data1", 0x3CA, dat)["LkaActvStats_D2_Req"] for dat in _frames(car_interface, toggles, CC, 0x3CA)]
    toggles.transit_lka_intervention = int(TransitLkaIntervention.INCREASING)
    after = [_decode("Lane_Assist_Data1", 0x3CA, dat)["LkaActvStats_D2_Req"] for dat in _frames(car_interface, toggles, CC, 0x3CA)]
    assert set(before) <= {2, 4} and set(after) <= {1, 6}


class TestTransitLkaAvailability:
  """LaActAvail_D_Actl is not in get_can_parsers' pt list on this (non-CAN-FD) platform; CarState
  registers Lane_Assist_Data3_FD1 lazily on first cp.vl access, which _transit_interface's setup
  update() already does, so a frame sent on any later update() is parsed normally."""

  @staticmethod
  def _lane_assist_data1_while(la_act_avail):
    car_interface, toggles = _transit_interface()
    packer = CANPacker(DBC[CAR.FORD_TRANSIT_MK5][Bus.pt])
    msg = packer.make_can_msg("Lane_Assist_Data3_FD1", 0, {"LaActAvail_D_Actl": la_act_avail})
    car_interface.update([(1_000_000_000, [msg])], toggles)
    CC = structs.CarControl()
    CC.enabled = True
    CC.latActive = True
    CC.actuators.steeringAngleDeg = 4.0
    return [_decode("Lane_Assist_Data1", 0x3CA, dat) for dat in _frames(car_interface, toggles, CC.as_reader(), 0x3CA)]

  def test_a_suppressed_pscm_gets_an_empty_frame(self):
    for v in self._lane_assist_data1_while(0):
      assert v["LkaActvStats_D2_Req"] == 0
      assert v["LaRefAng_No_Req"] == 0.0

  def test_an_offering_pscm_gets_the_request(self):
    seen = self._lane_assist_data1_while(3)
    assert any(v["LkaActvStats_D2_Req"] != 0 for v in seen)
    assert any(abs(v["LaRefAng_No_Req"]) > 1.0 for v in seen)

  def test_state_2_is_now_accepted(self):
    # TRANSIT_LKA_AVAIL_VALUES == (2, 3) widened this from the shipped ==3-only check.
    seen = self._lane_assist_data1_while(2)
    assert any(v["LkaActvStats_D2_Req"] != 0 for v in seen)

  def test_state_1_is_still_suppressed(self):
    seen = self._lane_assist_data1_while(1)
    assert all(v["LkaActvStats_D2_Req"] == 0 for v in seen)


class TestTransitLkaContinuation:
  ENGAGED, STANDBY, OFF = 5, 3, 0

  @staticmethod
  def _carstate(on):
    toggles = _transit_toggles(transit_lka_continuation=on)
    CP = _transit_params(toggles=toggles)
    FPCP = CarInterface.get_starpilot_params(CAR.FORD_TRANSIT_MK5, {0: {0x176: 8}, 1: {}, 2: {}}, [], CP, toggles)
    cs = CarState(CP, FPCP)
    can_parsers = cs.get_can_parsers(CP)
    return cs, can_parsers, toggles

  @staticmethod
  def _driver(cs, can_parsers, toggles):
    """Returns a step(cruise_state, speed, brake) closure that drives CarState.update with
    synthesized EngBrakeData/BrakeSysFeatures frames -- the same three signals panda's own
    latch reads -- and returns the real ret.cruiseState.enabled, not a private copy of it."""
    packer = CANPacker(DBC[CAR.FORD_TRANSIT_MK5][Bus.pt])
    nanos = itertools.count(100_000_000, 100_000_000)

    def step(cruise_state, speed, brake=False):
      frames = [
        packer.make_can_msg("EngBrakeData", 0, {
          "CcStat_D_Actl": cruise_state,
          "BpedDrvAppl_D_Actl": 2 if brake else 0,
        }),
        packer.make_can_msg("BrakeSysFeatures", 0, {"Veh_V_ActlBrk": speed / CV.KPH_TO_MS}),
      ]
      can_parsers[Bus.pt].update([(next(nanos), frames)])
      ret, _ = cs.update(can_parsers, toggles)
      return ret.cruiseState.enabled

    return step

  def test_enabled_comes_from_safety_param_not_toggles(self):
    assert self._carstate(True)[0].lka_continuation_enabled is True
    assert self._carstate(False)[0].lka_continuation_enabled is False

  def test_latches_on_the_cancel_and_holds_down_to_walking_pace(self):
    cs, can_parsers, toggles = self._carstate(True)
    step = self._driver(cs, can_parsers, toggles)
    assert step(self.ENGAGED, 12.0)
    assert step(self.ENGAGED, 5.2)
    assert step(self.STANDBY, 4.9)
    assert cs.lka_continuation
    for v in (4.0, 3.0, 2.0, 1.0, 0.6):
      assert step(self.STANDBY, v), f"dropped at {v} m/s"

  def test_never_latches_from_a_standing_start_in_standby(self):
    cs, can_parsers, toggles = self._carstate(True)
    step = self._driver(cs, can_parsers, toggles)
    for _ in range(5):
      assert not step(self.STANDBY, 3.0)
    assert not cs.lka_continuation

  def test_off_is_the_shipped_behaviour(self):
    cs, can_parsers, toggles = self._carstate(False)
    step = self._driver(cs, can_parsers, toggles)
    assert step(self.ENGAGED, 6.0)
    assert not step(self.STANDBY, 4.5)

  @pytest.mark.parametrize(("what", "cruise_state", "speed", "brake"), [
    ("brake pressed", STANDBY, 3.0, True),
    ("back above the exit speed", STANDBY, 9.5, False),
    ("cruise re-engaged", ENGAGED, 3.0, False),
    ("cruise switched off", OFF, 3.0, False),
    ("stopped", STANDBY, 0.2, False),
  ])
  def test_every_exit_condition_releases_the_latch(self, what, cruise_state, speed, brake):
    cs, can_parsers, toggles = self._carstate(True)
    step = self._driver(cs, can_parsers, toggles)
    step(self.ENGAGED, 6.0)
    step(self.STANDBY, 4.5)
    assert cs.lka_continuation
    step(cruise_state, speed, brake)
    assert not cs.lka_continuation, f"latch survived {what}"

  @staticmethod
  def _accdata_while(latched, accel=-2.0):
    toggles = _transit_toggles(transit_lka_continuation=True)
    car_interface, _ = _transit_interface(toggles, alpha_long=True)
    assert car_interface.CP.openpilotLongitudinalControl
    if latched:
      # Same arming sequence as test_latches_on_the_cancel_...: drive the real parse rather
      # than poking CS.lka_continuation, so this test exercises the condition that triggers
      # the ACCDATA suppression, not just the suppression itself.
      packer = CANPacker(DBC[CAR.FORD_TRANSIT_MK5][Bus.pt])
      for i, (cruise_state, speed) in enumerate(((TestTransitLkaContinuation.ENGAGED, 6.0),
                                                  (TestTransitLkaContinuation.STANDBY, 4.5))):
        frames = [
          packer.make_can_msg("EngBrakeData", 0, {"CcStat_D_Actl": cruise_state, "BpedDrvAppl_D_Actl": 0}),
          packer.make_can_msg("BrakeSysFeatures", 0, {"Veh_V_ActlBrk": speed / CV.KPH_TO_MS}),
        ]
        car_interface.update([((i + 1) * 100_000_000, frames)], toggles)
    assert car_interface.CS.lka_continuation is latched
    CC = structs.CarControl()
    CC.enabled = True
    CC.longActive = True
    CC.actuators.accel = accel
    return [_decode("ACCDATA", 0x186, dat) for dat in _frames(car_interface, toggles, CC.as_reader(), 0x186)]

  def test_longitudinal_is_forced_inactive_while_latched(self):
    for v in self._accdata_while(True):
      assert v["Cmbb_B_Enbl"] == 0
      assert v["AccBrkPrchg_B_Rq"] == 0 and v["AccBrkDecel_B_Rq"] == 0
      assert abs(v["AccBrkTot_A_Rq"]) < 0.005

  def test_the_same_request_does_get_out_when_not_latched(self):
    free = self._accdata_while(False)
    assert any(v["AccBrkTot_A_Rq"] < -0.5 for v in free)
