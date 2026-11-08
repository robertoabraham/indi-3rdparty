"""
Protocol tests for the emulated Celestron Smart DewHeater/Power Controller
(AUX target 0x17) added to the NSE simulator.

These deliberately do NOT involve the INDI driver or indiserver: the driver has
no 0x17 support yet, and per AGENTS.md the simulator must be trusted before the
driver is suspected. They check two things:

  1. DewPowerController in isolation -- every command's response payload,
     sizes and byte order, against the layouts recorded in INDI core's
     standalone driver (drivers/power/indi-celestron-dewpower/).
  2. The full framing and dispatch path through NexStarScope.handle_msg(),
     feeding real AUX byte frames in and parsing real frames out. This is what
     catches the collision risk: 0x17's command bytes overlap those of the
     other non-motor AUX targets.

Run with:  pytest -v tests/system/test_dewpower_protocol.py
"""

import os
import struct
import sys

import pytest

SIM_DIR = os.path.abspath(
    os.path.join(os.path.dirname(__file__), '..', '..', 'simulator'))
if SIM_DIR not in sys.path:
    sys.path.insert(0, SIM_DIR)

import nse_dewpower as dp                                    # noqa: E402
from nse_dewpower import DewPowerController                  # noqa: E402

APP = 0x20
DEW = 0x17
PREAMBLE = 0x3b


# --------------------------------------------------------------------------- #
# AUX framing helpers
# --------------------------------------------------------------------------- #
def checksum(payload):
    """Matches nse_telescope.make_checksum: two's complement of the sum."""
    return ((~sum(payload) + 1)) & 0xFF


def build(cmd, data=b'', src=APP, dst=DEW):
    """A full AUX packet: 0x3b <len> <src> <dst> <cmd> <data..> <checksum>"""
    body = bytes((3 + len(data), src, dst, cmd)) + bytes(data)
    return bytes((PREAMBLE,)) + body + bytes((checksum(body),))


def parse_frames(buf):
    """
    Split a response buffer into frames by declared length.

    Deliberately not reusing nse_telescope.split_cmds(): that searches for the
    0x3b preamble byte, which can also occur inside a payload. Length-driven
    parsing is what the C++ driver does.
    """
    frames = []
    i = 0
    while i < len(buf):
        assert buf[i] == PREAMBLE, 'frame %d does not start with 0x3b' % len(frames)
        length = buf[i + 1]
        total = length + 3                      # preamble + len byte + body + csum
        frame = buf[i:i + total]
        body, csum = frame[1:-1], frame[-1]
        assert checksum(body) == csum, 'bad checksum on frame %d' % len(frames)
        frames.append({
            'len': length, 'src': frame[2], 'dst': frame[3],
            'cmd': frame[4], 'data': frame[5:-1],
        })
        i += total
    return frames


def ask(scope, cmd, data=b''):
    """
    Send one command to the simulated controller and return the reply frame.

    The AUX bus echoes the request before the reply, which the simulator
    reproduces, so the reply is the *second* frame.
    """
    frames = parse_frames(scope.handle_msg(build(cmd, data)))
    assert len(frames) == 2, 'expected echo + reply, got %d frames' % len(frames)
    echo, reply = frames
    assert echo['cmd'] == cmd and echo['dst'] == DEW
    assert reply['src'] == DEW, 'reply should come from the controller'
    assert reply['dst'] == APP, 'reply should be addressed back to the app'
    return reply


@pytest.fixture
def scope_4x(monkeypatch):
    monkeypatch.setenv('NSE_DEW_MODEL', '4x')
    from nse_telescope import NexStarScope
    return NexStarScope(tui=False)


@pytest.fixture
def ctrl_4x():
    return DewPowerController('4x')


@pytest.fixture
def ctrl_2x():
    return DewPowerController('2x')


# --------------------------------------------------------------------------- #
# Model profiles
# --------------------------------------------------------------------------- #
def test_2x_has_two_dew_ports_and_one_dc_port(ctrl_2x):
    types = [p.type for p in ctrl_2x.ports]
    assert len(types) == 3, 'the 2X manual describes 2 dew + 1 power output'
    assert types.count(dp.PORT_DEW) == 2
    assert types.count(dp.PORT_DC) == 1
    assert dp.PORT_VARIABLE not in types, 'the 2X has no variable port'


def test_4x_has_four_dew_four_power_one_variable(ctrl_4x):
    types = [p.type for p in ctrl_4x.ports]
    assert len(types) == 8, 'the 4X manual describes 4 dew + 4 power outputs'
    assert types.count(dp.PORT_DEW) == 4
    assert types.count(dp.PORT_VARIABLE) == 1, 'power port #1 is the variable one'
    assert types.count(dp.PORT_DC) == 3


def test_unknown_model_rejected():
    with pytest.raises(ValueError):
        DewPowerController('8x')


def test_model_none_means_no_controller(monkeypatch):
    """The driver must be able to see a mount with no controller attached."""
    monkeypatch.setenv('NSE_DEW_MODEL', 'none')
    assert dp.from_env() is None


def test_absent_controller_answers_nothing(monkeypatch):
    monkeypatch.setenv('NSE_DEW_MODEL', 'none')
    from nse_telescope import NexStarScope
    scope = NexStarScope(tui=False)
    assert scope.dewpower is None
    # Only the bus echo comes back -- no reply frame. This is the timeout path
    # the driver's detection probe must degrade through.
    frames = parse_frames(scope.handle_msg(build(dp.GET_VERSION)))
    assert len(frames) == 1


# --------------------------------------------------------------------------- #
# Response sizes -- these are the contract the C++ driver asserts on
# --------------------------------------------------------------------------- #
@pytest.mark.parametrize('cmd,data,size', [
    (dp.GET_VERSION,           b'',       2),
    (dp.GET_NUMBER_OF_PORTS,   b'',       1),
    (dp.GET_INPUT_POWER,       b'',       6),
    (dp.GET_ENVIRONMENT,       b'',       9),
    (dp.GET_POWER_ENABLED,     b'',       1),
    (dp.GET_EXT_CURRENT_LIMIT, b'',       4),
    (dp.SELF_HEATER_STATUS,    b'',       1),
    (dp.GET_HEATER_LEVEL,      b'',       1),
    (dp.GET_LED_BRIGHTNESS,    b'',       1),
    (dp.SET_PORT_ENABLED,      b'\x00\x01', 0),
    (dp.DH_ENABLE_MANUAL,      b'\x00\x32', 0),
    (dp.DH_ENABLE_AUTO,        b'\x00\x01\x05', 0),
    (dp.SET_LED_BRIGHTNESS,    b'\x40',   0),
    (dp.RESET_FUSE,            b'',       0),
])
def test_response_sizes_match_driver_expectations(scope_4x, cmd, data, size):
    reply = ask(scope_4x, cmd, data)
    assert reply['cmd'] == cmd, 'should not have been NAKd'
    assert len(reply['data']) == size


def test_get_version_is_two_bytes(ctrl_4x):
    assert ctrl_4x.handle(dp.GET_VERSION, b'') == (dp.GET_VERSION, b'\x01\x07')


def test_number_of_ports_matches_profile(scope_4x):
    assert ask(scope_4x, dp.GET_NUMBER_OF_PORTS)['data'] == b'\x08'


# --------------------------------------------------------------------------- #
# Port info, including the idle-port quirk
# --------------------------------------------------------------------------- #
def test_active_port_returns_full_telemetry(ctrl_4x):
    """Port 0 is a dew heater with a load; drive it so it draws current."""
    ctrl_4x.handle(dp.DH_ENABLE_MANUAL, b'\x00\x64')      # port 0, 100%
    _, data = ctrl_4x.handle(dp.GET_PORT_INFO, b'\x00')
    assert len(data) == 7
    ptype, enabled, shorted = data[0], data[1], data[2]
    power_mW, voltage_mV = struct.unpack('!HH', data[3:7])
    assert ptype == dp.PORT_DEW
    assert enabled == 1 and shorted == 0
    assert power_mW == 1800
    assert voltage_mV == 12000


def test_idle_port_returns_only_its_type(ctrl_4x):
    """
    The real controller reports just the 1-byte port type for an idle port.
    Reproducing this is the point: it is what previously made every idle port
    look like portType 0 (DC) to the standalone driver.
    """
    _, data = ctrl_4x.handle(dp.GET_PORT_INFO, b'\x02')   # dew heater 3, no load
    assert len(data) == 1
    assert data[0] == dp.PORT_DEW


def test_disabling_a_port_makes_it_idle(ctrl_4x):
    ctrl_4x.handle(dp.DH_ENABLE_MANUAL, b'\x00\x64')
    assert len(ctrl_4x.handle(dp.GET_PORT_INFO, b'\x00')[1]) == 7
    ctrl_4x.handle(dp.SET_PORT_ENABLED, b'\x00\x00')
    assert len(ctrl_4x.handle(dp.GET_PORT_INFO, b'\x00')[1]) == 1


def test_port_info_out_of_range_is_nakd(scope_4x):
    reply = ask(scope_4x, dp.GET_PORT_INFO, b'\x7f')
    assert reply['cmd'] == dp.NAK
    assert reply['data'] == bytes((dp.GET_PORT_INFO,))


# --------------------------------------------------------------------------- #
# Dew heater ports
# --------------------------------------------------------------------------- #
def test_dh_port_info_layout(ctrl_4x):
    ctrl_4x.handle(dp.DH_ENABLE_AUTO, b'\x00\x01\x07')     # auto above dew point
    _, data = ctrl_4x.handle(dp.GET_DH_PORT_INFO, b'\x00')
    assert len(data) >= 6
    assert data[0] == dp.PORT_DEW
    assert struct.unpack('!b', data[1:2])[0] == dp.DH_AUTO_ABOVE_DEWPT
    assert 0 <= data[2] <= 100
    assert data[5] == 7, 'aggression level should round-trip'


def test_dh_port_info_nakd_on_non_dew_port(ctrl_4x):
    """Port 5 is a fixed 12V output on the 4X profile."""
    assert ctrl_4x.ports[5].type == dp.PORT_DC
    assert ctrl_4x.handle(dp.GET_DH_PORT_INFO, b'\x05') == (
        dp.NAK, bytes((dp.GET_DH_PORT_INFO,)))


def test_manual_mode_sets_duty_cycle(ctrl_4x):
    ctrl_4x.handle(dp.DH_ENABLE_MANUAL, b'\x00\x19')       # 25%
    _, data = ctrl_4x.handle(dp.GET_DH_PORT_INFO, b'\x00')
    assert struct.unpack('!b', data[1:2])[0] == dp.DH_MANUAL
    assert data[2] == 25
    # Draw should scale with duty cycle: 25% of the 1800 mW load.
    assert struct.unpack('!H', data[3:5])[0] == 450


def test_manual_duty_cycle_above_100_rejected(ctrl_4x):
    assert ctrl_4x.handle(dp.DH_ENABLE_MANUAL, b'\x00\x65')[0] == dp.NAK


def test_auto_mode_with_bad_mode_rejected(ctrl_4x):
    """Only auto_above_dewpoint (1) and auto_above_ambient (2) are valid."""
    assert ctrl_4x.handle(dp.DH_ENABLE_AUTO, b'\x00\x00\x05')[0] == dp.NAK
    assert ctrl_4x.handle(dp.DH_ENABLE_AUTO, b'\x00\x09\x05')[0] == dp.NAK


def test_auto_dew_raises_duty_when_lens_is_below_target(ctrl_4x):
    ctrl_4x.handle(dp.DH_ENABLE_AUTO, b'\x00\x02\x05')     # 5C above ambient
    ctrl_4x.ports[0].heater_temp_mC = 0                     # lens at 0C
    ctrl_4x.tick(1.0)
    assert ctrl_4x.ports[0].power_level > 0, 'should be heating'


def test_manual_mode_is_not_overridden_by_the_control_loop(ctrl_4x):
    ctrl_4x.handle(dp.DH_ENABLE_MANUAL, b'\x00\x0a')       # 10%
    ctrl_4x.tick(5.0)
    assert ctrl_4x.ports[0].power_level == 10


def test_shorted_dew_port_reports_mode_minus_one(ctrl_4x):
    ctrl_4x.ports[0].shorted = True
    _, data = ctrl_4x.handle(dp.GET_DH_PORT_INFO, b'\x00')
    assert struct.unpack('!b', data[1:2])[0] == dp.DH_SHORTED


def test_heater_temperature_appended_when_thermistor_present(ctrl_4x):
    ctrl_4x.ports[0].heater_temp_mC = 4250
    _, data = ctrl_4x.handle(dp.GET_DH_PORT_INFO, b'\x00')
    assert len(data) == 10
    assert struct.unpack('!i', data[6:10])[0] == 4250


# --------------------------------------------------------------------------- #
# Variable voltage port
# --------------------------------------------------------------------------- #
def test_variable_port_accepts_documented_voltages(ctrl_4x):
    for mV in (3000, 5000, 11000, 12000):
        assert ctrl_4x.handle(dp.SET_PORT_VOLTAGE,
                              b'\x04' + struct.pack('!H', mV))[0] \
            == dp.SET_PORT_VOLTAGE, '%d mV should be accepted' % mV
        assert ctrl_4x.ports[4].voltage_mV == mV


def test_variable_port_rejects_out_of_range_voltage(ctrl_4x):
    """The manual allows 3.0-11.0V or 12.0V -- nothing else."""
    for mV in (2000, 11500, 13000):
        assert ctrl_4x.handle(dp.SET_PORT_VOLTAGE,
                              b'\x04' + struct.pack('!H', mV))[0] == dp.NAK


def test_setting_voltage_on_fixed_port_is_nakd(ctrl_4x):
    assert ctrl_4x.ports[5].type == dp.PORT_DC
    assert ctrl_4x.handle(dp.SET_PORT_VOLTAGE,
                          b'\x05' + struct.pack('!H', 5000))[0] == dp.NAK


# --------------------------------------------------------------------------- #
# Environment sensor
# --------------------------------------------------------------------------- #
def test_environment_layout_and_plausibility(scope_4x):
    data = ask(scope_4x, dp.GET_ENVIRONMENT)['data']
    assert len(data) == 9
    ambient_mC = struct.unpack('!i', data[0:4])[0]
    dewpoint_mC = struct.unpack('!i', data[4:8])[0]
    humidity = data[8]
    assert -40000 < ambient_mC < 50000
    assert 0 <= humidity <= 100
    assert dewpoint_mC <= ambient_mC, 'dew point cannot exceed ambient'


def test_dew_point_formula_against_known_values():
    # 20C / 50% RH is ~9.3C; 25C / 100% RH is 25C by definition.
    assert dp.dew_point_c(20.0, 50.0) == pytest.approx(9.27, abs=0.2)
    assert dp.dew_point_c(25.0, 100.0) == pytest.approx(25.0, abs=0.1)


def test_environment_drifts_over_time(ctrl_4x):
    first = ctrl_4x.handle(dp.GET_ENVIRONMENT, b'')[1]
    ctrl_4x.tick(60.0)
    assert ctrl_4x.handle(dp.GET_ENVIRONMENT, b'')[1] != first


# --------------------------------------------------------------------------- #
# Input power and protection
# --------------------------------------------------------------------------- #
def test_input_power_layout(scope_4x):
    data = ask(scope_4x, dp.GET_INPUT_POWER)['data']
    voltage_mV, current_mA = struct.unpack('!HH', data[0:4])
    v_status = struct.unpack('!b', data[4:5])[0]
    overcurrent = data[5]
    assert 11000 <= voltage_mV <= dp.MAX_INPUT_MV
    assert current_mA >= 0
    assert v_status in (-1, 0, 1)
    assert overcurrent in (0, 1)


def test_current_reflects_enabled_loads(ctrl_4x):
    before = struct.unpack('!H', ctrl_4x.handle(dp.GET_INPUT_POWER, b'')[1][2:4])[0]
    # Turn off the 6 W fixed-output port and the draw must fall.
    ctrl_4x.handle(dp.SET_PORT_ENABLED, b'\x05\x00')
    after = struct.unpack('!H', ctrl_4x.handle(dp.GET_INPUT_POWER, b'')[1][2:4])[0]
    assert after < before


def test_overcurrent_flag_set_when_fuse_blown(ctrl_4x):
    ctrl_4x.fuse_blown = True
    assert ctrl_4x.handle(dp.GET_INPUT_POWER, b'')[1][5] == 1
    ctrl_4x.handle(dp.RESET_FUSE, b'')
    assert ctrl_4x.handle(dp.GET_INPUT_POWER, b'')[1][5] == 0


def test_reset_fuse_clears_shorts(ctrl_4x):
    ctrl_4x.ports[0].shorted = True
    ctrl_4x.handle(dp.RESET_FUSE, b'')
    assert not ctrl_4x.ports[0].shorted


def test_external_current_limit_round_trips(ctrl_4x):
    ctrl_4x.handle(dp.SET_EXT_CURRENT_LIMIT, struct.pack('!H', 3500))
    _, data = ctrl_4x.handle(dp.GET_EXT_CURRENT_LIMIT, b'')
    limit_mA, max_mA = struct.unpack('!HH', data)
    assert limit_mA == 3500
    assert max_mA == dp.MAX_CURRENT_MA


# --------------------------------------------------------------------------- #
# LED, self heater, factory reset
# --------------------------------------------------------------------------- #
def test_led_brightness_round_trips(scope_4x):
    ask(scope_4x, dp.SET_LED_BRIGHTNESS, b'\x2a')
    assert ask(scope_4x, dp.GET_LED_BRIGHTNESS)['data'] == b'\x2a'


def test_self_heater_round_trips(ctrl_4x):
    assert ctrl_4x.handle(dp.SELF_HEATER_STATUS, b'')[1] == b'\x00'
    ctrl_4x.handle(dp.ENABLE_SELF_HEATER, b'\x01')
    assert ctrl_4x.handle(dp.SELF_HEATER_STATUS, b'')[1] == b'\x01'


def test_heater_level_tracks_duty_cycles(ctrl_4x):
    for port in range(4):
        ctrl_4x.handle(dp.DH_ENABLE_MANUAL, bytes((port, 100)))
    assert ctrl_4x.handle(dp.GET_HEATER_LEVEL, b'')[1] == b'\xff'
    for port in range(4):
        ctrl_4x.handle(dp.DH_ENABLE_MANUAL, bytes((port, 0)))
    assert ctrl_4x.handle(dp.GET_HEATER_LEVEL, b'')[1] == b'\x00'


def test_factory_reset_restores_defaults(ctrl_4x):
    ctrl_4x.handle(dp.SET_LED_BRIGHTNESS, b'\x01')
    ctrl_4x.handle(dp.SET_PORT_ENABLED, b'\x00\x00')
    ctrl_4x.handle(dp.FACTORY_RESET, b'')
    assert ctrl_4x.led_brightness == 128
    assert ctrl_4x.ports[0].enabled


def test_unimplemented_command_is_ignored(ctrl_4x):
    """An unknown command gets silence, as on the real bus."""
    assert ctrl_4x.handle(0x77, b'') is None


# --------------------------------------------------------------------------- #
# The reason dispatch had to be destination-aware
# --------------------------------------------------------------------------- #
@pytest.mark.parametrize('cmd,expected_len', [
    (0x10, 1),      # GET_NUMBER_OF_PORTS -- collides with BAT GET_VOLTAGE
    (0x18, 9),      # GET_ENVIRONMENT     -- collides with BAT GET_SET_CURRENT
    (0xfe, 2),      # GET_VERSION         -- mount fw_version returns 4 bytes
])
def test_colliding_command_bytes_route_to_the_controller(scope_4x, cmd, expected_len):
    """
    _other_handlers is keyed on the command byte alone and shared across every
    non-motor target, so these three would otherwise be answered by the mount's
    own handlers with the wrong payload entirely.
    """
    reply = ask(scope_4x, cmd, b'')
    assert reply['cmd'] == cmd
    assert len(reply['data']) == expected_len


def test_mount_targets_still_answer_normally(scope_4x):
    """Adding 0x17 must not disturb the existing targets."""
    frames = parse_frames(scope_4x.handle_msg(build(0xfe, b'', src=APP, dst=0x10)))
    assert len(frames) == 2
    reply = frames[1]
    assert reply['src'] == 0x10
    assert len(reply['data']) == 4, 'motor controller version is 4 bytes'


def test_battery_target_still_answers_normally(scope_4x):
    """0x10 to the battery must still mean GET_VOLTAGE, not port count."""
    frames = parse_frames(scope_4x.handle_msg(build(0x10, b'', src=APP, dst=0xb6)))
    reply = frames[1]
    assert reply['src'] == 0xb6
    assert len(reply['data']) != 1, 'should be battery voltage, not a port count'


def test_bad_checksum_is_ignored(scope_4x):
    pkt = bytearray(build(dp.GET_VERSION))
    pkt[-1] ^= 0xFF
    assert scope_4x.handle_msg(bytes(pkt)) == b''
