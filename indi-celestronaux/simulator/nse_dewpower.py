#!/bin/env python3
"""
Emulation of the Celestron Smart DewHeater & Power Controller (2X / 4X) as an
AUX bus device, for use with nse_simulator.py.

The real controllers answer on AUX destination address 0x17. The mount
forwards AUX packets addressed to accessories through its PC/HC/USB port, which
is how the Celestron focuser (0x12) is reached by the indi-celestronaux driver,
and how CPWI reaches the dew/power controller. This module lets us develop and
test that path without the hardware powered up.

PROVENANCE OF THE PROTOCOL
--------------------------
The command set and payload layouts here are taken from INDI core's standalone
driver, drivers/power/indi-celestron-dewpower/ (Jasem Mutlaq), whose comments
record them as verified against real hardware on 2026-08-20 -- a 3-port unit
attached to a CGX. Byte orders are big-endian throughout, matching that
driver's AUXCommand::getData()/setData().

WHAT IS CONFIRMED vs ASSUMED
----------------------------
Confirmed by that driver (and so reproduced faithfully here):
  * target address 0x17, the command bytes, and every response size
  * port types are classified 0 = DC output, 1 = dew heater, 2 = variable
  * GET_PORT_INFO on an *idle* port returns only the 1-byte port type; the
    full 7-byte telemetry appears only while the port is drawing current.
    This is reproduced deliberately -- it is a real quirk that previously
    caused idle ports to be misclassified as DC.
  * NAK (0xF0) is returned *in place of* the requested command byte, with one
    data byte naming the rejected command, when a command does not apply to
    the addressed port.

ASSUMED, and flagged for confirmation against Bob's hardware:
  * The ORDER of ports within the single flat index space. The manuals number
    the 4X's *power* ports 1..4 (#1 variable, #2-4 fixed 12V) and its dew
    ports 1..4 separately, but they say nothing about how the two groups are
    interleaved in the AUX port index space. PORT_PROFILES below puts dew
    ports first; if hardware disagrees, that table is the only thing to change.
  * Whether the 4X's three USB ports appear in the index space at all. They
    are omitted here -- the standalone driver counts USB ports but never
    classifies any port as USB, which suggests they are not enumerated.
  * GET_VERSION's 2 bytes are rendered as major.minor.

Port counts come from the manuals in docs/:
  2X (#94035): 2 dew heater ports + 1 fixed 12V power output        -> 3 ports
  4X (#94036): 4 dew heater ports + 1 variable + 3 fixed 12V output -> 8 ports
The 2X's 3 ports happen to match the "3-port unit" the protocol was verified
against, which is a reassuring consistency check.
"""

import math
import os
import struct

# AUX destination address of the controller.
DEW_POWER_CTRL = 0x17

# PORTCTRL_* command bytes.
GET_INPUT_POWER       = 0x00
SET_POWER_ENABLED     = 0x01   # not accepted unless connected via USB
GET_POWER_ENABLED     = 0x02
SET_EXT_CURRENT_LIMIT = 0x03
GET_EXT_CURRENT_LIMIT = 0x04
GET_NUMBER_OF_PORTS   = 0x10
GET_PORT_INFO         = 0x11
GET_DH_PORT_INFO      = 0x12
RESET_FUSE            = 0x13
SET_PORT_ENABLED      = 0x14
SET_PORT_VOLTAGE      = 0x15
DH_ENABLE_AUTO        = 0x16
DH_ENABLE_MANUAL      = 0x17
GET_ENVIRONMENT       = 0x18
ENABLE_SELF_HEATER    = 0x19
SELF_HEATER_STATUS    = 0x1A
GET_HEATER_LEVEL      = 0x1B
SET_LED_BRIGHTNESS    = 0x20
GET_LED_BRIGHTNESS    = 0x21
FACTORY_RESET         = 0xAA
GET_VERSION           = 0xFE
NAK                   = 0xF0

cmd_names = {
    GET_INPUT_POWER:       'GET_INPUT_POWER',
    SET_POWER_ENABLED:     'SET_POWER_ENABLED',
    GET_POWER_ENABLED:     'GET_POWER_ENABLED',
    SET_EXT_CURRENT_LIMIT: 'SET_EXT_CURRENT_LIMIT',
    GET_EXT_CURRENT_LIMIT: 'GET_EXT_CURRENT_LIMIT',
    GET_NUMBER_OF_PORTS:   'GET_NUMBER_OF_PORTS',
    GET_PORT_INFO:         'GET_PORT_INFO',
    GET_DH_PORT_INFO:      'GET_DH_PORT_INFO',
    RESET_FUSE:            'RESET_FUSE',
    SET_PORT_ENABLED:      'SET_PORT_ENABLED',
    SET_PORT_VOLTAGE:      'SET_PORT_VOLTAGE',
    DH_ENABLE_AUTO:        'DH_ENABLE_AUTO',
    DH_ENABLE_MANUAL:      'DH_ENABLE_MANUAL',
    GET_ENVIRONMENT:       'GET_ENVIRONMENT',
    ENABLE_SELF_HEATER:    'ENABLE_SELF_HEATER',
    SELF_HEATER_STATUS:    'SELF_HEATER_STATUS',
    GET_HEATER_LEVEL:      'GET_HEATER_LEVEL',
    SET_LED_BRIGHTNESS:    'SET_LED_BRIGHTNESS',
    GET_LED_BRIGHTNESS:    'GET_LED_BRIGHTNESS',
    FACTORY_RESET:         'FACTORY_RESET',
    GET_VERSION:           'GET_VERSION',
    NAK:                   'NAK',
}

# Port types, as classified by the standalone driver.
PORT_DC       = 0
PORT_DEW      = 1
PORT_VARIABLE = 2

# Dew heater modes, as reported in byte 1 of GET_DH_PORT_INFO (signed).
DH_SHORTED            = -1
DH_MANUAL             = 0
DH_AUTO_ABOVE_DEWPT   = 1
DH_AUTO_ABOVE_AMBIENT = 2

# Hardware limits from the manuals (both models): 13.8V max input, 120W / 10A.
MAX_INPUT_MV   = 13800
MAX_CURRENT_MA = 10000

# (type, nominal_mV, idle_load_mW) per port. See the ASSUMED note above about
# ordering; swapping these rows is all it takes to match real hardware.
PORT_PROFILES = {
    # Smart DewHeater Controller 2X, #94035.
    '2x': [
        (PORT_DEW,       12000, 1800),   # dew heater 1 -- a ring, drawing
        (PORT_DEW,       12000,    0),   # dew heater 2 -- nothing plugged in
        (PORT_DC,        12000, 6000),   # 12V output -- e.g. the mount
    ],
    # Smart DewHeater and Power Controller 4X, #94036.
    '4x': [
        (PORT_DEW,       12000, 1800),   # dew heater 1
        (PORT_DEW,       12000, 2400),   # dew heater 2
        (PORT_DEW,       12000,    0),   # dew heater 3 -- idle
        (PORT_DEW,       12000,    0),   # dew heater 4 -- idle
        (PORT_VARIABLE,  12000, 2500),   # power port #1, variable 3.0-11.0/12.0V
        (PORT_DC,        12000, 6000),   # power port #2
        (PORT_DC,        12000,    0),   # power port #3 -- idle
        (PORT_DC,        12000,    0),   # power port #4 -- idle
    ],
}


def describe(cmd, data):
    """One-line log string, in the style of nse_telescope's command log."""
    return 'DEW[%s] %s' % (cmd_names.get(cmd, '%02x' % cmd),
                           ''.join('%02x' % b for b in data))


def dew_point_c(temp_c, humidity_pct):
    """Magnus-Tetens dew point, so auto-dew behaviour is physically sensible."""
    humidity_pct = min(max(humidity_pct, 1.0), 100.0)
    a, b = 17.27, 237.7
    gamma = (a * temp_c) / (b + temp_c) + math.log(humidity_pct / 100.0)
    return (b * gamma) / (a - gamma)


class Port:
    def __init__(self, ptype, voltage_mV, load_mW):
        self.type = ptype
        self.enabled = True
        self.shorted = False
        self.voltage_mV = voltage_mV
        self.load_mW = load_mW
        # Dew-heater-only state.
        self.mode = DH_AUTO_ABOVE_DEWPT if ptype == PORT_DEW else DH_MANUAL
        self.power_level = 0          # 0..100 %
        self.aggression_C = 5         # degrees above the target to hold
        self.heater_temp_mC = None    # only present when a thermistor is fitted

    @property
    def is_dew(self):
        return self.type == PORT_DEW

    def power_mW(self):
        """Instantaneous draw. Zero unless enabled and something is plugged in."""
        if not self.enabled or self.shorted:
            return 0
        if self.is_dew:
            # A dew heater's draw scales with its duty cycle.
            return int(self.load_mW * self.power_level / 100.0)
        return self.load_mW

    def is_active(self):
        """Whether the controller will report full telemetry for this port."""
        return self.power_mW() > 0


class DewPowerController:
    """
    Emulates one Smart DewHeater/Power Controller on the AUX bus.

    handle() takes a command byte plus its data payload and returns
    (response_command_byte, response_payload) -- the command byte is returned
    separately because a NAK replies under 0xF0 rather than the requested
    command. Returns None for commands this device does not implement, which
    the caller should treat the way the bus does: silence.
    """

    #: 2 bytes, rendered major.minor
    VERSION = (1, 7)

    def __init__(self, model='4x'):
        model = str(model).lower()
        if model not in PORT_PROFILES:
            raise ValueError('unknown dew/power controller model %r (want one of %s)'
                             % (model, ', '.join(sorted(PORT_PROFILES))))
        self.model = model
        self.ports = [Port(t, v, l) for (t, v, l) in PORT_PROFILES[model]]
        self.power_enabled = True
        self.led_brightness = 128
        self.self_heater = False
        self.ext_current_limit_mA = 5000
        self.fuse_blown = False
        self.log = []

        # Environment sensor. Starts at a plausible damp-evening state and
        # drifts in tick() so repeated reads are not identical.
        self.ambient_C = 8.0
        self.humidity = 72.0
        self._t = 0.0

        self.cmd_handlers = {
            GET_INPUT_POWER:       self._get_input_power,
            SET_POWER_ENABLED:     self._set_power_enabled,
            GET_POWER_ENABLED:     self._get_power_enabled,
            SET_EXT_CURRENT_LIMIT: self._set_ext_current_limit,
            GET_EXT_CURRENT_LIMIT: self._get_ext_current_limit,
            GET_NUMBER_OF_PORTS:   self._get_number_of_ports,
            GET_PORT_INFO:         self._get_port_info,
            GET_DH_PORT_INFO:      self._get_dh_port_info,
            RESET_FUSE:            self._reset_fuse,
            SET_PORT_ENABLED:      self._set_port_enabled,
            SET_PORT_VOLTAGE:      self._set_port_voltage,
            DH_ENABLE_AUTO:        self._dh_enable_auto,
            DH_ENABLE_MANUAL:      self._dh_enable_manual,
            GET_ENVIRONMENT:       self._get_environment,
            ENABLE_SELF_HEATER:    self._enable_self_heater,
            SELF_HEATER_STATUS:    self._self_heater_status,
            GET_HEATER_LEVEL:      self._get_heater_level,
            SET_LED_BRIGHTNESS:    self._set_led_brightness,
            GET_LED_BRIGHTNESS:    self._get_led_brightness,
            FACTORY_RESET:         self._factory_reset,
            GET_VERSION:           self._get_version,
        }

    # ------------------------------------------------------------------ #
    # Bus interface
    # ------------------------------------------------------------------ #
    def handle(self, cmd, data):
        handler = self.cmd_handlers.get(cmd)
        if handler is None:
            return None
        result = handler(bytes(data))
        if result is _NAK:
            return (NAK, bytes((cmd,)))
        return (cmd, result)

    def describe(self, cmd, data):
        """One-line log string, in the style of nse_telescope's command log."""
        return describe(cmd, data)

    # ------------------------------------------------------------------ #
    # Simulation
    # ------------------------------------------------------------------ #
    def tick(self, interval):
        """Drift the environment and run the auto-dew control loop."""
        self._t += interval
        # Slow, bounded wander -- a couple of degrees over several minutes.
        self.ambient_C = 8.0 + 2.0 * math.sin(self._t / 180.0)
        self.humidity = 72.0 + 8.0 * math.sin(self._t / 240.0)

        dp = dew_point_c(self.ambient_C, self.humidity)
        for p in self.ports:
            if not p.is_dew or not p.enabled:
                continue
            if p.mode == DH_MANUAL or p.mode == DH_SHORTED:
                continue
            target = (dp if p.mode == DH_AUTO_ABOVE_DEWPT else self.ambient_C) + p.aggression_C
            # Lens temperature trails ambient; heat it toward the target.
            lens = p.heater_temp_mC / 1000.0 if p.heater_temp_mC is not None else self.ambient_C
            deficit = target - lens
            # Proportional, 10% of duty per degree of deficit.
            p.power_level = int(min(max(deficit * 10.0, 0), 100))

    def total_current_mA(self):
        total_mW = sum(p.power_mW() for p in self.ports)
        return int(total_mW / 12.0)  # at the 12V rail

    # ------------------------------------------------------------------ #
    # Command implementations
    # ------------------------------------------------------------------ #
    def _get_version(self, data):
        return bytes(self.VERSION)

    def _get_number_of_ports(self, data):
        return bytes((len(self.ports),))

    def _get_input_power(self, data):
        """<0:1 voltage mV><2:3 current mA><4 voltage status><5 overcurrent>"""
        voltage_mV = 12400
        current_mA = self.total_current_mA()
        if voltage_mV > MAX_INPUT_MV:
            v_status = 1            # over voltage
        elif voltage_mV < 11000:
            v_status = -1           # under voltage
        else:
            v_status = 0
        overcurrent = 1 if (current_mA > MAX_CURRENT_MA or self.fuse_blown) else 0
        return (struct.pack('!HH', voltage_mV, current_mA)
                + struct.pack('!bB', v_status, overcurrent))

    def _set_power_enabled(self, data):
        if len(data) >= 1:
            self.power_enabled = bool(data[0])
        return b''

    def _get_power_enabled(self, data):
        return bytes((1 if self.power_enabled else 0,))

    def _set_ext_current_limit(self, data):
        if len(data) >= 2:
            self.ext_current_limit_mA = struct.unpack('!H', data[:2])[0]
        return b''

    def _get_ext_current_limit(self, data):
        """<0:1 limit mA><2:3 max limit mA>"""
        return struct.pack('!HH', self.ext_current_limit_mA, MAX_CURRENT_MA)

    def _port(self, data):
        """Resolve the 1-byte port index argument, or None if out of range."""
        if len(data) < 1 or data[0] >= len(self.ports):
            return None
        return self.ports[data[0]]

    def _get_port_info(self, data):
        """
        <0 type><1 enabled><2 isShorted><3:4 power mW><5:6 voltage mV>

        An idle port returns only byte 0. Reproducing that is the point --
        see the module docstring.
        """
        p = self._port(data)
        if p is None:
            return _NAK
        if not p.is_active():
            return bytes((p.type,))
        return (bytes((p.type, 1 if p.enabled else 0, 1 if p.shorted else 0))
                + struct.pack('!HH', p.power_mW(), p.voltage_mV))

    def _get_dh_port_info(self, data):
        """
        <0 type><1 mode (signed)><2 power level><3:4 power mW><5 aggression C>
        and, when a thermistor is fitted, <6:9 heater temperature mC>.

        NAK'd on a non-dew port, which is the behaviour the standalone driver
        records observing on real hardware.
        """
        p = self._port(data)
        if p is None or not p.is_dew:
            return _NAK
        mode = DH_SHORTED if p.shorted else p.mode
        out = (bytes((p.type,)) + struct.pack('!b', mode)
               + bytes((p.power_level,))
               + struct.pack('!H', p.power_mW())
               + bytes((p.aggression_C,)))
        if p.heater_temp_mC is not None:
            out += struct.pack('!i', p.heater_temp_mC)
        return out

    def _reset_fuse(self, data):
        self.fuse_blown = False
        for p in self.ports:
            p.shorted = False
        return b''

    def _set_port_enabled(self, data):
        """<0 port><1 enabled>"""
        p = self._port(data)
        if p is None or len(data) < 2:
            return _NAK
        p.enabled = bool(data[1])
        return b''

    def _set_port_voltage(self, data):
        """<0 port><1:2 voltage mV>. Only the variable port accepts this."""
        p = self._port(data)
        if p is None or len(data) < 3:
            return _NAK
        if p.type != PORT_VARIABLE:
            return _NAK
        mV = struct.unpack('!H', data[1:3])[0]
        # Manual: 3.0-11.0V in steps, or 12.0V.
        if not (3000 <= mV <= 11000 or mV == 12000):
            return _NAK
        p.voltage_mV = mV
        return b''

    def _dh_enable_auto(self, data):
        """<0 port><1 mode><2 aggression C>"""
        p = self._port(data)
        if p is None or not p.is_dew or len(data) < 3:
            return _NAK
        mode = data[1]
        if mode not in (DH_AUTO_ABOVE_DEWPT, DH_AUTO_ABOVE_AMBIENT):
            return _NAK
        p.mode = mode
        p.aggression_C = data[2]
        return b''

    def _dh_enable_manual(self, data):
        """<0 port><1 power level 0-100>"""
        p = self._port(data)
        if p is None or not p.is_dew or len(data) < 2:
            return _NAK
        if data[1] > 100:
            return _NAK
        p.mode = DH_MANUAL
        p.power_level = data[1]
        return b''

    def _get_environment(self, data):
        """<0:3 ambient mC><4:7 dew point mC><8 humidity %>"""
        ambient_mC = int(round(self.ambient_C * 1000))
        dewpoint_mC = int(round(dew_point_c(self.ambient_C, self.humidity) * 1000))
        return (struct.pack('!i', ambient_mC) + struct.pack('!i', dewpoint_mC)
                + bytes((int(round(self.humidity)),)))

    def _enable_self_heater(self, data):
        if len(data) >= 1:
            self.self_heater = bool(data[0])
        return b''

    def _self_heater_status(self, data):
        return bytes((1 if self.self_heater else 0,))

    def _get_heater_level(self, data):
        """Aggregate heater level, 0-255."""
        dew = [p for p in self.ports if p.is_dew]
        if not dew:
            return bytes((0,))
        avg = sum(p.power_level for p in dew) / float(len(dew))
        return bytes((int(min(avg * 255 / 100.0, 255)),))

    def _set_led_brightness(self, data):
        if len(data) >= 1:
            self.led_brightness = data[0]
        return b''

    def _get_led_brightness(self, data):
        return bytes((self.led_brightness,))

    def _factory_reset(self, data):
        self.__init__(self.model)
        return b''


class _Nak:
    """Sentinel: reply with NAK (0xF0) naming the rejected command."""
    def __repr__(self):
        return '<NAK>'


_NAK = _Nak()


def from_env(default='4x'):
    """
    Build a controller from $NSE_DEW_MODEL ('2x', '4x', or 'none').

    'none' returns None, which makes the simulator behave like a mount with no
    dew/power controller attached -- the case where the driver must degrade
    cleanly rather than misbehave.
    """
    model = os.environ.get('NSE_DEW_MODEL', default).lower()
    if model in ('none', 'off', ''):
        return None
    return DewPowerController(model)
