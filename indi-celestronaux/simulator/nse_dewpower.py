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

VERIFIED AGAINST REAL HARDWARE (2026-11-08)
-------------------------------------------
Probed on Bob's Smart DewHeater Controller 2X (#94035, firmware 1.0) over the
controller's own USB CDC port at 19200 baud, no flow control. Confirmed:

  * 0x17 answers; GET_VERSION returns 2 bytes (01 00 = v1.0)
  * GET_NUMBER_OF_PORTS = 3 for the 2X
  * response sizes for GET_INPUT_POWER (6), GET_ENVIRONMENT (9),
    GET_POWER_ENABLED (1), GET_EXT_CURRENT_LIMIT (4), SELF_HEATER_STATUS (1),
    GET_LED_BRIGHTNESS (1), GET_DH_PORT_INFO (6)
  * the idle-port quirk is real: an idle port returns ONLY the 1-byte type,
    and the full 7-byte telemetry appears only while it draws current
  * an out-of-range port index is NAK'd
  * NAK (0xF0) replaces the requested command byte, carrying one data byte
    naming the rejected command
  * 10000 mA maximum current, matching the manual's 10 A / 120 W
  * the dew point the controller reports agrees with the Magnus-Tetens
    formula in dew_point_c() to 0.01 C

CORRECTED BY THAT PROBE
-----------------------
  * PORT TYPE CODES are not 0/1/2 -- see the long note below. The standalone
    driver's mapping is wrong, and on a 2X it drops the real power port.
  * GET_HEATER_LEVEL (0x1B) is NAK'd by firmware 1.0, not supported.
  * Defaults: LED brightness 0xFF, external current limit 8000 mA, both dew
    ports resting in auto-above-dew-point at 0% duty with per-port aggression
    levels (3 and 5, so it is genuinely per port and not a global setting).

STILL UNVERIFIED
----------------
  * Everything about the 4X: its port count, type codes, and how its
    variable-voltage port is identified over AUX. The '4x' profile is an
    extrapolation -- probe one before trusting it.
  * Whether the 4X's three USB ports are enumerated at all (omitted here).
  * All the SET_* commands. Only read commands have been exercised on
    hardware; nothing here has been confirmed to *change* anything.
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

# ---------------------------------------------------------------------------
# PORT TYPE CODES
#
# Measured on Bob's Smart DewHeater Controller 2X (#94035, firmware 1.0) on
# 2026-11-08, over the controller's own USB CDC port:
#
#   port 0 -> type 0x01     GET_DH_PORT_INFO answered
#   port 1 -> type 0x02     GET_DH_PORT_INFO answered
#   port 2 -> type 0x11     GET_DH_PORT_INFO NAK'd; the only port drawing
#                           current (489 mW at 12197 mV -- it powers the mount)
#
# This does NOT match the mapping in INDI core's standalone driver, which
# assumes 0 = DC, 1 = dew, 2 = variable and says in a comment that the mapping
# "needs to be clarified from the protocol if different". It is different. On
# this unit that driver reads port 1 as a variable-voltage port (the 2X has
# none -- that is a 4X feature) and silently drops port 2 entirely, because
# nothing classifies type 0x11. The dropped port is the one actually delivering
# power. Do not copy that mapping into our driver.
#
# HYPOTHESIS that fits every observation: the type byte is
#
#       (category << 4) | one-based index within that category
#
# so 0x01/0x02 are dew heater ports 1 and 2, and 0x11 is power output port 1.
# That gives the 2X exactly the 2 dew ports + 1 power output its manual
# describes, and explains why DH_PORT_INFO is accepted on 0x01/0x02 and
# refused on 0x11.
#
# Still UNVERIFIED: the 4X's codes. The hypothesis predicts dew 0x01-0x04 and
# power 0x11-0x14, but how the 4X distinguishes its variable-voltage port
# (power port #1 per the manual) is unknown -- it may be a third category, or
# a flag, or not distinguished at all over AUX. Hence `variable` is an explicit
# per-port flag in PORT_PROFILES below rather than being inferred from a code.
# ---------------------------------------------------------------------------
CAT_DEW   = 0x0
CAT_POWER = 0x1


def port_category(type_code):
    return type_code >> 4


def port_index(type_code):
    return type_code & 0x0F

# Byte 1 of GET_DH_PORT_INFO (signed). Measured on the 2X: it is a plain
# auto/manual flag, NOT the four-valued "nmode" the standalone driver
# documents. DH_ENABLE_MANUAL drives it to 0 and DH_ENABLE_AUTO to 1, and no
# write produced any other value. DH_SHORTED is retained as plausible but is
# UNVERIFIED -- we had nothing shorted to test with.
DH_SHORTED = -1
DH_MANUAL  = 0
DH_AUTO    = 1

# Kept so older code reads sensibly; both of these are "auto" on the wire. The
# distinction between auto-above-dew-point and auto-above-ambient is NOT
# visible in the 6-byte GET_DH_PORT_INFO response, and the third byte of
# DH_ENABLE_AUTO -- which the standalone driver believes carries the target
# temperature -- produced no observable change for any value tried (1, 3, 5,
# 7, 9, and a four-byte payload). Its meaning is unknown.
DH_AUTO_ABOVE_DEWPT   = 1
DH_AUTO_ABOVE_AMBIENT = 2

# Hardware limits from the manuals (both models): 13.8V max input, 120W / 10A.
# MAX_CURRENT_MA is confirmed: the 2X reports 0x2710 = 10000 mA as its maximum
# in GET_EXT_CURRENT_LIMIT, matching the manual's 10 A / 120 W ceiling.
MAX_INPUT_MV   = 13800
MAX_CURRENT_MA = 10000

# Per port: (type_code, nominal_mV, idle_load_mW, aggression_C, variable)
PORT_PROFILES = {
    # Smart DewHeater Controller 2X, #94035.
    # VERIFIED against Bob's unit (firmware 1.0) on 2026-11-08, including the
    # type codes, the per-port aggression levels, and which port draws current.
    '2x': [
        (0x01, 12000,   0, 3, False),   # dew heater 1 -- idle, aggression 3
        (0x02, 12000,   0, 5, False),   # dew heater 2 -- idle, aggression 5
        (0x11, 12197, 489, 0, False),   # 12V output -- powers the mount
    ],
    # Smart DewHeater and Power Controller 4X, #94036.
    # NOT VERIFIED. Port count (4 dew + 4 power) is from the manual; the type
    # codes are the hypothesis above extrapolated from the 2X, and the variable
    # flag on power port #1 is from the manual rather than from the bus. Probe
    # a real 4X with ./aux-probe.py before trusting any of this.
    '4x': [
        (0x01, 12000, 1800, 5, False),  # dew heater 1
        (0x02, 12000, 2400, 5, False),  # dew heater 2
        (0x03, 12000,    0, 5, False),  # dew heater 3 -- idle
        (0x04, 12000,    0, 5, False),  # dew heater 4 -- idle
        (0x11, 12000, 2500, 0, True),   # power #1 -- variable 3.0-11.0/12.0V
        (0x12, 12000, 6000, 0, False),  # power #2
        (0x13, 12000,    0, 0, False),  # power #3 -- idle
        (0x14, 12000,    0, 0, False),  # power #4 -- idle
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
    def __init__(self, type_code, voltage_mV, load_mW, aggression_C=5,
                 variable=False):
        self.type = type_code
        self.enabled = True
        self.shorted = False
        self.voltage_mV = voltage_mV
        self.load_mW = load_mW
        self.variable = variable
        # Dew-heater-only state. Both of the 2X's dew ports were found in auto
        # at 0% duty, which is the idle resting state.
        self.auto = self.is_dew
        self.power_level = 0          # 0..100 nominally; hardware does not clamp
        self.aggression_C = aggression_C
        self.heater_temp_mC = None    # only present when a thermistor is fitted
        self.auto_param = None        # DH_ENABLE_AUTO byte 2; meaning unknown

    @property
    def category(self):
        return port_category(self.type)

    @property
    def is_dew(self):
        return self.category == CAT_DEW

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

    #: 2 bytes, rendered major.minor. Bob's 2X reports 1.0.
    VERSION = (1, 0)

    def __init__(self, model='4x'):
        model = str(model).lower()
        if model not in PORT_PROFILES:
            raise ValueError('unknown dew/power controller model %r (want one of %s)'
                             % (model, ', '.join(sorted(PORT_PROFILES))))
        self.model = model
        self.ports = [Port(*spec) for spec in PORT_PROFILES[model]]
        self.power_enabled = True
        # Observed defaults on the real 2X: LED at full brightness, self heater
        # off, external current limit 8000 mA against a 10000 mA maximum.
        self.led_brightness = 0xFF
        self.self_heater = False
        self.ext_current_limit_mA = 8000
        self.fuse_blown = False
        self.log = []

        # Environment sensor. Seeded near what the real 2X reported indoors
        # (22.96 C / 51% RH) and drifted in tick() so repeated reads differ.
        self.ambient_C = 23.0
        self.humidity = 51.0
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
        self.ambient_C = 23.0 + 2.0 * math.sin(self._t / 180.0)
        self.humidity = 51.0 + 8.0 * math.sin(self._t / 240.0)

        dp = dew_point_c(self.ambient_C, self.humidity)
        for p in self.ports:
            if not p.is_dew or not p.enabled or not p.auto or p.shorted:
                continue
            # Hold the optic this far above the dew point. Whether the real
            # controller can instead track ambient is not observable over AUX,
            # so only the dew-point form is modelled.
            target = dp + p.aggression_C
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
        voltage_mV = 12197          # what Bob's unit reported on a 12V supply
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
        <0 type><1 auto flag (signed)><2 duty><3:4 power mW><5 aggression C>
        and, when a thermistor is fitted, <6:9 heater temperature mC>.

        Byte 1 and byte 5 are as MEASURED on the 2X, and differ from the
        standalone driver's reading: byte 1 is a plain auto(1)/manual(0) flag,
        and byte 5 is the aggression in C, which DH_ENABLE_AUTO sets from its
        *second* byte. Verified by writing aggressions 3, 5 and 9 and reading
        each back unchanged.

        NAK'd on a non-dew port, as observed.
        """
        p = self._port(data)
        if p is None or not p.is_dew:
            return _NAK
        flag = DH_SHORTED if p.shorted else (DH_AUTO if p.auto else DH_MANUAL)
        out = (bytes((p.type,)) + struct.pack('!b', flag)
               + bytes((p.power_level & 0xFF,))
               + struct.pack('!H', p.power_mW())
               + bytes((p.aggression_C & 0xFF,)))
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
        if not p.variable:
            return _NAK
        mV = struct.unpack('!H', data[1:3])[0]
        # Manual: 3.0-11.0V in steps, or 12.0V.
        if not (3000 <= mV <= 11000 or mV == 12000):
            return _NAK
        p.voltage_mV = mV
        return b''

    def _dh_enable_auto(self, data):
        """
        <0 port><1 aggression C><2 unknown>

        MEASURED: byte 1 is the aggression, not a mode -- writing 3/5/9 reads
        straight back in byte 5 of GET_DH_PORT_INFO. Byte 2 is accepted but had
        no observable effect for any value tried, so it is stored and ignored
        rather than validated. The standalone driver sends (port, mode, temp_C)
        here, which would write its *mode* into the aggression field.

        The real controller does not validate: every aggression value offered
        was accepted. So no range check here either.
        """
        p = self._port(data)
        if p is None or not p.is_dew or len(data) < 2:
            return _NAK
        p.auto = True
        p.aggression_C = data[1]
        if len(data) >= 3:
            p.auto_param = data[2]      # meaning unknown; recorded for later
        return b''

    def _dh_enable_manual(self, data):
        """
        <0 port><1 duty cycle>

        MEASURED: the controller does NOT validate the duty cycle. Sending 150
        stored 150 verbatim (read back as 0x96), so the documented 0-100 range
        is a convention, not something the firmware enforces. Reproduced
        faithfully -- our driver has to clamp before sending, and this is what
        makes a test for that meaningful.
        """
        p = self._port(data)
        if p is None or not p.is_dew or len(data) < 2:
            return _NAK
        p.auto = False
        p.power_level = data[1]
        return b''

    def _get_environment(self, data):
        """<0:3 ambient mC><4:7 dew point mC><8 humidity %>"""
        ambient_mC = int(round(self.ambient_C * 1000))
        dewpoint_mC = int(round(dew_point_c(self.ambient_C, self.humidity) * 1000))
        return (struct.pack('!i', ambient_mC) + struct.pack('!i', dewpoint_mC)
                + bytes((int(round(self.humidity)),)))

    def _enable_self_heater(self, data):
        """
        MEASURED: this is effectively one-way on firmware 1.0. Every payload
        offered (empty, 0x00, 0xFF, 0x00 repeated) was ACKed, but once the
        self heater had been switched on SELF_HEATER_STATUS stayed 0x01 and
        would not return to 0x00. So the sensor heater latches on, presumably
        until a power cycle or until the firmware's own humidity logic releases
        it. Reproduced so the driver cannot ship a working "off" switch that
        does nothing on real hardware -- expose it as one-way, or not at all.
        """
        self.self_heater = True
        return b''

    def _self_heater_status(self, data):
        return bytes((1 if self.self_heater else 0,))

    def _get_heater_level(self, data):
        """
        Not supported by the firmware we have seen.

        The real 2X (firmware 1.0) NAKs 0x1B outright, so the aggregate heater
        level has to be derived from the per-port duty cycles in
        GET_DH_PORT_INFO instead. If a 4X with newer firmware does answer it,
        implement it then -- do not guess at a payload we have never observed.
        """
        return _NAK

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
