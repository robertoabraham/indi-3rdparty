/*
    Celestron Aux Command

    Copyright (C) 2020 Paweł T. Jochym
    Copyright (C) 2020 Fabrizio Pollastri
    Copyright (C) 2021 Jasem Mutlaq

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA

*/

#pragma once

#include <vector>
#include <stddef.h>
#include <stdint.h>

typedef std::vector<uint8_t> AUXBuffer;

enum AUXCommands
{
    MC_GET_POSITION      = 0x01,
    MC_GOTO_FAST         = 0x02,
    MC_SET_POSITION      = 0x04,
    MC_GET_MODEL         = 0x05,
    MC_SET_POS_GUIDERATE = 0x06,
    MC_SET_NEG_GUIDERATE = 0x07,
    MC_LEVEL_START       = 0x0b,
    MC_LEVEL_DONE        = 0x12,
    MC_SLEW_DONE         = 0x13,
    MC_GOTO_SLOW         = 0x17,
    MC_SEEK_DONE         = 0x18,
    MC_SEEK_INDEX        = 0x19,
    MC_MOVE_POS          = 0x24,
    MC_MOVE_NEG          = 0x25,
    MC_AUX_GUIDE         = 0x26,
    MC_AUX_GUIDE_ACTIVE  = 0x27,
    MC_ENABLE_CORDWRAP   = 0x38,
    MC_DISABLE_CORDWRAP  = 0x39,
    MC_SET_CORDWRAP_POS  = 0x3a,
    MC_POLL_CORDWRAP     = 0x3b,
    MC_GET_CORDWRAP_POS  = 0x3c,
    MC_SET_AUTOGUIDE_RATE = 0x46,
    MC_GET_AUTOGUIDE_RATE = 0x47,
    GET_VER              = 0xfe,
    GPS_GET_LAT          = 0x01,
    GPS_GET_LONG         = 0x02,
    GPS_GET_DATE         = 0x03,
    GPS_GET_YEAR         = 0x04,
    GPS_GET_TIME         = 0x33,
    GPS_TIME_VALID       = 0x36,
    GPS_LINKED           = 0x37,
    FOC_GET_HS_POSITIONS = 0x2c,
    // Evolution commands
    GET_VOLTAGE          = 0x10,

    // Smart DewHeater & Power Controller (2X #94035 / 4X #94036), target
    // DEW_POWER_CTRL. Command bytes overlap the motor-controller and GPS sets
    // above, which is fine -- AUX commands are only meaningful relative to a
    // destination, and commandName()/responseDataSize() already branch on it.
    PORTCTRL_GET_INPUT_POWER       = 0x00,
    PORTCTRL_SET_POWER_ENABLED     = 0x01, // master switch; avoid
    PORTCTRL_GET_POWER_ENABLED     = 0x02,
    PORTCTRL_SET_EXT_CURRENT_LIMIT = 0x03,
    PORTCTRL_GET_EXT_CURRENT_LIMIT = 0x04,
    PORTCTRL_GET_NUMBER_OF_PORTS   = 0x10,
    PORTCTRL_GET_PORT_INFO         = 0x11,
    PORTCTRL_GET_DH_PORT_INFO      = 0x12,
    PORTCTRL_RESET_FUSE            = 0x13,
    PORTCTRL_SET_PORT_ENABLED      = 0x14,
    PORTCTRL_SET_PORT_VOLTAGE      = 0x15,
    PORTCTRL_DH_ENABLE_AUTO        = 0x16,
    PORTCTRL_DH_ENABLE_MANUAL      = 0x17,
    PORTCTRL_GET_ENVIRONMENT       = 0x18,
    PORTCTRL_ENABLE_SELF_HEATER    = 0x19,
    PORTCTRL_SELF_HEATER_STATUS    = 0x1a,
    PORTCTRL_GET_HEATER_LEVEL      = 0x1b, // NAK'd by 2X firmware 1.0
    PORTCTRL_SET_LED_BRIGHTNESS    = 0x20,
    PORTCTRL_GET_LED_BRIGHTNESS    = 0x21,
    PORTCTRL_FACTORY_RESET         = 0xaa, // destroys saved config; avoid
    PORTCTRL_NAK                   = 0xf0
};

// Port type byte returned by PORTCTRL_GET_PORT_INFO / GET_DH_PORT_INFO.
//
// Measured on a Smart DewHeater Controller 2X (firmware 1.0) on 2026-11-08:
// its three ports report 0x01, 0x02 and 0x11, and GET_DH_PORT_INFO is answered
// for the first two but NAK'd for the third -- which is also the only one
// drawing current, i.e. the 12V output. That fits the 2X's documented layout of
// two dew heater ports plus one power output if the byte is
//
//     (category << 4) | one-based index within that category
//
// Note this is NOT the 0 = DC / 1 = dew / 2 = variable mapping assumed by
// INDI core's standalone indi_celestron_dewpower driver, whose own comment
// concedes the mapping was unconfirmed. Under that reading a 2X appears to
// have a variable-voltage port it does not have, and its real power output is
// dropped because nothing classifies 0x11.
//
// CONFIRMED on a 4X (2026-11-08), which reports 11 ports:
//
//     0x01-0x04  four dew heaters
//     0x41       variable voltage -- the manual's "power port #1"
//     0x12-0x14  three fixed 12V outputs, indexed 2..4 precisely because
//                port #1 is the variable one and has its own category
//     0x31-0x33  three USB ports, reporting 5000 mV
//
// SET_PORT_VOLTAGE is accepted only on 0x4x, and NAK'd on 0x1x and 0x3x,
// which is what pins the variable port down. The USB ports' 5000 mV is what
// identifies them. Category 0x2 has never been seen.
#define PORTCTRL_CATEGORY(type) ((type) >> 4)
#define PORTCTRL_INDEX(type)    ((type) & 0x0f)

enum PortCtrlCategory
{
    PORTCTRL_CAT_DEW      = 0x0,   // 2X: 0x01-0x02   4X: 0x01-0x04
    PORTCTRL_CAT_POWER    = 0x1,   // fixed 12V output
    PORTCTRL_CAT_USB      = 0x3,   // 4X only: 0x31-0x33, reporting 5000 mV
    PORTCTRL_CAT_VARIABLE = 0x4    // 4X only: 0x41, 3.0-11.0 V or 12.0 V
};

// Byte 1 of PORTCTRL_GET_DH_PORT_INFO, signed. Measured as a plain
// auto/manual flag, not the four-valued mode the standalone driver documents.
enum PortCtrlDewMode
{
    PORTCTRL_DH_SHORTED = -1, // plausible, unverified -- nothing to test with
    PORTCTRL_DH_MANUAL  = 0,
    PORTCTRL_DH_AUTO    = 1
};

enum AUXTargets
{
    ANY   = 0x00,
    MB    = 0x01,
    HC    = 0x04,
    HCP   = 0x0d,
    AZM   = 0x10,
    ALT   = 0x11,
    FOCUS = 0x12,
    // Smart DewHeater & Power Controller. Confirmed on real hardware
    // 2026-11-08; not in the older documented AUX target list, which stops at
    // LIGHT = 0xbf.
    DEW_POWER_CTRL = 0x17,
    APP   = 0x20,
    GPS   = 0xb0,
    WiFi  = 0xb5,
    BAT   = 0xb6,
    CHG   = 0xb7,
    LIGHT = 0xbf
};

#define CAUX_DEFAULT_IP   "1.2.3.4"
#define CAUX_DEFAULT_PORT 2000

void logBytes(unsigned char *buf, int n, const char *deviceName, uint32_t debugLevel);

class AUXCommand
{
    public:
        AUXCommand();
        AUXCommand(const AUXBuffer &buf);
        AUXCommand(AUXCommands command, AUXTargets source, AUXTargets destination, const AUXBuffer &data);
        AUXCommand(AUXCommands command, AUXTargets source, AUXTargets destination);

        ///////////////////////////////////////////////////////////////////////////////
        /// Buffer Management
        ///////////////////////////////////////////////////////////////////////////////
        void fillBuf(AUXBuffer &buf);
        void parseBuf(AUXBuffer buf);
        void parseBuf(AUXBuffer buf, bool do_checksum);

        ///////////////////////////////////////////////////////////////////////////////
        /// Getters
        ///////////////////////////////////////////////////////////////////////////////
        const AUXTargets &source() const
        {
            return m_Source;
        }
        const AUXTargets &destination() const
        {
            return m_Destination;
        }
        const AUXBuffer &data() const
        {
            return m_Data;
        }
        AUXCommands command() const
        {
            return m_Command;
        }
        size_t dataSize() const
        {
            return m_Data.size();
        }
        const char * commandName() const
        {
            return commandName(m_Command);
        }

        ///////////////////////////////////////////////////////////////////////////////
        /// Set and Get data
        ///////////////////////////////////////////////////////////////////////////////
        /**
         * @brief getData Parses data packet and convert it to a 32bit unsigned integer
         * @param bytes How many bytes to interpret the data.
         * @return
         */
        uint32_t getData();
        void setData(uint32_t value, uint8_t bytes = 3);

        ///////////////////////////////////////////////////////////////////////////////
        /// Check sum
        ///////////////////////////////////////////////////////////////////////////////
        uint8_t checksum(AUXBuffer buf);

        ///////////////////////////////////////////////////////////////////////////////
        /// Logging
        ///////////////////////////////////////////////////////////////////////////////
        const char * commandName(AUXCommands command) const;
        const char * moduleName(AUXTargets n);
        int responseDataSize();
        void logResponse();
        void logCommand();
        static void setDebugInfo(const char *deviceName, uint8_t debugLevel);

        static uint8_t DEBUG_LEVEL;
        static char DEVICE_NAME[64];

    private:
        uint8_t len {0};
        bool valid {false};

        AUXCommands m_Command;
        AUXTargets m_Source, m_Destination;
        AUXBuffer m_Data;



};
