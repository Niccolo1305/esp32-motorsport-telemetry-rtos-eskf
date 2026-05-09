// SPDX-License-Identifier: GPL-3.0-or-later
// SerialGpsWrapper.h - Concrete IGpsProvider backed by HardwareSerial + TinyGPSPlus
//                    + NMEA DHV parser (1 Hz diagnostics on the tested AT6668 module)
//                    + CASIC NAV2-PVH binary diagnostics listener
//
// Encapsulates all GPS hardware concerns:
//   - HardwareSerial (UART1, RX/TX pins, baud rate, RX buffer size)
//   - TinyGPSPlus sentence decoding (position, NMEA SOG, sats, hdop)
//   - NMEA DHV parser (gdspd, dhv_fix_us)
//   - CASIC NAV2-PVH binary parser (speed2D, sAcc, velN/E, velValid)
//   - UART overflow ISR callback (atomic counter, captured by reference)
//   - AT6668 chip-startup delay + CASIC $PCAS02 rate command (10 Hz positioning)
//   - Explicit $PCAS03 sentence selection (GGA/RMC/DHV only)
//   - CFG-PRT + CFG-MSG NAV2-PVH diagnostic enable sequence
//
// CASIC Binary Protocol (CASBIN):
//   Frame: [0xBA][0xCE][len_lo][len_hi][class][id][payload x len][cksum x 4]
//   Checksum (32-bit, LE): ckSum = (id<<24)|(class<<16)|len;
//                          for each uint32 word in payload: ckSum += word
//
// NAV2-PVH payload layout (class=0x11, id=0x03, 88 bytes, all little-endian):
//   Offset  Bytes  Type  Name      Description
//   0       4      I4    tow       GPS time of week [ms]
//   8       1      U1    fixflags  Position validity flag
//   9       1      U1    velflags  Velocity validity flag, PVT validity scale
//   24      8      R8    lon       Longitude [deg]
//   32      8      R8    lat       Latitude [deg]
//   48      4      R4    velE      East velocity [m/s]
//   52      4      R4    velN      North velocity [m/s]
//   56      4      R4    velU      Up velocity [m/s]
//   60      4      R4    speed3D   3D speed [m/s]
//   64      4      R4    speed2D   2D ground speed [m/s]
//   68      4      R4    heading   Course over ground [deg]
//   72      4      R4    hAcc      Horizontal accuracy estimate [m]
//   76      4      R4    vAcc      Vertical accuracy estimate [m]
//   80      4      R4    sAcc      Speed accuracy estimate [m/s]
//   84      4      R4    cAcc      Course accuracy estimate [deg]
//
// Both NMEA and binary coexist on the same UART. TinyGPSPlus safely ignores
// 0xBA/0xCE bytes. The CASIC parser runs in parallel on every byte.
//
// Contract with Task_GPS:
//   update() returns true when a new NMEA fix, a new DHV sentence, a NAV2-PVH
//   frame, or a config ACK/NACK was parsed in this poll cycle. epoch is
//   incremented only on the NMEA fix path, so velocity-only updates refresh
//   shared_gps_data without triggering an extra GPS correction in Task_Filter.
//
// Lifetime: MUST be a static or global object. The onReceiveError lambda
// captures a reference to _overflowCount; if the wrapper is destroyed while
// the UART ISR is still active, the capture becomes dangling.
#pragma once

#include <Arduino.h>
#include <HardwareSerial.h>
#include <TinyGPSPlus.h>
#include <atomic>
#include <esp_timer.h>
#include <stdlib.h>
#include <string.h>

#include "IGpsProvider.h"
#include "types.h"   // GpsData
#include "config.h"  // GPS_BAUD, GPS_RX_PIN, GPS_TX_PIN (passed via ctor)

class SerialGpsWrapper : public IGpsProvider {
public:
    SerialGpsWrapper(uint8_t uart_num, uint32_t baud, int rxPin, int txPin)
        : _serial(uart_num), _baud(baud), _rxPin(rxPin), _txPin(txPin) {}

    void begin() override {
        _serial.setRxBufferSize(2048);  // absorb startup burst plus NMEA chatter
        _serial.begin(_baud, SERIAL_8N1, _rxPin, _txPin);

        _serial.onReceiveError([this](hardwareSerial_error_t err) {
            if (err == UART_BUFFER_FULL_ERROR || err == UART_FIFO_OVF_ERROR)
                _overflowCount++;
        });

        Serial.println("[GPS] UART initialized on pins RX=" + String(_rxPin) +
                       ", TX=" + String(_txPin) + " at " + String(_baud) + " baud.");

        vTaskDelay(pdMS_TO_TICKS(500));

        while (_serial.available()) (void)_serial.read();  // drop power-up chatter before config

        // NMEA checksum verified: XOR("PCAS02,100") = 0x1E
        _serial.print("$PCAS02,100*1E\r\n");
        Serial.println("[GPS] Update rate set to 10Hz (CASIC $PCAS02,100).");
        vTaskDelay(pdMS_TO_TICKS(200));

        // Enable only the NMEA sentences we actually use plus DHV:
        //   GGA = position/alt/sats/hdop
        //   RMC = valid + SOG (10 Hz fallback velocity)
        //   DHV = receiver-reported detailed velocity sentence (observed at 1 Hz)
        // Disables GLL/GSA/GSV/VTG/ZDA/ANT/LPS/UTC/GST/TIM to reduce UART load.
        _sendNmeaCommand("PCAS03,1,0,0,0,1,0,0,0,1,0,0,0,0,0,0,0,0,0",
                         "NMEA sentence config (GGA/RMC/DHV)");
        vTaskDelay(pdMS_TO_TICKS(200));

        // Enable CASBIN input/output while keeping NMEA input/output enabled.
        // Payload: portID=current, protoMask=B0+B1+B4+B5, mode=8N1, baud=current.
        uint8_t prt[8] = {};
        prt[0] = 0xFF;
        prt[1] = 0x33;
        prt[2] = 0xC0;
        prt[3] = 0x08;
        prt[4] = (uint8_t)(_baud & 0xFF);
        prt[5] = (uint8_t)((_baud >> 8) & 0xFF);
        prt[6] = (uint8_t)((_baud >> 16) & 0xFF);
        prt[7] = (uint8_t)((_baud >> 24) & 0xFF);

        _cfgPrtAcked = false;
        _cfgPrtNacked = false;
        _sendCasicFrame(0x06, 0x00, prt, sizeof(prt));
        Serial.println("[GPS] CFG-PRT sent (CASBIN+NMEA in/out, current baud).");

        const int prtAck = _waitForCfgAck(0x06, 0x00, 300);
        if (prtAck > 0) {
            Serial.println("[GPS] CFG-PRT ACK: binary output enabled.");

            // Enable NAV2-PVH at rate=1, meaning one message per positioning epoch.
            const uint8_t msg[4] = {0x11, 0x03, 0x01, 0x00};
            _cfgMsgAcked = false;
            _cfgMsgNacked = false;
            _sendCasicFrame(0x06, 0x01, msg, sizeof(msg));
            Serial.println("[GPS] CFG-MSG sent (NAV2-PVH 0x11/0x03 rate=1).");

            const int msgAck = _waitForCfgAck(0x06, 0x01, 300);
            if (msgAck > 0) {
                Serial.println("[GPS] CFG-MSG ACK: NAV2-PVH enabled.");
            } else if (msgAck < 0) {
                Serial.println("[GPS] CFG-MSG NACK: NAV2-PVH not enabled.");
            } else {
                Serial.println("[GPS] CFG-MSG: no ACK within 300 ms.");
            }
        } else if (prtAck < 0) {
            Serial.println("[GPS] CFG-PRT NACK: binary output not enabled; CFG-MSG skipped.");
        } else {
            Serial.println("[GPS] CFG-PRT: no ACK within 300 ms; CFG-MSG skipped.");
        }
    }

    // Drains UART, feeds TinyGPSPlus, the DHV parser, and the CASIC listener in
    // parallel. Returns true when any path produced a new snapshot for Task_GPS.
    bool update(GpsData& outData) override {
        _navPvUpdated = false;     // reset before draining this call
        _dhvUpdated = false;       // reset before draining this call
        _casicDiagUpdated = false; // ACK/NACK-only updates still publish MQTT diagnostics

        while (_serial.available()) {
            uint8_t b = (uint8_t)_serial.read();
            _gps.encode((char)b);
            _parseNmeaByte(b, outData);   // may set _dhvUpdated = true
            _parseCasicByte(b, outData);  // may set _navPvUpdated/_casicDiagUpdated
        }

        const bool location_updated = _gps.location.isUpdated();
        bool nmea_updated = _gps.satellites.isUpdated() || location_updated ||
                            _gps.speed.isUpdated()       || _gps.altitude.isUpdated();

        if (!nmea_updated && !_navPvUpdated && !_dhvUpdated && !_casicDiagUpdated) {
            return false;
        }

        // Always snapshot current NMEA state (valid or not) when returning true.
        // If only DHV or NAV2-PVH updated, NMEA fields stay unchanged and epoch
        // is NOT incremented, so Task_Filter sees refreshed velocity diagnostics
        // without a fresh position-correction trigger.
        outData.sats  = _gps.satellites.isValid() ? (uint8_t)_gps.satellites.value() : 0;
        outData.valid = _gps.location.isValid();

        if (nmea_updated) {
            outData.sog_kmh = _gps.speed.isValid() ? (float)_gps.speed.kmph() : outData.sog_kmh;
            outData.alt_m = _gps.altitude.isValid() ? (float)_gps.altitude.meters() : outData.alt_m;
            outData.hdop = _gps.hdop.isValid() ? (float)_gps.hdop.hdop() : outData.hdop;
        }

        // v1.8.4: Only a fresh location sentence may advance fix_us/epoch.
        // TinyGPSPlus keeps location.isValid() true after signal degradation;
        // speed/sats-only updates must not re-publish that stale coordinate as
        // a new position fix or the ESKF can accept kilometre-scale jumps.
        if (location_updated && outData.valid) {
            outData.lat      = _gps.location.lat();
            outData.lon      = _gps.location.lng();
            outData.fix_us   = esp_timer_get_time();
            outData.epoch++;  // only on NMEA; triggers ESKF correction in Task_Filter
        }

        _copyDiagnostics(outData);
        return true;
    }

    uint32_t getOverflowCount() const override {
        return _overflowCount.load();
    }

private:
    // CASIC Binary State Machine.
    // Accepted data frame: NAV2-PVH. ACK/NACK frames are parsed for diagnostics.
    // Other valid frames are consumed to maintain sync, then ignored.

    static constexpr uint16_t CASIC_MAX_PAYLOAD_SIZE = 88;
    static constexpr uint16_t NAV2_PVH_PAYLOAD_SIZE  = 88;
    static constexpr uint8_t  CASIC_CLASS_ACK        = 0x05;
    static constexpr uint8_t  CASIC_ID_ACK_NACK      = 0x00;
    static constexpr uint8_t  CASIC_ID_ACK_ACK       = 0x01;
    static constexpr uint8_t  CASIC_CLASS_CFG        = 0x06;
    static constexpr uint8_t  CASIC_ID_CFG_PRT       = 0x00;
    static constexpr uint8_t  CASIC_ID_CFG_MSG       = 0x01;
    static constexpr uint8_t  CASIC_CLASS_NAV2       = 0x11;
    static constexpr uint8_t  CASIC_ID_NAV2_PVH      = 0x03;
    static constexpr size_t   NMEA_LINE_MAX          = 128;

    enum class CasicState : uint8_t {
        IDLE, SYNC2, LEN_LO, LEN_HI, CLASS, ID, PAYLOAD, CKSUM0, CKSUM1, CKSUM2, CKSUM3
    };

    CasicState  _csState  = CasicState::IDLE;
    uint16_t    _csLen    = 0;
    uint8_t     _csClass  = 0;
    uint8_t     _csId     = 0;
    uint16_t    _csIdx    = 0;
    uint8_t     _csCkBuf[4] = {};
    uint8_t     _casicBuf[CASIC_MAX_PAYLOAD_SIZE] = {};
    bool        _navPvUpdated = false;
    bool        _casicDiagUpdated = false;
    char        _nmeaLineBuf[NMEA_LINE_MAX] = {};
    size_t      _nmeaLineIdx = 0;
    bool        _nmeaLineActive = false;
    bool        _dhvUpdated = false;
    bool        _loggedFirstNav2Pvh = false;
    bool        _loggedFirstNav2Vel = false;
    bool        _loggedFirstDhv = false;
    bool        _cfgPrtAcked = false;
    bool        _cfgPrtNacked = false;
    bool        _cfgMsgAcked = false;
    bool        _cfgMsgNacked = false;
    uint32_t    _nav2PvhCount = 0;
    uint32_t    _nav2VelCount = 0;

    void _sendNmeaCommand(const char* body, const char* label) {
        uint8_t checksum = 0;
        for (const char* p = body; *p != '\0'; ++p) {
            checksum ^= static_cast<uint8_t>(*p);
        }
        _serial.print('$');
        _serial.print(body);
        _serial.print('*');
        if (checksum < 0x10) _serial.print('0');
        _serial.print(checksum, HEX);
        _serial.print("\r\n");
        Serial.printf("[GPS] %s: $%s*%02X\n", label, body, (unsigned)checksum);
    }

    void _sendCasicFrame(uint8_t cls, uint8_t id, const uint8_t* payload, uint16_t len) {
        if ((len % 4U) != 0U) return;

        uint32_t ck = ((uint32_t)id << 24) | ((uint32_t)cls << 16) | (uint32_t)len;
        for (uint16_t i = 0; i < len; i += 4) {
            uint32_t word;
            memcpy(&word, payload + i, 4);
            ck += word;
        }

        const uint8_t hdr[6] = {
            0xBA, 0xCE,
            (uint8_t)(len & 0xFF), (uint8_t)((len >> 8) & 0xFF),
            cls, id
        };
        _serial.write(hdr, sizeof(hdr));
        if (len > 0) _serial.write(payload, len);
        uint8_t ckBytes[4];
        memcpy(ckBytes, &ck, sizeof(ckBytes));
        _serial.write(ckBytes, sizeof(ckBytes));
    }

    int _ackStateFor(uint8_t targetCls, uint8_t targetId) const {
        if (targetCls == CASIC_CLASS_CFG && targetId == CASIC_ID_CFG_PRT) {
            if (_cfgPrtAcked) return 1;
            if (_cfgPrtNacked) return -1;
        }
        if (targetCls == CASIC_CLASS_CFG && targetId == CASIC_ID_CFG_MSG) {
            if (_cfgMsgAcked) return 1;
            if (_cfgMsgNacked) return -1;
        }
        return 0;
    }

    int _waitForCfgAck(uint8_t targetCls, uint8_t targetId, uint32_t timeoutMs) {
        GpsData tmp{};
        const uint32_t start = millis();
        while ((uint32_t)(millis() - start) < timeoutMs) {
            while (_serial.available()) {
                _parseCasicByte((uint8_t)_serial.read(), tmp);
            }
            const int state = _ackStateFor(targetCls, targetId);
            if (state != 0) return state;
            vTaskDelay(pdMS_TO_TICKS(5));
        }
        return 0;
    }

    void _copyDiagnostics(GpsData& out) const {
        out.dbg_prt_ack = _cfgPrtAcked ? 1 : 0;
        out.dbg_prt_nack = _cfgPrtNacked ? 1 : 0;
        out.dbg_cfg_ack = _cfgMsgAcked ? 1 : 0;
        out.dbg_cfg_nack = _cfgMsgNacked ? 1 : 0;
        out.dbg_nav2pvh = _nav2PvhCount;
        out.dbg_nav2_vel = _nav2VelCount;
    }

    static int _hexNibble(char c) {
        if (c >= '0' && c <= '9') return c - '0';
        if (c >= 'A' && c <= 'F') return c - 'A' + 10;
        if (c >= 'a' && c <= 'f') return c - 'a' + 10;
        return -1;
    }

    static bool _nmeaChecksumOk(const char* line) {
        if (!line || line[0] != '$') return false;
        const char* star = strchr(line, '*');
        if (!star || (star - line) < 2 || !star[1] || !star[2]) return false;

        uint8_t checksum = 0;
        for (const char* p = line + 1; p < star; ++p) {
            checksum ^= static_cast<uint8_t>(*p);
        }
        int hi = _hexNibble(star[1]);
        int lo = _hexNibble(star[2]);
        if (hi < 0 || lo < 0) return false;
        return checksum == static_cast<uint8_t>((hi << 4) | lo);
    }

    void _dispatchNmeaLine(GpsData& out) {
        if (!_nmeaChecksumOk(_nmeaLineBuf)) return;

        char* star = strchr(_nmeaLineBuf, '*');
        char* comma = strchr(_nmeaLineBuf, ',');
        if (!star || !comma || comma <= (_nmeaLineBuf + 4)) return;
        if (strncmp(comma - 3, "DHV", 3) != 0) return;

        *star = '\0';
        char* save = nullptr;
        char* token = strtok_r(_nmeaLineBuf + 1, ",", &save);  // skip '$'
        int field = 0;
        double gdspd = 0.0;
        bool have_gdspd = false;

        while ((token = strtok_r(nullptr, ",", &save)) != nullptr) {
            field++;
            if (field == 6) {
                gdspd = strtod(token, nullptr);
                have_gdspd = true;
                break;
            }
        }

        if (!have_gdspd) return;

        out.dhv_gdspd = static_cast<float>(gdspd);
        out.dhv_fix_us = esp_timer_get_time();
        _dhvUpdated = true;

        if (!_loggedFirstDhv) {
            Serial.printf("[GPSDBG] First DHV parsed: gdspd=%.3f m/s\n", out.dhv_gdspd);
            _loggedFirstDhv = true;
        }
    }

    void _parseNmeaByte(uint8_t b, GpsData& out) {
        if (b == '$') {
            _nmeaLineActive = true;
            _nmeaLineIdx = 0;
            _nmeaLineBuf[_nmeaLineIdx++] = static_cast<char>(b);
            return;
        }

        if (!_nmeaLineActive) return;

        if (_nmeaLineIdx >= (NMEA_LINE_MAX - 1)) {
            if (b == '\n') {
                _nmeaLineActive = false;
                _nmeaLineIdx = 0;
            }
            return;
        }

        _nmeaLineBuf[_nmeaLineIdx++] = static_cast<char>(b);
        if (b != '\n') return;

        _nmeaLineBuf[_nmeaLineIdx] = '\0';
        _nmeaLineActive = false;
        _dispatchNmeaLine(out);
        _nmeaLineIdx = 0;
    }

    void _parseCasicByte(uint8_t b, GpsData& out) {
        switch (_csState) {
        case CasicState::IDLE:
            if (b == 0xBA) _csState = CasicState::SYNC2;
            break;
        case CasicState::SYNC2:
            _csState = (b == 0xCE) ? CasicState::LEN_LO : CasicState::IDLE;
            break;
        case CasicState::LEN_LO:
            _csLen = b;
            _csState = CasicState::LEN_HI;
            break;
        case CasicState::LEN_HI:
            _csLen |= ((uint16_t)b << 8);
            if (_csLen <= CASIC_MAX_PAYLOAD_SIZE && (_csLen % 4U) == 0U) {
                _csState = CasicState::CLASS;
            } else {
                _csState = CasicState::IDLE;
            }
            break;
        case CasicState::CLASS:
            _csClass = b;
            _csState = CasicState::ID;
            break;
        case CasicState::ID:
            _csId = b;
            _csIdx = 0;
            _csState = (_csLen > 0) ? CasicState::PAYLOAD : CasicState::CKSUM0;
            break;
        case CasicState::PAYLOAD:
            if (_csIdx < CASIC_MAX_PAYLOAD_SIZE) _casicBuf[_csIdx] = b;
            _csIdx++;
            if (_csIdx >= _csLen) _csState = CasicState::CKSUM0;
            break;
        case CasicState::CKSUM0: _csCkBuf[0] = b; _csState = CasicState::CKSUM1; break;
        case CasicState::CKSUM1: _csCkBuf[1] = b; _csState = CasicState::CKSUM2; break;
        case CasicState::CKSUM2: _csCkBuf[2] = b; _csState = CasicState::CKSUM3; break;
        case CasicState::CKSUM3:
            _csCkBuf[3] = b;
            _csState = CasicState::IDLE;
            _dispatchCasicFrame(out);
            break;
        }
    }

    void _dispatchCasicFrame(GpsData& out) {
        uint32_t ckExpect = ((uint32_t)_csId << 24)
                           | ((uint32_t)_csClass << 16)
                           | (uint32_t)_csLen;
        const uint16_t words = _csLen / 4;
        for (uint16_t i = 0; i < words; i++) {
            uint32_t word;
            memcpy(&word, _casicBuf + i * 4, 4);
            ckExpect += word;
        }
        uint32_t ckReceived;
        memcpy(&ckReceived, _csCkBuf, 4);
        if (ckExpect != ckReceived) {
            return;
        }

        if (_csClass == CASIC_CLASS_ACK &&
            (_csId == CASIC_ID_ACK_ACK || _csId == CASIC_ID_ACK_NACK) &&
            _csLen == 4) {
            const uint8_t targetCls = _casicBuf[0];
            const uint8_t targetId = _casicBuf[1];
            const bool ack = (_csId == CASIC_ID_ACK_ACK);

            if (targetCls == CASIC_CLASS_CFG && targetId == CASIC_ID_CFG_PRT) {
                _cfgPrtAcked = ack;
                _cfgPrtNacked = !ack;
                _casicDiagUpdated = true;
            } else if (targetCls == CASIC_CLASS_CFG && targetId == CASIC_ID_CFG_MSG) {
                _cfgMsgAcked = ack;
                _cfgMsgNacked = !ack;
                _casicDiagUpdated = true;
            }
            return;
        }

        if (_csClass != CASIC_CLASS_NAV2 || _csId != CASIC_ID_NAV2_PVH ||
            _csLen != NAV2_PVH_PAYLOAD_SIZE) {
            return;
        }

        const uint8_t* p = _casicBuf;
        const uint8_t velFlags = p[9];

        float velE, velN, speed2D, sAcc_mps;
        memcpy(&velE,     p + 48, 4);
        memcpy(&velN,     p + 52, 4);
        memcpy(&speed2D,  p + 64, 4);
        memcpy(&sAcc_mps, p + 80, 4);

        out.nav_speed2d = speed2D;
        out.nav_s_acc = sAcc_mps;
        out.nav_vel_n = velN;
        out.nav_vel_e = velE;
        out.nav_vel_valid = velFlags;
        out.nav_fix_us = esp_timer_get_time();

        _nav2PvhCount++;
        if (velFlags >= 6) _nav2VelCount++;
        _navPvUpdated = true;

        if (!_loggedFirstNav2Pvh) {
            Serial.printf(
                "[GPSDBG] First NAV2-PVH parsed: velflags=%u speed2D=%.3f m/s velN=%.3f velE=%.3f sAcc=%.3f m/s\n",
                velFlags, speed2D, velN, velE, sAcc_mps
            );
            _loggedFirstNav2Pvh = true;
        }
        if (velFlags >= 6 && !_loggedFirstNav2Vel) {
            Serial.printf(
                "[GPSDBG] First NAV2-PVH vel-valid: velflags=%u speed2D=%.3f m/s sAcc=%.3f m/s\n",
                velFlags, speed2D, sAcc_mps
            );
            _loggedFirstNav2Vel = true;
        }
    }

    HardwareSerial          _serial;
    TinyGPSPlus             _gps;
    std::atomic<uint32_t>   _overflowCount{0};
    const uint32_t          _baud;
    const int               _rxPin;
    const int               _txPin;
};
