// SPDX-License-Identifier: GPL-3.0-or-later
// SerialGpsWrapper.h - Concrete IGpsProvider backed by HardwareSerial + TinyGPSPlus
//                    + NMEA DHV parser (1 Hz diagnostics on the tested AT6668 module)
//                    + passive CASIC NAV-PV binary listener
//
// Encapsulates all GPS hardware concerns:
//   - HardwareSerial (UART1, RX/TX pins, baud rate, RX buffer size)
//   - TinyGPSPlus sentence decoding (position, NMEA SOG, sats, hdop)
//   - NMEA DHV parser (gdspd, dhv_fix_us) — 1 Hz on the tested AT6668 module
//   - CASIC NAV-PV binary parser (speed2D, sAcc, velN/E, velValid)
//   - UART overflow ISR callback (atomic counter, captured by reference)
//   - AT6668 chip-startup delay + CASIC $PCAS02 rate command (10 Hz positioning)
//   - Explicit $PCAS03 sentence selection (GGA/RMC/DHV only)
//   - Passive CASIC binary parsing only; no active binary reconfiguration in
//     the clean production build
//
// CASIC Binary Protocol (CSIP):
//   Frame: [0xBA][0xCE][len_lo][len_hi][class][id][payload × len][cksum × 4]
//   Checksum (32-bit, LE): ckSum = (id<<24)|(class<<16)|len;
//                          for each uint32 word in payload: ckSum += word
//
// NAV-PV payload layout (class=0x01, id=0x03, 80 bytes, all little-endian):
//   Offset  Bytes  Type  Name      Description
//   0       4      U4    iTOW      GPS time of week [ms]
//   4       1      U1    navMode   Fix type: 0=no fix, 1=DR, 2=2D, 3=3D, 4=GPS+DR
//   5       1      U1    velValid  Velocity validity: 0=invalid, non-zero=valid
//   6       2      U2    numSV     Satellites used
//   8       8      R8    lat       Latitude [deg]
//   16      8      R8    lon       Longitude [deg]
//   24      4      R4    hMSL      Height above MSL [m]
//   28      4      R4    hAcc      Horizontal accuracy [m]
//   32      4      R4    vAcc      Vertical accuracy [m]
//   36      4      R4    pDop      Position DOP
//   40      4      R4    hDop      Horizontal DOP
//   44      4      R4    vDop      Vertical DOP
//   48      4      R4    velN      North velocity [m/s]        ← used
//   52      4      R4    velE      East velocity [m/s]         ← used
//   56      4      R4    velD      Down velocity [m/s]
//   60      4      R4    speed3D   3D speed [m/s]
//   64      4      R4    speed2D   2D ground speed [m/s]       ← used
//   68      4      R4    heading   Course over ground [deg]
//   72      4      R4    sAcc      Speed accuracy estimate [m/s]  ← used
//   76      4      R4    cAcc      Course accuracy estimate [deg]
//
// ASSUMPTION (unverified on hardware): offsets 36–47 = pDop/hDop/vDop (3×float).
// The fields actually used (velValid@5, velN@48, velE@52, speed2D@64, sAcc@72)
// are specified by the plan and should be validated against a real AT6668 capture.
//
// Both NMEA and binary coexist on the same UART. TinyGPSPlus safely ignores
// 0xBA/0xCE bytes. The CASIC parser runs in parallel on every byte.
//
// Contract with Task_GPS (v1.5.2-clean):
//   update() returns true when a new NMEA fix, a new DHV sentence, or a passive
//   NAV-PV frame was parsed in this poll cycle. epoch is incremented only on the
//   NMEA fix path, so velocity-only updates refresh shared_gps_data without
//   triggering an extra GPS correction in Task_Filter.
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
        //   DHV = receiver-reported detailed velocity sentence (gdspd, observed at 1 Hz)
        // Disables GLL/GSA/GSV/VTG/ZDA/ANT/LPS/UTC/GST/TIM to reduce UART load.
        _sendNmeaCommand("PCAS03,1,0,0,0,1,0,0,0,1,0,0,0,0,0,0,0,0,0",
                         "NMEA sentence config (GGA/RMC/DHV)");
        vTaskDelay(pdMS_TO_TICKS(200));

        Serial.println("[GPS] Clean build: passive CASIC listener only (no CFG-PRT/CFG-MSG probing).");
    }

    // Drains UART, feeds TinyGPSPlus, the DHV parser, and the passive CASIC
    // listener in parallel. Returns true when any of those paths produced a
    // new snapshot for Task_GPS to publish.
    bool update(GpsData& outData) override {
        _navPvUpdated = false;  // reset before draining this call
        _dhvUpdated = false;    // reset before draining this call

        while (_serial.available()) {
            uint8_t b = (uint8_t)_serial.read();
            _gps.encode((char)b);
            _parseNmeaByte(b, outData);   // may set _dhvUpdated = true
            _parseCasicByte(b, outData);  // passive NAV-PV listener
        }

        bool nmea_updated = _gps.satellites.isUpdated() || _gps.location.isUpdated() ||
                            _gps.speed.isUpdated()       || _gps.altitude.isUpdated();

        if (!nmea_updated && !_navPvUpdated && !_dhvUpdated) {
            return false;
        }

        // Always snapshot current NMEA state (valid or not) when returning true.
        // If only DHV or passive NAV-PV updated, NMEA fields stay unchanged and
        // epoch is NOT incremented, so Task_Filter sees refreshed velocity data
        // without a fresh position-correction trigger.
        outData.sats  = _gps.satellites.isValid() ? (uint8_t)_gps.satellites.value() : 0;
        outData.valid = _gps.location.isValid();

        if (nmea_updated && outData.valid) {
            outData.lat      = _gps.location.lat();
            outData.lon      = _gps.location.lng();
            outData.sog_kmh  = _gps.speed.isValid()    ? (float)_gps.speed.kmph()      : 0.0f;
            outData.alt_m    = _gps.altitude.isValid()  ? (float)_gps.altitude.meters() : 0.0f;
            outData.hdop     = _gps.hdop.isValid()      ? (float)_gps.hdop.hdop()       : 99.9f;
            outData.fix_us   = esp_timer_get_time();
            outData.epoch++;  // only on NMEA — triggers ESKF correction in Task_Filter
        }

        return true;
    }

    uint32_t getOverflowCount() const override {
        return _overflowCount.load();
    }

private:
    // ── CASIC Binary State Machine ─────────────────────────────────────────
    // Processes one byte per call. Accepted frame types: NAV-PV only.
    // All other frame types are accepted through PAYLOAD→CKSUM states (to
    // maintain sync) but discarded in _dispatchCasicFrame().
    //
    // States: IDLE→SYNC2→LEN_LO→LEN_HI→CLASS→ID→PAYLOAD→CKSUM0→…→CKSUM3

    static constexpr uint16_t NAV_PV_PAYLOAD_SIZE   = 80;
    static constexpr uint8_t  CASIC_CLASS_NAV       = 0x01;
    static constexpr uint8_t  CASIC_ID_PV           = 0x03;
    static constexpr size_t   NMEA_LINE_MAX         = 128;

    enum class CasicState : uint8_t {
        IDLE, SYNC2, LEN_LO, LEN_HI, CLASS, ID, PAYLOAD, CKSUM0, CKSUM1, CKSUM2, CKSUM3
    };

    CasicState  _csState  = CasicState::IDLE;
    uint16_t    _csLen    = 0;
    uint8_t     _csClass  = 0;
    uint8_t     _csId     = 0;
    uint16_t    _csIdx    = 0;
    uint8_t     _csCkBuf[4] = {};
    uint8_t     _casicBuf[NAV_PV_PAYLOAD_SIZE];
    bool        _navPvUpdated = false;  // set by _dispatchCasicFrame on valid NAV-PV
    char        _nmeaLineBuf[NMEA_LINE_MAX] = {};
    size_t      _nmeaLineIdx = 0;
    bool        _nmeaLineActive = false;
    bool        _dhvUpdated = false;
    bool        _loggedFirstNavPv = false;
    bool        _loggedFirstNavPvVel = false;
    bool        _loggedFirstDhv = false;

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
            if (b == 0xCE) {
                _csState = CasicState::LEN_LO;
            } else {
                _csState = CasicState::IDLE;
            }
            break;
        case CasicState::LEN_LO:
            _csLen  = b;
            _csState = CasicState::LEN_HI;
            break;
        case CasicState::LEN_HI:
            _csLen |= ((uint16_t)b << 8);
            if (_csLen <= NAV_PV_PAYLOAD_SIZE) {
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
            _csId  = b;
            _csIdx = 0;
            _csState = (_csLen > 0) ? CasicState::PAYLOAD : CasicState::CKSUM0;
            break;
        case CasicState::PAYLOAD:
            if (_csIdx < NAV_PV_PAYLOAD_SIZE) _casicBuf[_csIdx] = b;
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
        // Verify checksum
        uint32_t ckExpect = ((uint32_t)_csId << 24)
                           | ((uint32_t)_csClass << 16)
                           | (uint32_t)_csLen;
        uint16_t words = _csLen / 4;
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

        if (_csClass != CASIC_CLASS_NAV || _csId != CASIC_ID_PV
                || _csLen != NAV_PV_PAYLOAD_SIZE) {
            return;
        }

        // NAV-PV 80-byte payload — corrected offsets (v1.5.0):
        //
        // [4]  U1  navMode   fix type (0=no fix, 2=2D, 3=3D, 4=GPS+DR)
        // [5]  U1  velValid  velocity validity: 0=invalid, non-zero=valid
        // [48] R4  velN      North velocity [m/s]
        // [52] R4  velE      East velocity [m/s]
        // [64] R4  speed2D   2D ground speed [m/s] — direct, not reconstructed
        // [72] R4  sAcc      speed accuracy estimate [m/s] — direct float

        const uint8_t* p = _casicBuf;

        uint8_t navMode  = p[4];
        uint8_t velValid = p[5];

        float velN, velE, speed2D, sAcc_mps;
        memcpy(&velN,     p + 48, 4);
        memcpy(&velE,     p + 52, 4);
        memcpy(&speed2D,  p + 64, 4);
        memcpy(&sAcc_mps, p + 72, 4);

        // nav_vel_valid encoding: 0=invalid, 6=2D fix with vel, 7=3D fix with vel
        uint8_t navVelValid = 0;
        if (velValid != 0) {
            navVelValid = (navMode >= 3) ? 7 : 6;
        }

        out.nav_speed2d   = speed2D;
        out.nav_s_acc     = sAcc_mps;  // speed accuracy estimate [m/s]
        out.nav_vel_n     = velN;
        out.nav_vel_e     = velE;
        out.nav_vel_valid = navVelValid;
        out.nav_fix_us    = esp_timer_get_time();

        if (!_loggedFirstNavPv) {
            Serial.printf(
                "[GPSDBG] First NAV-PV parsed: navMode=%u velValid=%u speed2D=%.3f m/s velN=%.3f velE=%.3f sAcc=%.3f m/s\n",
                navMode, velValid, speed2D, velN, velE, sAcc_mps
            );
            _loggedFirstNavPv = true;
        }
        if (navVelValid >= 6) {
            if (!_loggedFirstNavPvVel) {
                Serial.printf(
                    "[GPSDBG] First NAV-PV vel-valid: navVelValid=%u speed2D=%.3f m/s sAcc=%.3f m/s\n",
                    navVelValid, speed2D, sAcc_mps
                );
                _loggedFirstNavPvVel = true;
            }
        }

        _navPvUpdated = true;
    }

    HardwareSerial          _serial;
    TinyGPSPlus             _gps;
    std::atomic<uint32_t>   _overflowCount{0};
    const uint32_t          _baud;
    const int               _rxPin;
    const int               _txPin;
};
