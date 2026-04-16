// SPDX-License-Identifier: GPL-3.0-or-later
// SerialGpsWrapper.h — Concrete IGpsProvider backed by HardwareSerial + TinyGPSPlus
//                      + CASIC binary protocol parser for NAV-PV velocity (v1.5.0)
//
// Encapsulates all GPS hardware concerns:
//   - HardwareSerial (UART1, RX/TX pins, baud rate, RX buffer size)
//   - TinyGPSPlus sentence decoding (position, NMEA SOG, sats, hdop)
//   - CASIC binary NAV-PV parser (speed2D, sAcc, velN/E, velValid)
//   - UART overflow ISR callback (atomic counter, captured by reference)
//   - AT6668 chip-startup delay + CASIC $PCAS02 rate command (10 Hz NMEA)
//   - Boot CFG-MSG to enable periodic binary NAV-PV output
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
// Contract with Task_GPS (v1.5.0):
//   update() returns true when EITHER a new NMEA sentence OR a new NAV-PV frame
//   was fully parsed this call. This guarantees nav_* fields reach shared_gps_data
//   even when NAV-PV arrives in a poll cycle without a concurrent NMEA update.
//   epoch is incremented only on NMEA fix → ESKF correction is not triggered by
//   a NAV-PV-only update.
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
#include <string.h>

#include "IGpsProvider.h"
#include "types.h"   // GpsData
#include "config.h"  // GPS_BAUD, GPS_RX_PIN, GPS_TX_PIN (passed via ctor)

class SerialGpsWrapper : public IGpsProvider {
public:
    SerialGpsWrapper(uint8_t uart_num, uint32_t baud, int rxPin, int txPin)
        : _serial(uart_num), _baud(baud), _rxPin(rxPin), _txPin(txPin) {}

    void begin() override {
        _serial.setRxBufferSize(512);  // default 256 too tight for 10 Hz NMEA + binary
        _serial.begin(_baud, SERIAL_8N1, _rxPin, _txPin);

        _serial.onReceiveError([this](hardwareSerial_error_t err) {
            if (err == UART_BUFFER_FULL_ERROR || err == UART_FIFO_OVF_ERROR)
                _overflowCount++;
        });

        Serial.println("[GPS] UART initialized on pins RX=" + String(_rxPin) +
                       ", TX=" + String(_txPin) + " at " + String(_baud) + " baud.");

        vTaskDelay(pdMS_TO_TICKS(500));

        // NMEA checksum verified: XOR("PCAS02,100") = 0x1E
        _serial.print("$PCAS02,100*1E\r\n");
        Serial.println("[GPS] Update rate set to 10Hz (CASIC $PCAS02,100).");

        // Enable CASIC binary NAV-PV via CFG-MSG (class=0x06, id=0x01).
        // Payload: [msg_class=0x01, msg_id=0x03, rate=0x01, reserved=0x00]
        //
        // Checksum derivation:
        //   ckSum  = (0x01<<24)|(0x06<<16)|0x0004  = 0x01060004
        //   payload as uint32 LE: 01 03 01 00 → 0x00010301
        //   ckSum += 0x00010301 → 0x01070305 → LE bytes: 05 03 07 01
        //
        // Full frame: BA CE 04 00 06 01  01 03 01 00  05 03 07 01
        static const uint8_t cfgMsgNavPv[] = {
            0xBA, 0xCE,              // sync
            0x04, 0x00,              // payload length = 4
            0x06, 0x01,              // class=CFG-MSG, id=MSG
            0x01, 0x03, 0x01, 0x00,  // NAV class=1, id=PV=3, rate=1, rsv=0
            0x05, 0x03, 0x07, 0x01   // checksum LE
        };
        _serial.write(cfgMsgNavPv, sizeof(cfgMsgNavPv));
        Serial.println("[GPS] CFG-MSG sent: NAV-PV binary output enabled.");
    }

    // Drains UART, feeds TinyGPSPlus and CASIC parser in parallel.
    // Returns true when NMEA updated OR a new NAV-PV frame was parsed.
    // nav_* fields in outData are updated by the CASIC path independently.
    bool update(GpsData& outData) override {
        _navPvUpdated = false;  // reset before draining this call

        while (_serial.available()) {
            uint8_t b = (uint8_t)_serial.read();
            _gps.encode((char)b);
            _parseCasicByte(b, outData);  // may set _navPvUpdated = true
        }

        bool nmea_updated = _gps.satellites.isUpdated() || _gps.location.isUpdated() ||
                            _gps.speed.isUpdated()       || _gps.altitude.isUpdated();

        if (!nmea_updated && !_navPvUpdated) {
            return false;
        }

        // Always snapshot current NMEA state (valid or not) when returning true.
        // If only NAV-PV updated, NMEA fields stay unchanged and epoch is NOT
        // incremented — Task_Filter will see the same epoch, so no ESKF correction
        // is triggered; only nav_* fields are refreshed in shared_gps_data.
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

    static constexpr uint16_t NAV_PV_PAYLOAD_SIZE = 80;
    static constexpr uint8_t  CASIC_CLASS_NAV     = 0x01;
    static constexpr uint8_t  CASIC_ID_PV         = 0x03;

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

    void _parseCasicByte(uint8_t b, GpsData& out) {
        switch (_csState) {
        case CasicState::IDLE:
            if (b == 0xBA) _csState = CasicState::SYNC2;
            break;
        case CasicState::SYNC2:
            _csState = (b == 0xCE) ? CasicState::LEN_LO : CasicState::IDLE;
            break;
        case CasicState::LEN_LO:
            _csLen  = b;
            _csState = CasicState::LEN_HI;
            break;
        case CasicState::LEN_HI:
            _csLen |= ((uint16_t)b << 8);
            _csState = (_csLen <= NAV_PV_PAYLOAD_SIZE) ? CasicState::CLASS : CasicState::IDLE;
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
        if (ckExpect != ckReceived) return;

        if (_csClass != CASIC_CLASS_NAV || _csId != CASIC_ID_PV
                || _csLen != NAV_PV_PAYLOAD_SIZE) return;

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

        _navPvUpdated = true;
    }

    HardwareSerial          _serial;
    TinyGPSPlus             _gps;
    std::atomic<uint32_t>   _overflowCount{0};
    const uint32_t          _baud;
    const int               _rxPin;
    const int               _txPin;
};
