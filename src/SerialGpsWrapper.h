// SPDX-License-Identifier: GPL-3.0-or-later
// SerialGpsWrapper.h — Concrete IGpsProvider backed by HardwareSerial + TinyGPSPlus
//
// Encapsulates all GPS hardware concerns:
//   - HardwareSerial (UART1, RX/TX pins, baud rate, RX buffer size)
//   - TinyGPSPlus sentence decoding
//   - UART overflow ISR callback (atomic counter, captured by reference)
//   - AT6668 chip-startup delay + CASIC $PCAS02 rate command
//
// Lifetime: MUST be a static or global object. The onReceiveError lambda
// captures a reference to _overflowCount; if the wrapper is destroyed while
// the UART ISR is still active, the capture becomes dangling.
//
// Usage:
//   // In Telemetria.ino, file scope (static storage duration):
//   static SerialGpsWrapper gpsWrapper(1, GPS_BAUD, GPS_RX_PIN, GPS_TX_PIN);
//
//   // In setup():
//   gpsWrapper.begin();
//
//   // When creating Task_GPS:
//   xTaskCreatePinnedToCore(Task_GPS, "GPS", 4096, &gpsWrapper, 2, NULL, 0);
#pragma once

#include <Arduino.h>
#include <HardwareSerial.h>
#include <TinyGPSPlus.h>
#include <atomic>

#include "IGpsProvider.h"
#include "types.h"   // GpsData
#include "config.h"  // GPS_BAUD, GPS_RX_PIN, GPS_TX_PIN (for documentation; passed via ctor)

class SerialGpsWrapper : public IGpsProvider {
public:
    // uart_num: ESP32 UART number (1 = UART1, Grove connector on AtomS3)
    SerialGpsWrapper(uint8_t uart_num, uint32_t baud, int rxPin, int txPin)
        : _serial(uart_num), _baud(baud), _rxPin(rxPin), _txPin(txPin) {}

    // Initializes UART, registers the overflow callback, waits 500 ms for AT6668
    // chip startup, then sends the CASIC command to switch to 10 Hz update rate.
    // Blocking: ~500 ms due to AT6668 startup requirement.
    void begin() override {
        _serial.setRxBufferSize(512);  // default 256 too tight for 10 Hz NMEA at 115200
        _serial.begin(_baud, SERIAL_8N1, _rxPin, _txPin);

        // UART overflow callback — captures _overflowCount by reference.
        // HardwareSerial::onReceiveError() dispatches from the UART event task
        // (not directly from the hardware ISR), so no IRAM_ATTR is required.
        // The lambda captures only this atomic member; SerialGpsWrapper must outlive
        // the UART peripheral (i.e., static/global lifetime).
        _serial.onReceiveError([this](hardwareSerial_error_t err) {
            if (err == UART_BUFFER_FULL_ERROR || err == UART_FIFO_OVF_ERROR)
                _overflowCount++;
        });

        Serial.println("[GPS] UART initialized on pins RX=" + String(_rxPin) +
                       ", TX=" + String(_txPin) + " at " + String(_baud) + " baud.");

        // Wait for AT6668 chip startup before sending CASIC configuration.
        // Sending commands too early results in them being silently ignored.
        vTaskDelay(pdMS_TO_TICKS(500));

        // Force update rate to 10 Hz via CASIC protocol (AT6668 / ATGM336H-6N).
        // Reduces IMU dead-reckoning gap from ~1000 ms to ~100 ms.
        // NMEA checksum verified: XOR("PCAS02,100") = 0x1E
        _serial.print("$PCAS02,100*1E\r\n");
        Serial.println("[GPS] Update rate set to 10Hz (CASIC $PCAS02,100).");
    }

    // Drains the UART RX buffer into TinyGPSPlus. Returns true and populates
    // outData when a complete, updated fix has been decoded. Returns false otherwise.
    bool update(GpsData& outData) override {
        while (_serial.available()) {
            _gps.encode(_serial.read());
        }

        if (!_gps.satellites.isUpdated() && !_gps.location.isUpdated() &&
            !_gps.speed.isUpdated()      && !_gps.altitude.isUpdated()) {
            return false;
        }

        outData.sats  = _gps.satellites.isValid() ? (uint8_t)_gps.satellites.value() : 0;
        outData.valid = _gps.location.isValid();

        if (outData.valid) {
            outData.lat       = _gps.location.lat();
            outData.lon       = _gps.location.lng();
            outData.speed_kmh = _gps.speed.isValid()    ? (float)_gps.speed.kmph()     : 0.0f;
            outData.alt_m     = _gps.altitude.isValid() ? (float)_gps.altitude.meters() : 0.0f;
            outData.hdop      = _gps.hdop.isValid()     ? (float)_gps.hdop.hdop()       : 99.9f;
            outData.fix_ms    = millis();
            outData.epoch++;  // signals fresh fix to Task_Filter
        }

        return true;
    }

    uint32_t getOverflowCount() const override {
        return _overflowCount.load();
    }

private:
    HardwareSerial          _serial;
    TinyGPSPlus             _gps;
    std::atomic<uint32_t>   _overflowCount{0};
    const uint32_t          _baud;
    const int               _rxPin;
    const int               _txPin;
};
