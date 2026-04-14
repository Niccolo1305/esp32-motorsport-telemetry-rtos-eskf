// SPDX-License-Identifier: GPL-3.0-or-later
// IGpsProvider.h — Pure interface for GPS data acquisition (HAL)
//
// Decouples Task_GPS from the physical UART/TinyGPSPlus implementation.
// Concrete implementations: SerialGpsWrapper (hardware), or mock stubs for testing.
#pragma once

#include "types.h"  // GpsData

class IGpsProvider {
public:
    virtual ~IGpsProvider() = default;

    // Initialize hardware (UART, baud rate, chip-startup delay, CASIC commands).
    // Blocks for ~500 ms on hardware implementations (AT6668 startup sequence).
    virtual void begin() = 0;

    // Drain the UART buffer and decode NMEA sentences.
    // Returns true if a new complete fix was decoded this call; outData is populated.
    // Returns false if no new data is available; outData is unchanged.
    virtual bool update(GpsData& outData) = 0;

    // Returns the cumulative count of UART overflow events since begin().
    virtual uint32_t getOverflowCount() const = 0;
};
