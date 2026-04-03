// SPDX-License-Identifier: GPL-3.0-or-later
// display.h — LCD display helper functions
#pragma once

#include <stdint.h>

void setLabel(int y, const char *text, uint16_t txtColor = 0xFFFF,
              uint16_t bgColor = 0x0000);
void fillScreen(uint32_t color);
void update_wifi_indicator();
