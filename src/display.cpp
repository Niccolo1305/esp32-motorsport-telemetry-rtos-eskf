// SPDX-License-Identifier: GPL-3.0-or-later
// display.cpp — LCD display helper functions
#include "display.h"

#include <M5Unified.h>

#include "globals.h"

void setLabel(int y, const char *text, uint16_t txtColor,
              uint16_t bgColor) {
  if (text == nullptr) {
    text = "";
  }
  M5.Lcd.fillRect(0, y, M5.Lcd.width(), 16, bgColor);
  M5.Lcd.setTextColor(txtColor, bgColor);
  M5.Lcd.setCursor(2, y);
  M5.Lcd.print(text);
}

void fillScreen(uint32_t color) { M5.Lcd.fillScreen((uint16_t)color); }

// Redraws the Wi-Fi indicator circle (bottom-right corner of LCD).
// Called every time the state changes (including after triple-click toggle).
void update_wifi_indicator() {
  uint16_t color = wifi_enabled ? GREEN : RED;
  M5.Lcd.fillCircle(115, 115, 6, color);
}
