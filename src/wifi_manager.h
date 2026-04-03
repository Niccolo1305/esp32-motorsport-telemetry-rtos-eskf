// SPDX-License-Identifier: GPL-3.0-or-later
// wifi_manager.h — WiFi/MQTT connection management and radio control
#pragma once

bool load_wifi_config();
void connect_wifi();
void reconnect_network();
void shutdown_radio();
void startup_radio();
