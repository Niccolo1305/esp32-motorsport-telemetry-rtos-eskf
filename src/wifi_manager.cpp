// SPDX-License-Identifier: GPL-3.0-or-later
// wifi_manager.cpp — WiFi/MQTT connection management and radio control
//
// Loads credentials from /wifi_config.txt on SD, handles multi-SSID scanning,
// MQTT connect/reconnect, and full radio shutdown/startup for RAM recovery.
#include "wifi_manager.h"

#include <M5Unified.h>
#include <SD.h>
#include <WiFi.h>
#include <esp_wifi.h>

#include "globals.h"
#include "display.h"

// Loads WiFi credentials and MQTT settings from /wifi_config.txt on SD.
// Must be called after SD.begin(). Returns true if at least one SSID was found.
// On failure, wifi_enabled is set to false (WiFi silently disabled).
bool load_wifi_config() {
  File f = SD.open("/wifi_config.txt", FILE_READ);
  if (!f) {
    Serial.println("[CFG] /wifi_config.txt not found: WiFi disabled.");
    wifi_enabled = false;
    return false;
  }

  char tmp_ssid[2][33] = {{'\0'}, {'\0'}};
  char tmp_pass[2][65] = {{'\0'}, {'\0'}};

  char line[160];
  while (f.available()) {
    int len = 0;
    while (f.available() && len < (int)(sizeof(line) - 1)) {
      char c = (char)f.read();
      if (c == '\n') break;
      if (c != '\r') line[len++] = c;
    }
    line[len] = '\0';
    if (len == 0 || line[0] == '#') continue;

    char *eq = strchr(line, '=');
    if (!eq) continue;
    *eq = '\0';
    const char *key = line;
    const char *val = eq + 1;

    // Trim trailing whitespace from key
    char *end = eq - 1;
    while (end >= line && (*end == ' ' || *end == '\t')) *end-- = '\0';

    if      (strcmp(key, "SSID")        == 0) strncpy(tmp_ssid[0], val, 32);
    else if (strcmp(key, "PASSWORD")    == 0) strncpy(tmp_pass[0], val, 64);
    else if (strcmp(key, "SSID2")       == 0) strncpy(tmp_ssid[1], val, 32);
    else if (strcmp(key, "PASSWORD2")   == 0) strncpy(tmp_pass[1], val, 64);
    else if (strcmp(key, "MQTT_BROKER") == 0) strncpy(cfg_mqtt_broker, val, sizeof(cfg_mqtt_broker) - 1);
    else if (strcmp(key, "MQTT_PORT")   == 0) cfg_mqtt_port = atoi(val);
    else if (strcmp(key, "MQTT_TOPIC")  == 0) strncpy(cfg_mqtt_topic,  val, sizeof(cfg_mqtt_topic)  - 1);
  }
  f.close();

  for (int i = 0; i < 2; i++) {
    if (strlen(tmp_ssid[i]) > 0) {
      strncpy(wifi_networks[wifi_network_count].ssid,     tmp_ssid[i], 32);
      strncpy(wifi_networks[wifi_network_count].password, tmp_pass[i], 64);
      wifi_network_count++;
    }
  }

  if (wifi_network_count == 0) {
    Serial.println("[CFG] No valid SSID in wifi_config.txt: WiFi disabled.");
    wifi_enabled = false;
    return false;
  }

  // Generate unique client ID from last 3 bytes of chip MAC address
  uint64_t mac = ESP.getEfuseMac();
  snprintf(cfg_mqtt_client_id, sizeof(cfg_mqtt_client_id),
           "TelUnit_%06lX", (unsigned long)(mac & 0xFFFFFF));

  // Default topic if not specified in config
  if (strlen(cfg_mqtt_topic) == 0) {
    snprintf(cfg_mqtt_topic, sizeof(cfg_mqtt_topic),
             "telemetry/%s/data", cfg_mqtt_client_id);
  }

  mqtt_enabled = (strlen(cfg_mqtt_broker) > 0);

  Serial.printf("[CFG] %d network(s) loaded. Client: %s. MQTT: %s\n",
                wifi_network_count, cfg_mqtt_client_id,
                mqtt_enabled ? cfg_mqtt_broker : "disabled");
  return true;
}

void connect_wifi() {
  if (!wifi_enabled || wifi_network_count == 0) {
    wifi_enabled = false;
    return;
  }

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  vTaskDelay(pdMS_TO_TICKS(500));

  while (WiFi.status() != WL_CONNECTED && wifi_enabled) {
    for (int r = 0; r < wifi_network_count; r++) {
      snprintf(buf, sizeof(buf), "Scanning: %.10s", wifi_networks[r].ssid);
      setLabel(10, buf);
      setLabel(30, "");
      WiFi.begin(wifi_networks[r].ssid, wifi_networks[r].password);
      for (int i = 0; i < 12; i++) {
        if (!wifi_enabled)
          return;
        if (WiFi.status() == WL_CONNECTED) {
          setLabel(10, "WiFi Connected!");
          setLabel(30, "");
          vTaskDelay(pdMS_TO_TICKS(1000));
          return;
        }
        snprintf(buf, sizeof(buf), "Attempt %d/12", i + 1);
        setLabel(30, buf);
        M5.update();
        vTaskDelay(pdMS_TO_TICKS(500));
      }
      WiFi.disconnect();
      vTaskDelay(pdMS_TO_TICKS(500));
    }
  }
}

void reconnect_network() {
  if (!wifi_enabled || !mqtt_enabled)
    return;

  fillScreen(0xFF0000);
  setLabel(10, "Connection Lost!", RED, (uint16_t)0xFF0000);
  setLabel(30, "Reconnecting...", WHITE, (uint16_t)0xFF0000);
  vTaskDelay(pdMS_TO_TICKS(1000));

  bool connected = false;
  while (!connected && wifi_enabled) {
    M5.update();
    if (WiFi.status() != WL_CONNECTED) {
      setLabel(10, "WiFi Down!");
      vTaskDelay(pdMS_TO_TICKS(1000));
      connect_wifi();
      if (!wifi_enabled)
        return;
      fillScreen(0xFF0000);
    }
    setLabel(10, "Reconnecting MQTT...");
    setLabel(30, "");
    mqttClient.disconnect();
    if (mqttClient.connect(cfg_mqtt_client_id)) {
      fillScreen(0x66FF99);
      setLabel(10, "MQTT Online!");
      vTaskDelay(pdMS_TO_TICKS(1000));
      prev_system_state = -1;
      connected = true;
    } else {
      snprintf(buf, sizeof(buf), "Err MQTT: %d", mqttClient.state());
      setLabel(10, buf);
      vTaskDelay(pdMS_TO_TICKS(2000));
    }
  }
}

// Fully shuts down the Wi-Fi driver and frees ~45 KB of heap RAM.
// Called from loop() on triple click when wifi_enabled == true.
void shutdown_radio() {
  mqttClient.disconnect();
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  esp_wifi_stop();   // stop RF driver
  esp_wifi_deinit(); // free memory (~45 KB recovered)
  wifi_enabled = false;
  Serial.println("[WIFI] Driver deinitialized. RAM freed.");
}

// Powers the Wi-Fi driver back on and reconnects.
// WiFi.mode(WIFI_OFF) resets the internal WiFiClass state before
// connect_wifi(), ensuring WiFi.mode(WIFI_STA) reinitialises correctly.
void startup_radio() {
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  esp_wifi_init(&cfg);
  esp_wifi_start();
  wifi_enabled = true;
  WiFi.mode(WIFI_OFF); // reset WiFiClass internal state
  connect_wifi();     // WiFi.mode(WIFI_STA) now guaranteed to reinitialise
  if (wifi_enabled && mqtt_enabled && WiFi.status() == WL_CONNECTED) {
    mqttClient.connect(cfg_mqtt_client_id);
  }
  Serial.println("[WIFI] Driver reinitialized and connected.");
}
