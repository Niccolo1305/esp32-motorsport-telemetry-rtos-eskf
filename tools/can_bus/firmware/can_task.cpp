// SPDX-License-Identifier: GPL-3.0-or-later
// can_task.cpp — CAN bus acquisition task for Qashqai J10 ECU telemetry
//
// Runs on Core 0 at priority 2 in LISTEN_ONLY mode. Drains the TWAI RX
// queue, decodes known CAN IDs, and publishes a CanData snapshot under
// can_mutex for Task_Filter to read in Phase 2.
//
// Integration checklist:
//   1. Add to globals.h:
//        #include "can_task.h"
//        extern SemaphoreHandle_t can_mutex;
//        extern CanData shared_can_data;
//        extern TaskHandle_t TaskCANHandle;
//
//   2. Add to globals.cpp:
//        SemaphoreHandle_t can_mutex;
//        CanData shared_can_data;
//        TaskHandle_t TaskCANHandle;
//
//   3. Add to config.h:
//        #define CAN_TX_PIN  XX  // verify with Hub Grove pinout
//        #define CAN_RX_PIN  XX  // verify with Hub Grove pinout
//
//   4. Add to Telemetria.ino setup(), after SD init:
//        can_mutex = xSemaphoreCreateMutex();
//        twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
//            (gpio_num_t)CAN_TX_PIN, (gpio_num_t)CAN_RX_PIN,
//            TWAI_MODE_LISTEN_ONLY);
//        g_config.rx_queue_len = 32;
//        twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
//        twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
//        if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
//            if (twai_start() == ESP_OK) {
//                Serial.println("[CAN] TWAI started at 500 kbps (LISTEN_ONLY)");
//                xTaskCreatePinnedToCore(Task_CAN, "Task_CAN", 4096,
//                                        NULL, 2, &TaskCANHandle, 0);
//            }
//        }
//
//   5. In filter_task.cpp Phase 2 (after GPS snapshot, outside telemetry_mutex):
//        CanData local_can;
//        if (xSemaphoreTake(can_mutex, pdMS_TO_TICKS(2)) == pdTRUE) {
//            local_can = shared_can_data;
//            xSemaphoreGive(can_mutex);
//        }

#include "can_task.h"
#include "can_decode.h"
#include "driver/twai.h"
#include "globals.h"

void Task_CAN(void *pvParameters) {
    CanData data = {};
    twai_message_t msg;

    for (;;) {
        // Drain all available frames from the TWAI RX queue
        while (twai_receive(&msg, pdMS_TO_TICKS(5)) == ESP_OK) {
            uint64_t now_us = esp_timer_get_time();
            data.frames_received++;
            data.last_frame_us = now_us;
            data.bus_active = true;

            switch (msg.identifier) {

            case CAN_ID_RPM:
                if (msg.data_length_code >= 2) {
                    data.engine_rpm = ((msg.data[0] << 8) | msg.data[1]) / 4.0f;
                    data.rpm_us = now_us;
                }
                break;

            case CAN_ID_THROTTLE:
                if (msg.data_length_code >= 2) {
                    data.throttle_pct =
                        ((msg.data[0] << 8) | msg.data[1]) * 100.0f / 1024.0f;
                }
                break;

            case CAN_ID_TEMP:
                if (msg.data_length_code >= 2) {
                    data.coolant_temp_c    = msg.data[0] - 40.0f;
                    data.intake_air_temp_c = msg.data[1] - 40.0f;
                }
                break;

            case CAN_ID_WHEELS_FRONT:
                if (msg.data_length_code >= 4) {
                    data.wheel_fl_kmh =
                        ((msg.data[0] << 8) | msg.data[1]) * 0.01f;
                    data.wheel_fr_kmh =
                        ((msg.data[2] << 8) | msg.data[3]) * 0.01f;
                    data.wheels_us = now_us;
                }
                break;

            case CAN_ID_WHEELS_REAR:
                if (msg.data_length_code >= 4) {
                    data.wheel_rl_kmh =
                        ((msg.data[0] << 8) | msg.data[1]) * 0.01f;
                    data.wheel_rr_kmh =
                        ((msg.data[2] << 8) | msg.data[3]) * 0.01f;
                }
                break;

            case CAN_ID_STEERING:
                if (msg.data_length_code >= 2) {
                    data.steering_angle_deg =
                        (int16_t)((msg.data[0] << 8) | msg.data[1]) * 0.1f;
                    data.steering_us = now_us;
                }
                break;

            case CAN_ID_BRAKE:
                if (msg.data_length_code >= 1) {
                    data.brake_pressed = (msg.data[0] & 0x01) != 0;
                }
                break;

            default:
                break;
            }
        }

        // Bus timeout detection
        uint64_t now = esp_timer_get_time();
        if (now - data.last_frame_us > 500000) {  // 500 ms
            data.bus_active = false;
        }

        // Publish snapshot under mutex
        if (xSemaphoreTake(can_mutex, pdMS_TO_TICKS(2)) == pdTRUE) {
            shared_can_data = data;
            xSemaphoreGive(can_mutex);
        }

        vTaskDelay(pdMS_TO_TICKS(5));  // ~200 Hz polling to drain CAN bursts
    }
}
