// SPDX-License-Identifier: GPL-3.0-or-later
// sd_writer.h — FreeRTOS Task_SD_Writer (Core 1, priority 1)
#pragma once

#include <stddef.h>

bool create_next_log_segment(char *filename, size_t filename_size);
void Task_SD_Writer(void *pvParameters);
