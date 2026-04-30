// MPU6886 static bench logger for M5Stack AtomS3.
// Purpose-built sensor characterization firmware: no vehicle pipeline, no software
// filters, no runtime bias correction, no axis remap, no GPS, no MQTT.

#include <Arduino.h>
#include <FS.h>
#include <SD.h>
#include <SPI.h>
#include <Wire.h>
#include <esp_timer.h>

static constexpr char FW_VERSION[] = "mpu6886-lpf20-v0.2.0";

static constexpr int PIN_I2C_SDA = 38;
static constexpr int PIN_I2C_SCL = 39;
static constexpr uint32_t I2C_HZ = 400000;

static constexpr int PIN_SD_SCK = 7;
static constexpr int PIN_SD_MISO = 8;
static constexpr int PIN_SD_MOSI = 6;
static constexpr int PIN_SD_CS = 5;
static constexpr uint32_t SD_SPI_HZ = 25000000;

static constexpr uint8_t MPU_ADDR = 0x68;
static constexpr uint8_t REG_SMPLRT_DIV = 0x19;
static constexpr uint8_t REG_CONFIG = 0x1A;
static constexpr uint8_t REG_GYRO_CONFIG = 0x1B;
static constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;
static constexpr uint8_t REG_ACCEL_CONFIG2 = 0x1D;
static constexpr uint8_t REG_FIFO_EN = 0x23;
static constexpr uint8_t REG_INT_ENABLE = 0x38;
static constexpr uint8_t REG_INT_STATUS = 0x3A;
static constexpr uint8_t REG_USER_CTRL = 0x6A;
static constexpr uint8_t REG_PWR_MGMT_1 = 0x6B;
static constexpr uint8_t REG_PWR_MGMT_2 = 0x6C;
static constexpr uint8_t REG_FIFO_COUNTH = 0x72;
static constexpr uint8_t REG_FIFO_R_W = 0x74;
static constexpr uint8_t REG_WHO_AM_I = 0x75;

static constexpr uint8_t EXPECTED_WHO_AM_I = 0x19;

// 1 kHz FIFO source, decimated by dropping older frames and keeping the newest
// frame every 20 ms output period. This is not a software averaging filter.
static constexpr uint16_t RAW_ODR_HZ = 1000;
static constexpr uint16_t OUTPUT_ODR_HZ = 50;
static constexpr uint16_t DECIMATION_RATIO = RAW_ODR_HZ / OUTPUT_ODR_HZ;
static constexpr uint16_t SAMPLE_PERIOD_US = 1000000UL / OUTPUT_ODR_HZ;
static constexpr uint16_t FIFO_FRAME_SIZE = 14; // accel(6) + temp(2) + gyro(6)

static constexpr uint16_t ACCEL_RANGE_G = 8;
static constexpr uint16_t GYRO_RANGE_DPS = 2000;

static constexpr uint16_t SD_QUEUE_DEPTH = 300; // 6 s at 50 Hz, 256 B/record.
static constexpr uint16_t SD_FLUSH_EVERY_RECORDS = 250;
static constexpr uint32_t SD_REOPEN_DELAY_MS = 100;

static constexpr uint32_t RECORD_MAGIC = 0x4236384DUL; // "M86B" little-endian.
static constexpr uint16_t HEADER_SIZE = 256;
static constexpr uint16_t RECORD_SIZE = 256;

enum RecordFlags : uint32_t {
  FLAG_SAMPLE_FRESH = 1UL << 0,
  FLAG_FIFO_OVERRUN = 1UL << 1,
  FLAG_FIFO_MISALIGNED = 1UL << 2,
  FLAG_I2C_ERROR = 1UL << 3,
  FLAG_SD_QUEUE_DROP = 1UL << 4,
};

#pragma pack(push, 1)
struct LogHeader {
  char magic[8];
  uint16_t header_version;
  uint16_t header_size;
  uint16_t record_size;
  uint16_t endian_marker;
  uint32_t firmware_build;
  uint64_t log_start_us;
  char firmware_version[24];
  char sensor_type[16];
  char board[16];
  uint16_t raw_odr_hz;
  uint16_t output_odr_hz;
  uint16_t decimation_ratio;
  uint16_t fifo_frame_size;
  uint16_t accel_range_g;
  uint16_t gyro_range_dps;
  uint8_t reg_dump[128];
  uint8_t reserved[30];
  uint16_t crc16;
};

struct BenchRecord {
  uint32_t magic;
  uint32_t seq;
  uint64_t timestamp_us;
  uint16_t fifo_count_before;
  uint16_t fifo_count_after;
  uint16_t fifo_frames_drained;
  uint16_t decimation_counter;
  uint32_t flags;
  uint32_t read_error_count;
  int16_t acc_x;
  int16_t acc_y;
  int16_t acc_z;
  int16_t gyro_x;
  int16_t gyro_y;
  int16_t gyro_z;
  int16_t temp_raw;
  uint32_t sd_queue_high_watermark;
  uint32_t sd_records_written;
  uint32_t sd_records_dropped;
  uint32_t sd_partial_write_count;
  uint32_t sd_stall_count;
  uint32_t sd_reopen_count;
  uint32_t sd_flush_worst_us;
  uint32_t sd_records_enqueued;
  uint32_t sd_partial_current_record;
  uint32_t fifo_overrun_count;
  uint32_t fifo_reset_count;
  uint32_t sd_stall_worst_ms;
  uint16_t int_status;
  uint16_t sample_period_us;
  uint32_t imu_loop_worst_us;
  uint32_t sd_reopen_fail_count;
  uint32_t sd_write_zero_count;
  uint8_t reserved[144];
  uint16_t crc16;
};
#pragma pack(pop)

static_assert(sizeof(LogHeader) == HEADER_SIZE, "LogHeader must be 256 bytes");
static_assert(sizeof(BenchRecord) == RECORD_SIZE, "BenchRecord must be 256 bytes");

struct FifoFrame {
  int16_t acc_x = 0;
  int16_t acc_y = 0;
  int16_t acc_z = 0;
  int16_t temp_raw = 0;
  int16_t gyro_x = 0;
  int16_t gyro_y = 0;
  int16_t gyro_z = 0;
};

static QueueHandle_t sd_queue = nullptr;
static char log_filename[32] = {};

static volatile uint32_t sd_queue_high_watermark = 0;
static volatile uint32_t sd_records_written = 0;
static volatile uint32_t sd_records_dropped = 0;
static volatile uint32_t sd_partial_write_count = 0;
static volatile uint32_t sd_stall_count = 0;
static volatile uint32_t sd_reopen_count = 0;
static volatile uint32_t sd_flush_worst_us = 0;
static volatile uint32_t sd_records_enqueued = 0;
static volatile uint32_t sd_partial_current_record = 0;
static volatile uint32_t sd_stall_worst_ms = 0;
static volatile uint32_t sd_reopen_fail_count = 0;
static volatile uint32_t sd_write_zero_count = 0;

static volatile uint32_t read_error_count = 0;
static volatile uint32_t fifo_overrun_count = 0;
static volatile uint32_t fifo_reset_count = 0;
static volatile uint32_t imu_loop_worst_us = 0;

static uint16_t crc16_ccitt(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; ++i) {
    crc ^= (uint16_t)data[i] << 8;
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
  }
  return crc;
}

static int16_t be_i16(const uint8_t *p) {
  return (int16_t)((uint16_t)p[0] << 8 | p[1]);
}

static bool write_reg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

static bool read_regs(uint8_t reg, uint8_t *dst, size_t len) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  size_t got = Wire.requestFrom((int)MPU_ADDR, (int)len);
  if (got != len) {
    while (Wire.available()) (void)Wire.read();
    return false;
  }
  for (size_t i = 0; i < len; ++i) dst[i] = (uint8_t)Wire.read();
  return true;
}

static bool read_reg(uint8_t reg, uint8_t &value) {
  return read_regs(reg, &value, 1);
}

static bool read_fifo_count(uint16_t &count) {
  uint8_t b[2] = {};
  if (!read_regs(REG_FIFO_COUNTH, b, sizeof(b))) return false;
  count = ((uint16_t)b[0] << 8) | b[1];
  return true;
}

static bool read_fifo_bytes(uint8_t *dst, size_t len) {
  // Keep chunks below the ESP32 Wire buffer while still burst-reading FIFO.
  static constexpr size_t CHUNK_MAX = 112; // 8 complete 14-byte FIFO frames.
  size_t offset = 0;
  while (offset < len) {
    const size_t chunk = min(CHUNK_MAX, len - offset);
    if (!read_regs(REG_FIFO_R_W, dst + offset, chunk)) return false;
    offset += chunk;
  }
  return true;
}

static bool reset_fifo() {
  bool ok = write_reg(REG_FIFO_EN, 0x00);
  ok = write_reg(REG_USER_CTRL, 0x04) && ok; // FIFO_RST, auto-clears.
  delay(2);
  ok = write_reg(REG_USER_CTRL, 0x40) && ok; // FIFO_EN.
  ok = write_reg(REG_FIFO_EN, 0x18) && ok;   // gyro + accel; temp included.
  fifo_reset_count++;
  return ok;
}

static bool configure_mpu6886() {
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, I2C_HZ);
  delay(50);

  uint8_t who = 0;
  if (!read_reg(REG_WHO_AM_I, who) || who != EXPECTED_WHO_AM_I) {
    Serial.printf("[IMU] WHO_AM_I failed: 0x%02X, expected 0x%02X\n", who, EXPECTED_WHO_AM_I);
    return false;
  }

  bool ok = true;
  ok = write_reg(REG_PWR_MGMT_1, 0x80) && ok; // device reset.
  delay(100);
  ok = write_reg(REG_PWR_MGMT_1, 0x01) && ok; // awake, PLL clock.
  ok = write_reg(REG_PWR_MGMT_2, 0x00) && ok; // all accel/gyro axes on.
  delay(20);

  ok = write_reg(REG_FIFO_EN, 0x00) && ok;
  ok = write_reg(REG_USER_CTRL, 0x04) && ok;
  delay(2);

  ok = write_reg(REG_SMPLRT_DIV, 0x00) && ok;     // 1 kHz with DLPF 1..6.
  ok = write_reg(REG_CONFIG, 0x44) && ok;         // FIFO stop-on-full + gyro DLPF_CFG=4 (~20 Hz).
  ok = write_reg(REG_GYRO_CONFIG, 0x18) && ok;    // +/-2000 dps, FCHOICE_B=00.
  ok = write_reg(REG_ACCEL_CONFIG, 0x10) && ok;   // +/-8 g.
  ok = write_reg(REG_ACCEL_CONFIG2, 0x04) && ok;  // accel DLPF ~21.2 Hz, 1 kHz rate.
  ok = write_reg(REG_INT_ENABLE, 0x10) && ok;     // FIFO overflow interrupt flag.
  ok = write_reg(REG_USER_CTRL, 0x40) && ok;      // FIFO access enabled.
  ok = write_reg(REG_FIFO_EN, 0x18) && ok;        // accel + temp + gyro frames.

  if (!ok) {
    Serial.println("[IMU] register configuration failed");
    return false;
  }

  uint8_t regs[5] = {};
  read_reg(REG_CONFIG, regs[0]);
  read_reg(REG_GYRO_CONFIG, regs[1]);
  read_reg(REG_ACCEL_CONFIG, regs[2]);
  read_reg(REG_ACCEL_CONFIG2, regs[3]);
  read_reg(REG_FIFO_EN, regs[4]);
  Serial.printf("[IMU] MPU6886 WHO=0x%02X CONFIG=0x%02X GYRO=0x%02X ACCEL=0x%02X ACCEL2=0x%02X FIFO_EN=0x%02X\n",
                who, regs[0], regs[1], regs[2], regs[3], regs[4]);
  Serial.println("[IMU] FIFO frame: accel,temp,gyro = 14 bytes @ 1 kHz; HW LPF ~20 Hz; log output @ 50 Hz.");
  return true;
}

static void fill_register_dump(uint8_t dump[128]) {
  memset(dump, 0xFF, 128);
  const uint8_t safe_regs[] = {
      REG_SMPLRT_DIV, REG_CONFIG, REG_GYRO_CONFIG, REG_ACCEL_CONFIG,
      REG_ACCEL_CONFIG2, REG_FIFO_EN, REG_INT_ENABLE, REG_USER_CTRL,
      REG_PWR_MGMT_1, REG_PWR_MGMT_2, REG_FIFO_COUNTH, (uint8_t)(REG_FIFO_COUNTH + 1),
      REG_WHO_AM_I,
  };
  for (uint8_t reg : safe_regs) {
    uint8_t value = 0xFF;
    if (read_reg(reg, value)) dump[reg] = value;
  }
}

static void make_header(LogHeader &hdr) {
  memset(&hdr, 0, sizeof(hdr));
  memcpy(hdr.magic, "MPU6886B", 8);
  hdr.header_version = 1;
  hdr.header_size = sizeof(LogHeader);
  hdr.record_size = sizeof(BenchRecord);
  hdr.endian_marker = 0x1234;
  hdr.firmware_build = 1;
  hdr.log_start_us = esp_timer_get_time();
  strncpy(hdr.firmware_version, FW_VERSION, sizeof(hdr.firmware_version) - 1);
  strncpy(hdr.sensor_type, "MPU6886", sizeof(hdr.sensor_type) - 1);
  strncpy(hdr.board, "M5Stack AtomS3", sizeof(hdr.board) - 1);
  hdr.raw_odr_hz = RAW_ODR_HZ;
  hdr.output_odr_hz = OUTPUT_ODR_HZ;
  hdr.decimation_ratio = DECIMATION_RATIO;
  hdr.fifo_frame_size = FIFO_FRAME_SIZE;
  hdr.accel_range_g = ACCEL_RANGE_G;
  hdr.gyro_range_dps = GYRO_RANGE_DPS;
  fill_register_dump(hdr.reg_dump);
  hdr.crc16 = crc16_ccitt(reinterpret_cast<const uint8_t *>(&hdr), sizeof(hdr) - sizeof(hdr.crc16));
}

static bool write_all(File &file, const uint8_t *data, size_t len) {
  size_t offset = 0;
  while (offset < len) {
    const size_t written = file.write(data + offset, len - offset);
    if (written == 0) return false;
    offset += written;
  }
  return true;
}

static bool choose_log_filename() {
  for (int i = 0; i < 1000; ++i) {
    snprintf(log_filename, sizeof(log_filename), "/MPU6886_%03d.BIN", i);
    if (!SD.exists(log_filename)) return true;
  }
  return false;
}

static bool create_log_file() {
  if (!choose_log_filename()) {
    Serial.println("[SD] no free MPU6886_###.BIN name");
    return false;
  }

  LogHeader hdr;
  make_header(hdr);

  File file = SD.open(log_filename, FILE_WRITE);
  if (!file) {
    Serial.printf("[SD] cannot create %s\n", log_filename);
    return false;
  }
  const bool ok = write_all(file, reinterpret_cast<const uint8_t *>(&hdr), sizeof(hdr));
  file.flush();
  file.close();
  if (!ok) {
    Serial.println("[SD] failed to write complete header");
    return false;
  }
  Serial.printf("[SD] logging to %s header=%u record=%u\n", log_filename, HEADER_SIZE, RECORD_SIZE);
  return true;
}

static void fill_diag(BenchRecord &rec) {
  rec.sd_queue_high_watermark = sd_queue_high_watermark;
  rec.sd_records_written = sd_records_written;
  rec.sd_records_dropped = sd_records_dropped;
  rec.sd_partial_write_count = sd_partial_write_count;
  rec.sd_stall_count = sd_stall_count;
  rec.sd_reopen_count = sd_reopen_count;
  rec.sd_flush_worst_us = sd_flush_worst_us;
  rec.sd_records_enqueued = sd_records_enqueued;
  rec.sd_partial_current_record = sd_partial_current_record;
  rec.fifo_overrun_count = fifo_overrun_count;
  rec.fifo_reset_count = fifo_reset_count;
  rec.sd_stall_worst_ms = sd_stall_worst_ms;
  rec.imu_loop_worst_us = imu_loop_worst_us;
  rec.sd_reopen_fail_count = sd_reopen_fail_count;
  rec.sd_write_zero_count = sd_write_zero_count;
}

static void parse_fifo_frame(const uint8_t b[FIFO_FRAME_SIZE], FifoFrame &out) {
  out.acc_x = be_i16(&b[0]);
  out.acc_y = be_i16(&b[2]);
  out.acc_z = be_i16(&b[4]);
  out.temp_raw = be_i16(&b[6]);
  out.gyro_x = be_i16(&b[8]);
  out.gyro_y = be_i16(&b[10]);
  out.gyro_z = be_i16(&b[12]);
}

static bool drain_fifo_frames(uint16_t frames_to_read, FifoFrame &latest) {
  uint8_t buf[112];
  uint16_t remaining = frames_to_read;
  while (remaining > 0) {
    const uint16_t chunk_frames = min<uint16_t>(remaining, sizeof(buf) / FIFO_FRAME_SIZE);
    const size_t chunk_bytes = (size_t)chunk_frames * FIFO_FRAME_SIZE;
    if (!read_fifo_bytes(buf, chunk_bytes)) return false;
    for (uint16_t i = 0; i < chunk_frames; ++i) {
      parse_fifo_frame(&buf[i * FIFO_FRAME_SIZE], latest);
    }
    remaining -= chunk_frames;
  }
  return true;
}

static void TaskIMU(void *) {
  TickType_t last_wake = xTaskGetTickCount();
  FifoFrame latest;
  uint32_t seq = 0;

  for (;;) {
    const uint64_t loop_t0 = esp_timer_get_time();
    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1000 / OUTPUT_ODR_HZ));

    BenchRecord rec = {};
    rec.magic = RECORD_MAGIC;
    rec.seq = seq++;
    rec.timestamp_us = esp_timer_get_time();
    rec.sample_period_us = SAMPLE_PERIOD_US;
    rec.decimation_counter = 0;
    rec.read_error_count = read_error_count;

    uint8_t int_status = 0;
    if (!read_reg(REG_INT_STATUS, int_status)) {
      rec.flags |= FLAG_I2C_ERROR;
      read_error_count++;
    }
    rec.int_status = int_status;
    const bool fifo_overrun = (int_status & 0x10) != 0;
    if (fifo_overrun) {
      rec.flags |= FLAG_FIFO_OVERRUN;
      fifo_overrun_count++;
      reset_fifo();
    }

    uint16_t fifo_count_before = 0;
    if (!read_fifo_count(fifo_count_before)) {
      rec.flags |= FLAG_I2C_ERROR;
      read_error_count++;
    }
    rec.fifo_count_before = fifo_count_before;

    if (!fifo_overrun && fifo_count_before > 0) {
      const uint16_t aligned_bytes = fifo_count_before - (fifo_count_before % FIFO_FRAME_SIZE);
      const uint16_t frames = aligned_bytes / FIFO_FRAME_SIZE;

      if ((fifo_count_before % FIFO_FRAME_SIZE) != 0) {
        rec.flags |= FLAG_FIFO_MISALIGNED;
        read_error_count++;
      }

      if (frames > 0) {
        if (drain_fifo_frames(frames, latest)) {
          rec.flags |= FLAG_SAMPLE_FRESH;
          rec.fifo_frames_drained = frames;
          rec.decimation_counter = frames;
        } else {
          rec.flags |= FLAG_I2C_ERROR;
          read_error_count++;
          reset_fifo();
        }
      }

      if ((fifo_count_before % FIFO_FRAME_SIZE) != 0) {
        reset_fifo();
      }
    }

    uint16_t fifo_count_after = 0;
    if (read_fifo_count(fifo_count_after)) {
      rec.fifo_count_after = fifo_count_after;
    } else {
      rec.flags |= FLAG_I2C_ERROR;
      read_error_count++;
    }

    rec.acc_x = latest.acc_x;
    rec.acc_y = latest.acc_y;
    rec.acc_z = latest.acc_z;
    rec.gyro_x = latest.gyro_x;
    rec.gyro_y = latest.gyro_y;
    rec.gyro_z = latest.gyro_z;
    rec.temp_raw = latest.temp_raw;
    rec.read_error_count = read_error_count;
    fill_diag(rec);

    if (xQueueSend(sd_queue, &rec, 0) == pdTRUE) {
      sd_records_enqueued++;
      UBaseType_t pending = uxQueueMessagesWaiting(sd_queue);
      if ((uint32_t)pending > sd_queue_high_watermark) sd_queue_high_watermark = pending;
    } else {
      sd_records_dropped++;
      rec.flags |= FLAG_SD_QUEUE_DROP;
    }

    const uint32_t loop_us = (uint32_t)(esp_timer_get_time() - loop_t0);
    if (loop_us > imu_loop_worst_us) imu_loop_worst_us = loop_us;
  }
}

static bool reopen_append(File &file) {
  file.close();
  delay(SD_REOPEN_DELAY_MS);
  sd_reopen_count++;
  file = SD.open(log_filename, FILE_APPEND);
  if (!file) {
    sd_reopen_fail_count++;
    return false;
  }
  return true;
}

static void TaskSD(void *) {
  File file = SD.open(log_filename, FILE_APPEND);
  if (!file) {
    Serial.println("[SD] writer cannot open log file");
    vTaskDelete(nullptr);
  }

  uint16_t unsynced_records = 0;
  BenchRecord rec;

  for (;;) {
    if (xQueueReceive(sd_queue, &rec, portMAX_DELAY) != pdTRUE) continue;

    fill_diag(rec);
    rec.sd_records_written = sd_records_written + 1; // Count this record once it is fully accepted.
    rec.crc16 = crc16_ccitt(reinterpret_cast<const uint8_t *>(&rec), sizeof(rec) - sizeof(rec.crc16));

    const uint8_t *bytes = reinterpret_cast<const uint8_t *>(&rec);
    size_t offset = 0;
    uint32_t stall_start_ms = 0;
    sd_partial_current_record = 0;

    while (offset < sizeof(rec)) {
      if (!file) {
        while (!reopen_append(file)) {
          uint32_t now = millis();
          if (stall_start_ms == 0) stall_start_ms = now;
          const uint32_t stalled_ms = now - stall_start_ms;
          if (stalled_ms > sd_stall_worst_ms) sd_stall_worst_ms = stalled_ms;
        }
      }

      const size_t remaining = sizeof(rec) - offset;
      size_t written = file.write(bytes + offset, remaining);
      if (written > remaining) written = remaining;

      if (written > 0) {
        offset += written;
        stall_start_ms = 0;
        if (offset < sizeof(rec)) {
          sd_partial_write_count++;
          sd_partial_current_record = 1;
        }
        continue;
      }

      sd_write_zero_count++;
      sd_stall_count++;
      uint32_t now = millis();
      if (stall_start_ms == 0) stall_start_ms = now;
      const uint32_t stalled_ms = now - stall_start_ms;
      if (stalled_ms > sd_stall_worst_ms) sd_stall_worst_ms = stalled_ms;
      reopen_append(file);
    }

    sd_partial_current_record = 0;
    sd_records_written++;
    unsynced_records++;

    if (unsynced_records >= SD_FLUSH_EVERY_RECORDS) {
      const uint64_t t0 = esp_timer_get_time();
      file.flush();
      const uint32_t dt_us = (uint32_t)min<uint64_t>(UINT32_MAX, esp_timer_get_time() - t0);
      if (dt_us > sd_flush_worst_us) sd_flush_worst_us = dt_us;
      unsynced_records = 0;
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println();
  Serial.printf("[BOOT] %s\n", FW_VERSION);

  if (!configure_mpu6886()) {
    Serial.println("[FATAL] MPU6886 init failed");
    while (true) delay(1000);
  }

  SPI.begin(PIN_SD_SCK, PIN_SD_MISO, PIN_SD_MOSI, PIN_SD_CS);
  if (!SD.begin(PIN_SD_CS, SPI, SD_SPI_HZ)) {
    Serial.println("[FATAL] SD mount failed");
    while (true) delay(1000);
  }

  if (!create_log_file()) {
    Serial.println("[FATAL] log file creation failed");
    while (true) delay(1000);
  }

  sd_queue = xQueueCreate(SD_QUEUE_DEPTH, sizeof(BenchRecord));
  if (!sd_queue) {
    Serial.println("[FATAL] SD queue allocation failed");
    while (true) delay(1000);
  }

  if (!reset_fifo()) {
    Serial.println("[FATAL] FIFO reset failed");
    while (true) delay(1000);
  }

  xTaskCreatePinnedToCore(TaskSD, "Task_SD", 8192, nullptr, 2, nullptr, 1);
  xTaskCreatePinnedToCore(TaskIMU, "Task_IMU", 8192, nullptr, 5, nullptr, 0);
  Serial.println("[RUN] static bench logger started");
}

void loop() {
  static uint32_t last_ms = 0;
  const uint32_t now = millis();
  if (now - last_ms >= 1000) {
    last_ms = now;
    Serial.printf("[STAT] written=%lu enq=%lu drop=%lu q_hwm=%lu fifo_ovr=%lu read_err=%lu partial=%lu stall=%lu flush_worst_us=%lu\n",
                  (unsigned long)sd_records_written,
                  (unsigned long)sd_records_enqueued,
                  (unsigned long)sd_records_dropped,
                  (unsigned long)sd_queue_high_watermark,
                  (unsigned long)fifo_overrun_count,
                  (unsigned long)read_error_count,
                  (unsigned long)sd_partial_write_count,
                  (unsigned long)sd_stall_count,
                  (unsigned long)sd_flush_worst_us);
  }
  delay(50);
}
