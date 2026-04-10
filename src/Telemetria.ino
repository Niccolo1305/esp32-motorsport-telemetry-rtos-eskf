// SPDX-License-Identifier: GPL-3.0-or-later
// ================================================================================
// VEHICLE TELEMETRY — C++ for M5Stack Atom S3 (ESP32-S3)
// FreeRTOS Multi-Core Architecture (Zero Jitter) and Q_rsqrt
// ================================================================================
//
//
// ==============================================================================================
// CONFIGURATION MANIFESTO (v0.9.7) - Tuning Variables, Calibration and Constants
// ==============================================================================================
//
// --- 1. HARDWARE CALIBRATION AND MATRICES (telemetria.ino) ---
// VARIABLE                     VALUE           DESCRIPTION
// CALIB_W[3][3]                {...}           Soft-Iron / Scale Factor compensation. Corrects ellipsoidal distortion of the sensor.
// CALIB_B[3]                   {...}           Hard-Iron / Zero-Rate Bias compensation. Corrects fixed offset error.
//
// --- 2. TIMING AND HARDWARE (telemetria.ino) ---
// VARIABLE                     VALUE           DESCRIPTION
// FREQ_HZ                      50.0f           Core system sampling frequency (IMU + ESKF).
// DT                           0.02f           Delta Time in seconds (1 / FREQ_HZ).
// DT_MS                        20              Delta Time in milliseconds for FreeRTOS task delay.
// CALIB_SAMPLES               100             Number of samples for initial static calibration (2 sec).
// GPS_BAUD                     115200          Baud rate for serial communication with the ATGM336H-6N module.
// SD_FLUSH_EVERY               50              Samples between one flush() and the next (1s at 50Hz), in Task_SD_Writer.
//
// --- 3. NETWORK AND DATA TRANSMISSION (telemetria.ino) ---
// VARIABLE                     VALUE           DESCRIPTION
// wifi_networks[]              {...}           SSID and Password credentials loaded at boot from /wifi_config.txt on SD.
// cfg_mqtt_broker              "..."           MQTT broker IP or hostname (loaded from config file, disabled if absent).
// cfg_mqtt_port                1883            Standard MQTT port (unencrypted).
// cfg_mqtt_topic               "..."           Topic where the telemetry JSON payload is published.
//
// --- 4. SIGNAL FILTERS (telemetria.ino) ---
// VARIABLE                     VALUE           DESCRIPTION
// beta (Madgwick)              0.1f            Madgwick AHRS gain. Adaptive: decays to 0 under G-forces in cornering.
// alpha (EMA)                  0.06f           Single EMA filter on all accel + gyro axes (tau ~313ms at 50Hz).
//                                              Does NOT feed the ESKF (Data Fork): used only for display/MQTT/SD/ZARU.
//
// --- 5. ZARU STATISTICAL ENGINE (telemetria.ino) ---
// VARIABLE                     VALUE           DESCRIPTION
// VAR_BUF_SIZE                 50              Variance sliding window size (1 sec at 50Hz).
// VAR_STILLNESS_THRESHOLD             0.05f           [(deg/s)^2] var_gz threshold below which the vehicle is "stationary".
// ZUPT_GPS_MAX_KMH             2.0f            [km/h] Threshold below which GPS agrees vehicle is stationary.
//
// --- 6. PHYSICAL AND GEOGRAPHICAL CONSTANTS (eskf.h) ---
// VARIABLE                     VALUE           DESCRIPTION
// G_ACCEL                      9.80665f        [m/s^2] Multiplier to convert IMU G values to m/s^2.
// EARTH_RADIUS_M               6371000.0       [m] Mean Earth radius for spherical WGS84->ENU projection.
//
// --- 7. KALMAN FILTER TUNING (eskf.h) ---
// VARIABLE                     VALUE           DESCRIPTION
// Qd_(2,2) / Qd_(3,3)         1.0e-2f         Accel Process Covariance. Uncertainty on integration of a_x and a_y.
// Qd_(4,4)                     5.0e-4f         Gyro Process Covariance. Uncertainty on integration of g_z.
// R_base                       0.05f           [m^2] GPS covariance floor. R_dyn = 0.05 * HDOP^2 (v0.9.6).
//
// --- 8. KALMAN SEQUENTIAL UPDATE THRESHOLDS (eskf.h) ---
// VARIABLE                     VALUE           DESCRIPTION
// R_v                          1.0f            [(m/s)^2] Uncertainty on GPS velocity in Update 2.
// (vel trigger threshold)      > 5.0 km/h      Minimum GPS speed to trigger velocity correction (Update 2).
// sigma_pos                    1.2f            [m] Estimated mean GPS positional error (R_th calculation).
// R_th (Min / Max)             0.005f / 0.3f   [rad^2] Clamp of adaptive uncertainty on GPS angle (Update 3).
// (angle trigger threshold)    > 5.0 km/h      Minimum GPS speed to trigger heading correction (Update 3).
//
//
// ==============================================================================================
// IMU PIPELINE — Data flow in Task_Filter (Core 1, 50Hz)
// ==============================================================================================
//
// Raw ImuRawData from xQueueOverwrite queue (Task_I2C, Core 0)
//   |
//   | STEP 1: Ellipsoidal Calibration (native chip frame)
//   |   a_cal = CALIB_W * (a_raw - CALIB_B)
//   |   Corrects hard-iron (-0.065g on Z) and soft-iron (cross-axis < 0.002).
//   |   Sigma(||a||) from 0.042g to 0.023g.
//   |
//   | STEP 2: Geometric Alignment (chip frame -> vehicle frame)
//   |   rotate_3d(a_cal, cos_phi, sin_phi, cos_theta, sin_theta)
//   |   phi/theta angles computed in calibrate_alignment() at boot (2s, 100 samples).
//   |
//   | STEP 3: Mounting Bias (residuals post-ellipsoid)
//   |   a_r = a_r_raw - bias_a{x,y,z}
//   |   Mean of 100 calibrate_alignment() samples = mechanical zero.
//   |
//   | STEP 4: Gyroscope — static bias + vehicle frame rotation
//   |   g_clean = g_raw - bias_g{x,y,z}    (static bias from boot)
//   |   rotate_3d(g_clean, ...) -> g_r       (same frame as STEP 2)
//   |   g_rad = g_r * DEG2RAD
//   |
//   | STEP 5: Madgwick AHRS (quaternion, adaptive beta)
//   |   ahrs.update_imu(gx_rad, gy_rad, gz_rad, ax_r, ay_r, az_r)
//   |   Linear beta: full when ||a|| ~ 1g, zero when deviation > 0.15g.
//   |   Output: quaternion q[4] representing 3D orientation.
//   |
//   | STEP 6: Gravity Cancellation via Quaternion
//   |   ahrs.get_gravity_vector(grav_x, grav_y, grav_z)
//   |   lin_a{x,y,z} = a_r - grav  [g, gravity removed]
//   |
//   +----------- DATA FORK -----------+
//   |                                 |
//   | (RAW, zero latency)             | (EMA, tau ~313ms)
//   |                                 |
//   | STEP 7: EMA alpha=0.06          |
//   | ema_a = alpha*lin_a+(1-a)*prev  | -> Display 5Hz
//   | ema_g = alpha*g_r+(1-a)*prev    | -> MQTT 10Hz
//   |                                 | -> SD Log 50Hz
//   | STEP 8: Statistical Engine      |
//   | Circular buffer 50 samples ema_gz |
//   | var_gz = E[x^2] - E[x]^2       |
//   | mean_gz = buffer mean           |
//   |                                 |
//   | STEP 9: Stillness Condition     |
//   | is_stationary = (var_gz < 0.05) |
//   |       AND (gps_speed < 2 km/h)  |
//   |                                 |
//   | STEP 10: Adaptive ZARU          |
//   | If is_stationary:               |
//   |   thermal_bias_gz=mean_gz[deg/s]|
//   |                                 |
//   +---------------------------------+
//   |
//   | STEP 11: ESKF Predict (raw post-Madgwick data)
//   |   gz_eskf = gz_rad - thermal_bias_gz * DEG2RAD
//   |   eskf.predict(lin_ax, lin_ay, gz_eskf, dt, is_stationary)
//   |   -> Integrates position and velocity in ENU frame at 50Hz
//   |   -> Internal ZUPT: if is_stationary, forces vx=vy=0
//   |
//   | STEP 12: ESKF Correct (on fresh GPS fix, ~10Hz)
//   |   WGS84 -> ENU (wgs84_to_enu)
//   |   eskf.correct(east, north, speed_kmh, hdop)
//   |   -> UPDATE 1: 2D Position (always active)
//   |   -> UPDATE 2: Scalar velocity (if > 5 km/h)
//   |   -> UPDATE 3: Course Over Ground (if > 5 km/h and dist > 1m)
//   |   -> Dynamic R: R_dyn = 0.05 * HDOP^2
//   |
// ==============================================================================================
//
// ==========================================================
// --- PATCH NOTES ---
// ==========================================================
//
// v0.1.0 — MicroPython → C++ Conversion (baseline)
//   - Full port from MicroPython to C++ Arduino/ESP-IDF
//   - MPU-6886 IMU reading via M5Unified
//   - Basic pipeline: read → EMA filter → MQTT JSON at 10 Hz
//
// v0.2.0 — FreeRTOS Multi-Core Architecture
//   - Task_I2C on Core 0 (priority 3): IMU read every 20 ms with vTaskDelayUntil
//   - Task_Filter on Core 1 (priority 2): full math pipeline
//   - Inter-task communication via xQueueOverwrite (depth 1, always fresh data)
//   - telemetry_mutex to protect shared_telemetry between Task_Filter and loop()
//
// v0.2.1 — Critical fixes and optimisations
//   - Linear adaptive Madgwick beta (fixed accel cutoff error):
//     old on/off version with 0.3 g threshold was wrong (Pythagoras:
//     0.3 g lateral → only 0.044 g deviation from norm, not 0.3 g)
//     new version: beta decays linearly from 0 to 0.15 g distance from 1 g
//   - fast_inv_sqrt (Quake III) with UB-safe memcpy, 2 Newton-Raphson iterations
//   - Fix buf[128] (was 80, JSON worst-case ~87 chars → silent truncation)
//   - Fix DEBOUNCE_CYCLES calibrated on LOOP_HZ not FREQ_HZ (correct 200 ms)
//   - Multi-SSID Wi-Fi with deterministic scan (home + hotspot)
//   - ZARU: raw gyro bias update after 3 s of continuous stillness
//
// v0.3.0 — Ellipsoidal Calibration (Tumble Test)
//   - Ellipsoid fitting on 16,750 samples (tumble_test_visivo.csv)
//   - σ(‖a‖) reduction: 0.042 g → 0.023 g (−45.6 %)
//   - Hard-iron offset: b = [-0.00125, +0.00429, -0.06491] g
//     AZ bias = -0.065 g was the root cause of the anomaly found in the pendulum test
//   - Soft-iron matrix W 3×3 (off-diagonal < 0.002, excellent MEMS quality)
//   - apply_ellipsoidal_calibration() applied BEFORE rotate_3d()
//     and BEFORE calibrate_alignment() for accurate phi/theta angles
//   - EMA α restored to 0.06 f (production) from 0.20 f (testing)
//
// v0.4.0 — SD Card Logging + atomic system_state
//   - Micro-SD logging over SPI (TF Card Reader base)
//   - struct TelemetryRecord (CSV, 30 bytes) with 200-element FreeRTOS queue
//   - Task_SD_Writer on Core 1 (priority 1, async background)
//   - SPI.begin() and SD.begin() in setup() single-thread (no race condition)
//   - Sequential file naming /tel_N.csv with snprintf (no heap fragmentation)
//   - system_state converted to std::atomic<int> (formal C++11 correctness)
//   - Guard sd_queue != NULL: missing SD does not block the system
//   - lap_snap captured inside the mutex; xQueueSend called outside the lock
//   - Timestamp from data.timestamp_us (IMU hardware clock) instead of millis()
//
// v0.5.0 — Binary Logging + IMU Temperature
//   - struct TelemetryRecord converted to packed binary format (33 bytes/sample)
//     __attribute__((packed)): zero padding, exact in-memory layout
//     vs CSV ~50 bytes → 34 % saving, 110 h logging at 50 Hz on 16 GB SD
//   - Added float temp_c field (MPU-6886 internal temperature)
//   - Task_SD_Writer: logFile.printf() → logFile.write() (direct RAM→SD copy)
//   - File extension .bin (no text header)
//   - Python read: struct.unpack('<I6ffB', chunk_33_byte)
//
// v0.6.0 — Hardware DLPF 20 Hz, Dynamic Wi-Fi, I2C Race Condition Fix
//   - Hardware DLPF forced to 20 Hz via I2C (reg 0x1A, 0x1D = 0x04):
//     engine/chassis vibrations suppressed in silicon before sampling.
//     Used Wire (AtomS3 internal bus) instead of Wire1 (wrong bus fix)
//   - IMU temperature moved to Task_I2C (critical I2C race condition fix):
//     getTemp() was called in Task_Filter outside the mutex, contending
//     with Task_I2C getAccelData/getGyroData on the same physical bus.
//     Fix: temp_c added to ImuRawData, read in the exclusive I2C context
//   - Dynamic Wi-Fi with triple click:
//     shutdown_radio(): esp_wifi_stop + deinit → frees ~45 KB RAM
//     startup_radio(): reinit + WiFi.mode(WIFI_OFF) reset
//     before connect_wifi() to guarantee correct reinitialisation
//   - Wi-Fi indicator on display (green/red circle in LCD corner)
//   - std::atomic<bool> wifi_enabled for thread-safe control between loop() and tasks
//   - Guard wifi_enabled in connect_wifi() and reconnect_network()
//
// v0.6.1 — Compilation fix: removed invalid M5Unified APIs
//   - Removed M5.Imu.setAccelFsr() and M5.Imu.setGyroFsr(): these methods
//     belong to the direct MPU6886 driver and do not exist in m5::IMU_Class
//     of M5Unified (error: 'has no member named setAccelFsr/setGyroFsr')
//   - ±8 g / ±2000 dps full-scale is already the M5.begin() default:
//     the two calls were redundant as well as incorrect
//   - DLPF block via Wire.beginTransmission(0x68) unchanged and works
//     independently of the full-scale setting
//   - Added Wire.endTransmission() check with diagnostic output
//
// v0.6.2 — Black display crash fix: explicit Wire.begin() + display first
//   - Root cause: M5Unified uses an internal protected I2C bus and leaves the
//     Wire object uninitialised. Wire.beginTransmission() on an uninitialised bus
//     caused an immediate ESP32 crash before the display was turned on,
//     resulting in a permanently black screen at boot.
//   - Fix: added Wire.begin(38, 39) (AtomS3 internal SDA/SCL pins) before any
//     Wire.beginTransmission(), to explicitly initialise the bus before use.
//   - Fix setup() order: display moved as first operation after M5.begin(),
//     before the DLPF block. Any future error will be visible on screen
//     instead of producing a black screen.
//   - Removed stale comment "try Wire1": the correct bus is Wire with
//     Wire.begin(38, 39); Wire1 is not needed.
//
// v0.7.0 — GPS Integration (TinyGPSPlus, Grove UART1 RX=2/TX=1)
//   - TinyGPSPlus library (Mikal Hart) for NMEA parsing from GPS module
//   - HardwareSerial gpsSerial(UART1) on Grove pins: RX=2, TX=1 at 9600 baud
//   - Task_GPS: Core 0, priority 2 (below Task_I2C p3), UART polling every 20 ms
//     Decodes NMEA sentences and updates shared_gps_data under gps_mutex
//   - GpsData struct with lat/lon (double WGS84), speed_kmh (float), sats (uint8)
//     and valid flag: Task_Filter and loop() always read coherent data
//   - TelemetryRecord extended: 33 → 54 bytes/sample
//     Added: double gps_lat (8 B), double gps_lon (8 B),
//            float gps_speed_kmh (4 B), uint8_t gps_sats (1 B)
//     Python read: struct.unpack('<I7fBddfB', chunk_54_byte)
//   - Display: third GPS row (y=50) in states 0 and 2
//     "Sats:N Spd:XXX" — "Sats:0 NO FIX" when GPS has no fix
//   - MQTT JSON expanded (buf 128→256 bytes):
//     added lat, lon, spd, sats to payload for live monitoring via MQTTX
//   - gps_mutex separate from telemetry_mutex: very short lock (struct copy 21 B)
//     to avoid interfering with the IMU cycle at 50 Hz
//
// v0.7.1 — "Null Island" fix and GPS display flickering (static mutex cache)
//   - Bug: if gps_mutex was held by Task_GPS at the IMU sampling instant,
//     the else branch wrote lat=0/lon=0 into the SD record. In Python/Kalman
//     traces this produced spurious points in the Gulf of Guinea (0°N 0°E)
//     every time Task_GPS and Task_Filter collided on the mutex (~rare but deterministic)
//   - Fix Task_Filter: GpsData last_gps declared static — retains the last valid
//     fix across IMU cycles. If the mutex is busy, the SD record contains data
//     from 20 ms ago, not zeros. The GPS updates at 1 Hz: for 49 out of 50 cycles
//     caching is the correct behaviour anyway.
//   - Display bug: GpsData gps_disp declared local (uninitialised) — if the mutex
//     was busy, gps_disp.valid was undefined → display flickered to "NO FIX" for
//     one frame on every collision.
//   - Fix display + MQTT: GpsData gps_disp and gps_snap declared static — if the
//     mutex is busy, display and MQTT show data from the previous cycle (200 ms
//     ago for display, 100 ms ago for MQTT) without flickering.
//   - Note: Python format remains '<I7fBddfB' = 54 bytes (unchanged from v0.7.0).
//     The external-analysis suggested format '<I7fIddfBB' = 58 bytes was based on
//     a struct with an extra gps_timestamp_ms field not present in the firmware.
//
// v0.7.2 — GPS hardware fix: swapped pins and correct baud rate (ATGM336H-6N)
//   - Identified via diagnostic "GPS Radar" firmware (baud+pin scanner):
//     the ATGM336H-6N module ships configured at 115200 baud instead of the
//     standard NMEA 9600 baud — undocumented in the module's public datasheet.
//   - Grove pins swapped relative to the standard M5Stack convention:
//     RX_PIN: 2 → 1 (module TX enters AtomS3 GPIO1)
//     TX_PIN: 1 → 2 (module RX exits AtomS3 GPIO2)
//     Confirmed empirically: at 9600/RX=2 = slow illegible characters,
//     at 38400/RX=1 = fast characters with repetitions,
//     at 115200/RX=1 = readable NMEA with $GN/$GP strings and
//     chip "antenna open #25" message.
//   - GPS_BAUD: 9600 → 115200
//   - No software logic changes: TinyGPSPlus transparently handles any baud rate
//     once the bytes arrive correctly.
//
// v0.7.3 — GPS Altitude (satellite Z-axis for future 3D ESKF)
//   - float gps_alt_m added to TelemetryRecord: 54 → 58 bytes/sample
//   - float alt_m added to GpsData (shared RAM under gps_mutex)
//   - Task_GPS: reads gps.altitude.meters() in the valid-fix branch
//   - Task_Filter: copies alt_m into SD record (same static anti-Null cache)
//   - MQTT JSON: "alt" field added to payload for live monitoring
//   - Python read: struct.unpack('<I7fBddffB', chunk_58_byte)
//
// v0.8.0 — Planar ESKF 2D (IMU 50 Hz + GPS 1 Hz fusion)
//   - New file eskf.h: ESKF2D header-only class using BasicLinearAlgebra
//     State X=[px, py, vx, vy, θ]ᵀ — prediction at 50 Hz, correction at 1 Hz
//     R and Qd matrices from empirical System Identification on static data
//   - WGS84→ENU conversion: first valid GPS coordinate = origin (0, 0)
//     Spherical formula with mean Earth radius, error <0.1 % within 10 km
//   - GpsData.epoch: monotonic counter to detect a fresh fix without a flag
//   - TelemetryRecord: 58 → 74 bytes (+4 floats: kf_x, kf_y, kf_vel, kf_heading)
//   - Task_Filter STEP 9: predict() at 50 Hz with post-Madgwick linear
//     accelerations converted g→m/s²; correct() at 1 Hz on new GPS ENU fix
//   - MQTT JSON: kfx, kfy, kfv, kfh fields for live monitoring
//   - Python read: struct.unpack('<I7fBddffB4f', chunk_74_byte)
//
// v0.9.0 — Data Fork, Statistical ZUPT/ZARU Engine, ESKF API update
//   - DATA FORK (phase-delay resolution):
//     The ESKF receives raw post-Madgwick IMU data (lin_ax, lin_ay, gz_rad)
//     at zero latency. EMA data (ema_ax, ema_ay, ema_gz) is used exclusively
//     for display, MQTT, and SD logging. EMA never enters the Kalman filter.
//   - STATISTICAL ENGINE (circular-buffer variance):
//     Circular buffer of 50 samples (1 s at 50 Hz) of ema_gz (EMA-filtered).
//     Warm-up: variance not computed until the buffer is full.
//     is_stationary = (variance < threshold) AND (GPS speed < 2 km/h).
//   - ADAPTIVE ZARU (thermal bias learning):
//     When is_stationary == true, the ema_gz buffer mean becomes the new
//     thermal_bias_gz. This value is subtracted from gz_rad before feeding
//     the ESKF: thermal drift is learned at standstill and compensated in motion.
//   - ESKF API: predict() now accepts bool is_stationary. ZUPT is governed
//     externally by statistical variance, no longer by hardcoded acceleration
//     thresholds.
//   - Removed old fixed-threshold ZARU (stillness_counter, ACCEL_THRESHOLD,
//     GYRO_THRESHOLD, ZARU_3S_CYCLES).
//   - Gimbal Lock documentation added to eskf.h.
//
// v0.9.1 — Fix calibrate_alignment(): buffer safety and mutex comment
//   - samples_ax/ay/az/gx/gy/gz arrays now sized with CALIB_SAMPLES (constexpr)
//     instead of hardcoded [100]: if FREQ_HZ changes, arrays resize automatically
//     without stack buffer overflow.
//   - static_assert(CALIB_SAMPLES <= 200) as stack-overflow guard.
//   - Corrected mutex comment: the lock covers only the trigonometric calculations
//     (~50 µs), not the 2 s of I2C sampling that happens without the mutex.
//
// v0.9.2 — Pre-Flight Audit Fixes: ESKF race condition, I2C contention, variance
//   - CRITICAL FIX #1 (ESKF Race Condition):
//     loop() read eskf.px()/py()/speed_ms()/heading() directly from the global
//     object without a mutex in the MQTT payload, while Task_Filter updated
//     predict()/correct() at 50 Hz. Task_Filter preemption (priority 2) over
//     loop() (priority 1) could interleave values from different timesteps into
//     the same JSON payload.
//     Fix: 4 ESKF fields added to FilteredTelemetry (kf_x, kf_y, kf_vel,
//     kf_heading), copied under telemetry_mutex after predict/correct.
//     loop() now reads from the thread-safe local copy, not the global object.
//   - CRITICAL FIX #2 (I2C Bus Contention):
//     calibrate_alignment() called M5.Imu.getAccelData/getGyroData from
//     loop() (Core 1) while Task_I2C (Core 0) made the same calls at 50 Hz
//     on the same physical bus. Two cores on I2C without a lock = interleaved
//     reads → corrupted calibration → wrong biases for the entire session.
//     Fix: vTaskSuspend(TaskI2CHandle) before the sampling loop,
//     vTaskResume(TaskI2CHandle) immediately after. Bus is exclusive for 2 s.
//   - NOTICE FIX #9 (floating-point variance):
//     Formula E[x²]-E[x]² can yield slightly negative results due to
//     numerical cancellation when samples are nearly identical.
//     Fix: fmaxf(0.0f, var_gz) clamped to zero.
//
// v0.9.3 — GPS-ESKF Discrepancy & Kinematic Drift Fix (Spiral)
//   Problem: v0.8.0 correct() used only position [px, py] as observable →
//   P(pos,vel) cross-covariance too small to transfer information to velocity
//   → kf_vel underestimated (~30 %), heading free to drift → spiral trajectory.
//
//   Root cause analysed on tel_21.bin:
//     • mean kf_vel: 3.55 m/s vs GPS 5.26 m/s (−32 %)
//     • kf_heading: monotonic drift, does not follow GPS curves
//     • KF trajectory: tight spiral vs actual straight path
//
//   Fix — 3-stage Sequential Kalman Update:
//     UPDATE 1 (2D Position): identical to v0.8.0, corrects [px, py]
//       H = [I₂ 0], z = [east, north]
//     UPDATE 2 (1D scalar speed): new, closes the velocity loop
//       h(X) = ‖[vx, vy]‖, z = gps_speed [m/s]
//       H_v = [0, 0, vx/‖v‖, vy/‖v‖, 0] — non-linear Jacobian
//       Active only above 5 km/h (below that GPS noise dominates)
//       R_v = 1.0 m²/s²
//     UPDATE 3 (1D Course Over Ground): new, pseudo-magnetometer
//       θ_cog = atan2(Δnorth, Δeast) from GPS trail (previous → current fix)
//       Active only above 10 km/h and distance > 1 m
//       Adaptive R_th: σ²_pos / dist², clamped [0.005, 0.300]
//
//   Qd_ tuning (process covariance):
//     accel: 1.05e-7 / 8.75e-8 → 1.0e-2 (P no longer collapses)
//     gyro:  6.0e-10 → 5.0e-4 (heading follows curves)
//
//   Each stage calls symmetrize_P() for numerical stability.
//
// v0.9.4 — Hardware GPS Overclock (10 Hz) & Sawtooth Fix
//   Diagnosis (tel_23.bin): the "Sawtooth Effect" on the ESKF trajectory was
//   caused by along-track error accumulating during IMU dead-reckoning.
//   Pure IMU integration only covered 93 % of the real distance, and at 1 Hz
//   GPS correction the error built up for ~1.16 s before being cancelled
//   with a sharp jump (sawtooth tooth).
//
//   Hardware fix: the AT6668 GNSS chip (Zhongke/CASIC) supports up to 10 Hz.
//   CASIC command $PCAS02,100*1E sends a fix every 100 ms instead of 1000 ms.
//   Pure IMU dead-reckoning drops from ~1160 ms to ~100 ms: along-track error
//   is reduced by an order of magnitude, physically eliminating the sawtooth
//   before it can accumulate.
//   LNA MAX2659 pre-amplified antenna: guarantees sufficient CNR for 10 Hz.
//
//   5 Hz fallback (if 10 Hz saturates the NMEA parser or GPS queue):
//   // Serial2.print("$PCAS02,200*1D\r\n");
//
// v0.9.5 — Wi-Fi Hardware Bypass on BtnA + HDOP + Removed Alpha Menu
//   - Wi-Fi Bypass: pressing BtnA during the Wi-Fi connection loop in setup()
//     aborts the attempt. The system runs WiFi.disconnect(true) + WiFi.mode(WIFI_OFF),
//     disabling the radio (thermal/power saving). Firmware continues in SD-only offline mode.
//   - HDOP: added float gps_hdop to GpsData and TelemetryRecord (74 → 78 bytes).
//     Read from gps.hdop.hdop() in Task_GPS. Tracks geometric GPS fix quality
//     for precision diagnostics (1.0 = ideal, >5.0 = poor).
//   - Removed interactive ALPHA menu (system_state == 3): EMA filter tuning via
//     tilt/BtnA was removed. Alpha constants remain hardcoded at 0.06 f.
//     Single BtnA click is now free for other use.
//     Rationale: the ALPHA menu was impractical and added no real value to the demo;
//     filter tuning was done from real data and does not require runtime adjustment
//     at this stage of development.
//
// v0.9.6 — Dynamic R Matrix based on HDOP
//   - R_dyn = 0.05 * HDOP² injected into correct() before the Kalman updates.
//   - Signature becomes correct(east, north, speed, hdop): the gps_hdop field
//     acquired in v0.9.5 is now propagated from Task_Filter to the ESKF.
//   - Motivation: analysis of the v0.9.5 static dataset at 10 Hz (tel_25) revealed
//     that the AT6668 receiver with LNA MAX2659 is limited by NMEA 6-decimal
//     quantisation (~11 cm/digit at 45°N). The actual base covariance is excellent
//     (σ ≈ 0.22 m → R_base = 0.05 m²). The old R = diag(1.35, 1.40) penalised
//     the filter with 6× more uncertainty than the real data.
//   - R_base initialised to 0.05 m² in eskf.h (no longer 1.35/1.40).
//   - Guard: HDOP < 0.5 or > 50 → fallback HDOP = 1.0 (conservative).
//   - Expected behaviour: open sky (HDOP ~1.0) → R ≈ 0.05 m², filter follows
//     GPS with sub-metre accuracy. Covered areas (HDOP > 3.0) →
//     R ≈ 0.45+ m², ESKF trusts the IMU vector smoothness at 50 Hz more than GPS,
//     eliminating lateral jolts.
//
// v0.9.7 — COG threshold 5 km/h, Straight-line ZARU, Innovation Gate, SD FileHeader
//   - UPDATE 3 (COG): threshold lowered from 10 km/h to 5 km/h.
//     Adaptive R_th (sigma_pos²/dist²) handles COG noise at low speed:
//     small inter-fix distance → high R_th → gentle correction.
//     Fixes spikes/artefacts in low-speed cornering (issue #3).
//   - Straight-line ZARU: when the vehicle is on a fast straight
//     (speed > 20 km/h AND |ema_ay| < 0.02 g AND |ema_gz| < 2 deg/s),
//     the ema_gz residual is the current thermal bias. Updated via slow EMA
//     (alpha=0.01, convergence ~5 s at 50 Hz) to filter micro steering
//     corrections. Captures thermal drift in motion without touching the ESKF.
//   - Innovation Gate (Mahalanobis) in UPDATE 1 (position):
//     d² = yᵀ · S⁻¹ · y. If d² > 11.83 (χ²(2, 0.997) at 99.7 %),
//     the GPS fix is statistically incompatible with the prediction.
//     Instead of discarding the fix, R is inflated ×50 to dampen the correction.
//     Prevents spikes from GPS glitch/stall (issue #4).
//   - Binary SD FileHeader (25 bytes): written at the start of every .bin file.
//     Magic "TEL" + header_version + firmware_version[16] + record_size +
//     start_time_ms. The Python converter auto-detects the header and uses
//     record_size to parse records (backwards-compatible with legacy files).
//   - TelemetryRecord unchanged: 78 bytes/sample.
//   - Python read: struct.unpack('<I7fBddffBf4f', chunk_78_byte) unchanged.
//
// v0.9.8 — Shadow Mode 6D + Raw IMU Logging
//   - New ESKF_6D class in eskf.h: augmented state X=[px,py,vx,vy,theta,b_gz].
//     The sixth state (b_gz) models gyro bias as a random walk (Qd=1e-7).
//     predict() subtracts X(5) internally: receives raw gz_rad, no double subtraction.
//     correct_bias(): Kalman observation of bias at standstill (H=[0,0,0,0,0,1]).
//   - Shadow Mode: ESKF_6D runs in parallel with the 5D on the same IMU+GPS data.
//     The 5D remains the primary filter (display, MQTT). The 6D is SD-log only
//     for offline A/B testing. Zero impact on existing data flow.
//   - Raw IMU logging: 6 post-Madgwick channels without EMA (lin_ax/ay/az, gx_r/gy_r/gz_r)
//     added to SD record for offline pipeline analysis without EMA phase delay.
//   - TelemetryRecord: 78 → 122 bytes/sample (+44 bytes: 6 raw + 5 shadow 6D).
//     Python read: struct.unpack('<I7fBddffBf4f6f5f', chunk_122_byte).
//   - FileHeader.record_size now 122 (Python converter uses this field).
//
// v0.9.8-hotfix — Audit: Joseph Form, θ While, GPS Buffer, Cleanup
//   - Joseph Form for all Kalman updates (ESKF2D + ESKF_6D):
//     the previous form P = (I-KH)·P is numerically fragile on float32 because
//     rounding errors accumulate unilaterally with each correction. In long sessions
//     (>1 h at 50 Hz = 180 k steps) matrix P can lose positive semi-definiteness
//     and diverge. Joseph Form P = (I-KH)·P·(I-KH)ᵀ + K·R·Kᵀ is structurally PSD
//     by construction: the K·R·Kᵀ term (outer product scaled by measurement
//     covariance) always adds a PSD matrix that compensates the numerical loss of the
//     triple product. The extra cost is negligible (~0.1 % Flash) compared to the
//     stability guarantee. Applied to all 8 updates (4 per class).
//   - Post-update theta normalisation: if/else if → while.
//     With K·y potentially large (e.g. after long GPS-free dead-reckoning), a single
//     if does not normalise theta outside [-3π, 3π]. The while converges in 1-2
//     iterations in the normal case and correctly handles the degenerate case.
//     predict() keeps if because gz_rad·dt << 2π by definition.
//   - GPS UART buffer: 256 → 512 bytes. At 115200 baud and 10 Hz fix rate the AT6668
//     chip transmits ~230 bytes of NMEA sentences every 100 ms. Task_GPS drains the
//     buffer every 20 ms, but under CPU load (e.g. ESKF 6D + SD write burst) polling
//     can slip to 40 ms, accumulating ~460 bytes. The 256-byte buffer caused silent
//     overflow with lost sentences → sporadic missing GPS fixes.
//     512 bytes gives 2× margin. With ~320 KB of available RAM (15.2 % used),
//     1024 bytes would be a zero-cost future upgrade if extra NMEA sentences are added
//     (GSA, GSV for SNR).
//   - alpha: float → const float. The runtime tuning menu was removed in v0.9.5
//     but the variable remained mutable. The compiler can now propagate the constant
//     and optimise the 12 FMA operations of the EMA.
//   - Renamed CSV field 'ay_r_raw' → 'temp_c' in the Python converter.
//     The field has been the MPU-6886 temperature since v0.5.0; the name was a
//     fossil from v0.4 when that position held ay_r_raw.
//   - Fixed stale comments: COG 10→5 km/h in ASCII pipeline diagram,
//     SD Writer 78→122 bytes/record.
//
// v0.9.9 — ZUPT Sanity Gate + MoTeC Exporter
//   - Sanity Gate in is_stationary condition: added criterion
//     fabsf(mean_gz) < 2.5 deg/s in addition to var_gz and GPS speed.
//     Prevents false ZUPT/ZARU triggers during slow rotational manoeuvres
//     (1-2 km/h) where variance stays low (constant rotation) and GPS confirms
//     low speed. Without the gate, ZARU overwrote thermal_bias_gz with the actual
//     manoeuvre rate, corrupting the estimate for the entire subsequent session.
//     The 2.5 deg/s threshold is derived from the MPU-6886 maximum thermal drift
//     (30 °C × 0.032°/s/°C ≈ 1°/s) with a 2.5× margin. The gate protects both
//     the 5D ZARU and the 6D correct_bias().
//   - New script tools/motec_exporter.py: converts .bin/.csv telemetry to MoTeC
//     i2 Pro CSV format. 13 channels (8@50 Hz IMU/ESKF + 4@10 Hz GPS + 1@50 Hz
//     temp). Supports multi-format (78/122 bytes) with EMA→raw fallback for
//     legacy files. Regular 20 ms timestep enforced.
//
// v0.9.10 — SD Card Health Gate + Bypass
//   - SD Health Gate: if the SD is absent or has dirty contacts at boot,
//     the system blocks startup with a RED screen ("INSERT SD!") and retries
//     automatically every 2 s. Prevents entire sessions lost without logging.
//   - Double-click bypass: pressing BtnA twice on the red screen lets boot
//     continue without SD (sd_mounted=false, sd_queue stays NULL). Only live
//     MQTT is available — confirmed with an orange screen.
//   - No conflict with the existing Wi-Fi bypass: the SD gate precedes
//     connect_wifi(), so SD double-click and Wi-Fi single-click operate in
//     separate boot phases and do not interfere.
//   - sd_mounted flag: global variable indicating whether the SD is mounted.
//     The existing sd_queue != NULL guard in Task_Filter already prevents writes
//     to an absent SD; sd_mounted is available for future use (e.g. display indicator).
//   - Low Space Warning: at boot, if free space < 50 MB, YELLOW screen for 3 s
//     ("SD ALMOST FULL! XX MB free") + MQTT JSON warning with "warning":"SD_LOW_SPACE"
//     and "free_mb" fields. One-time check, zero runtime overhead.
//     50 MB threshold ≈ 400 k records at 122 bytes (~2 h @50 Hz).
//
// v0.9.11 — Runtime Error Detection + Robustness
//   - SD Write Error Detection: Task_SD_Writer now checks the return value of
//     logFile.write(). On failure it retries reopen up to 3 times, then declares
//     the SD lost (sd_write_error=true) and terminates the task.
//     Flashing red visual alarm on the display.
//   - GPS UART Overflow Monitoring: onReceiveError() callback registered on the
//     GPS UART, increments an atomic counter on BUFFER_FULL/FIFO_OVF events.
//     Task_GPS logs to Serial when the counter advances.
//   - Innovation Gate div-by-zero guard: in eskf.h, after recomputing S with R
//     inflated ×50, if det < 1e-10 the update is skipped rather than producing inf
//     that would corrupt P for the rest of the session.
//   - GPS Staleness Detection: if the last valid GPS fix is > 5 seconds old,
//     the ESKF switches to predict-only (no correct) and the display flashes
//     RED "GPS LOST!". Pure IMU dead-reckoning diverges in metres, so the visual
//     alarm is critical to warn the driver.
//   - Session Stats MQTT Heartbeat: every 60 s publishes a JSON with records written,
//     uptime, GPS state, and SD state. Allows verifying from the dashboard that
//     logging is working throughout the session.
//   - Fix calibrate_alignment() mutex: portMAX_DELAY replaced with a 2 s timeout.
//     Prevents loop() freeze if a long-press calibration coincides with a long
//     Kalman correct in Task_Filter.
// v0.9.12 — Stack Overflow Fix + Runtime Stack Monitoring
//   - Task_Filter stack increased from 12288 to 16384 bytes. Temporary 6×6
//     matrices in ESKF_6D.correct() (~2.5 KB per full cycle with 3 sequential
//     updates) + ESKF 5D (~1.5 KB) + 12-step IMU pipeline caused stack overflow
//     at ~570 s of continuous runtime, corrupting the last_gps struct and SD
//     records from that point onwards.
//   - Created global TaskSDHandle for Task_SD_Writer stack monitoring.
//   - MQTT Heartbeat extended: stk_filter, stk_i2c, stk_sd (free words in each
//     stack via uxTaskGetStackHighWaterMark). Values below ~500 are concerning.
//     Enables remote diagnostics via dashboard.
//   - Stack Overflow Guard: if stk_filter < 2000 bytes, Task_Filter self-terminates.
//
// v1.0.0 — Full Audit: 9 Bug Fixes (Math, Physics, Robustness)
//   Full line-by-line audit of Telemetria.ino and eskf.h.
//   9 findings fixed (0 critical, 0 major, 5 minor, 4 info).
//
//   FEATURE — Display Sleep (single-click toggle with confirmation):
//     Single tap shows a red full-screen warning "DISPLAY OFF?" for 5 s.
//     A second single tap within the window confirms: LCD enters sleep mode
//     (backlight off + controller sleep) to reduce internal heat and IMU
//     thermal drift. Any single tap while sleeping wakes the display.
//     Double/triple clicks cancel the pending confirmation and work normally.
//     Critical alarms (GPS lost, SD write error) auto-wake the display.
//     All data acquisition, logging, and MQTT continue unaffected.
//
//   BUG-1 (MINOR, Math) — ESKF Qd not scaled by dt:
//     Process noise matrix Qd_ was constant regardless of actual dt. When the
//     scheduler jitters or a timestamp gap occurs (dt ≠ 0.02 s), uncertainty
//     propagation was incorrect. Fix: Qd diagonal elements now scaled by
//     dt / DT_NOMINAL (0.02 s) on a local copy inside predict(), both ESKF2D
//     and ESKF_6D. The member Qd_ is never modified (no cumulative drift).
//
//   BUG-2 (MINOR, Edge-case) — Early return in correct() skipped COG update:
//     If the Innovation Gate fired (d² > 11.83) AND the recomputed S after
//     R×50 had det < 1e-10, the function returned early, skipping UPDATE 2
//     (speed), UPDATE 3 (COG), and the last_gps_east_/north_ assignment.
//     The next COG would use stale reference positions. Practically impossible
//     (S = P + 50·R always has large det) but logically incomplete.
//     Fix: UPDATE 1 wrapped in do{}while(0); return → break. Execution
//     continues to UPDATE 2, UPDATE 3, and last_gps_ bookkeeping.
//
//   BUG-3 (MINOR, Physics) — Straight-line ZARU on gentle curves:
//     The straight-line ZARU could corrupt thermal_bias_gz on gentle highway
//     curves. Two issues fixed:
//     (a) Speed threshold raised 20 → 40 km/h. At 20 km/h (5.5 m/s),
//         ω = 2°/s corresponds to R = 157 m — a real curve, not a straight.
//         At 40 km/h (11.1 m/s), R = 318 m: a genuine highway straight.
//     (b) Lateral gate changed from ema_ay to lin_ay (raw post-Madgwick).
//         EMA τ ≈ 313 ms smooths a rapid lane change (100 ms) below the
//         0.02 g threshold, making the gate too permeable. Raw lin_ay
//         captures instantaneous lateral load without phase delay.
//
//   BUG-4 (MINOR, Documentation) — ESKF_6D predict comment misleading:
//     Comment said "PURE gyroscope rate (no external bias subtracted)" but
//     gz_rad has boot bias already subtracted. X(5) estimates residual thermal
//     drift only, not the full bias. Corrected in predict() and class header.
//
//   BUG-5 (MINOR, Edge-case) — Madgwick subnormal input guard:
//     norm_accel_sq == 0.0f was an exact zero comparison. Subnormal inputs
//     (non-zero but near-zero) would pass and overflow fast_inv_sqrt.
//     Fix: threshold changed to < 1e-8f for both norm_accel_sq and norm_s_sq.
//
//   BUG-6 (INFO, Documentation) — EMA tau comment stale:
//     Comment said τ = 1.66 s (from an old α ≈ 0.012). Corrected to
//     τ ≈ 313 ms for the current α = 0.06 at 50 Hz.
//
//   BUG-7 (INFO, Edge-case) — uint8_t len in load_wifi_config:
//     len was uint8_t (max 255). If line[] were ever enlarged past 256 bytes,
//     the cast would silently overflow. Changed to int.
//
//   BUG-8 (INFO, Edge-case) — FileHeader record_size overflow:
//     record_size was uint8_t (max 255). If TelemetryRecord exceeds 255 bytes
//     in a future version, the field would overflow silently and the Python
//     converter would parse corrupted data. Changed to uint16_t.
//     header_version incremented 1 → 2. FileHeader size: 25 → 26 bytes.
//     Python bin_to_csv.py updated to handle both v1 and v2 headers.
//
//   BUG-9 (INFO, Edge-case) — SD write failure lost the failed record:
//     When a write failed and reopen succeeded, the loop returned to
//     xQueueReceive for a NEW record — the failed one was silently dropped.
//     Fix: pending_retry flag retries the same record after successful reopen.
//
// v1.1.0 — Modular Refactoring
//   - Codebase split from monolithic Telemetria.ino (~2,418 lines) into
//     13 separate header/source files with clear module boundaries.
//   - Shared state managed via globals.h/globals.cpp (extern linkage).
//   - Zero logic changes: all signal processing, FreeRTOS timing, mutex
//     protocols, and Kalman filter math are byte-for-byte identical.
//   - Telemetria.ino now contains only setup() and loop().
//
// v1.1.1 — Madgwick Dual-Gate Adaptive Beta Fix
//   - BUG (MINOR, Physics) — Adaptive beta gate too wide during cornering:
//     The single accel-deviation gate used a 0.15 g divisor. At 0.3–0.5 g
//     of sustained lateral acceleration ‖a‖ stays near 1 g by Pythagoras
//     (0.3 g lateral → only ~0.045 g deviation from 1 g), so beta_eff
//     remained non-zero and Madgwick kept pulling the quaternion toward the
//     tilted net-force vector. Observed effect: ~7°/s gravity-reference drift
//     in roundabout logs.
//     Fix (madgwick.h): dual gate via fminf of two independent factors:
//       k_acc = fmaxf(0, 1 − dist_from_1g / 0.05f)   [tightened: 0.15→0.05]
//       k_gyr = fmaxf(0, 1 − |gz| / 0.2618f)          [new: 15°/s cutoff]
//       beta_eff = beta × fminf(k_acc, k_gyr)
//     k_acc fully suppresses beta above ~0.32 g lateral; k_gyr suppresses
//     it above 15°/s yaw rate, catching sustained cornering before the accel
//     magnitude has time to wander. Both must be quiet simultaneously for
//     Madgwick to apply any accelerometer correction.
//     No changes to gradient descent math, quaternion integration, or
//     get_gravity_vector().
//
// v1.2.0 — G-Sensitivity Correction (K_gs 3×3 matrix, MPU-6886)
//   - G-sensitivity characterisation: dedicated test firmware (version "test-gsen")
//     logs chip-frame IMU data (ax_cal, ay_cal, az_cal, gx_clean, gy_clean, gz_clean)
//     bypassing rotate_3d and Madgwick. 6-face static test at thermal equilibrium
//     with WiFi disabled (eliminates thermal contamination from radio events).
//   - K_gs matrix measured by differential 6-face method:
//     K[i][j] = (gyro_i at +1g on axis_j − gyro_i at −1g on axis_j) / ΔG_j
//     Data sources: tel_gsen4.csv (Z, X pairs) + tel_82.csv (Y pair, WiFi-off).
//     Trim: 30 s time-trim each end + 5 % value-trim (scipy.trim_mean).
//
//            az(j=0)    ax(j=1)    ay(j=2)   [(°/s)/g]
//   gx(i=0): +0.6056    +0.7969    -0.1346
//   gy(i=1): +0.0712    +0.3539    -0.0908
//   gz(i=2): -0.1547    -0.0372    -0.0003   <-- ESKF heading axis
//
//   - Boot calibration interaction: bias_gz (trimmed mean of raw gyro at boot)
//     absorbs K_gz · a_boot at whatever orientation the device was during the
//     2 s calibration window. The runtime correction therefore uses the
//     incremental form Δa = a_cal − a_boot, not the absolute accel.
//     This is orientation-independent at boot (device does not need to be flat).
//   - New globals ax_boot_cal, ay_boot_cal, az_boot_cal (chip-frame ellipsoid-
//     calibrated accel trimmed means) saved by calibrate_alignment() under
//     telemetry_mutex and read by Task_Filter.
//   - Correction applied in filter_task.cpp STEP 4, after bias subtraction and
//     before rotate_3d(), on gx_clean / gy_clean / gz_clean (chip frame).
//   - Analysis tools added to Tool/:
//       gsen_analysis.py  — auto-segments 6 faces from CSV, outputs per-face
//                           gyro/accel/temperature table + 4-subplot PNG.
//       kgs_calc.py       — computes the 3×3 K_gs matrix from per-face means.
//
// ==========================================================

#include <M5Unified.h>
#include <SD.h>
#include <SPI.h>
#include <Wire.h>

#include "globals.h"
#include "display.h"
#include "calibration.h"
#include "wifi_manager.h"
#include "imu_task.h"
#include "filter_task.h"
#include "gps_task.h"
#include "sd_writer.h"

// ==========================================================
// --- SETUP ---
// ==========================================================

void setup() {
  auto cfg = M5.config();
  M5.begin(cfg);

  // ── 1. DISPLAY FIRST ─────────────────────────────────────────────────────
  // Display turned on as the very first operation: any subsequent error will
  // be visible on screen instead of producing a black screen at boot.
  M5.Lcd.setTextSize(1);
  M5.Lcd.setRotation(0);

  fillScreen(0x222222);
  M5.Lcd.setTextColor(WHITE, (uint16_t)0x222222);
  M5.Lcd.setCursor(12, 15);
  M5.Lcd.print("TELEMETRY");
  M5.Lcd.setTextColor(GREEN, (uint16_t)0x222222);
  M5.Lcd.setCursor(28, 40);
  M5.Lcd.print(FIRMWARE_VERSION);
  vTaskDelay(pdMS_TO_TICKS(1000));

  fillScreen(BLACK);
  setLabel(10, "System Init...");
  setLabel(30, "");

  // ── 2. HARDWARE DLPF AT 20 Hz ────────────────────────────────────────────
  // M5Unified uses a protected internal I2C bus and leaves Wire uninitialised.
  // Wire.begin(38, 39) explicitly initialises the bus on the AtomS3 internal
  // pins (SDA=38, SCL=39) before any transmission.
  // Without this line Wire.beginTransmission() causes an immediate crash.
  //
  // MPU-6886 registers:
  //   0x1A CONFIG        → DLPF_CFG = 4 → gyro BW  ~20 Hz
  //   0x1D ACCEL_CONFIG2 → A_DLPF_CFG = 4 → accel BW ~21 Hz
  Wire.begin(38, 39);

  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x04);
  uint8_t dlpf_gyro_err = Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x1D);
  Wire.write(0x04);
  uint8_t dlpf_accel_err = Wire.endTransmission();

  if (dlpf_gyro_err == 0 && dlpf_accel_err == 0) {
    Serial.println("[IMU] Hardware DLPF forced to 20Hz (ACK OK).");
    setLabel(30, "DLPF: 20Hz OK");
  } else {
    Serial.printf("[IMU] DLPF WARN: gyro_err=%d accel_err=%d\n", dlpf_gyro_err,
                  dlpf_accel_err);
    setLabel(30, "DLPF: ERROR!", RED, BLACK);
  }
  vTaskDelay(pdMS_TO_TICKS(1000));
  setLabel(30, "");
  // ─────────────────────────────────────────────────────────────────────────

  // ── EARLY SD MOUNT — reads /wifi_config.txt before WiFi setup ────────────
  // SPI is initialized here once. Full logging setup (file creation, queue,
  // task) happens later after calibration.
  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, CS_PIN);
  if (SD.begin(CS_PIN, SPI, 25000000)) {
    sd_mounted = true;
    load_wifi_config(); // populates wifi_networks[], cfg_mqtt_*, mqtt_enabled
  } else {
    Serial.println("[SD] Early mount failed - WiFi disabled (no config available).");
    wifi_enabled = false; // no config available, disable WiFi
  }
  // ─────────────────────────────────────────────────────────────────────────

  connect_wifi();

  telemetry_mutex = xSemaphoreCreateMutex();
  imuQueue = xQueueCreate(1, sizeof(ImuRawData));

  if (telemetry_mutex == NULL || imuQueue == NULL) {
    setLabel(10, "FreeRTOS Memory");
    setLabel(30, "Exhausted!");
    while (true)
      vTaskDelay(pdMS_TO_TICKS(100));
  }

  setLabel(10, "Calibrating...");
  setLabel(30, "Hold still...");
  calibrate_alignment();

  if (mqtt_enabled) {
    mqttClient.setServer(cfg_mqtt_broker, cfg_mqtt_port);
    mqttClient.setKeepAlive(60);
  }

  if (wifi_enabled && mqtt_enabled) {
    if (!mqttClient.connect(cfg_mqtt_client_id)) {
      reconnect_network();
    } else {
      fillScreen(0x66FF99);
      setLabel(10, "Ready!");
      vTaskDelay(pdMS_TO_TICKS(800));
    }
  }

  // ── SETUP LOGGING SD ─────────────────────────────────────────────────────
  // SPI already initialized during early boot. If early mount succeeded
  // (sd_mounted = true), skip health gate and go straight to logging setup.
  if (!sd_mounted && !SD.begin(CS_PIN, SPI, 25000000)) {
    Serial.println("[ERR] SD Card not found or defective!");
    // ── SD HEALTH GATE (v0.9.10) ──────────────────────────────────────────
    // Fixed red screen + retry every 2 s. Blocks calibration and task start
    // to prevent sessions lost without logging.
    // Bypass: double-click BtnA → continues with MQTT only (sd_mounted=false).
    fillScreen(0xFF0000);
    setLabel(10, "INSERT SD!", WHITE, (uint16_t)0xFF0000);
    setLabel(30, "2xClick=bypass", WHITE, (uint16_t)0xFF0000);
    Serial.println("[SD] Boot blocked: SD absent. Double-click to bypass.");

    bool sd_bypass = false;
    int sd_click_count = 0;
    int sd_click_release = 0;

    while (!sd_mounted && !sd_bypass) {
      // Retry mount every 2 s (dirty contacts → reinsert → recovers)
      SD.end();
      if (SD.begin(CS_PIN, SPI, 25000000)) {
        Serial.println("[SD] Card detected after retry!");
        sd_mounted = true;
        break;
      }

      // Button polling for bypass (double-click) — 2 s in 50 ms steps.
      // Bypass triggers immediately on second click (no post-click wait).
      // Single-click timeout extended to 600 ms (12 × 50 ms) to absorb
      // the dead time caused by the blocking SD.begin() call above.
      for (int i = 0; i < 40 && !sd_bypass && !sd_mounted; i++) {
        M5.update();
        if (M5.BtnA.wasPressed()) {
          sd_click_count++;
          sd_click_release = 0;
          if (sd_click_count >= 2) {   // immediate bypass — no extra wait
            sd_bypass = true;
            break;
          }
        }
        if (sd_click_count > 0) {
          sd_click_release++;
          // 600 ms window (12 × 50 ms) — extra margin for SD.begin() dead time
          if (sd_click_release > 12) {
            sd_click_count = 0;
            sd_click_release = 0;
          }
        }
        vTaskDelay(pdMS_TO_TICKS(50));
      }
    }

    if (sd_bypass) {
      Serial.println("[SD] User bypass: booting without SD (MQTT only).");
      fillScreen(0xFF8800);
      setLabel(10, "SD BYPASS", BLACK, (uint16_t)0xFF8800);
      setLabel(30, "MQTT-only mode", BLACK, (uint16_t)0xFF8800);
      vTaskDelay(pdMS_TO_TICKS(1500));
    }
    // ───────────────────────────────────────────────────────────────────────
  } else {
    sd_mounted = true;
  }

  if (sd_mounted) {
    Serial.println("[OK] SD card mounted.");

    // ── Low Space Warning (v0.9.10) ───────────────────────────────��──────
    // One-time check at boot: if free space < 50 MB, yellow warning screen
    // for 3 s + serial warning. The MQTT warning is sent after connection
    // (see sd_low_space flag).
    uint64_t sd_total = SD.totalBytes();
    uint64_t sd_used  = SD.usedBytes();
    uint64_t sd_free  = sd_total - sd_used;
    Serial.printf("[SD] Space: %.1f MB free / %.1f MB total\n",
                  sd_free / 1048576.0, sd_total / 1048576.0);
    if (sd_free < 50ULL * 1024 * 1024) {
      Serial.println("[SD] WARNING: free space < 50 MB!");
      fillScreen(0xFFE000);  // yellow
      setLabel(10, "SD ALMOST FULL!", BLACK, (uint16_t)0xFFE000);
      char space_buf[32];
      snprintf(space_buf, sizeof(space_buf), "%.0f MB free",
               sd_free / 1048576.0);
      setLabel(30, space_buf, BLACK, (uint16_t)0xFFE000);
      sd_low_space = true;
      vTaskDelay(pdMS_TO_TICKS(3000));
      // MQTT warning (only if WiFi+MQTT are already connected at this point)
      if (wifi_enabled && mqtt_enabled && mqttClient.connected()) {
        char warn_buf[96];
        snprintf(warn_buf, sizeof(warn_buf),
                 "{\"warning\":\"SD_LOW_SPACE\",\"free_mb\":%.0f}",
                 sd_free / 1048576.0);
        mqttClient.publish(cfg_mqtt_topic, warn_buf);
        Serial.println("[SD] Low-space warning sent via MQTT.");
      }
    }
    // ─────────────────────────────────────────────────────────────────────

    int idx = 0;
    while (true) {
      snprintf(current_log_filename, sizeof(current_log_filename), "/tel_%d.bin", idx);
      if (!SD.exists(current_log_filename))
        break;
      idx++;
    }

    File newFile = SD.open(current_log_filename, FILE_WRITE);
    if (newFile) {
      // Write FileHeader (26 bytes) as the first block of the .bin file.
      // The Python converter detects the "TEL" magic and uses record_size
      // from the header to parse subsequent records.
      FileHeader hdr = {};
      hdr.magic[0] = 0x54; hdr.magic[1] = 0x45; hdr.magic[2] = 0x4C; // "TEL"
      hdr.header_version = 2; // v2: record_size is uint16_t (was uint8_t in v1)
      strncpy(hdr.firmware_version, FIRMWARE_VERSION, sizeof(hdr.firmware_version) - 1);
      hdr.record_size = sizeof(TelemetryRecord);
      hdr.start_time_ms = millis();
      newFile.write((const uint8_t *)&hdr, sizeof(FileHeader));
      newFile.close();
      Serial.printf("[OK] SD binary file: %s (header %dB + %d bytes/record)\n",
                    current_log_filename, (int)sizeof(FileHeader),
                    (int)sizeof(TelemetryRecord));
      sd_queue = xQueueCreate(200, sizeof(TelemetryRecord));
      xTaskCreatePinnedToCore(Task_SD_Writer, "Task_SD", 8192, NULL, 1,
                              &TaskSDHandle, 1);
    } else {
      Serial.println("[ERR] Failed to create initial binary log file!");
    }
  }
  // ─────────────────────────────────────────────────────────────────────────

  // ── GPS SETUP ─────────────────────────────────────────────────────────────
  // UART1 on AtomS3 Grove pins (RX=1, TX=2) at 115200 baud (ATGM336H-6N default).
  // Buffer, baud rate, and update rate are configured below before task launch.
  gps_mutex = xSemaphoreCreateMutex();
  gpsSerial.setRxBufferSize(512);  // default 256 too tight for 10 Hz NMEA at 115200
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  // UART overflow callback: increments atomic counter read by Task_GPS.
  // Runs in ISR context: atomic operation only, no Serial/mutex.
  gpsSerial.onReceiveError([](hardwareSerial_error_t err) {
    if (err == UART_BUFFER_FULL_ERROR || err == UART_FIFO_OVF_ERROR)
      gps_uart_overflow_count++;
  });
  Serial.println("[GPS] UART1 initialized on pins RX=1, TX=2 at 115200 baud.");
  setLabel(30, "GPS: init...");
  vTaskDelay(pdMS_TO_TICKS(500)); // wait for AT6668 chip startup before sending CASIC commands
  // Force update rate to 10 Hz via CASIC protocol (AT6668).
  // Reduces IMU dead-reckoning from ~1000 ms to ~100 ms, eliminating the Sawtooth Effect.
  // NMEA checksum verified: XOR("PCAS02,100") = 0x1E
  gpsSerial.print("$PCAS02,100*1E\r\n");
  Serial.println("[GPS] Update rate set to 10Hz (CASIC $PCAS02,100).");
  setLabel(30, "");
  // ─────────────────────────────────────────────────────────────────────────

  xTaskCreatePinnedToCore(Task_I2C, "I2C", 4096, NULL, 3, &TaskI2CHandle, 0);
  // Stack 16 KB: ESKF 6D uses temporary 6×6 matrices (~2.5 KB per full correct())
  // + ESKF 5D (~1.5 KB) + 12-step IMU pipeline + static variables.
  // Increased from 12288 to 16384 to prevent stack overflow at ~570 s (v0.9.12).
  xTaskCreatePinnedToCore(Task_Filter, "Filter", 16384, NULL, 2,
                          &TaskFilterHandle, 1);
  // Task_GPS on Core 0 alongside Task_I2C: both lightweight, no contention.
  // Priority 2: below Task_I2C (3) — IMU timing is never disturbed.
  xTaskCreatePinnedToCore(Task_GPS, "GPS", 4096, NULL, 2, NULL, 0);
}

// ==========================================================
// --- MAIN LOOP (Core 1, Arduino task) ---
// ==========================================================

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000 / LOOP_HZ));
  M5.update();

  // ---- BUTTON ----
  if (M5.BtnA.isPressed()) {
    if (!btn_locked) {
      btn_press_cycles++;
      btn_release_cycles = 0;
      // Long press ≥ 1.5 s: manual recalibration
      if (btn_press_cycles >= (int)(LOOP_HZ * 1.5f)) {
        fillScreen(0xFFFF00);
        for (int i = 20; i >= 0; i--) {
          snprintf(buf, sizeof(buf), "Cal: %.1fs", i / 10.0f);
          setLabel(10, buf, BLACK, (uint16_t)0xFFFF00);
          setLabel(30, "", BLACK, (uint16_t)0xFFFF00);
          vTaskDelay(pdMS_TO_TICKS(100));
        }
        calibrate_alignment();
        prev_system_state = -1;
        btn_locked = true;
        btn_press_cycles = 0;
        btn_click_count = 0;
      }
    }
  } else {
    btn_locked = false;
    if (btn_press_cycles > 0)
      btn_click_count++;
    if (btn_press_cycles != 0)
      btn_press_cycles = 0;

    if (btn_click_count > 0) {
      btn_release_cycles++;
      // 400 ms window: on expiry, process all accumulated clicks
      if (btn_release_cycles > (int)(LOOP_HZ * 0.4f)) {

        if (btn_click_count == 1) {
          // Single click: display sleep toggle (with confirmation)
          if (display_off) {
            // Wake up immediately — no confirmation needed
            M5.Lcd.wakeup();
            M5.Lcd.setBrightness(64);
            display_off = false;
            display_confirm_pending = false;
            prev_system_state = -1; // force full redraw
          } else if (display_confirm_pending) {
            // Second single click within 5 s window: confirmed, sleep now
            display_confirm_pending = false;
            display_confirm_cycles = 0;
            M5.Lcd.setBrightness(0);
            M5.Lcd.sleep();
            display_off = true;
          } else {
            // First single click: show confirmation warning
            display_confirm_pending = true;
            display_confirm_cycles = (int)(LOOP_HZ * 5.0f); // 5 s
            fillScreen(0xFF0000);
            M5.Lcd.setTextSize(2);
            M5.Lcd.setTextColor(WHITE, (uint16_t)0xFF0000);
            M5.Lcd.setCursor(4, 20);
            M5.Lcd.print("DISPLAY");
            M5.Lcd.setCursor(22, 44);
            M5.Lcd.print("OFF?");
            M5.Lcd.setTextSize(1);
            M5.Lcd.setTextColor(WHITE, (uint16_t)0xFF0000);
            M5.Lcd.setCursor(4, 80);
            M5.Lcd.print("Tap again to");
            M5.Lcd.setCursor(4, 96);
            M5.Lcd.print("confirm (5s)");
          }

        } else if (btn_click_count == 2) {
          // Double click: start/stop lap (cancels pending confirm)
          display_confirm_pending = false;
          display_confirm_cycles = 0;
          if (system_state == 0 || system_state == 1) {
            system_state = (system_state == 0) ? 1 : 0;
            if (system_state == 1)
              countdown_cycles = (int)(LOOP_HZ * 5.0f);
          } else if (system_state == 2) {
            system_state = 0;
          }

        } else if (btn_click_count == 3) {
          // Triple click: toggle Wi-Fi (cancels pending confirm)
          display_confirm_pending = false;
          display_confirm_cycles = 0;
          if (wifi_enabled) {
            shutdown_radio();
            fillScreen(0x0000FF);
            setLabel(10, "WiFi OFF", WHITE, 0x0000FF);
            vTaskDelay(pdMS_TO_TICKS(1500));
          } else {
            fillScreen(0x00A5FF);
            setLabel(10, "WiFi ON", WHITE, 0x00A5FF);
            setLabel(30, "Connecting...", WHITE, 0x00A5FF);
            startup_radio();
          }
          prev_system_state = -1; // force full redraw + Wi-Fi indicator
        }

        btn_click_count = 0;
        btn_release_cycles = 0;
      }
    }
  }

  // ---- DISPLAY CONFIRM TIMEOUT ----
  // Counts down the 5 s confirmation window. On expiry, cancels the pending
  // confirmation and forces a full display redraw. No blocking — runs inline
  // at loop() frequency alongside all other processing.
  if (display_confirm_pending) {
    display_confirm_cycles--;
    if (display_confirm_cycles <= 0) {
      display_confirm_pending = false;
      prev_system_state = -1; // force full redraw to restore normal screen
    }
  }

  // ---- READ DATA FROM TASK FILTER ----
  FilteredTelemetry local_data;
  if (xSemaphoreTake(telemetry_mutex, pdMS_TO_TICKS(5)) != pdTRUE) {
    return;
  }

  local_data = shared_telemetry;

  xSemaphoreGive(telemetry_mutex);

  // ---- DISPLAY UPDATE ----
  // Skip all LCD writes when display is sleeping or confirmation is pending.
  // Critical alarms (GPS lost, SD error) force the display back on.
  if (!display_off && !display_confirm_pending) {
    if (system_state != prev_system_state) {
      switch (system_state) {
      case 0:
        fillScreen(BLACK);
        break;
      case 1:
        fillScreen(RED);
        setLabel(30, "", WHITE, RED);
        break;
      case 2:
        fillScreen(BLUE);
        break;
      }
      // Wi-Fi circle: redrawn on every state change (including post-toggle)
      if (system_state == 0 || system_state == 2) {
        update_wifi_indicator();
      }
      prev_system_state = system_state;
    }
  } else {
    // Track state transitions even while display is off
    prev_system_state = system_state;
  }

  // ---- COUNTDOWN TO RACE START ----
  // Always runs (even with display off) to keep system_state machine correct.
  if (system_state == 1) {
    countdown_cycles--;
    if (!display_off && !display_confirm_pending) {
      if (countdown_cycles % LOOP_HZ == 0) {
        int sec = countdown_cycles / LOOP_HZ;
        snprintf(buf, sizeof(buf), "START IN: %d", sec);
        setLabel(10, buf, WHITE, RED);
      }
    }
    if (countdown_cycles <= 0)
      system_state = 2;
  }

  // ---- MQTT at 10 Hz (only if Wi-Fi active and MQTT configured) ----
  if (wifi_enabled && mqtt_enabled) {
    mqtt_cycle_count++;
    if (mqtt_cycle_count >= MQTT_10HZ_CYCLES) {
      mqttClient.loop();
      int lap_flag = (system_state == 2) ? 1 : 0;
      // GPS snapshot for MQTT: static cache — if the mutex is busy this cycle,
      // use data from the previous cycle (10 ms ago).
      // Avoids uninitialised memory reads and JSON flickering.
      static GpsData gps_snap;
      if (xSemaphoreTake(gps_mutex, pdMS_TO_TICKS(3)) == pdTRUE) {
        gps_snap = shared_gps_data;
        xSemaphoreGive(gps_mutex);
      }
      snprintf(buf, sizeof(buf),
               "{\"ax\":%.2f,\"ay\":%.2f,\"az\":%.2f,"
               "\"gx\":%.2f,\"gy\":%.2f,\"gz\":%.2f,"
               "\"lap\":%d,\"lat\":%.6f,\"lon\":%.6f,"
               "\"spd\":%.1f,\"alt\":%.1f,\"sats\":%d,\"hdop\":%.1f,"
               "\"kfx\":%.2f,\"kfy\":%.2f,\"kfv\":%.2f,\"kfh\":%.3f}",
               local_data.ema_ax, local_data.ema_ay, local_data.ema_az,
               local_data.ema_gx, local_data.ema_gy, local_data.ema_gz, lap_flag,
               gps_snap.lat, gps_snap.lon, gps_snap.speed_kmh, gps_snap.alt_m,
               (int)gps_snap.sats, gps_snap.hdop,
               local_data.kf_x, local_data.kf_y,
               local_data.kf_vel, local_data.kf_heading);
      if (!mqttClient.publish(cfg_mqtt_topic, buf)) {
        reconnect_network();
      }
      mqtt_cycle_count = 0;

      // ── MQTT Heartbeat every 60 s (v0.9.11) ───────────────────────
      // Records written, uptime, active alarms. Allows verifying from the
      // dashboard that SD logging is working throughout the session.
      static int heartbeat_counter = 0;
      heartbeat_counter++;
      if (heartbeat_counter >= 600) { // 600 × 100ms = 60s
        heartbeat_counter = 0;
        UBaseType_t stk_filter = (TaskFilterHandle != NULL)
            ? uxTaskGetStackHighWaterMark(TaskFilterHandle) : 0;
        UBaseType_t stk_i2c = (TaskI2CHandle != NULL)
            ? uxTaskGetStackHighWaterMark(TaskI2CHandle) : 0;
        UBaseType_t stk_sd = (TaskSDHandle != NULL)
            ? uxTaskGetStackHighWaterMark(TaskSDHandle) : 0;
        snprintf(buf, sizeof(buf),
                 "{\"heartbeat\":true,\"records\":%u,\"uptime_s\":%u,"
                 "\"gps_stale\":%s,\"sd_err\":%s,"
                 "\"stk_filter\":%u,\"stk_i2c\":%u,\"stk_sd\":%u}",
                 (unsigned)sd_records_written.load(),
                 (unsigned)(millis() / 1000),
                 gps_stale ? "true" : "false",
                 sd_write_error ? "true" : "false",
                 (unsigned)stk_filter,
                 (unsigned)stk_i2c,
                 (unsigned)stk_sd);
        mqttClient.publish(cfg_mqtt_topic, buf);
      }
      // ─────────────────────────────────���─────────────────────────────
    }
  }

  // ---- Display at 5 Hz ----
  display_cycle_count++;
  if (display_cycle_count >= DISPLAY_5HZ_CYCLES) {
    // ── Critical alarm auto-wake ─────────────────────────────���───────
    // GPS lost or SD write error: force display back on so the driver
    // sees the flashing alarm even if display sleep was active.
    if ((gps_stale || sd_write_error) && display_off) {
      M5.Lcd.wakeup();
      M5.Lcd.setBrightness(64);
      display_off = false;
      display_confirm_pending = false;
    }
    // ── GPS STALE: flashing visual alarm (v0.9.11) ────────────────
    // Pure IMU dead-reckoning diverges in a few metres: red screen flashing
    // at ~2.5 Hz (toggled every frame at 5 Hz) to immediately alert the
    // driver/engineer. SD write error uses the same alarm mechanism.
    if (gps_stale || sd_write_error) {
      static bool alert_toggle = false;
      alert_toggle = !alert_toggle;
      if (alert_toggle) {
        fillScreen(0xFF0000);
        if (gps_stale) {
          setLabel(10, "GPS LOST!", WHITE, (uint16_t)0xFF0000);
          setLabel(30, "IMU only!", WHITE, (uint16_t)0xFF0000);
        } else {
          setLabel(10, "SD ERROR!", WHITE, (uint16_t)0xFF0000);
          setLabel(30, "Data lost!", WHITE, (uint16_t)0xFF0000);
        }
      } else {
        fillScreen(BLACK);
        if (gps_stale)
          setLabel(10, "GPS LOST!", RED, BLACK);
        else
          setLabel(10, "SD ERROR!", RED, BLACK);
        setLabel(30, "");
      }
    } else if (!display_off && !display_confirm_pending &&
               (system_state == 0 || system_state == 2)) {
      snprintf(buf, sizeof(buf), "A:%.1f %.1f %.1f", local_data.ema_ax,
               local_data.ema_ay, local_data.ema_az);
      setLabel(10, buf);
      snprintf(buf, sizeof(buf), "G:%.0f %.0f %.0f", local_data.ema_gx,
               local_data.ema_gy, local_data.ema_gz);
      setLabel(30, buf);
      // GPS row: static cache — if the mutex is busy this cycle (GPS update
      // in progress), the display shows the last valid data instead of
      // flickering to "NO FIX" for one frame.
      static GpsData gps_disp;
      if (xSemaphoreTake(gps_mutex, pdMS_TO_TICKS(3)) == pdTRUE) {
        gps_disp = shared_gps_data;
        xSemaphoreGive(gps_mutex);
      }
      if (gps_disp.valid) {
        snprintf(buf, sizeof(buf), "Sats:%d Spd:%.0f", (int)gps_disp.sats,
                 gps_disp.speed_kmh);
      } else {
        snprintf(buf, sizeof(buf), "Sats:%d NO FIX", (int)gps_disp.sats);
      }
      setLabel(50, buf, gps_disp.valid ? GREEN : YELLOW, BLACK);
    }
    display_cycle_count = 0;
  }
}
