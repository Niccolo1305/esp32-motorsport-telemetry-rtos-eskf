// SPDX-License-Identifier: GPL-3.0-or-later
// ================================================================================
// PLANAR ESKF 2D — Header-Only for ESP32-S3
// Error-State Kalman Filter: IMU (50 Hz) + GPS (10 Hz) fusion
// ================================================================================
//
// Classes:
//   ESKF2D  — 5D state: X = [px, py, vx, vy, theta]  (primary filter)
//   ESKF_6D — 6D state: X = [px, py, vx, vy, theta, b_gz]  (shadow mode)
//
//   px, py: ENU position [m] relative to GPS origin
//   vx, vy: ENU velocity [m/s]
//   theta:  heading (yaw) [rad]
//   b_gz:   gyroscope Z bias [rad/s] (6D only, estimated by the filter)
//
// Prediction (50 Hz): integrates acceleration and gyroscope in the global ENU frame.
// Correction (~10 Hz): 3-stage Sequential Update (position, speed, COG).
//
// Dependency: BasicLinearAlgebra (namespace BLA)
//   Install via PlatformIO: tomstewart89/BasicLinearAlgebra
//
// Changelog:
//   v0.8.0  — Original ESKF2D class, position-only correction
//   v0.9.3  — 3-stage Sequential Update + recalibrated Qd
//   v0.9.6  — Dynamic R based on HDOP
//   v0.9.7  — COG threshold 5 km/h, Mahalanobis Innovation Gate
//   v0.9.8  — Shadow Mode: ESKF_6D class with augmented state b_gz
//   v0.9.8h — Joseph form P update, theta while-normalisation, cleanup
//   v0.9.11 — Div-by-zero guard after Innovation Gate R × 50
//   v0.9.12 — Task_Filter stack increased to 16384, heartbeat stack monitor
//   v1.0.0  — Full Audit: Qd scaled by dt (BUG-1), return→break in Innovation
//             Gate (BUG-2), ESKF_6D predict comment corrected (BUG-4)
//
// ================================================================================
//
// ================================================================================
// ARCHITECTURAL NOTE: GIMBAL LOCK IMMUNITY
// ================================================================================
//
// This system is structurally immune to Gimbal Lock for two independent reasons:
//
// 1. 3D ATTITUDE (Pitch/Roll) — Quaternion Madgwick AHRS
//    Gravity compensation (removing g from the acceleration vector) is handled by
//    the Madgwick filter, which represents orientation as a unit quaternion q in H
//    (4 parameters, 1 constraint). Quaternions have no singularities: the map
//    SO(3) -> S3 is a double covering with no degenerate points, unlike the Euler
//    parameterisation (phi, theta, psi) which is singular at theta = ±90 deg.
//
// 2. 2D NAVIGATION (Heading/Yaw) — Planar ESKF
//    The ESKF integrates a single angle theta (yaw) on a single rotation axis.
//    Gimbal Lock requires alignment of TWO axes in a chain of three Euler
//    rotations: with a single axis the condition is physically impossible.
//    theta can take any value in [-pi, pi] without singularities.
//
// ================================================================================

#pragma once

#include <BasicLinearAlgebra.h>
#include <math.h>

using namespace BLA;

// -- Physical constants --
static constexpr float G_ACCEL = 9.80665f;
static constexpr double EARTH_RADIUS_M = 6371000.0;
static constexpr double DEG2RAD_D = M_PI / 180.0;

// -- WGS84 -> ENU conversion --
inline void wgs84_to_enu(double lat, double lon,
                         double lat0, double lon0,
                         float &east_m, float &north_m) {
    double dlat = (lat - lat0) * DEG2RAD_D;
    double dlon = (lon - lon0) * DEG2RAD_D;
    double cos_lat0 = cos(lat0 * DEG2RAD_D);
    east_m  = (float)(dlon * cos_lat0 * EARTH_RADIUS_M);
    north_m = (float)(dlat * EARTH_RADIUS_M);
}

// ========================================================================
// ESKF2D — Primary 5D filter
// ========================================================================
class ESKF2D {
public:
    Matrix<5, 1> X;
    Matrix<5, 5> P;

    ESKF2D() { reset(); }

    void reset() {
        X.Fill(0.0f);
        P.Fill(0.0f);
        P(0, 0) = 100.0f;
        P(1, 1) = 100.0f;
        P(2, 2) = 1.0f;
        P(3, 3) = 1.0f;
        P(4, 4) = 0.1f;
        has_last_gps_ = false;
    }

    // -- Predict (50 Hz) --
    void predict(float ax_body, float ay_body, float gz_rad, float dt,
                 bool is_stationary) {
        float ax_ms2 = ax_body * G_ACCEL;
        float ay_ms2 = ay_body * G_ACCEL;

        if (is_stationary) {
            X(2) = 0.0f;
            X(3) = 0.0f;
        }

        float vx = X(2);
        float vy = X(3);
        float theta = X(4);

        float theta_new = theta + gz_rad * dt;
        if (theta_new > (float)M_PI)       theta_new -= 2.0f * (float)M_PI;
        else if (theta_new < -(float)M_PI) theta_new += 2.0f * (float)M_PI;

        float cos_th = cosf(theta);
        float sin_th = sinf(theta);
        float ax_enu = ax_ms2 * cos_th - ay_ms2 * sin_th;
        float ay_enu = ax_ms2 * sin_th + ay_ms2 * cos_th;

        float vx_new = vx + ax_enu * dt;
        float vy_new = vy + ay_enu * dt;
        X(0) += vx * dt + 0.5f * ax_enu * dt * dt;
        X(1) += vy * dt + 0.5f * ay_enu * dt * dt;
        X(2) = vx_new;
        X(3) = vy_new;
        X(4) = theta_new;

        float dax_dth = -ax_ms2 * sin_th - ay_ms2 * cos_th;
        float day_dth =  ax_ms2 * cos_th - ay_ms2 * sin_th;

        Matrix<5, 5> Fj;
        Fj.Fill(0.0f);
        Fj(0, 0) = 1.0f;  Fj(0, 2) = dt;    Fj(0, 4) = 0.5f * dax_dth * dt * dt;
        Fj(1, 1) = 1.0f;  Fj(1, 3) = dt;    Fj(1, 4) = 0.5f * day_dth * dt * dt;
        Fj(2, 2) = 1.0f;                     Fj(2, 4) = dax_dth * dt;
        Fj(3, 3) = 1.0f;                     Fj(3, 4) = day_dth * dt;
        Fj(4, 4) = 1.0f;

        // BUG-1 fix: scale Qd by dt/DT_NOMINAL for correct uncertainty
        // propagation when dt varies (scheduling jitter, timestamp gaps).
        // Qd_ is tuned for DT_NOMINAL = 0.02 s; for different dt the
        // continuous-time Qc·dt relationship requires linear scaling.
        // Applied to a local copy (diagonal-only add) to avoid cumulative drift.
        float dt_scale = dt / DT_NOMINAL_;
        P = Fj * P * (~Fj);
        P(2, 2) += Qd_(2, 2) * dt_scale;
        P(3, 3) += Qd_(3, 3) * dt_scale;
        P(4, 4) += Qd_(4, 4) * dt_scale;
    }

    // -- Correct (~10 Hz, 3-stage Sequential Update) --
    void correct(float gps_east_m, float gps_north_m, float gps_speed_kmh,
                 float hdop = 1.0f) {

        if (hdop < 0.5f || hdop > 50.0f) hdop = 1.0f;
        float r_dynamic = 0.05f * (hdop * hdop);
        R_(0, 0) = r_dynamic;
        R_(1, 1) = r_dynamic;
        R_(0, 1) = 0.0f;
        R_(1, 0) = 0.0f;

        // UPDATE 1: 2D POSITION
        // BUG-2 fix: do-while(0) allows break instead of return on degenerate
        // S after Innovation Gate R×50, so UPDATE 2/3 and last_gps_ are not skipped.
        do {
            Matrix<2, 1> z;
            z(0) = gps_east_m;
            z(1) = gps_north_m;

            Matrix<2, 1> HX;
            HX(0) = X(0);
            HX(1) = X(1);

            Matrix<2, 1> y = z - HX;

            Matrix<2, 2> S;
            S(0, 0) = P(0, 0) + R_(0, 0);
            S(0, 1) = P(0, 1) + R_(0, 1);
            S(1, 0) = P(1, 0) + R_(1, 0);
            S(1, 1) = P(1, 1) + R_(1, 1);

            float det = S(0, 0) * S(1, 1) - S(0, 1) * S(1, 0);
            if (fabsf(det) > 1e-10f) {
                float inv_det = 1.0f / det;
                Matrix<2, 2> S_inv;
                S_inv(0, 0) =  S(1, 1) * inv_det;
                S_inv(0, 1) = -S(0, 1) * inv_det;
                S_inv(1, 0) = -S(1, 0) * inv_det;
                S_inv(1, 1) =  S(0, 0) * inv_det;

                // Innovation Gate (Mahalanobis, v0.9.7)
                float d2 = y(0) * (S_inv(0, 0) * y(0) + S_inv(0, 1) * y(1))
                         + y(1) * (S_inv(1, 0) * y(0) + S_inv(1, 1) * y(1));
                if (d2 > 11.83f) {
                    R_(0, 0) *= 50.0f;
                    R_(1, 1) *= 50.0f;
                    S(0, 0) = P(0, 0) + R_(0, 0);
                    S(0, 1) = P(0, 1);
                    S(1, 0) = P(1, 0);
                    S(1, 1) = P(1, 1) + R_(1, 1);
                    det = S(0, 0) * S(1, 1) - S(0, 1) * S(1, 0);
                    if (fabsf(det) < 1e-10f) break; // BUG-2: was return (v0.9.11)
                    inv_det = 1.0f / det;
                    S_inv(0, 0) =  S(1, 1) * inv_det;
                    S_inv(0, 1) = -S(0, 1) * inv_det;
                    S_inv(1, 0) = -S(1, 0) * inv_det;
                    S_inv(1, 1) =  S(0, 0) * inv_det;
                }

                Matrix<5, 2> PHt;
                for (int i = 0; i < 5; i++) {
                    PHt(i, 0) = P(i, 0);
                    PHt(i, 1) = P(i, 1);
                }
                Matrix<5, 2> K = PHt * S_inv;
                X = X + K * y;

                // Joseph form: P = (I-KH) P (I-KH)^T + K R K^T (v0.9.8h)
                Matrix<5, 5> I_KH;
                I_KH.Fill(0.0f);
                for (int i = 0; i < 5; i++) {
                    I_KH(i, i) = 1.0f;
                    I_KH(i, 0) -= K(i, 0);
                    I_KH(i, 1) -= K(i, 1);
                }
                P = I_KH * P * (~I_KH) + K * R_ * (~K);
                symmetrize_P();
            }
        } while (0);

        // UPDATE 2: SCALAR SPEED (1D)
        {
            float gps_speed_ms = gps_speed_kmh / 3.6f;
            float vx_ = X(2), vy_ = X(3);
            float eskf_speed = sqrtf(vx_ * vx_ + vy_ * vy_);

            if (gps_speed_kmh > 5.0f && eskf_speed > 0.1f) {
                Matrix<1, 5> H_v;
                H_v.Fill(0.0f);
                H_v(0, 2) = vx_ / eskf_speed;
                H_v(0, 3) = vy_ / eskf_speed;

                float y_v = gps_speed_ms - eskf_speed;
                float R_v = 1.0f;

                Matrix<1, 1> S_v_mat = H_v * P * (~H_v);
                float S_v = S_v_mat(0, 0) + R_v;

                if (fabsf(S_v) > 1e-10f) {
                    Matrix<5, 1> K_v = P * (~H_v) * (1.0f / S_v);
                    X = X + K_v * y_v;

                    // Scalar Joseph form (v0.9.8h)
                    Matrix<5, 5> I_KH_v;
                    I_KH_v.Fill(0.0f);
                    for (int i = 0; i < 5; i++) I_KH_v(i, i) = 1.0f;
                    I_KH_v = I_KH_v - K_v * H_v;
                    Matrix<5, 5> KKt_v;
                    for (int i = 0; i < 5; i++)
                        for (int j = 0; j < 5; j++)
                            KKt_v(i, j) = K_v(i) * K_v(j);
                    P = I_KH_v * P * (~I_KH_v) + KKt_v * R_v;
                    symmetrize_P();
                }
            }
        }

        // UPDATE 3: COURSE OVER GROUND (1D)
        // Threshold 5 km/h (v0.9.7), adaptive R_th (v0.9.3)
        if (has_last_gps_ && gps_speed_kmh > 5.0f) {
            float dx = gps_east_m - last_gps_east_;
            float dy = gps_north_m - last_gps_north_;
            float dist = sqrtf(dx * dx + dy * dy);

            if (dist > 1.0f) {
                float gps_heading = atan2f(dy, dx);
                float y_th = gps_heading - X(4);

                while (y_th > (float)M_PI)  y_th -= 2.0f * (float)M_PI;
                while (y_th < -(float)M_PI) y_th += 2.0f * (float)M_PI;

                Matrix<1, 5> H_th;
                H_th.Fill(0.0f);
                H_th(0, 4) = 1.0f;

                float sigma_pos = 1.2f;
                float R_th = (sigma_pos * sigma_pos) / (dist * dist);
                R_th = fmaxf(R_th, 0.005f);
                R_th = fminf(R_th, 0.300f);

                Matrix<1, 1> S_th_mat = H_th * P * (~H_th);
                float S_th = S_th_mat(0, 0) + R_th;
                Matrix<5, 1> K_th = P * (~H_th) * (1.0f / S_th);

                X = X + K_th * y_th;

                while (X(4) > (float)M_PI)  X(4) -= 2.0f * (float)M_PI;
                while (X(4) < -(float)M_PI) X(4) += 2.0f * (float)M_PI;

                // Scalar Joseph form (v0.9.8h)
                Matrix<5, 5> I_KH_th;
                I_KH_th.Fill(0.0f);
                for (int i = 0; i < 5; i++) I_KH_th(i, i) = 1.0f;
                I_KH_th = I_KH_th - K_th * H_th;
                Matrix<5, 5> KKt_th;
                for (int i = 0; i < 5; i++)
                    for (int j = 0; j < 5; j++)
                        KKt_th(i, j) = K_th(i) * K_th(j);
                P = I_KH_th * P * (~I_KH_th) + KKt_th * R_th;
                symmetrize_P();
            }
        }

        last_gps_east_  = gps_east_m;
        last_gps_north_ = gps_north_m;
        has_last_gps_   = true;

        while (X(4) > (float)M_PI)  X(4) -= 2.0f * (float)M_PI;
        while (X(4) < -(float)M_PI) X(4) += 2.0f * (float)M_PI;
    }

    // -- Getter --
    float px()       const { return X(0); }
    float py()       const { return X(1); }
    float vx()       const { return X(2); }
    float vy()       const { return X(3); }
    float heading()  const { return X(4); }

    float speed_ms() const {
        float vx_ = X(2), vy_ = X(3);
        return sqrtf(vx_ * vx_ + vy_ * vy_);
    }

private:
    Matrix<2, 2> R_ = []() {
        Matrix<2, 2> m;
        m.Fill(0.0f);
        m(0, 0) = 0.05f;
        m(1, 1) = 0.05f;
        return m;
    }();

    Matrix<5, 5> Qd_ = []() {
        Matrix<5, 5> m;
        m.Fill(0.0f);
        m(2, 2) = 1.0e-2f;
        m(3, 3) = 1.0e-2f;
        m(4, 4) = 5.0e-4f;
        return m;
    }();

    static constexpr float DT_NOMINAL_ = 0.02f; // Qd_ tuned for this dt

    bool  has_last_gps_   = false;
    float last_gps_east_  = 0.0f;
    float last_gps_north_ = 0.0f;

    void symmetrize_P() {
        for (int i = 0; i < 5; i++) {
            for (int j = i + 1; j < 5; j++) {
                float avg = 0.5f * (P(i, j) + P(j, i));
                P(i, j) = avg;
                P(j, i) = avg;
            }
        }
    }
};

// ========================================================================
// ESKF_6D — Shadow Mode with gyroscope bias in the state (v0.9.8)
// ========================================================================
//
// State: X = [px, py, vx, vy, theta, b_gz]
//   b_gz: gyroscope Z bias [rad/s] — estimated by the filter, not external
//
// Identical to the 5D for the first 5 states. The sixth state models the
// bias as a random walk and estimates it via the theta <-> b_gz cross-covariance.
// predict() receives gz_rad with boot bias already subtracted; X(5)
// estimates residual thermal drift only.
//
// ========================================================================
class ESKF_6D {
public:
    Matrix<6, 1> X;
    Matrix<6, 6> P;

    ESKF_6D() { reset(); }

    void reset() {
        X.Fill(0.0f);
        P.Fill(0.0f);
        P(0, 0) = 100.0f;
        P(1, 1) = 100.0f;
        P(2, 2) = 1.0f;
        P(3, 3) = 1.0f;
        P(4, 4) = 0.1f;
        P(5, 5) = 0.01f;
        has_last_gps_ = false;
    }

    // -- Predict --
    // gz_rad has boot bias already subtracted (gz_clean * DEG2RAD).
    // X(5) estimates residual thermal drift only, not the full bias.
    // Bias is in the state: theta_new = theta + (gz_rad - X(5)) * dt
    void predict(float ax_body, float ay_body, float gz_rad, float dt,
                 bool is_stationary) {
        float ax_ms2 = ax_body * G_ACCEL;
        float ay_ms2 = ay_body * G_ACCEL;

        if (is_stationary) {
            X(2) = 0.0f;
            X(3) = 0.0f;
        }

        float vx = X(2);
        float vy = X(3);
        float theta = X(4);
        float bgz = X(5);

        float theta_new = theta + (gz_rad - bgz) * dt;
        if (theta_new > (float)M_PI)       theta_new -= 2.0f * (float)M_PI;
        else if (theta_new < -(float)M_PI) theta_new += 2.0f * (float)M_PI;

        float cos_th = cosf(theta);
        float sin_th = sinf(theta);
        float ax_enu = ax_ms2 * cos_th - ay_ms2 * sin_th;
        float ay_enu = ax_ms2 * sin_th + ay_ms2 * cos_th;

        float vx_new = vx + ax_enu * dt;
        float vy_new = vy + ay_enu * dt;
        X(0) += vx * dt + 0.5f * ax_enu * dt * dt;
        X(1) += vy * dt + 0.5f * ay_enu * dt * dt;
        X(2) = vx_new;
        X(3) = vy_new;
        X(4) = theta_new;
        // X(5) = bgz — random walk, unchanged in predict

        float dax_dth = -ax_ms2 * sin_th - ay_ms2 * cos_th;
        float day_dth =  ax_ms2 * cos_th - ay_ms2 * sin_th;

        Matrix<6, 6> Fj;
        Fj.Fill(0.0f);
        Fj(0, 0) = 1.0f;  Fj(0, 2) = dt;    Fj(0, 4) = 0.5f * dax_dth * dt * dt;
        Fj(1, 1) = 1.0f;  Fj(1, 3) = dt;    Fj(1, 4) = 0.5f * day_dth * dt * dt;
        Fj(2, 2) = 1.0f;                     Fj(2, 4) = dax_dth * dt;
        Fj(3, 3) = 1.0f;                     Fj(3, 4) = day_dth * dt;
        Fj(4, 4) = 1.0f;  Fj(4, 5) = -dt;   // d(theta)/d(b_gz) = -dt
        Fj(5, 5) = 1.0f;                     // bias as random walk

        // BUG-1 fix: scale Qd by dt/DT_NOMINAL (see ESKF2D::predict)
        float dt_scale = dt / DT_NOMINAL_;
        P = Fj * P * (~Fj);
        P(2, 2) += Qd_(2, 2) * dt_scale;
        P(3, 3) += Qd_(3, 3) * dt_scale;
        P(4, 4) += Qd_(4, 4) * dt_scale;
        P(5, 5) += Qd_(5, 5) * dt_scale;
    }

    // -- Correct (Sequential Update, 6×N matrices) --
    void correct(float gps_east_m, float gps_north_m, float gps_speed_kmh,
                 float hdop = 1.0f) {

        if (hdop < 0.5f || hdop > 50.0f) hdop = 1.0f;
        float r_dynamic = 0.05f * (hdop * hdop);
        R_(0, 0) = r_dynamic;
        R_(1, 1) = r_dynamic;
        R_(0, 1) = 0.0f;
        R_(1, 0) = 0.0f;

        // UPDATE 1: 2D POSITION
        // BUG-2 fix: do-while(0) allows break instead of return (see ESKF2D)
        do {
            Matrix<2, 1> z;
            z(0) = gps_east_m;
            z(1) = gps_north_m;

            Matrix<2, 1> HX;
            HX(0) = X(0);
            HX(1) = X(1);

            Matrix<2, 1> y = z - HX;

            Matrix<2, 2> S;
            S(0, 0) = P(0, 0) + R_(0, 0);
            S(0, 1) = P(0, 1) + R_(0, 1);
            S(1, 0) = P(1, 0) + R_(1, 0);
            S(1, 1) = P(1, 1) + R_(1, 1);

            float det = S(0, 0) * S(1, 1) - S(0, 1) * S(1, 0);
            if (fabsf(det) > 1e-10f) {
                float inv_det = 1.0f / det;
                Matrix<2, 2> S_inv;
                S_inv(0, 0) =  S(1, 1) * inv_det;
                S_inv(0, 1) = -S(0, 1) * inv_det;
                S_inv(1, 0) = -S(1, 0) * inv_det;
                S_inv(1, 1) =  S(0, 0) * inv_det;

                // Innovation Gate (Mahalanobis)
                float d2 = y(0) * (S_inv(0, 0) * y(0) + S_inv(0, 1) * y(1))
                         + y(1) * (S_inv(1, 0) * y(0) + S_inv(1, 1) * y(1));
                if (d2 > 11.83f) {
                    R_(0, 0) *= 50.0f;
                    R_(1, 1) *= 50.0f;
                    S(0, 0) = P(0, 0) + R_(0, 0);
                    S(0, 1) = P(0, 1);
                    S(1, 0) = P(1, 0);
                    S(1, 1) = P(1, 1) + R_(1, 1);
                    det = S(0, 0) * S(1, 1) - S(0, 1) * S(1, 0);
                    if (fabsf(det) < 1e-10f) break; // BUG-2: was return (v0.9.11)
                    inv_det = 1.0f / det;
                    S_inv(0, 0) =  S(1, 1) * inv_det;
                    S_inv(0, 1) = -S(0, 1) * inv_det;
                    S_inv(1, 0) = -S(1, 0) * inv_det;
                    S_inv(1, 1) =  S(0, 0) * inv_det;
                }

                Matrix<6, 2> PHt;
                for (int i = 0; i < 6; i++) {
                    PHt(i, 0) = P(i, 0);
                    PHt(i, 1) = P(i, 1);
                }
                Matrix<6, 2> K = PHt * S_inv;
                X = X + K * y;

                // Joseph form (v0.9.8h)
                Matrix<6, 6> I_KH;
                I_KH.Fill(0.0f);
                for (int i = 0; i < 6; i++) {
                    I_KH(i, i) = 1.0f;
                    I_KH(i, 0) -= K(i, 0);
                    I_KH(i, 1) -= K(i, 1);
                }
                P = I_KH * P * (~I_KH) + K * R_ * (~K);
                symmetrize_P();
            }
        } while (0);

        // UPDATE 2: SCALAR SPEED (1D)
        {
            float gps_speed_ms = gps_speed_kmh / 3.6f;
            float vx_ = X(2), vy_ = X(3);
            float eskf_speed = sqrtf(vx_ * vx_ + vy_ * vy_);

            if (gps_speed_kmh > 5.0f && eskf_speed > 0.1f) {
                Matrix<1, 6> H_v;
                H_v.Fill(0.0f);
                H_v(0, 2) = vx_ / eskf_speed;
                H_v(0, 3) = vy_ / eskf_speed;

                float y_v = gps_speed_ms - eskf_speed;
                float R_v = 1.0f;

                Matrix<1, 1> S_v_mat = H_v * P * (~H_v);
                float S_v = S_v_mat(0, 0) + R_v;

                if (fabsf(S_v) > 1e-10f) {
                    Matrix<6, 1> K_v = P * (~H_v) * (1.0f / S_v);
                    X = X + K_v * y_v;

                    // Scalar Joseph form (v0.9.8h)
                    Matrix<6, 6> I_KH_v;
                    I_KH_v.Fill(0.0f);
                    for (int i = 0; i < 6; i++) I_KH_v(i, i) = 1.0f;
                    I_KH_v = I_KH_v - K_v * H_v;
                    Matrix<6, 6> KKt_v;
                    for (int i = 0; i < 6; i++)
                        for (int j = 0; j < 6; j++)
                            KKt_v(i, j) = K_v(i) * K_v(j);
                    P = I_KH_v * P * (~I_KH_v) + KKt_v * R_v;
                    symmetrize_P();
                }
            }
        }

        // UPDATE 3: COURSE OVER GROUND (1D)
        if (has_last_gps_ && gps_speed_kmh > 5.0f) {
            float dx = gps_east_m - last_gps_east_;
            float dy = gps_north_m - last_gps_north_;
            float dist = sqrtf(dx * dx + dy * dy);

            if (dist > 1.0f) {
                float gps_heading = atan2f(dy, dx);
                float y_th = gps_heading - X(4);

                while (y_th > (float)M_PI)  y_th -= 2.0f * (float)M_PI;
                while (y_th < -(float)M_PI) y_th += 2.0f * (float)M_PI;

                Matrix<1, 6> H_th;
                H_th.Fill(0.0f);
                H_th(0, 4) = 1.0f;

                float sigma_pos = 1.2f;
                float R_th = (sigma_pos * sigma_pos) / (dist * dist);
                R_th = fmaxf(R_th, 0.005f);
                R_th = fminf(R_th, 0.300f);

                Matrix<1, 1> S_th_mat = H_th * P * (~H_th);
                float S_th = S_th_mat(0, 0) + R_th;
                Matrix<6, 1> K_th = P * (~H_th) * (1.0f / S_th);

                X = X + K_th * y_th;

                while (X(4) > (float)M_PI)  X(4) -= 2.0f * (float)M_PI;
                while (X(4) < -(float)M_PI) X(4) += 2.0f * (float)M_PI;

                // Scalar Joseph form (v0.9.8h)
                Matrix<6, 6> I_KH_th;
                I_KH_th.Fill(0.0f);
                for (int i = 0; i < 6; i++) I_KH_th(i, i) = 1.0f;
                I_KH_th = I_KH_th - K_th * H_th;
                Matrix<6, 6> KKt_th;
                for (int i = 0; i < 6; i++)
                    for (int j = 0; j < 6; j++)
                        KKt_th(i, j) = K_th(i) * K_th(j);
                P = I_KH_th * P * (~I_KH_th) + KKt_th * R_th;
                symmetrize_P();
            }
        }

        last_gps_east_  = gps_east_m;
        last_gps_north_ = gps_north_m;
        has_last_gps_   = true;

        while (X(4) > (float)M_PI)  X(4) -= 2.0f * (float)M_PI;
        while (X(4) < -(float)M_PI) X(4) += 2.0f * (float)M_PI;
    }

    // -- Correct bias (ZUPT on bias at standstill) --
    // When is_stationary, measured gz is entirely bias (zero true rotation).
    // Kalman observation: H=[0,0,0,0,0,1], z=gz_measured, R=R_bias.
    // Does NOT overwrite X(5) — updates via Kalman to keep P consistent.
    void correct_bias(float gz_measured_rad, float R_bias) {
        float y_b = gz_measured_rad - X(5);
        float S_b = P(5, 5) + R_bias;
        if (fabsf(S_b) < 1e-10f) return;

        Matrix<6, 1> K_b;
        for (int i = 0; i < 6; i++) K_b(i) = P(i, 5) / S_b;
        X = X + K_b * y_b;

        // Joseph form scalare (v0.9.8h)
        Matrix<6, 6> I_KH_b;
        I_KH_b.Fill(0.0f);
        for (int i = 0; i < 6; i++) I_KH_b(i, i) = 1.0f;
        for (int i = 0; i < 6; i++) I_KH_b(i, 5) -= K_b(i);
        Matrix<6, 6> KKt_b;
        for (int i = 0; i < 6; i++)
            for (int j = 0; j < 6; j++)
                KKt_b(i, j) = K_b(i) * K_b(j);
        P = I_KH_b * P * (~I_KH_b) + KKt_b * R_bias;
        symmetrize_P();
    }

    // -- Getter --
    float px()       const { return X(0); }
    float py()       const { return X(1); }
    float vx()       const { return X(2); }
    float vy()       const { return X(3); }
    float heading()  const { return X(4); }
    float bias_gz()  const { return X(5); }

    float speed_ms() const {
        float vx_ = X(2), vy_ = X(3);
        return sqrtf(vx_ * vx_ + vy_ * vy_);
    }

private:
    Matrix<2, 2> R_ = []() {
        Matrix<2, 2> m;
        m.Fill(0.0f);
        m(0, 0) = 0.05f;
        m(1, 1) = 0.05f;
        return m;
    }();

    Matrix<6, 6> Qd_ = []() {
        Matrix<6, 6> m;
        m.Fill(0.0f);
        m(2, 2) = 1.0e-2f;
        m(3, 3) = 1.0e-2f;
        m(4, 4) = 5.0e-4f;
        m(5, 5) = 1.0e-7f;   // random walk bias gz
        return m;
    }();

    static constexpr float DT_NOMINAL_ = 0.02f; // Qd_ tuned for this dt

    bool  has_last_gps_   = false;
    float last_gps_east_  = 0.0f;
    float last_gps_north_ = 0.0f;

    void symmetrize_P() {
        for (int i = 0; i < 6; i++) {
            for (int j = i + 1; j < 6; j++) {
                float avg = 0.5f * (P(i, j) + P(j, i));
                P(i, j) = avg;
                P(j, i) = avg;
            }
        }
    }
};
