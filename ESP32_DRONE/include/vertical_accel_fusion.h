#pragma once
// Earth-up linear acceleration from MPU6050_light, for Kalman Z prediction.
// MPU6050_light angles X/Y are sensor-frame tilt angles, NOT command signs.
// Acceleration in g; gravity is +1 g on a level, face-up sensor.
// Bench validation of physical sensor orientation and vibrations is mandatory.
#include <cmath>
#include <cstdint>
#include <algorithm>

namespace vertical_accel_fusion {
struct State {
    bool calibrated=false, valid=false;
    float earth_z_raw_mps2=0.0f;
    float earth_z_linear_mps2=0.0f;
    float bias_mps2=0.0f;
    float acc_norm_g=0.0f;
    uint16_t stationary_samples=0;
    uint32_t rejected_samples=0;
};

class Processor {
public:
    void reset() { s_={}; bias_sum_=0; filter_initialized_=false;
                   filtered_raw_=0; last_update_ms_=0; }
    Processor() { reset(); }
    // Angles are DIRECT getAngleX/getAngleY from MPU6050_light. Use the IMU's
    // sensor coordinate convention; do not apply controller stick/axis signs.
    // disarmed enables one-time zero-g bias calibration on a level, quiet desk.
    const State& update(uint32_t now_ms, float ax_g, float ay_g, float az_g,
                        float sensor_angle_x_deg, float sensor_angle_y_deg,
                        float gyro_norm_dps, bool disarmed, bool timing_ok) {
        s_.valid=false;
        const bool finite=std::isfinite(ax_g) && std::isfinite(ay_g) &&
            std::isfinite(az_g) && std::isfinite(sensor_angle_x_deg) &&
            std::isfinite(sensor_angle_y_deg) && std::isfinite(gyro_norm_dps);
        if (!finite || !timing_ok) { ++s_.rejected_samples; return s_; }
        const float norm=std::sqrt(ax_g*ax_g+ay_g*ay_g+az_g*az_g);
        s_.acc_norm_g=norm;
        if (!(norm>0.55f && norm<1.55f)) {
            ++s_.rejected_samples; return s_;
        }
        const float phi=sensor_angle_x_deg*0.017453292519943295f;
        const float theta=sensor_angle_y_deg*0.017453292519943295f;
        // Specific force along gravity-up world Z: world Z row of R_body_to_world.
        const float upward_g=-std::sin(theta)*ax_g +
            std::cos(theta)*std::sin(phi)*ay_g +
            std::cos(theta)*std::cos(phi)*az_g;
        const float raw=(upward_g-1.0f)*9.80665f;
        s_.earth_z_raw_mps2=raw;
        if (!std::isfinite(raw)) { ++s_.rejected_samples; return s_; }
        const uint32_t elapsed=last_update_ms_ ? uint32_t(now_ms-last_update_ms_) : 4;
        last_update_ms_=now_ms;
        if (elapsed==0 || elapsed>30) {
            filter_initialized_=false;
            ++s_.rejected_samples;
            return s_;
        }
        // 18 Hz low-pass (~250 Hz sample loop) to limit high-frequency noise.
        const float dt=elapsed*0.001f;
        const float tau=1.0f/(2.0f*3.14159265359f*18.0f);
        const float alpha=dt/(tau+dt);
        if (!filter_initialized_) { filtered_raw_=raw; filter_initialized_=true; }
        else filtered_raw_ += alpha*(raw-filtered_raw_);

        // Calibrate only while DISARMED and almost flat/stationary. Never adapt
        // the bias during flight (would mistake sustained thrust for bias).
        const bool stationary=disarmed &&
            std::fabs(sensor_angle_x_deg)<8.0f &&
            std::fabs(sensor_angle_y_deg)<8.0f &&
            std::fabs(norm-1.0f)<0.09f &&
            gyro_norm_dps<3.0f && std::fabs(raw)<0.80f;
        if (!s_.calibrated) {
            if (stationary) {
                if (s_.stationary_samples<400) {
                    ++s_.stationary_samples;
                    bias_sum_+=raw;
                }
                if (s_.stationary_samples>=300) {
                    s_.bias_mps2=bias_sum_/s_.stationary_samples;
                    s_.calibrated=true;
                }
            } else {
                s_.stationary_samples=0;
                bias_sum_=0.0f;
            }
        }
        const float corrected=filtered_raw_-s_.bias_mps2;
        s_.earth_z_linear_mps2=corrected;
        // Keep KF accelerometer out of thrust path on big tilts, excessive
        // vibration/impulse or uncalibrated IMU. RAW LIDAR still gates ALT.
        if (!s_.calibrated || std::fabs(sensor_angle_x_deg)>25.0f ||
            std::fabs(sensor_angle_y_deg)>25.0f ||
            gyro_norm_dps>250.0f || !std::isfinite(corrected) ||
            std::fabs(corrected)>4.5f) {
            ++s_.rejected_samples;
            return s_;
        }
        s_.valid=true;
        return s_;
    }
    const State& state()const{return s_;}
private:
    State s_{};
    float bias_sum_=0.0f, filtered_raw_=0.0f;
    bool filter_initialized_=false;
    uint32_t last_update_ms_=0;
};
} // namespace vertical_accel_fusion
