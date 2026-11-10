#pragma once
// V6 NAV SHADOW. No Arduino dependencies; no motor/PWM connection.
// Range = slant distance along lidar beam; correction only for downward aligned sensor.
#include <cmath>
#include <cstdint>

struct NavInput {
    uint32_t now_ms = 0;
    uint32_t range_time_ms = 0;
    uint32_t flow_time_ms = 0;
    int32_t distance_mm = -1;
    int32_t flow_x_raw = 0;
    int32_t flow_y_raw = 0;
    uint8_t range_quality = 0;
    uint8_t flow_quality = 0;
    bool range_valid = false;
    bool flow_valid = false;
    bool imu_valid = false;
    float roll_deg = 0.0f;
    float pitch_deg = 0.0f;
    // Captured from control_task at 250 Hz, consumed at 50 Hz.
    // A norm far from 1 g means the acceleration cannot be treated as gravity alone.
    float acc_norm_g = 1.0f;
    float gyro_norm_dps = 0.0f;
};

enum class HeightReason : uint8_t {
    WAIT = 0, WARMUP = 1, READY = 2, STALE = 3,
    RANGE_BAD = 4, TILT_BAD = 5, OUTLIER = 6, IMU_BAD = 7,
    MOTION_BAD = 8, SETTLING = 9
};
inline const char* heightReasonName(HeightReason s) {
    switch(s) {
        case HeightReason::WAIT: return "WAIT";
        case HeightReason::WARMUP: return "WARMUP";
        case HeightReason::READY: return "READY";
        case HeightReason::STALE: return "STALE";
        case HeightReason::RANGE_BAD: return "RANGE_BAD";
        case HeightReason::TILT_BAD: return "TILT_BAD";
        case HeightReason::OUTLIER: return "OUTLIER";
        case HeightReason::MOTION_BAD: return "MOTION_BAD";
        case HeightReason::SETTLING: return "SETTLING";
        default: return "IMU_BAD";
    }
}

struct NavOutput {
    bool height_valid = false;
    bool flow_fresh = false;
    bool xy_valid = false; // ALWAYS false in V6.2: flow scale + axes uncalibrated
    bool motion_clear = false; // true only after a quiet recovery interval
    float acc_norm_g = 0.0f;
    float gyro_norm_dps = 0.0f;
    float height_m = 0.0f;
    float climb_mps = 0.0f;
    float range_vertical_m = 0.0f; // current tilt-corrected *raw* distance
    float tilt_compensation = 1.0f;
    uint32_t accepted_samples = 0;
    uint32_t rejected_samples = 0;
    uint32_t warmups_completed = 0;
    uint8_t warmup_count = 0;
    float last_outlier_h_m = 0.0f, last_outlier_delta_m = 0.0f;
    uint32_t outlier_count = 0;
    uint32_t range_age_ms = 0;
    HeightReason reason = HeightReason::WAIT;
};

struct NavConfig {
    uint32_t max_sample_age_ms = 120;
    float min_height_m = 0.08f;  // navigation test interval (not sensor full range)
    float max_height_m = 3.0f;
    float max_tilt_deg = 25.0f;
    float height_tau_s = 0.12f;
    float climb_tau_s = 0.20f;
    float max_abs_climb_mps = 2.0f;
    float outlier_base_m = 0.08f;
    float warmup_max_spread_m = 0.075f;
    uint8_t warmup_samples = 3;
    // Bench diagnostics only; NOT certified flight-time thresholds.
    float max_accel_deviation_g = 0.20f; // gate if |norm - 1| > 0.20 g
    float max_body_rate_dps = 180.0f;
    uint32_t motion_settle_ms = 300; // continuous quiet time after disturbed IMU
    uint32_t outlier_settle_ms = 180; // quiet time after abrupt range jump
};

class NavEstimator {
public:
    explicit NavEstimator(NavConfig cfg = {}) : cfg_(cfg) {}
    const NavOutput& state() const { return out_; }
    void reset() {
        out_ = {}; last_seen_ms_ = last_accepted_ms_ = 0;
        warmup_avg_ = 0.0f; initialized_ = false;
        motion_blocked_ = false; motion_good_since_ms_ = 0;
        range_outlier_blocked_ = false; range_outlier_since_ms_ = 0;
    }

    const NavOutput& step(const NavInput& in) {
        out_.xy_valid = false; // intentionally locked until separate calibration
        out_.flow_fresh = in.flow_valid && in.flow_time_ms != 0 &&
            (uint32_t)(in.now_ms - in.flow_time_ms) <= cfg_.max_sample_age_ms;
        out_.range_age_ms = in.range_time_ms == 0 ? UINT32_MAX :
            (uint32_t)(in.now_ms - in.range_time_ms);
        const bool new_sample = in.range_time_ms != 0 && in.range_time_ms != last_seen_ms_;
        if (new_sample) last_seen_ms_ = in.range_time_ms;
        out_.acc_norm_g = in.acc_norm_g;
        out_.gyro_norm_dps = in.gyro_norm_dps;
        out_.motion_clear = false;

        // Order is important: invalid raw readings must never become accepted merely
        // because the UART frame passed its CRC or range_valid was stale/incorrect.
        if (!in.imu_valid || !std::isfinite(in.roll_deg) || !std::isfinite(in.pitch_deg) ||
            !std::isfinite(in.acc_norm_g) || !std::isfinite(in.gyro_norm_dps)) {
            motion_blocked_ = true;
            motion_good_since_ms_ = 0;
            invalidate(HeightReason::IMU_BAD, new_sample); return out_;
        }
        const bool motion_bad =
            std::fabs(in.acc_norm_g - 1.0f) > cfg_.max_accel_deviation_g ||
            in.gyro_norm_dps > cfg_.max_body_rate_dps;
        if (motion_bad) {
            motion_blocked_ = true;
            motion_good_since_ms_ = 0;
            invalidate(HeightReason::MOTION_BAD, new_sample); return out_;
        }
        if (motion_blocked_) {
            if (motion_good_since_ms_ == 0) motion_good_since_ms_ = in.now_ms;
            if ((uint32_t)(in.now_ms - motion_good_since_ms_) < cfg_.motion_settle_ms) {
                invalidate(HeightReason::SETTLING, new_sample); return out_;
            }
            motion_blocked_ = false;
            motion_good_since_ms_ = 0;
        }
        out_.motion_clear = true;
        if (std::fabs(in.roll_deg) > cfg_.max_tilt_deg ||
            std::fabs(in.pitch_deg) > cfg_.max_tilt_deg) {
            invalidate(HeightReason::TILT_BAD, new_sample); return out_;
        }
        if (in.range_time_ms == 0 || out_.range_age_ms > cfg_.max_sample_age_ms) {
            invalidate(in.range_time_ms == 0 ? HeightReason::WAIT : HeightReason::STALE, false);
            return out_;
        }
        if (!in.range_valid || !in.range_quality || in.distance_mm < 0) {
            invalidate(HeightReason::RANGE_BAD, new_sample); return out_;
        }
        const float rad = 0.01745329251994329577f;
        const float scale = std::cos(in.roll_deg * rad) * std::cos(in.pitch_deg * rad);
        const float h = ((float)in.distance_mm * 0.001f) * scale;
        out_.tilt_compensation = scale;
        out_.range_vertical_m = h;
        if (!std::isfinite(h) || scale <= 0.0f ||
            h < cfg_.min_height_m || h > cfg_.max_height_m) {
            invalidate(HeightReason::RANGE_BAD, new_sample); return out_;
        }
        if (range_outlier_blocked_) {
            if ((uint32_t)(in.now_ms - range_outlier_since_ms_) < cfg_.outlier_settle_ms) {
                invalidate(HeightReason::OUTLIER, new_sample); return out_;
            }
            range_outlier_blocked_ = false;
        }
        if (!new_sample) {
            // A rejected frame must not be made valid again by repeated calls with
            // the same timestamp. Real incoming frames must earn validity.
            out_.height_valid = initialized_ &&
                out_.reason == HeightReason::READY &&
                (uint32_t)(in.now_ms - last_accepted_ms_) <= cfg_.max_sample_age_ms;
            if (!out_.height_valid && out_.reason == HeightReason::READY) {
                invalidate(HeightReason::STALE, false);
            }
            return out_;
        }
        // After a sensor dropout or rejected step, reacquire from a short stable
        // warmup instead of carrying derivative/integrator across a discontinuity.
        if (!initialized_) {
            warmup(h);
            return out_;
        }
        uint32_t interval_ms = (uint32_t)(in.range_time_ms - last_accepted_ms_);
        if (interval_ms == 0 || interval_ms > cfg_.max_sample_age_ms) {
            invalidate(HeightReason::STALE, false);
            warmup(h); return out_;
        }
        const float dt = interval_ms * 0.001f;
        const float max_jump = cfg_.outlier_base_m + cfg_.max_abs_climb_mps * dt;
        if (std::fabs(h - last_raw_h_) > max_jump) {
            // Do not turn a single large spike into a new altitude target.
            // The next stable plateau must survive a cooldown + warmup.
            out_.last_outlier_h_m = h;
            out_.last_outlier_delta_m = h - last_raw_h_;
            ++out_.outlier_count;
            range_outlier_blocked_ = true;
            range_outlier_since_ms_ = in.now_ms;
            invalidate(HeightReason::OUTLIER, true);
            return out_;
        }
        const float alpha_h = dt / (cfg_.height_tau_s + dt);
        out_.height_m += alpha_h * (h - out_.height_m);
        // Differentiate raw consecutive measurements, not the slowly filtered
        // altitude (reduces systematic vertical-speed lag).
        float raw_climb = (h - last_raw_h_) / dt;
        if (raw_climb > cfg_.max_abs_climb_mps) raw_climb = cfg_.max_abs_climb_mps;
        if (raw_climb < -cfg_.max_abs_climb_mps) raw_climb = -cfg_.max_abs_climb_mps;
        const float alpha_v = dt / (cfg_.climb_tau_s + dt);
        out_.climb_mps += alpha_v * (raw_climb - out_.climb_mps);
        last_raw_h_ = h;
        last_accepted_ms_ = in.range_time_ms;
        ++out_.accepted_samples;
        out_.reason = HeightReason::READY;
        out_.height_valid = true;
        return out_;
    }
private:
    void invalidate(HeightReason reason, bool count_reject) {
        if (count_reject) ++out_.rejected_samples;
        out_.height_valid = false;
        out_.climb_mps = 0.0f;
        out_.reason = reason;
        initialized_ = false;
        out_.warmup_count = 0;
    }
    void warmup(float h) {
        if (out_.warmup_count == 0 ||
            std::fabs(h - warmup_avg_) > cfg_.warmup_max_spread_m) {
            out_.warmup_count = 1;
            warmup_avg_ = h;
        } else {
            warmup_avg_ += (h - warmup_avg_) / (out_.warmup_count + 1);
            ++out_.warmup_count;
        }
        out_.height_valid = false;
        out_.reason = HeightReason::WARMUP;
        if (out_.warmup_count >= cfg_.warmup_samples) {
            out_.height_m = warmup_avg_;
            out_.climb_mps = 0.0f;
            last_raw_h_ = h;
            last_accepted_ms_ = last_seen_ms_;
            initialized_ = true;
            ++out_.warmups_completed;
            ++out_.accepted_samples;
            out_.height_valid = true;
            out_.reason = HeightReason::READY;
        }
    }
    NavConfig cfg_;
    NavOutput out_{};
    uint32_t last_seen_ms_ = 0;
    uint32_t last_accepted_ms_ = 0;
    float last_raw_h_ = 0.0f;
    float warmup_avg_ = 0.0f;
    bool initialized_ = false;
    bool motion_blocked_ = false;
    uint32_t motion_good_since_ms_ = 0;
    bool range_outlier_blocked_ = false;
    uint32_t range_outlier_since_ms_ = 0;
};

enum class ShadowState : uint8_t {
    OFF = 0, WAIT_HEIGHT = 1, CAPTURE = 2, TRACK = 3,
    SENSOR_LOSS = 4, RECOVERY_PENDING = 5
};
inline const char* shadowStateName(ShadowState s) {
    switch (s) {
        case ShadowState::OFF: return "OFF";
        case ShadowState::WAIT_HEIGHT: return "WAIT_HEIGHT";
        case ShadowState::CAPTURE: return "CAPTURE";
        case ShadowState::TRACK: return "TRACK";
        case ShadowState::SENSOR_LOSS: return "SENSOR_LOSS";
        default: return "RECOVERY_PENDING";
    }
}
enum class TargetCaptureReason : uint8_t {
    NONE = 0, MODE_ENTRY = 1, MODE_SWITCH = 2, SENSOR_RECOVERY = 3
};
inline const char* captureReasonName(TargetCaptureReason r) {
    switch (r) {
        case TargetCaptureReason::MODE_ENTRY: return "MODE_ENTRY";
        case TargetCaptureReason::MODE_SWITCH: return "MODE_SWITCH";
        case TargetCaptureReason::SENSOR_RECOVERY: return "SENSOR_RECOVERY";
        default: return "NONE";
    }
}
enum class CaptureGate : uint8_t {
    NONE = 0, HEIGHT_INVALID = 1, HEIGHT_BAND = 2, MOTION = 3,
    CLIMB = 4, SPREAD = 5, WAIT_SAMPLES = 6, WAIT_TIME = 7,
    NO_NEW_FRAME = 8, CONTROL_GAP = 9
};
inline const char* captureGateName(CaptureGate r) {
    switch (r) {
        case CaptureGate::HEIGHT_INVALID: return "HEIGHT_INVALID";
        case CaptureGate::HEIGHT_BAND: return "HEIGHT_BAND";
        case CaptureGate::MOTION: return "MOTION";
        case CaptureGate::CLIMB: return "CLIMB";
        case CaptureGate::SPREAD: return "SPREAD";
        case CaptureGate::WAIT_SAMPLES: return "WAIT_SAMPLES";
        case CaptureGate::WAIT_TIME: return "WAIT_TIME";
        case CaptureGate::NO_NEW_FRAME: return "NO_NEW_FRAME";
        case CaptureGate::CONTROL_GAP: return "CONTROL_GAP";
        default: return "NONE";
    }
}
struct ShadowOutput {
    ShadowState state = ShadowState::OFF;
    TargetCaptureReason last_capture_reason = TargetCaptureReason::NONE;
    CaptureGate gate = CaptureGate::NONE;
    uint32_t capture_count = 0;
    uint32_t capture_reject_count = 0;
    uint32_t stable_ms = 0, last_capture_window_ms = 0;
    uint16_t stable_samples = 0, last_capture_window_samples = 0;
    float target_m = 0.0f;
    float desired_climb_mps = 0.0f;
    float velocity_error_mps = 0.0f;
    float i_term = 0.0f;
    float normalized_correction = 0.0f; // dimensionless, never sent to ESC
    bool pos_hold_available = false;
};
struct ShadowConfig {
    float outer_kp = 1.0f;
    float max_desired_climb = 0.5f;
    float inner_kp = 0.18f;
    float inner_ki = 0.025f;
    float max_correction = 0.20f;
    float max_i = 0.07f;
    // Separate sensor validity from safe-to-capture conditions.
    // Bench diagnostic thresholds, NOT flight tuning/arming criteria.
    float capture_min_height_m = 0.30f;
    float capture_max_height_m = 1.80f;
    float track_min_height_m = 0.25f; // hysteresis (broader tracking band)
    float track_max_height_m = 2.00f;
    float capture_max_climb_mps = 0.08f;
    float capture_max_rate_dps = 20.0f;
    float capture_max_tilt_deg = 12.0f;
    float capture_max_accel_error_g = 0.10f;
    float capture_max_spread_m = 0.035f;
    uint32_t capture_stable_ms = 800;
    uint16_t capture_min_samples = 20;
    uint32_t max_new_frame_gap_ms = 120;
};
class ShadowAltHold {
public:
    explicit ShadowAltHold(ShadowConfig cfg = {}) : cfg_(cfg) {}
    void reset() {
        out_ = {}; last_ms_ = 0; last_mode_ = 0;
        clearPending(); last_observed_accepted_ = 0;
        previously_captured_ = false;
        pending_mode_reason_ = TargetCaptureReason::MODE_ENTRY;
    }
    const ShadowOutput& state() const { return out_; }
    const ShadowOutput& step(uint32_t now_ms, int requested_mode,
                              bool rc_fresh, const NavOutput& nav) {
        // Called only for explicitly disarmed bench. No motor connection.
        if ((requested_mode != 1 && requested_mode != 2) || !rc_fresh) {
            reset(); return out_;
        }
        if (last_mode_ != requested_mode) {
            const bool switching = last_mode_ != 0;
            out_ = {};
            clearPending(); last_ms_ = 0;
            previously_captured_ = false;
            last_observed_accepted_ = nav.accepted_samples;
            pending_mode_reason_ = switching ? TargetCaptureReason::MODE_SWITCH : TargetCaptureReason::MODE_ENTRY;
            last_mode_ = requested_mode;
        }
        const bool in_track_band = std::isfinite(nav.height_m) &&
            nav.height_m >= cfg_.track_min_height_m &&
            nav.height_m <= cfg_.track_max_height_m;
        if (!nav.height_valid || !in_track_band) {
            if (out_.state == ShadowState::TRACK || out_.state == ShadowState::CAPTURE ||
                out_.state == ShadowState::RECOVERY_PENDING || previously_captured_) {
                out_.state = ShadowState::SENSOR_LOSS;
            } else {
                out_.state = ShadowState::WAIT_HEIGHT;
            }
            zeroController(); clearPending();
            out_.gate = nav.height_valid ? CaptureGate::HEIGHT_BAND : CaptureGate::HEIGHT_INVALID;
            last_ms_ = 0;
            return out_;
        }
        if (out_.state == ShadowState::CAPTURE) out_.state = ShadowState::TRACK;
        if (out_.state == ShadowState::TRACK) {
            float dt = (uint32_t)(now_ms - last_ms_) * 0.001f;
            if (last_ms_ == 0 || dt <= 0.0f || dt > 0.15f) {
                out_.state = ShadowState::SENSOR_LOSS;
                out_.gate = CaptureGate::CONTROL_GAP;
                zeroController(); clearPending(); last_ms_ = 0;
                return out_;
            }
            last_ms_ = now_ms;
            const float err_h = out_.target_m - nav.height_m;
            out_.desired_climb_mps = bound(cfg_.outer_kp * err_h,
                -cfg_.max_desired_climb, cfg_.max_desired_climb);
            const float ev = out_.desired_climb_mps - nav.climb_mps;
            out_.velocity_error_mps = ev;
            const float without_i = cfg_.inner_kp * ev + out_.i_term;
            if (!((without_i >= cfg_.max_correction && ev > 0.0f) ||
                  (without_i <= -cfg_.max_correction && ev < 0.0f))) {
                out_.i_term = bound(out_.i_term + cfg_.inner_ki * ev * dt,
                                    -cfg_.max_i, cfg_.max_i);
            }
            out_.normalized_correction = bound(cfg_.inner_kp * ev + out_.i_term,
                -cfg_.max_correction, cfg_.max_correction);
            out_.gate = CaptureGate::NONE;
            return out_;
        }
        // No correction may be emitted during a new capture or any recovery.
        zeroController();
        out_.state = ShadowState::RECOVERY_PENDING;
        if (nav.height_m < cfg_.capture_min_height_m ||
            nav.height_m > cfg_.capture_max_height_m) {
            reject(CaptureGate::HEIGHT_BAND); return out_;
        }
        if (!nav.motion_clear || !std::isfinite(nav.acc_norm_g) ||
            !std::isfinite(nav.gyro_norm_dps) ||
            std::fabs(nav.acc_norm_g - 1.0f) > cfg_.capture_max_accel_error_g ||
            nav.gyro_norm_dps > cfg_.capture_max_rate_dps ||
            !std::isfinite(nav.tilt_compensation) ||
            nav.tilt_compensation < std::cos(cfg_.capture_max_tilt_deg * 0.01745329252f)) {
            reject(CaptureGate::MOTION); return out_;
        }
        if (!std::isfinite(nav.climb_mps) ||
            std::fabs(nav.climb_mps) > cfg_.capture_max_climb_mps) {
            reject(CaptureGate::CLIMB); return out_;
        }
        const bool has_fresh_accepted_sample = nav.accepted_samples != last_observed_accepted_;
        if (has_fresh_accepted_sample) last_observed_accepted_ = nav.accepted_samples;
        if (!has_fresh_accepted_sample) {
            if (pending_since_ms_ != 0 &&
                (uint32_t)(now_ms - last_pending_sample_ms_) > cfg_.max_new_frame_gap_ms) {
                reject(CaptureGate::NO_NEW_FRAME);
            } else {
                out_.gate = CaptureGate::NO_NEW_FRAME;
            }
            return out_;
        }
        if (pending_since_ms_ == 0) {
            pending_since_ms_ = now_ms;
            low_h_ = high_h_ = nav.height_m;
            out_.stable_samples = 1;
        } else {
            if (nav.height_m < low_h_) low_h_ = nav.height_m;
            if (nav.height_m > high_h_) high_h_ = nav.height_m;
            if (high_h_ - low_h_ > cfg_.capture_max_spread_m) {
                reject(CaptureGate::SPREAD); return out_;
            }
            if (out_.stable_samples < UINT16_MAX) ++out_.stable_samples;
        }
        last_pending_sample_ms_ = now_ms;
        out_.stable_ms = (uint32_t)(now_ms - pending_since_ms_);
        if (out_.stable_samples < cfg_.capture_min_samples) {
            out_.gate = CaptureGate::WAIT_SAMPLES; return out_;
        }
        if (out_.stable_ms < cfg_.capture_stable_ms) {
            out_.gate = CaptureGate::WAIT_TIME; return out_;
        }
        // Capture only after a stable, NEW-sample window. Never capture on mere READY.
        out_.last_capture_window_ms = out_.stable_ms;
        out_.last_capture_window_samples = out_.stable_samples;
        out_.target_m = nav.height_m;
        out_.last_capture_reason = previously_captured_ ?
            TargetCaptureReason::SENSOR_RECOVERY : pending_mode_reason_;
        ++out_.capture_count;
        previously_captured_ = true;
        out_.gate = CaptureGate::NONE;
        out_.state = ShadowState::CAPTURE;
        last_ms_ = now_ms;
        clearPending();
        return out_;
    }
private:
    static float bound(float x, float lo, float hi) {
        return x < lo ? lo : (x > hi ? hi : x);
    }
    void zeroController() {
        out_.desired_climb_mps = 0.0f;
        out_.velocity_error_mps = 0.0f;
        out_.i_term = 0.0f;
        out_.normalized_correction = 0.0f;
    }
    void clearPending() {
        pending_since_ms_ = 0;
        last_pending_sample_ms_ = 0;
        low_h_ = high_h_ = 0.0f;
        out_.stable_ms = 0;
        out_.stable_samples = 0;
    }
    void reject(CaptureGate gate) {
        ++out_.capture_reject_count;
        clearPending();
        out_.gate = gate;
    }
    ShadowConfig cfg_{};
    ShadowOutput out_{};
    uint32_t last_ms_ = 0;
    uint32_t pending_since_ms_ = 0;
    uint32_t last_pending_sample_ms_ = 0;
    uint32_t last_observed_accepted_ = 0;
    float low_h_ = 0, high_h_ = 0;
    int last_mode_ = 0;
    bool previously_captured_ = false;
    TargetCaptureReason pending_mode_reason_ = TargetCaptureReason::MODE_ENTRY;
};
