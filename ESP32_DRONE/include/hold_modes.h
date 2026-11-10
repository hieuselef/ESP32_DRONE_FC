#pragma once
// V6.3B ALT/POS controller: flight-control math for BENCH simulation only.
// Actuator output remains hard-blocked in config.h; gains/signs/hover throttle
// are NOT tuned for the real airframe. No assertion of flight readiness.
#include <cmath>
#include <cstdint>

namespace v63hold {

enum class Reason : uint8_t {
    OFF=0, RC_BLOCKED, HEIGHT_UNAVAILABLE, CAPTURE_WAIT, ALT_TRACK,
    FLOW_UNVERIFIED, FLOW_STALE, POS_TRACK, TIME_GAP
};
inline const char *reasonName(Reason r) {
    switch (r) {
        case Reason::RC_BLOCKED: return "RC_BLOCKED";
        case Reason::HEIGHT_UNAVAILABLE: return "HEIGHT_UNAVAILABLE";
        case Reason::CAPTURE_WAIT: return "CAPTURE_WAIT";
        case Reason::ALT_TRACK: return "ALT_TRACK";
        case Reason::FLOW_UNVERIFIED: return "FLOW_UNVERIFIED";
        case Reason::FLOW_STALE: return "FLOW_STALE";
        case Reason::POS_TRACK: return "POS_TRACK";
        case Reason::TIME_GAP: return "TIME_GAP";
        default: return "OFF";
    }
}
struct Input {
    uint32_t now_ms = 0;
    int requested_mode = 0; // 0 ANGLE, 1 ALT, 2 POS
    bool authorized = false; // strictly disarmed bench, RC fresh and CH5 LOW
    bool height_valid = false;
    uint32_t range_age_ms = UINT32_MAX;
    float h_m = 0, vz_mps = 0;
    float roll_deg = 0, pitch_deg = 0;
    int throttle_us = 1500, roll_us = 1500, pitch_us = 1500;
    bool flow_verified = false; // independently verified axes AND scale, NEVER candidate flag
    bool flow_valid = false;
    uint32_t flow_age_ms = UINT32_MAX;
    float vx_body_mps = 0, vy_body_mps = 0;
    float yaw_rate_dps = 0;
};
struct Output {
    int active_mode = 0; // 0 ANGLE fallback, 1 ALT, 2 POS
    Reason reason = Reason::OFF;
    bool target_captured = false;
    float target_h_m = 0, z_sp_mps = 0;
    float throttle_correction_us = 0; // virtual mixer units, NOT validated ESC command
    float target_x_m = 0, target_y_m = 0;
    float estimated_x_m = 0, estimated_y_m = 0;
    float vx_sp_mps = 0, vy_sp_mps = 0;
    float roll_target_deg = 0, pitch_target_deg = 0;
};
struct Config {
    uint32_t capture_ms = 600;
    uint32_t max_range_age_ms = 100, max_flow_age_ms = 110;
    float min_height_m = 0.25f, max_height_m = 1.80f;
    float capture_max_climb_mps = 0.12f;
    float capture_max_tilt_deg = 12.0f;
    float max_climb_sp_mps = 0.35f;
    float alt_position_kp = 1.0f;
    float alt_velocity_kp_us = 110.0f, alt_velocity_ki_us = 18.0f;
    float alt_i_max_us = 40.0f, alt_max_correction_us = 100.0f;
    float max_planar_setpoint_mps = 0.35f;
    float pos_position_kp = 0.75f, pos_velocity_kp_deg = 9.0f;
    float pos_max_angle_deg = 7.0f;
    // Signs are UNVERIFIED: only diagnostic motor mix may consume these.
    float pos_roll_sign = 1.0f, pos_pitch_sign = 1.0f;
};

class Controller {
public:
    explicit Controller(Config cfg = {}) : cfg_(cfg) {}
    void reset() { out_ = {}; requested_ = 0; capture_start_ms_ = 0; last_ms_ = 0; z_i_ = 0; yaw_rad_ = 0; pos_captured_ = false; }
    const Output& state() const { return out_; }
    const Output& step(const Input &s) {
        if (!s.authorized || s.requested_mode == 0 || s.requested_mode > 2) {
            reset(); out_.reason = s.requested_mode ? Reason::RC_BLOCKED : Reason::OFF; return out_;
        }
        if (requested_ != s.requested_mode) {
            reset(); requested_ = s.requested_mode;
        }
        if (!s.height_valid || s.range_age_ms > cfg_.max_range_age_ms ||
            !finite(s.h_m) || !finite(s.vz_mps) ||
            s.h_m < cfg_.min_height_m || s.h_m > cfg_.max_height_m) {
            resetTracking(); out_.reason = Reason::HEIGHT_UNAVAILABLE; return out_;
        }
        if (!out_.target_captured) {
            const bool stable = std::fabs(s.vz_mps) < cfg_.capture_max_climb_mps &&
                std::fabs(s.roll_deg) < cfg_.capture_max_tilt_deg &&
                std::fabs(s.pitch_deg) < cfg_.capture_max_tilt_deg;
            if (!stable) capture_start_ms_ = 0;
            else if (!capture_start_ms_) capture_start_ms_ = s.now_ms;
            if (!stable || s.now_ms - capture_start_ms_ < cfg_.capture_ms) {
                out_.active_mode = 0; out_.reason = Reason::CAPTURE_WAIT; return out_;
            }
            out_.target_h_m = s.h_m; out_.target_captured = true;
            last_ms_ = s.now_ms;
            out_.reason = Reason::ALT_TRACK;
            return out_;
        }
        const float dt = (uint32_t)(s.now_ms - last_ms_) * 0.001f;
        if (last_ms_ == 0 || !finite(dt) || dt < 0.005f || dt > 0.12f) {
            resetTracking(); out_.reason = Reason::TIME_GAP; return out_;
        }
        last_ms_ = s.now_ms;
        out_.active_mode = 1;
        out_.reason = Reason::ALT_TRACK;
        // Pilot throttle around center commands a measured climb/setpoint change.
        const float climb_cmd = deadbandStick(s.throttle_us, 55) * cfg_.max_climb_sp_mps;
        out_.target_h_m = clamp(out_.target_h_m + climb_cmd * dt,
                                 cfg_.min_height_m, cfg_.max_height_m);
        out_.z_sp_mps = clamp(cfg_.alt_position_kp*(out_.target_h_m - s.h_m) + climb_cmd,
                              -cfg_.max_climb_sp_mps, cfg_.max_climb_sp_mps);
        const float error_vz = out_.z_sp_mps - s.vz_mps;
        const float trial = cfg_.alt_velocity_kp_us * error_vz + z_i_;
        if (!((trial > cfg_.alt_max_correction_us && error_vz > 0) ||
              (trial < -cfg_.alt_max_correction_us && error_vz < 0))) {
            z_i_ = clamp(z_i_ + cfg_.alt_velocity_ki_us * error_vz * dt,
                         -cfg_.alt_i_max_us, cfg_.alt_i_max_us);
        }
        out_.throttle_correction_us = clamp(cfg_.alt_velocity_kp_us*error_vz + z_i_,
                              -cfg_.alt_max_correction_us, cfg_.alt_max_correction_us);
        if (s.requested_mode != 2) { resetPlanar(); return out_; }
        if (!s.flow_verified) { resetPlanar(); out_.reason = Reason::FLOW_UNVERIFIED; return out_; }
        if (!s.flow_valid || s.flow_age_ms > cfg_.max_flow_age_ms ||
            !finite(s.vx_body_mps) || !finite(s.vy_body_mps) ||
            !finite(s.yaw_rate_dps) || std::fabs(s.yaw_rate_dps) > 15.0f) {
            resetPlanar(); out_.reason = Reason::FLOW_STALE; return out_;
        }
        // Local frame origin is the point at which POS first engages.
        // Relative yaw is integrated from gyro and can drift. No GPS absolute position.
        if (!pos_captured_) { resetPlanar(); pos_captured_ = true; }
        yaw_rad_ += s.yaw_rate_dps * (0.017453292519943f * dt);
        const float cy = std::cos(yaw_rad_), sy = std::sin(yaw_rad_);
        const float vx_world = cy*s.vx_body_mps - sy*s.vy_body_mps;
        const float vy_world = sy*s.vx_body_mps + cy*s.vy_body_mps;
        out_.estimated_x_m += vx_world*dt;
        out_.estimated_y_m += vy_world*dt;
        const float stick_vx = deadbandStick(s.pitch_us, 40) * cfg_.max_planar_setpoint_mps;
        const float stick_vy = deadbandStick(s.roll_us, 40) * cfg_.max_planar_setpoint_mps;
        out_.target_x_m += (cy*stick_vx - sy*stick_vy)*dt;
        out_.target_y_m += (sy*stick_vx + cy*stick_vy)*dt;
        const float vx_sp_world = clamp(cfg_.pos_position_kp*(out_.target_x_m-out_.estimated_x_m),
                                        -cfg_.max_planar_setpoint_mps,cfg_.max_planar_setpoint_mps);
        const float vy_sp_world = clamp(cfg_.pos_position_kp*(out_.target_y_m-out_.estimated_y_m),
                                        -cfg_.max_planar_setpoint_mps,cfg_.max_planar_setpoint_mps);
        out_.vx_sp_mps = cy*vx_sp_world + sy*vy_sp_world;
        out_.vy_sp_mps = -sy*vx_sp_world + cy*vy_sp_world;
        out_.pitch_target_deg = clamp(cfg_.pos_pitch_sign*cfg_.pos_velocity_kp_deg *
                                     (out_.vx_sp_mps - s.vx_body_mps),
                                     -cfg_.pos_max_angle_deg,cfg_.pos_max_angle_deg);
        out_.roll_target_deg = clamp(cfg_.pos_roll_sign*cfg_.pos_velocity_kp_deg *
                                    (out_.vy_sp_mps - s.vy_body_mps),
                                    -cfg_.pos_max_angle_deg,cfg_.pos_max_angle_deg);
        out_.active_mode = 2;
        out_.reason = Reason::POS_TRACK;
        return out_;
    }
private:
    static bool finite(float x) { return std::isfinite(x); }
    static float clamp(float x,float lo,float hi) { return x<lo ? lo : (x>hi ? hi : x); }
    static float deadbandStick(int us, int db) {
        int d=us-1500;
        if (d >= -db && d <= db) return 0.0f;
        return clamp(static_cast<float>(d)/500.0f,-1.0f,1.0f);
    }
    void resetPlanar() {
        out_.roll_target_deg=out_.pitch_target_deg=0;
        out_.estimated_x_m=out_.estimated_y_m=0;
        out_.target_x_m=out_.target_y_m=0;
        out_.vx_sp_mps=out_.vy_sp_mps=0;
        yaw_rad_=0; pos_captured_=false;
    }
    void resetTracking() {
        out_={}; capture_start_ms_=last_ms_=0; z_i_=0; resetPlanar();
    }
    Config cfg_{};
    Output out_{};
    int requested_=0;
    uint32_t capture_start_ms_=0,last_ms_=0;
    float z_i_=0,yaw_rad_=0;
    bool pos_captured_=false;
};
} // namespace v63hold
