#pragma once
// Opt-in Kalman-Z -> ALT feedback selection (height and vertical speed).
// No ESC or RC access. The already-validated RangeTracker is ALWAYS the safety
// gate; a predicted Kalman state alone is NEVER a valid range measurement.
#include "flight_altitude.h"
#include "vertical_estimator.h"
#include <algorithm>
#include <cmath>
#include <cstdint>

namespace alt_kalman_authority {
enum Phase : uint8_t {
    RAW_ONLY=0, KF_WARMUP=1, KF_BLEND=2, KF_LIVE=3,
    RAW_FALLBACK_LATCHED=4, RANGE_INVALID=5
};
struct State {
    flight_alt::RangeState feedback{};
    Phase phase=RAW_ONLY;
    bool kalman_used=false;
    bool fallback_latched=false;
    uint16_t good_frames=0;
    float blend=0.0f;
};
class Gate {
public:
    void reset() {
        s_={};
        last_step_ms_=0;
        last_checked_frame_ms_=0;
        good_frames_=0;
        blending_=0.0f;
        ever_used_kf_=false;
        fallback_latched_=false;
    }
    const State& update(uint32_t now_ms, bool altitude_requested, bool enabled,
                        const flight_alt::RangeState& range,
                        const vertical_estimator::State& kf,
                        bool require_imu_accel=false) {
        if (!altitude_requested || !enabled) {
            reset(); s_.feedback=range; return s_;
        }
        s_.feedback=range;
        s_.kalman_used=false;
        s_.fallback_latched=fallback_latched_;
        s_.good_frames=good_frames_;
        s_.blend=blending_;
        if (!range.valid || range.age_ms>140U || !std::isfinite(range.height_m) ||
            !std::isfinite(range.vz_mps)) {
            s_.phase=RANGE_INVALID;
            // Range loss is a mandatory ALT-controller fault regardless of KF.
            // If KF had ever reached the ALT path, do not resume it until CH6
            // exits ALT/POS. The existing ALT range-fault latch still applies.
            if (ever_used_kf_) fallback_latched_=true;
            good_frames_=0;
            blending_=0.0f;
            s_.fallback_latched=fallback_latched_;
            s_.good_frames=0;
            s_.blend=0.0f;
            s_.feedback.valid=false;
            return s_;
        }
        // Conservative provisional confidence and agreement bounds. Tightening
        // must follow real logs; do not assume these constitute flight tuning.
        const bool good = kf.initialized && kf.healthy &&
            (!require_imu_accel || kf.imu_accel_used) &&
            kf.last_measurement_age_ms<=140U &&
            std::isfinite(kf.height_m) && std::isfinite(kf.velocity_mps) &&
            std::isfinite(kf.sigma_height_m) && std::isfinite(kf.sigma_velocity_mps) &&
            kf.sigma_height_m<=0.10f && kf.sigma_velocity_mps<=0.65f &&
            kf.height_m>=0.20f && kf.height_m<=1.80f &&
            std::fabs(kf.velocity_mps)<=1.50f &&
            std::fabs(kf.height_m-range.height_m)<=0.12f &&
            std::fabs(kf.velocity_mps-range.vz_mps)<=0.80f;
        // A rejected UNIQUE frame or loss of confidence during KF actuation
        // immediately latches raw fallback (requires CH6 exit to re-enable).
        const bool new_raw = range.new_sample && range.frame_ms &&
                             range.frame_ms != last_checked_frame_ms_;
        if (new_raw) {
            last_checked_frame_ms_=range.frame_ms;
            if (good && kf.measurement_used)
                good_frames_=std::min<uint16_t>(good_frames_+1,1000);
            else
                good_frames_=0;
        }
        if (ever_used_kf_ && (!good || (new_raw && !kf.measurement_used)))
            fallback_latched_=true;
        if (fallback_latched_) {
            s_.phase=RAW_FALLBACK_LATCHED;
            s_.fallback_latched=true;
            s_.blend=blending_;
            s_.good_frames=good_frames_;
            return s_;
        }
        if (!good || good_frames_<8) {
            s_.phase=KF_WARMUP;
            s_.good_frames=good_frames_;
            return s_;
        }
        // Gradual transition from existing range-filter feedback; Kalman is
        // allowed to reach mixer ONLY via the existing AltController limits.
        const uint32_t delta=last_step_ms_ ? uint32_t(now_ms-last_step_ms_) : 0U;
        last_step_ms_=now_ms;
        blending_=std::min(1.0f,blending_ + std::min(delta,30U)/600.0f);
        const float a=blending_;
        s_.feedback.height_m=(1.0f-a)*range.height_m+a*kf.height_m;
        s_.feedback.vz_mps=(1.0f-a)*range.vz_mps+a*kf.velocity_mps;
        // range.valid and frame age remain RAW RangeTracker state.
        s_.phase=a>=1.0f ? KF_LIVE : KF_BLEND;
        s_.kalman_used=a>0.0f;
        s_.blend=a;
        s_.good_frames=good_frames_;
        ever_used_kf_=ever_used_kf_ || s_.kalman_used;
        return s_;
    }
    const State& state() const { return s_; }
private:
    State s_{};
    uint32_t last_step_ms_=0, last_checked_frame_ms_=0;
    uint16_t good_frames_=0;
    float blending_=0.0f;
    bool ever_used_kf_=false, fallback_latched_=false;
};
} // namespace alt_kalman_authority
