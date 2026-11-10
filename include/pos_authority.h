#pragma once
// POS -> ANGLE setpoint bridge; NO motors controlled here.
// Default firmware supplies enabled=false. The only goal is to make a future,
// *verified* integration fail-closed without changing the existing ALT path.
#include <cmath>
#include <cstdint>
#include <climits>

// Explicit, single namespace opening; keep paired with closing brace below.
namespace pos_authority {
// True age includes NAV->CTRL snapshot latency (not just age when NAV ran).
inline uint32_t effectiveAge(uint32_t sensor_age_ms, uint32_t nav_lag_ms) {
    if (sensor_age_ms > 1000 || nav_lag_ms > 1000 ||
        sensor_age_ms > UINT32_MAX - nav_lag_ms) return UINT32_MAX;
    return sensor_age_ms + nav_lag_ms;
}

// Avoid enum member DISABLED: esp32-hal-gpio.h defines DISABLED as a macro.
enum class Reason : uint8_t {
    OFF=0, POS_DISABLED, ALT_INACTIVE, UNVERIFIED, PILOT_OVERRIDE,
    NAV_STALE, FLOW_LOST, BAD_TARGET, ACQUIRING, ACTIVE, LOSS_LATCHED
};
inline const char *reasonName(Reason r) {
    switch (r) {
        case Reason::POS_DISABLED: return "DISABLED";
        case Reason::ALT_INACTIVE: return "ALT_INACTIVE";
        case Reason::UNVERIFIED: return "UNVERIFIED";
        case Reason::PILOT_OVERRIDE: return "PILOT_OVERRIDE";
        case Reason::NAV_STALE: return "NAV_STALE";
        case Reason::FLOW_LOST: return "FLOW_LOST";
        case Reason::BAD_TARGET: return "BAD_TARGET";
        case Reason::ACQUIRING: return "ACQUIRING";
        case Reason::ACTIVE: return "ACTIVE";
        case Reason::LOSS_LATCHED: return "LOSS_LATCHED";
        default: return "OFF";
    }
}
struct Input {
    uint32_t now_ms=0, nav_updated_ms=0;
    bool enabled=false, armed=false, rc_ok=false, imu_ok=false, timing_ok=false;
    bool alt_active=false, scale_verified=false, signs_verified=false;
    bool rotation_compensation_verified=false;
    int requested_mode=0, roll_us=1500, pitch_us=1500, yaw_us=1500;
    bool xy_valid=false, flow_valid=false;
    uint8_t pos_active_mode=0;
    uint32_t flow_age_ms=UINT32_MAX, range_age_ms=UINT32_MAX;
    float roll_cmd_deg=0, pitch_cmd_deg=0;
};
struct Output {
    bool applied=false, latched=false;
    Reason reason=Reason::OFF;
    float roll_deg=0, pitch_deg=0;
};
class Gate {
public:
    const Output &state() const { return out_; }
    void reset() { out_={}; qualifying_since_ms_=0; ever_applied_=false; }
    const Output &step(const Input &in, float dt_s) {
        // Leaving CH6 high always clears a POS-only fault latch. Other flight
        // modes and ALT control are not touched by this gate.
        if (in.requested_mode != 2 || !in.armed) {
            reset(); return out_;
        }
        if (!in.enabled) { reset(); out_.reason=Reason::POS_DISABLED; return out_; }
        if (out_.latched) return idle(Reason::LOSS_LATCHED, true);
        if (!in.rc_ok || !in.imu_ok || !in.timing_ok || !in.alt_active)
            return reject(Reason::ALT_INACTIVE);
        if (!in.scale_verified || !in.signs_verified ||
            !in.rotation_compensation_verified)
            return reject(Reason::UNVERIFIED);
        // Stick input takes control immediately; reselect CH6 to try POS again.
        // Do NOT send 2D commands on top of a deliberate pilot ANGLE command.
        if (std::abs(in.roll_us-1500)>80 || std::abs(in.pitch_us-1500)>80 ||
            std::abs(in.yaw_us-1500)>80)
            return reject(Reason::PILOT_OVERRIDE, true);
        if (!in.nav_updated_ms || (uint32_t)(in.now_ms-in.nav_updated_ms)>80)
            return reject(Reason::NAV_STALE);
        if (!in.xy_valid || !in.flow_valid || in.pos_active_mode!=2 ||
            in.flow_age_ms>100 || in.range_age_ms>100)
            return reject(Reason::FLOW_LOST);
        if (!std::isfinite(in.roll_cmd_deg) || !std::isfinite(in.pitch_cmd_deg) ||
            std::fabs(in.roll_cmd_deg)>7.01f || std::fabs(in.pitch_cmd_deg)>7.01f ||
            !std::isfinite(dt_s) || dt_s<=0.0f || dt_s>0.03f)
            return reject(Reason::BAD_TARGET);
        if (!qualifying_since_ms_) qualifying_since_ms_=in.now_ms;
        if (in.now_ms-qualifying_since_ms_<300)
            return idle(Reason::ACQUIRING, false);
        // Conservative command envelope + slew; true PID gains & signs still
        // require real hardware validation. This is not a flight certification.
        const float cap=5.0f, max_step=12.0f*dt_s;
        const float roll=clamp(in.roll_cmd_deg,-cap,cap);
        const float pitch=clamp(in.pitch_cmd_deg,-cap,cap);
        out_.roll_deg+=clamp(roll-out_.roll_deg,-max_step,max_step);
        out_.pitch_deg+=clamp(pitch-out_.pitch_deg,-max_step,max_step);
        out_.applied=true; out_.reason=Reason::ACTIVE; ever_applied_=true;
        return out_;
    }
private:
    static float clamp(float x,float a,float b){return x<a?a:(x>b?b:x);}
    const Output &idle(Reason why,bool latch) {
        out_.applied=false; out_.roll_deg=out_.pitch_deg=0;
        out_.reason=why; out_.latched=latch; return out_;
    }
    const Output &reject(Reason why, bool force_latch=false) {
        const bool latch=force_latch || ever_applied_;
        qualifying_since_ms_=0;
        return idle(why,latch);
    }
    Output out_{};
    uint32_t qualifying_since_ms_=0;
    bool ever_applied_=false;
};
} // end namespace pos_authority (single closing brace)
