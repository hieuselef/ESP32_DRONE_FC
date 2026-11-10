#pragma once
// Throttle output transition after leaving ALT HOLD. Host-testable, no hardware IO.
// This only adjusts collective throttle; never touches RC arm/failsafe, rate PID,
// mixer mapping or the permanently disabled POS HOLD motor path.
#include <cstdint>
#include <algorithm>

namespace throttle_handover {
// 0 SAFE_STOP, 1 MANUAL, 2 ALT, 3 BLENDING, 4 PILOT_FULL_THROTTLE_BYPASS
constexpr uint8_t SAFE_STOP=0, MANUAL=1, ALT=2, BLENDING=3, EMERGENCY_BYPASS=4;
constexpr float SLEW_US_PER_S = 1500.0f; // max 6 logical us / 4ms loop
constexpr int DIRECT_PILOT_HIGH_US = 1800; // deliberate high-stick authority
constexpr int THROTTLE_MIN_US = 1100;
constexpr int THROTTLE_MAX_US = 1850;
struct State {
    int output_us=1000;
    int pilot_us=1000;
    uint8_t phase=SAFE_STOP;
    bool blending=false;
    uint32_t transitions=0;
    uint32_t emergency_bypasses=0;
    uint32_t last_transition_ms=0;
    int last_step_us=0;
    uint32_t max_blend_step_us=0;
};
class Controller {
public:
    void reset() { last_us_=1000; last_ms_=0; previous_alt_=false;
                   blending_=false; state_={};
                   state_.transitions=transitions_;
                   state_.emergency_bypasses=emergency_bypasses_;
                   state_.last_transition_ms=last_transition_ms_; }
    const State& update(uint32_t now_ms, bool motor_active, bool alt_controlling,
                        int alt_us, int pilot_us) {
        // Stop and disarm take precedence. Never delay safety/failsafe shutdown.
        if (!motor_active) { reset(); return state_; }
        const int pilot=clampUs(pilot_us);
        const int old_output=last_us_;
        state_.last_step_us=0;
        state_.pilot_us=pilot;
        if (alt_controlling) {
            const int next=clampUs(alt_us);
            state_.output_us=last_us_=next;
            state_.phase=ALT; state_.blending=false;
            previous_alt_=true; blending_=false; last_ms_=now_ms;
            return state_;
        }
        if (previous_alt_) { // first manual tick following ALT->manual fallback
            previous_alt_=false;
            blending_=true;
            ++transitions_;
            last_transition_ms_=now_ms;
        }
        if (blending_ && pilot>=DIRECT_PILOT_HIGH_US) {
            // Strong intentional command takes priority over smoothing.
            blending_=false;
            last_us_=pilot;
            state_.output_us=pilot;
            state_.phase=EMERGENCY_BYPASS;
            ++emergency_bypasses_;
        } else if (blending_) {
            // Limit dt to prevent a large step after a delayed scheduler tick.
            const uint32_t elapsed=now_ms-last_ms_;
            const uint32_t capped=std::min<uint32_t>(elapsed,20U);
            const int step=std::max(1, static_cast<int>((SLEW_US_PER_S*capped)/1000.0f));
            if (last_us_ < pilot) last_us_=std::min(pilot,last_us_+step);
            else if (last_us_ > pilot) last_us_=std::max(pilot,last_us_-step);
            if (last_us_==pilot) blending_=false;
            state_.output_us=last_us_;
            const uint32_t step_us=static_cast<uint32_t>(
                last_us_>old_output ? last_us_-old_output : old_output-last_us_);
            state_.max_blend_step_us=std::max(state_.max_blend_step_us,step_us);
            state_.phase=blending_ ? BLENDING : MANUAL;
        } else {
            last_us_=pilot;
            state_.output_us=pilot;
            state_.phase=MANUAL;
        }
        state_.last_step_us=last_us_-old_output;
        state_.blending=blending_;
        state_.transitions=transitions_;
        state_.emergency_bypasses=emergency_bypasses_;
        state_.last_transition_ms=last_transition_ms_;
        last_ms_=now_ms;
        return state_;
    }
    const State& state()const{return state_;}
private:
    static int clampUs(int us) {
        return us<THROTTLE_MIN_US ? THROTTLE_MIN_US :
               (us>THROTTLE_MAX_US ? THROTTLE_MAX_US : us);
    }
    int last_us_=1000;
    uint32_t last_ms_=0;
    bool previous_alt_=false, blending_=false;
    uint32_t transitions_=0, emergency_bypasses_=0, last_transition_ms_=0;
    State state_{};
};
} // namespace throttle_handover
