#pragma once
// Experimental rangefinder-based ALT algorithm. Not flight-proven.
// Independent of motion-gated NAV shadow (which is for calibration).
#include <cmath>
#include <cstdint>

namespace flight_alt {
inline float clip(float x,float a,float b){return x<a?a:(x>b?b:x);}
struct RangeState {
    bool valid=false;
    float height_m=0.0f;
    float measured_height_m=0.0f; // validated tilt-compensated NEW sample, before tracker LPF
    float vz_mps=0.0f;
    uint32_t age_ms=UINT32_MAX;
    uint32_t rejected=0;
    uint32_t frame_ms=0;   // timestamp of last accepted unique range frame
    bool new_sample=false; // true for one RangeTracker::update call only
};
class RangeTracker {
public:
    void reset(){ state_={}; last_frame_ms_=0; last_valid_frame_ms_=0; holdoff_until_ms_=0; }
    const RangeState& update(uint32_t now,uint32_t frame_ms,int32_t mm,
                             bool range_ok,float roll_deg,float pitch_deg){
        state_.new_sample=false;
        state_.age_ms = frame_ms ? (uint32_t)(now-frame_ms) : UINT32_MAX;
        if (!frame_ms || state_.age_ms>140 || !range_ok) {
            state_.valid=false; return state_;
        }
        if (frame_ms != last_frame_ms_) {
            last_frame_ms_=frame_ms;
            if (static_cast<int32_t>(now-holdoff_until_ms_)<0) {
                state_.valid=false; return state_;
            }
            const float cr=std::cos(roll_deg*0.0174532925f),
                        cp=std::cos(pitch_deg*0.0174532925f);
            const float height=mm*0.001f*cr*cp;
            if (!std::isfinite(height) || height<0.20f || height>1.80f ||
                std::fabs(roll_deg)>25.0f || std::fabs(pitch_deg)>25.0f) {
                ++state_.rejected; state_.valid=false; return state_;
            }
            const float frame_dt=last_valid_frame_ms_ ? (uint32_t)(frame_ms-last_valid_frame_ms_)*0.001f : 0.0f;
            if (state_.valid && frame_dt<0.20f && std::fabs(height-state_.height_m)>0.16f) {
                ++state_.rejected; state_.valid=false; holdoff_until_ms_=now+300; return state_;
            }
            if (!state_.valid || frame_dt<=0 || frame_dt>0.20f) {
                state_.height_m=height; state_.vz_mps=0.0f;
            } else {
                // Range LPF ~ 0.12s; vertical speed LPF ~ 0.25s.
                const float alpha=clip(frame_dt/(0.12f+frame_dt),0.02f,1.0f);
                const float prior=state_.height_m;
                state_.height_m+=alpha*(height-state_.height_m);
                const float vel=(state_.height_m-prior)/frame_dt;
                const float av=clip(frame_dt/(0.25f+frame_dt),0.01f,1.0f);
                state_.vz_mps+=av*(clip(vel,-2.0f,2.0f)-state_.vz_mps);
            }
            last_valid_frame_ms_=frame_ms;
            state_.measured_height_m=height;
            state_.frame_ms=frame_ms;
            state_.new_sample=true;
            state_.valid=true;
        }
        return state_;
    }
    const RangeState& state() const{return state_;}
private:
    RangeState state_{};
    uint32_t last_frame_ms_=0,last_valid_frame_ms_=0,holdoff_until_ms_=0;
};
struct AltState {
    bool controlling=false;
    float target_m=0.0f, height_m=0.0f, vz_mps=0.0f;
    int hover_us=0, throttle_us=0;
    float correction_us=0.0f;
    uint8_t reason=0; // 0 manual/off, 1 holding, 2 range invalid, 3 pilot override
};
class AltController {
public:
    void reset(){ clearTracking(); override_latched_=false; range_fault_latched_=false; }
    // Keep faults latched until the pilot deliberately exits ALT mode.
    // A recovered/stale laser reading must never re-engage ALT unexpectedly.
    void clearTracking(){ st_={}; integral_=0; last_ms_=0; }
    const AltState& update(uint32_t now,bool requested,bool pilot_active,
                           const RangeState& range,int stick_throttle_us,
                           bool alt_v2_low_stick_descent=false) {
        if (!requested) { reset(); return st_; }
        // Disarm/throttle stop never clears a latched ALT fault by itself.
        if (!pilot_active) {
            clearTracking();
            st_.reason = override_latched_ ? 3 : (range_fault_latched_ ? 2 : 0);
            return st_;
        }
        if (override_latched_) { clearTracking(); st_.reason=3; return st_; }
        if (range_fault_latched_) { clearTracking(); st_.reason=2; return st_; }
        if (!range.valid) {
            clearTracking(); range_fault_latched_=true; st_.reason=2; return st_;
        }
        if (!st_.controlling) {
            clearTracking(); st_.controlling=true; st_.reason=1;
            st_.height_m=range.height_m; st_.vz_mps=range.vz_mps;
            st_.target_m=range.height_m;
            st_.hover_us=stick_throttle_us;
            st_.throttle_us=stick_throttle_us;
            last_ms_=now;
            return st_;
        }
        // Large stick deviation means intentional pilot throttle take-over.
        // V2: low stick is a descent command, NOT an implicit throttle-cut.
        // High stick remains immediate pilot override (>300us above capture).
        // Leaving ALT on CH6 or DISARM remains available at all times.
        const bool pilotOverride = alt_v2_low_stick_descent ?
            (stick_throttle_us - st_.hover_us > 300) :
            (std::abs(stick_throttle_us-st_.hover_us) > 300);
        if (pilotOverride) {
            clearTracking(); override_latched_=true; st_.reason=3; return st_;
        }
        const float dt=(uint32_t)(now-last_ms_)*0.001f;
        last_ms_=now;
        if (dt<0.001f || dt>0.050f) {
            clearTracking(); range_fault_latched_=true; st_.reason=2; return st_;
        }
        int deviation=stick_throttle_us-st_.hover_us;
        if (std::abs(deviation)<65) deviation=0;
        const float climb_cmd=clip(deviation/400.0f,-1.0f,1.0f)*0.25f;
        st_.target_m=clip(st_.target_m+climb_cmd*dt,0.25f,1.7f);
        const float vz_sp=clip(1.2f*(st_.target_m-range.height_m)+climb_cmd,-0.30f,0.30f);
        const float verr=vz_sp-range.vz_mps;
        const float raw=75.0f*verr + integral_;
        if (!((raw>70.0f && verr>0) || (raw<-70.0f && verr<0)))
            integral_=clip(integral_+14.0f*verr*dt,-28.0f,28.0f);
        st_.correction_us=clip(75.0f*verr+integral_,-70.0f,70.0f);
        st_.height_m=range.height_m; st_.vz_mps=range.vz_mps;
        st_.throttle_us=static_cast<int>(std::lround(clip(st_.hover_us+st_.correction_us,1100.0f,1850.0f)));
        st_.reason=1;
        return st_;
    }
    const AltState& state()const{return st_;}
private:
    AltState st_{};
    float integral_=0.0f;
    uint32_t last_ms_=0;
    bool override_latched_=false;
    bool range_fault_latched_=false;
};
}
