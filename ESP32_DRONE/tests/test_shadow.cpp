#include "../include/nav_estimator.h"
#include <cassert>
#include <cmath>
#include <cstdio>

static NavOutput good(float height, uint32_t count) {
    NavOutput n{};
    n.height_valid=true;
    n.height_m=height;
    n.climb_mps=0.0f;
    n.motion_clear=true;
    n.acc_norm_g=1.0f;
    n.gyro_norm_dps=0.2f;
    n.tilt_compensation=1.0f;
    n.accepted_samples=count;
    return n;
}
int main() {
    ShadowAltHold hold;
    NavOutput n=good(0.50f,1);
    auto out=hold.step(20,0,true,n);
    assert(out.state==ShadowState::OFF);
    // The first 800ms must NEVER output a correction or capture prematurely.
    for (uint32_t t=40,k=2;t<=800;t+=20,++k) {
        n=good(0.5f,k);
        out=hold.step(t,1,true,n);
        assert(out.state==ShadowState::RECOVERY_PENDING);
        assert(out.normalized_correction==0.0f && out.i_term==0.0f);
        assert(out.capture_count==0);
    }
    for (uint32_t t=820,k=41;t<=880;t+=20,++k) {
        n=good(0.5f,k);
        out=hold.step(t,1,true,n);
    }
    assert(out.capture_count==1 && out.target_m==0.5f);
    assert(out.last_capture_reason==TargetCaptureReason::MODE_ENTRY);
    assert(out.state==ShadowState::TRACK);
    n=good(0.40f,45); n.climb_mps=-0.10f;
    out=hold.step(900,1,true,n);
    assert(out.state==ShadowState::TRACK && out.normalized_correction>0.0f);
    n=good(0.60f,46); n.climb_mps=0.05f;
    out=hold.step(920,1,true,n);
    assert(out.normalized_correction<0.0f);
    n.height_valid=false;
    out=hold.step(940,1,true,n);
    assert(out.state==ShadowState::SENSOR_LOSS);
    assert(out.normalized_correction==0 && out.i_term==0);
    assert(out.target_m==0.5f); // keep only for history; output is zero
    // Height 9cm is sensor-READY but not safe to capture/track.
    n=good(0.09f,50);
    out=hold.step(960,1,true,n);
    assert(out.state==ShadowState::SENSOR_LOSS && out.gate==CaptureGate::HEIGHT_BAND);
    // Long stable recovery. Not every 3 samples!
    for (uint32_t t=980,k=51;t<=1760;t+=20,++k) {
        n=good(0.55f,k);
        out=hold.step(t,1,true,n);
        assert(out.normalized_correction==0.0f);
        assert(out.capture_count==1);
    }
    n=good(0.55f,91);
    out=hold.step(1800,1,true,n);
    assert(out.capture_count==2 && std::fabs(out.target_m-.55f)<.001f);
    assert(out.last_capture_reason==TargetCaptureReason::SENSOR_RECOVERY);
    // Mode switch forces a separate stable capture.
    n=good(0.65f,92);
    out=hold.step(1820,2,true,n);
    assert(out.state==ShadowState::RECOVERY_PENDING && out.normalized_correction==0);
    assert(out.capture_count==0);
    // RC lost immediately disables hold.
    out=hold.step(1840,2,false,n);
    assert(out.state==ShadowState::OFF && out.normalized_correction==0);
    puts("PASS V6.3 target stable capture / loss / recovery / low-height / mode switch");
}
