#include "../include/nav_estimator.h"
#include <cassert>
#include <cstdio>
static NavOutput ok(uint32_t count, float h=.50f, float v=0.0f) {
    NavOutput n{};
    n.height_valid=true; n.height_m=h; n.climb_mps=v;
    n.acc_norm_g=1; n.gyro_norm_dps=.5f;
    n.tilt_compensation=1; n.motion_clear=true; n.accepted_samples=count;
    return n;
}
int main() {
    // 3 READY frames must not permit a capture, nor can stale frames advance the window.
    ShadowAltHold c;
    for (uint32_t t=100; t<=1200; t+=20) {
        auto x=ok(t<=140 ? t/20 : 7);
        auto out=c.step(t,1,true,x);
        assert(out.capture_count==0 && out.normalized_correction==0);
    }
    assert(c.state().gate==CaptureGate::NO_NEW_FRAME);
    c.reset();
    for(uint32_t t=100, k=1; t<2000; t+=20, ++k) {
        auto x=ok(k,.51f,.15f);
        auto out=c.step(t,1,true,x);
        assert(out.capture_count==0 && out.normalized_correction==0);
        assert(out.gate==CaptureGate::CLIMB);
    }
    c.reset();
    for (uint32_t t=100,k=1;t<1500;t+=20,++k) {
        float h=(k%4==0)?0.59f:0.50f;
        auto x=ok(k,h);
        auto out=c.step(t,1,true,x);
        assert(out.capture_count==0);
    }
    assert(c.state().capture_reject_count>0);
    c.reset();
    for (uint32_t t=100,k=1;t<1500;t+=20,++k) {
        auto x=ok(k,0.09f);
        auto out=c.step(t,1,true,x);
        assert(out.state==ShadowState::WAIT_HEIGHT);
        assert(out.gate==CaptureGate::HEIGHT_BAND);
        assert(out.capture_count==0);
    }
    puts("PASS V6.3 no stale capture, motion, spread, low-height guards");
}
