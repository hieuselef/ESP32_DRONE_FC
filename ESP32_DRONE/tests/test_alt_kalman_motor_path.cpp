#include "alt_kalman_authority.h"
#include "throttle_handover.h"
#include <cassert>
#include <cmath>
#include <cstdio>

using alt_kalman_authority::Gate;
using alt_kalman_authority::KF_LIVE;
using alt_kalman_authority::RAW_FALLBACK_LATCHED;
using alt_kalman_authority::RANGE_INVALID;
int main() {
    flight_alt::RangeTracker raw;
    vertical_estimator::Kalman2 kalman;
    Gate gate;
    flight_alt::AltController alt;
    throttle_handover::Controller collective;
    uint32_t frame=1000;
    bool seenKalman=false, changedThrottle=false;
    int minOut=9999,maxOut=0;
    // 250Hz control with a 50Hz lidar. An increasing target altitude creates
    // a non-zero throttle correction. The only feedback is gate.feedback.
    for (unsigned i=0;i<550;i++) {
        const uint32_t now=1000+i*4;
        if (i%5==0) frame=now;
        const int mm = i<240 ? 800 : 800 + static_cast<int>(0.001f*(i-240)*4*60.0f);
        const auto& r=raw.update(now,frame,mm,true,0,0);
        const auto& k=kalman.update(now,r.valid,r.new_sample,r.frame_ms,r.measured_height_m);
        const auto& g=gate.update(now,true,true,r,k);
        const int stick=i<230 ? 1450 : 1540;
        const auto& a=alt.update(now,true,true,g.feedback,stick,true);
        assert(a.controlling && a.reason==1);
        const auto& t=collective.update(now,true,a.controlling,a.throttle_us,stick);
        assert(t.phase==throttle_handover::ALT);
        assert(t.output_us>=1100 && t.output_us<=1850);
        minOut=std::min(minOut,t.output_us); maxOut=std::max(maxOut,t.output_us);
        assert(std::fabs(a.correction_us)<=70.01f);
        if (g.phase==KF_LIVE) {
            seenKalman=true;
            assert(g.kalman_used && g.blend>.99f);
            assert(std::fabs(a.height_m-g.feedback.height_m)<.00001f);
            assert(std::fabs(a.vz_mps-g.feedback.vz_mps)<.00001f);
            if (std::fabs(a.correction_us)>1.0f) changedThrottle=true;
        }
    }
    assert(seenKalman && changedThrottle);
    assert(maxOut>minOut);
    // Kalman stops being credible: use raw range, and latch KF off until ALT exit.
    auto r=raw.update(3200,3200,825,true,0,0);
    auto k=kalman.update(3200,r.valid,r.new_sample,r.frame_ms,r.measured_height_m);
    // Simulate rejected correction despite syntactically valid range.
    auto fake=k;
    fake.healthy=false;
    const auto& g=gate.update(3200,true,true,r,fake);
    assert(g.phase==RAW_FALLBACK_LATCHED && g.fallback_latched);
    assert(!g.kalman_used && std::fabs(g.feedback.height_m-r.height_m)<1e-6f);
    const auto& g2=gate.update(3204,true,true,r,k);
    assert(g2.phase==RAW_FALLBACK_LATCHED);
    // Switching to ANGLE explicitly clears KF fallback latch.
    const auto& off=gate.update(3210,false,true,r,k);
    assert(off.phase==alt_kalman_authority::RAW_ONLY);
    assert(!off.fallback_latched);
    // Invalid raw ranging always denies height feedback even if KF is healthy.
    r=raw.update(3400,3200,825,true,0,0);
    assert(!r.valid);
    const auto& bad=gate.update(3400,true,true,r,k);
    assert(bad.phase==RANGE_INVALID && !bad.feedback.valid);
    const auto& stopped=alt.update(3400,true,true,bad.feedback,1450,true);
    assert(!stopped.controlling && stopped.reason==2);
    // Existing ALT fault latch refuses automatic reactivation on recovered range.
    r=raw.update(3420,3420,825,true,0,0);
    const auto& krec=kalman.update(3420,r.valid,r.new_sample,r.frame_ms,r.measured_height_m);
    const auto& grec=gate.update(3420,true,true,r,krec);
    const auto& stillOff=alt.update(3420,true,true,grec.feedback,1450,true);
    assert(!stillOff.controlling && stillOff.reason==2);
    std::puts("PASS Kalman Z drives guarded ALT throttle, output bounds, raw fallback latch, stale and ALT latch");
}
