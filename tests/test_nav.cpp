#include "../include/nav_estimator.h"
#include <cassert>
#include <cmath>
#include <cstdio>
static void sample(NavEstimator& n, NavInput& in, uint32_t t, int32_t mm) {
    in.now_ms=t; in.range_time_ms=t; in.distance_mm=mm; n.step(in);
}
static NavInput initial() {
    NavInput in{}; in.imu_valid=true; in.range_valid=true;
    in.range_quality=255; in.flow_valid=true; in.flow_quality=100;
    in.acc_norm_g=1.0f; return in;
}
int main() {
    NavEstimator n;
    NavInput in=initial();
    sample(n,in,20,300); assert(!n.state().height_valid);
    sample(n,in,40,300); assert(!n.state().height_valid);
    sample(n,in,60,300); assert(n.state().height_valid);
    assert(n.state().reason==HeightReason::READY);
    assert(std::fabs(n.state().height_m-0.3f)<.001f);
    assert(n.state().accepted_samples==1);
    in.now_ms=70; n.step(in); assert(n.state().accepted_samples==1);
    sample(n,in,80,320); assert(n.state().height_valid);
    assert(n.state().climb_mps>0);
    sample(n,in,100,-3); assert(!n.state().height_valid);
    assert(n.state().reason==HeightReason::RANGE_BAD);
    sample(n,in,120,20); assert(!n.state().height_valid);
    sample(n,in,140,300); sample(n,in,160,300); sample(n,in,180,300);
    assert(n.state().height_valid);
    // 1m sudden jump must not be reaccepted after just three packets.
    sample(n,in,200,1400); assert(!n.state().height_valid);
    assert(n.state().reason==HeightReason::OUTLIER);
    for(uint32_t t=220;t<380;t+=20) {
        sample(n,in,t,1400);
        assert(!n.state().height_valid);
        assert(n.state().reason==HeightReason::OUTLIER);
    }
    sample(n,in,380,1400); assert(!n.state().height_valid);
    sample(n,in,400,1400); assert(!n.state().height_valid);
    sample(n,in,420,1400); assert(n.state().height_valid);
    assert(std::fabs(n.state().height_m-1.4f)<.001f);
    // Stale frame: never carry old velocity.
    in.now_ms=700; n.step(in); assert(!n.state().height_valid);
    assert(n.state().reason==HeightReason::STALE);
    in.roll_deg=30.0f;
    sample(n,in,720,300); assert(!n.state().height_valid);
    assert(n.state().reason==HeightReason::TILT_BAD);
    in.roll_deg=0;
    in.pitch_deg=20.f;
    sample(n,in,740,1000); sample(n,in,760,1000); sample(n,in,780,1000);
    assert(n.state().height_valid);
    assert(std::fabs(n.state().height_m-std::cos(20.*3.14159265359/180.))<.01f);
    assert(!n.state().xy_valid);
    puts("PASS v6.2 nav estimator tests");
}
