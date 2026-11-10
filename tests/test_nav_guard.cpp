#include "../include/nav_estimator.h"
#include <cassert>
#include <cstdio>
static void push(NavEstimator& n, NavInput& i, uint32_t t, float acc, float rate=0) {
    i.now_ms=t; i.range_time_ms=t; i.acc_norm_g=acc; i.gyro_norm_dps=rate;
    n.step(i);
}
int main() {
    NavEstimator n;
    NavInput i{}; i.imu_valid=true; i.range_valid=true;
    i.range_quality=120; i.distance_mm=500;
    push(n,i,100,1.0f); push(n,i,120,1.0f); push(n,i,140,1.0f);
    assert(n.state().height_valid);
    // User observed norm 1.552g while moving quickly: must gate hold.
    push(n,i,160,1.552f);
    assert(!n.state().height_valid && n.state().reason==HeightReason::MOTION_BAD);
    assert(!n.state().motion_clear);
    push(n,i,180,1.0f);
    assert(n.state().reason==HeightReason::SETTLING && !n.state().height_valid);
    // Repeated nav_task cycles on same ToF sample must not defeat guard.
    i.now_ms=240; n.step(i);
    assert(n.state().reason==HeightReason::SETTLING);
    for(uint32_t t=260;t<480;t+=20) {
        push(n,i,t,1.0f);
        assert(!n.state().height_valid);
    }
    push(n,i,480,1.0f);
    assert(n.state().reason==HeightReason::WARMUP && !n.state().height_valid);
    push(n,i,500,1.0f); assert(!n.state().height_valid);
    push(n,i,520,1.0f); assert(n.state().height_valid);
    // Angular-rate gate works separately even if accel norm stays ~1g.
    push(n,i,540,1.0f,300.0f);
    assert(!n.state().height_valid && n.state().reason==HeightReason::MOTION_BAD);
    push(n,i,560,1.0f,0.f);
    assert(n.state().reason==HeightReason::SETTLING);
    // Invalid (NaN) norm must invalidate the estimator, not bypass threshold.
    i.acc_norm_g=__builtin_nanf(""); i.now_ms=580; i.range_time_ms=580;
    n.step(i); assert(n.state().reason==HeightReason::IMU_BAD);
    assert(!n.state().height_valid);
    puts("PASS v6.2 motion guard tests");
}
