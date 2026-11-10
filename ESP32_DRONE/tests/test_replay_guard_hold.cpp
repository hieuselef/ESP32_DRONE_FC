#include "../include/nav_estimator.h"
#include <cassert>
#include <cstdio>

static void step(NavEstimator& est, ShadowAltHold& sh, NavInput& i,
                 uint32_t t, int32_t mm, float acc=1.0f, float gyro=0.0f) {
    i.now_ms=t; i.range_time_ms=t; i.distance_mm=mm;
    i.acc_norm_g=acc; i.gyro_norm_dps=gyro;
    const NavOutput& n=est.step(i);
    const ShadowOutput& o=sh.step(t,1,true,n);
    if (!n.height_valid) assert(o.normalized_correction==0 && o.i_term==0);
}
int main() {
    NavEstimator est;
    ShadowAltHold hold;
    NavInput i{};
    i.imu_valid=true; i.range_valid=true; i.range_quality=100;
    // 1.0s stationary capture after range warmup.
    for(uint32_t t=100;t<=1200;t+=20) step(est,hold,i,t,520);
    assert(hold.state().state==ShadowState::TRACK);
    assert(hold.state().capture_count==1);
    assert(hold.state().target_m>.51f && hold.state().target_m<.53f);
    // Strong acceleration forces a sensor-loss phase and u=0.
    step(est,hold,i,1220,300,1.55f);
    assert(est.state().reason==HeightReason::MOTION_BAD);
    assert(hold.state().state==ShadowState::SENSOR_LOSS);
    // Recovery with 0.40m plateau: cannot recapture during first 800ms of READY.
    bool saw_pending=false;
    for (uint32_t t=1240;t<=1980;t+=20) {
        step(est,hold,i,t,400);
        if (est.state().height_valid && hold.state().state==ShadowState::RECOVERY_PENDING)
            saw_pending=true;
        assert(hold.state().capture_count==1);
    }
    assert(saw_pending);
    for(uint32_t t=2000;t<=2600;t+=20) step(est,hold,i,t,400);
    assert(hold.state().capture_count==2);
    assert(hold.state().last_capture_reason==TargetCaptureReason::SENSOR_RECOVERY);
    assert(hold.state().target_m>.39f && hold.state().target_m<.41f);
    puts("PASS V6.3 replay: disturbance -> no correction -> delayed fresh target");
}
