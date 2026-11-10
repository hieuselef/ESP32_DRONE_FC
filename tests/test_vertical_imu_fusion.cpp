#include "vertical_accel_fusion.h"
#include "vertical_estimator.h"
#include "alt_kalman_authority.h"
#include "throttle_handover.h"
#include <cassert>
#include <cmath>
#include <cstdio>

int main() {
    using vertical_accel_fusion::Processor;
    Processor imu;
    const float phi=5.0f,theta=-6.0f;
    const float a=theta*0.01745329252f, b=phi*0.01745329252f;
    // Proper body-frame reading of gravity on a stationary tilted MPU.
    const float x=-std::sin(a),y=std::cos(a)*std::sin(b),z=std::cos(a)*std::cos(b);
    for(unsigned i=0;i<350;++i) {
        const auto &s=imu.update(1000+i*4,x,y,z,phi,theta,0.0f,true,true);
        assert(std::fabs(s.earth_z_raw_mps2)<0.0002f);
        if(i<299) assert(!s.calibrated);
    }
    assert(imu.state().calibrated && imu.state().valid);
    assert(std::fabs(imu.state().earth_z_linear_mps2)<0.001f);
    assert(imu.state().stationary_samples>=300);
    // Upward acceleration after takeoff: only the correct body->world Z
    // projection should yield positive linear upward acceleration.
    for(unsigned i=350;i<390;++i) {
        const float gfactor=1.0f+1.20f/9.80665f;
        const auto &s=imu.update(1000+i*4,x*gfactor,y*gfactor,z*gfactor,
                                 phi,theta,0.0f,false,true);
        assert(s.valid);
    }
    assert(std::fabs(imu.state().earth_z_linear_mps2-1.20f)<0.02f);
    const auto& neg=imu.update(2560,x*.91f,y*.91f,z*.91f,phi,theta,0.0f,false,true);
    assert(neg.valid && neg.earth_z_linear_mps2 < 1.20f);
    const auto& bad=imu.update(2564,NAN,0,1,phi,theta,0,false,true);
    assert(!bad.valid && bad.rejected_samples>0);
    const auto& spike=imu.update(2568,x*1.9f,y*1.9f,z*1.9f,phi,theta,0,false,true);
    assert(!spike.valid);

    vertical_estimator::Kalman2 rangeOnly,fused;
    auto r=rangeOnly.update(1000,true,true,1000,.8f);
    auto k=fused.update(1000,true,true,1000,.8f,0.0f,true);
    assert(r.healthy && k.healthy);
    r=rangeOnly.update(1004,true,false,1000,.8f);
    k=fused.update(1004,true,false,1000,.8f,1.2f,true);
    assert(!r.imu_accel_used && k.imu_accel_used);
    assert(k.velocity_mps>r.velocity_mps+.004f);
    assert(k.height_m>r.height_m);
    assert(k.imu_accel_predictions==1);
    const auto& k2=fused.update(1008,true,false,1000,.8f,0.0f,false);
    assert(!k2.imu_accel_used);
    const auto& reused=fused.update(1020,true,true,1000,.8f,1.0f,true);
    assert(!reused.measurement_used); // must reject duplicated timestamps

    flight_alt::RangeTracker tracker;
    vertical_estimator::Kalman2 kf;
    alt_kalman_authority::Gate gate;
    flight_alt::AltController alt;
    throttle_handover::Controller collective;
    uint32_t frame=5000;
    bool reachedLive=false, motorFromFused=false;
    for(unsigned i=0;i<390;++i){
        const uint32_t now=5000+i*4;
        if(i%5==0) frame=now;
        const auto &raw=tracker.update(now,frame,800,true,0,0);
        const auto &est=kf.update(now,raw.valid,raw.new_sample,
            raw.frame_ms,raw.measured_height_m,0.0f,true);
        const auto &selected=gate.update(now,true,true,raw,est,true);
        const int stick=i<220 ? 1450 : 1545;
        const auto &aOut=alt.update(now,true,true,selected.feedback,stick,true);
        const auto &out=collective.update(now,true,aOut.controlling,aOut.throttle_us,stick);
        assert(out.phase==throttle_handover::ALT);
        assert(out.output_us>=1100 && out.output_us<=1850);
        assert(std::fabs(aOut.correction_us)<=70.01f);
        if(selected.phase==alt_kalman_authority::KF_LIVE) {
            reachedLive=true;
            assert(est.imu_accel_used);
            assert(std::fabs(aOut.height_m-est.height_m)<0.00001f);
            // The selected feedback (rather than unfiltered raw) is the
            // actual input to the altitude->collective path.
            if(aOut.throttle_us==out.output_us) motorFromFused=true;
        }
    }
    assert(reachedLive && motorFromFused);
    const auto &rFault=tracker.update(6564,6564,800,true,0,0);
    const auto &imuFault=kf.update(6564,rFault.valid,rFault.new_sample,
        rFault.frame_ms,rFault.measured_height_m,0.0f,false);
    const auto &gateFault=gate.update(6564,true,true,rFault,imuFault,true);
    assert(!imuFault.imu_accel_used);
    assert(gateFault.phase==alt_kalman_authority::RAW_FALLBACK_LATCHED);
    assert(gateFault.fallback_latched && !gateFault.kalman_used);
    const auto &badRange=tracker.update(6800,6564,800,true,0,0);
    const auto &noRange=gate.update(6800,true,true,badRange,imuFault,true);
    assert(noRange.phase==alt_kalman_authority::RANGE_INVALID && !noRange.feedback.valid);
    std::puts("PASS MPU body->earth, bias calibration, motion prediction, range correction, ALT actuator and faults");
}
