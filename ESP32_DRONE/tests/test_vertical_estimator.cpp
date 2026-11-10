#include "vertical_estimator.h"
#include "flight_altitude.h"
#include <cassert>
#include <cmath>
#include <cstdio>
int main(){
    flight_alt::RangeTracker tracker;
    vertical_estimator::Kalman2 est;
    uint32_t frame=1000;
    auto r=tracker.update(1000,frame,780,true,0,0);
    assert(r.new_sample && r.valid);
    auto k=est.update(1000,r.valid,r.new_sample,r.frame_ms,r.height_m);
    assert(k.healthy && k.measurement_used && k.corrections==1);
    // A repeated frame CANNOT create extra independent Kalman corrections.
    r=tracker.update(1004,frame,780,true,0,0);
    k=est.update(1004,r.valid,r.new_sample,r.frame_ms,r.height_m);
    assert(!r.new_sample && !k.measurement_used && k.corrections==1);
    for (uint32_t i=2;i<500;i++){
      const uint32_t now=1000+i*4;
      if(i%5==0) frame=now;
      const int mm=780+((i/5)%3-1); // 1 mm regular measurement scatter
      r=tracker.update(now,frame,mm,true,0,0);
      k=est.update(now,r.valid,r.new_sample,r.frame_ms,r.height_m);
      assert(std::isfinite(k.height_m) && std::isfinite(k.velocity_mps));
    }
    assert(k.healthy && k.corrections>20);
    assert(std::fabs(k.height_m-.780f)<.02f);
    assert(std::fabs(k.velocity_mps)<.10f);
    // New raw range is too old: tracker's age gate and estimator health stop.
    r=tracker.update(3500,frame,780,true,0,0);
    k=est.update(3500,r.valid,r.new_sample,r.frame_ms,r.height_m);
    assert(!r.valid && !k.healthy);
    // New valid sensor frame can reacquire in shadow only.
    r=tracker.update(3520,3520,780,true,0,0);
    k=est.update(3520,r.valid,r.new_sample,r.frame_ms,r.height_m);
    assert(k.healthy);
    // Large but syntactically valid shadow measurement rejected by innovation gate.
    // RangeTracker itself rejects jumps; directly test KF gate to detect disagreements.
    const uint32_t prior=k.corrections;
    k=est.update(3540,true,true,3540,1.65f);
    assert(!k.measurement_used && k.outliers>=1 && k.corrections==prior);
    std::puts("PASS unique range frame Kalman shadow, stale guard, re-acquire and innovation gate");
}
