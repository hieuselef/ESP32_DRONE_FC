#include "alt_motor_gate.h"
#include "flight_altitude.h"
#include "throttle_handover.h"
#include <cassert>
#include <cstdio>
int main() {
    using namespace alt_motor_gate;
    auto d=decide(false,true,true,true,true,true,true,1600);
    assert(!d.motor_active && d.phase==HARD_STOP);
    d=decide(true,false,true,true,true,true,true,1600);
    assert(!d.motor_active);
    d=decide(true,true,false,true,true,true,true,1600);
    assert(!d.motor_active);
    d=decide(true,true,true,false,true,true,true,1600);
    assert(!d.motor_active);
    d=decide(true,true,true,true,false,true,true,1600);
    assert(!d.motor_active);
    d=decide(true,true,true,true,true,false,false,990);
    assert(!d.motor_active); // ANGLE without ALT: original low stick hard stop
    d=decide(true,true,true,true,true,false,false,1200);
    assert(d.motor_active && d.phase==MANUAL);
    d=decide(true,true,true,true,true,true,false,990);
    assert(d.motor_active && d.phase==ALT_LOW_STICK);
    d=decide(true,true,true,true,true,false,true,990);
    assert(d.motor_active && d.phase==MANUAL_HANDOVER);
    // Transition ALT to manual low throttle, using the existing bounded-slew transfer.
    flight_alt::AltController alt;
    flight_alt::RangeState r{}; r.valid=true; r.height_m=.80f;
    auto a=alt.update(1000,true,true,r,1450,true);
    assert(a.controlling);
    a=alt.update(1004,true,true,r,990,true);
    assert(a.controlling && a.reason==1); // not a pilot override
    // Upper end remains deliberate emergency pilot takeover.
    a=alt.update(1008,true,true,r,1800,true);
    assert(!a.controlling && a.reason==3);
    a=alt.update(1012,true,true,r,990,true);
    assert(!a.controlling && a.reason==3); // override latch persists
    a=alt.update(1016,false,true,r,990,true);
    assert(a.reason==0); // CH6 disengage resets fault latch
    a=alt.update(1020,true,true,r,1450,true);
    assert(a.controlling);
    auto out=throttle_handover::Controller{};
    auto s=out.update(2000,true,true,1450,990);
    assert(s.phase==throttle_handover::ALT);
    a=alt.update(1024,false,true,r,990,true);
    assert(!a.controlling);
    s=out.update(2004,true,false,0,990);
    assert(s.phase==throttle_handover::BLENDING && s.output_us>=1100);
    assert(s.last_step_us>=-6 && s.last_step_us<=6);
    int prev=s.output_us;
    for(int i=1;i<=70;++i){
      const auto d=decide(true,true,true,true,true,false,out.state().blending,990);
      if(!d.motor_active) break;
      s=out.update(2004+i*4,true,false,0,990);
      assert(s.output_us<=prev && prev-s.output_us<=6);
      prev=s.output_us;
    }
    assert(out.state().output_us==1100);
    d=decide(true,true,true,true,true,false,out.state().blending,990);
    assert(!d.motor_active); // low-throttle manual stop after completed handover
    // Always allow hard disarm immediately, even in ALT/BLEND.
    d=decide(false,true,true,true,true,true,true,990);
    assert(!d.motor_active);
    std::puts("PASS V2 low-stick ALT authority, manual handover, emergency/high and hard-stop priorities");
}
