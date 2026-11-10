#include "flight_altitude.h"
#include "throttle_handover.h"
#include <cassert>
#include <cstdlib>
#include <cstdio>
int main(){
    using namespace throttle_handover;
    Controller output;
    auto s=output.update(1000,true,false,0,1400);
    assert(s.phase==MANUAL && s.output_us==1400);
    s=output.update(1004,true,true,1375,1400);
    assert(s.phase==ALT && s.output_us==1375);
    // Loss of range/mode switch/pilot override: no instant 300+us jump.
    s=output.update(1008,true,false,0,1700);
    assert(s.phase==BLENDING && s.output_us>=1375 && s.output_us<=1381);
    assert(s.transitions==1 && s.last_transition_ms==1008);
    int previous=s.output_us;
    for(int i=1;i<100;++i){
        s=output.update(1008+i*4,true,false,0,1700);
        assert(s.output_us>=previous && s.output_us-previous<=6);
        previous=s.output_us;
    }
    assert(s.output_us==1700 && s.phase==MANUAL);
    // Next regular manual stick movement must be immediate, not permanently lagged.
    s=output.update(1410,true,false,0,1620);
    assert(s.output_us==1620 && s.phase==MANUAL);
    // Full deliberate throttle command bypasses blending, preserving authority.
    s=output.update(1414,true,true,1200,1200);
    s=output.update(1418,true,false,0,1810);
    assert(s.output_us==1810 && s.phase==EMERGENCY_BYPASS);
    assert(s.transitions==2 && s.emergency_bypasses==1);
    // Failsafe, disarm, and low-throttle cutoff must never ramp down slowly.
    s=output.update(1422,false,false,0,1800);
    assert(s.output_us==1000 && s.phase==SAFE_STOP);
    assert(s.transitions==2 && s.emergency_bypasses==1);
    s=output.update(1426,true,false,0,1300);
    assert(s.output_us==1300 && s.phase==MANUAL);
    s=output.update(1430,true,true,1350,1400);
    s=output.update(1434,true,false,0,1100);
    assert(s.output_us>=1100); // motor_active false is enforced by the FC at <=1099
    s=output.update(1438,false,false,0,1000);
    assert(s.output_us==1000 && s.phase==SAFE_STOP);
    // Latched range fault cannot restart ALT just because the range returns.
    flight_alt::AltController alt;
    flight_alt::RangeState r{}; r.valid=true; r.height_m=.70f;
    auto a=alt.update(2000,true,true,r,1250);
    assert(a.controlling);
    r.valid=false;
    a=alt.update(2004,true,true,r,1250);
    assert(!a.controlling && a.reason==2);
    r.valid=true;
    a=alt.update(2008,true,true,r,1250);
    assert(!a.controlling && a.reason==2);
    a=alt.update(2012,true,false,r,1000);
    assert(!a.controlling && a.reason==2); // even throttle-low holds fault latch
    a=alt.update(2016,true,true,r,1250);
    assert(!a.controlling && a.reason==2);
    a=alt.update(2020,false,true,r,1250);
    assert(!a.controlling && a.reason==0);
    a=alt.update(2024,true,true,r,1250);
    assert(a.controlling && a.reason==1);
    // Override latch likewise survives a temporary stop and clears on mode OFF.
    a=alt.update(2028,true,true,r,1590);
    assert(!a.controlling && a.reason==3);
    a=alt.update(2032,true,false,r,990);
    assert(!a.controlling && a.reason==3);
    a=alt.update(2036,true,true,r,1250);
    assert(!a.controlling && a.reason==3);
    a=alt.update(2040,false,true,r,1250);
    assert(a.reason==0);
    a=alt.update(2044,true,true,r,1250);
    assert(a.controlling);
    // A range fault before capture also cannot unexpectedly latch in later.
    alt.reset(); r.valid=false;
    a=alt.update(2050,true,true,r,1250);
    assert(!a.controlling && a.reason==2);
    r.valid=true;
    a=alt.update(2054,true,true,r,1250);
    assert(!a.controlling && a.reason==2);
    std::puts("PASS ALT->MANUAL smooth handover, full pilot bypass, hard stop and ALT fault latch");
}
