#include "flight_altitude.h"
#include "esc_output.h"
#include "controller_math.h"
#include <cassert>
#include <cmath>
#include <cstdio>
int main() {
    using namespace flight_alt;
    assert(flight_esc::logicalToOneshotUs(1000)==125);
    assert(flight_esc::logicalToOneshotUs(2000)==250);
    assert(flight_esc::logicalToOneshotUs(-50)==125);
    assert(flight_esc::logicalToOneshotUs(5000)==250);
    RangeTracker sensor;
    auto st=sensor.update(1000,1000,500,true,0,0);
    assert(st.valid && std::fabs(st.height_m-0.5f)<.001f);
    AltController c;
    auto a=c.update(1000,true,true,st,1450);
    assert(a.controlling && a.throttle_us==1450 && a.correction_us==0);
    for(int i=1;i<=20;i++){
        st=sensor.update(1000+i*20,1000+i*20,500,true,0,0);
        a=c.update(1000+i*20,true,true,st,1450);
        assert(a.controlling && std::abs(a.throttle_us-1450)<=1);
        assert(std::fabs(a.correction_us)<0.5f);
    }
    st=sensor.update(1420,1420,550,true,0,0);
    a=c.update(1420,true,true,st,1450);
    assert(a.controlling && a.correction_us<0); // climbing above setpoint -> reduce throttle
    assert(a.correction_us>=-70.01f);
    a=c.update(1424,true,true,st,1800);
    assert(!a.controlling && a.reason==3); // stick override -> manual
    a=c.update(1428,true,true,st,1450);
    assert(!a.controlling && a.reason==3); // latch until mode off
    a=c.update(1432,false,true,st,1450);
    assert(!a.controlling && a.reason==0);
    a=c.update(1436,true,true,st,1450);
    assert(a.controlling && a.throttle_us==1450); // re-enter bumplessly
    // Stale range forces fallback, not unbounded correction.
    st=sensor.update(1700,1436,500,true,0,0);
    assert(!st.valid);
    a=c.update(1700,true,true,st,1450);
    assert(!a.controlling && a.reason==2);
    // POS actuation gate stays compile-time false, tested in config separate.
    std::puts("PASS flight candidate OneShot125 mapping, ALT tracking / fallback / override / no jump");
}
