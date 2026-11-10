// Regression: Arduino-ESP32 esp32-hal-gpio.h defines DISABLED as 0x00.
#ifndef DISABLED
#define DISABLED 0x00
#endif
#include "pos_authority.h"
#include <cassert>
#include <cmath>
#include <cstdio>
using namespace pos_authority;
static Input good(uint32_t now) {
    Input x{}; x.now_ms=now; x.nav_updated_ms=now-10;
    x.enabled=true; x.armed=x.rc_ok=x.imu_ok=x.timing_ok=true;
    x.alt_active=x.scale_verified=x.signs_verified=true;
    x.rotation_compensation_verified=true;
    x.requested_mode=2; x.xy_valid=x.flow_valid=true;
    x.pos_active_mode=2; x.flow_age_ms=10; x.range_age_ms=10;
    x.roll_cmd_deg=4; x.pitch_cmd_deg=-3; return x;
}
int main() {
    assert(effectiveAge(60,50)==110);
    assert(effectiveAge(UINT32_MAX,20)==UINT32_MAX);
    Gate gate;
    Input x=good(1000);
    x.enabled=false;
    assert(!gate.step(x,.004f).applied && gate.state().reason==Reason::POS_DISABLED);
    x=good(1100); x.rotation_compensation_verified=false;
    assert(!gate.step(x,.004f).applied && gate.state().reason==Reason::UNVERIFIED);
    x=good(1100); x.scale_verified=false;
    assert(!gate.step(x,.004f).applied && gate.state().reason==Reason::UNVERIFIED);
    for (uint32_t t=1200;t<=1500;t+=4) {
        x=good(t); gate.step(x,.004f);
    }
    assert(gate.state().applied && gate.state().reason==Reason::ACTIVE);
    assert(gate.state().roll_deg>0 && gate.state().roll_deg<=5.0f);
    x=good(1504); x.flow_age_ms=250;
    assert(!gate.step(x,.004f).applied && gate.state().latched);
    x=good(1510);
    assert(gate.step(x,.004f).reason==Reason::LOSS_LATCHED);
    x=good(1514); x.requested_mode=1;
    assert(!gate.step(x,.004f).latched);
    x=good(1518); assert(gate.step(x,.004f).reason==Reason::ACQUIRING);
    x=good(1522); x.roll_us=1700;
    assert(gate.step(x,.004f).reason==Reason::PILOT_OVERRIDE);
    assert(gate.state().latched);
    x=good(1526); x.requested_mode=0; gate.step(x,.004f);
    x=good(1530); x.nav_updated_ms=1400;
    assert(gate.step(x,.004f).reason==Reason::NAV_STALE);
    x=good(1540); x.alt_active=false;
    assert(gate.step(x,.004f).reason==Reason::ALT_INACTIVE);
    x=good(1550); x.pitch_cmd_deg=NAN;
    assert(gate.step(x,.004f).reason==Reason::BAD_TARGET);
    x=good(1560); x.armed=false;
    assert(!gate.step(x,.004f).applied && !gate.state().latched);
    puts("PASS POS V2: disabled, calibration gate, 300ms acquire, bounded tilt, ALT fallback, stale/latch, pilot override");
}
