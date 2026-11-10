#include "controller_math.h"
#include <cassert>
#include <cmath>
#include <cstdio>
using namespace controller_math;

int main() {
    RatePid pid(0.8f, 0.1f, 0.002f, 60.0f, 25.0f, 200.0f, 80.0f);
    PidOutput out = pid.step(0.0f, 0.0f, 0.004f, true);
    assert(fabs(out.total) < 1e-5f);
    for (int i=0; i<20; ++i) out = pid.step(25.0f, 0.0f, 0.004f, true);
    assert(out.total > 0);
    pid.reset();
    for (int i=0; i<20; ++i) out = pid.step(0.0f, 50.0f, 0.004f, true);
    assert(out.total < 0);
    for (int i=0; i<10000; ++i) out = pid.step(9999.0f, 0.0f, 0.004f, true);
    assert(std::isfinite(out.total));
    assert(out.total <= 200.01f && out.total >= -200.01f);
    assert(std::fabs(out.i) <= 80.01f);
    assert(std::isfinite(lpAlpha(65.0f, 0.004f)));
    auto mix = mixX(1400, 50, 0, 0);
    // Roll+ lam m3,m4 tang; m1,m2 giam (theo quy uoc cua du an).
    assert(mix.us[0] == 1350 && mix.us[1] == 1350);
    assert(mix.us[2] == 1450 && mix.us[3] == 1450);
    auto sat = mixX(1900, 300, 0, 0);
    assert(sat.clipped);
    for (int m : sat.us) assert(m>=1000 && m<=1950);
    std::puts("PASS: rate PID signed feedback, filtered D, anti-windup, X mixer, clipping");
    return 0;
}
