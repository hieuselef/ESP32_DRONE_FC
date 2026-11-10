#include "../include/flow_calibration.h"
#include <cassert>
#include <cmath>
#include <cstdio>

static FlowDiagOutput goodFrame(int x,int y) {
    FlowDiagOutput f{};
    f.calibration_ready=true;
    f.x_raw=x; f.y_raw=y;
    return f;
}
int main() {
    FlowCalibrator cal;
    assert(!cal.begin('X',.30f,false,100));
    assert(cal.state().reason==FlowCalReason::WRONG_MODE);
    assert(!cal.begin('X',.01f,true,100));
    assert(cal.state().reason==FlowCalReason::BAD_DISTANCE);
    assert(cal.begin('X',.30f,true,100));
    auto f=goodFrame(5,1);
    for(uint32_t t=120;t<=720;t+=20) {
        cal.step(f,.50f,2.0f,true,t);
        cal.step(f,.50f,2.0f,true,t); // duplicate frames must NOT count
    }
    assert(cal.state().accepted==31);
    assert(!cal.finish()); // X-only measurement is not a 2-axis candidate
    assert(cal.state().x_measured && !cal.state().axes_candidate);
    assert(std::fabs(cal.state().x_counts_per_m_fx-516.6667f)<.1f);
    assert(cal.begin('Y',.30f,true,740));
    f=goodFrame(-1,4);
    for(uint32_t t=760;t<=1360;t+=20) cal.step(f,.50f,2.0f,true,t);
    assert(cal.finish());
    assert(cal.state().axes_candidate && cal.state().y_measured);
    assert(cal.state().determinant>0.0f);
    // Both axes observed, but NEVER enable real velocity/POS_HOLD.
    cal.reset(); assert(!cal.state().axes_candidate);
    assert(cal.begin('X',.30f,true,100));
    for(uint32_t t=120;t<=220;t+=20) cal.step(goodFrame(5,1),.50f,2.0f,true,t);
    assert(!cal.finish());
    assert(cal.state().reason==FlowCalReason::NEED_SAMPLES);
    cal.reset();
    assert(cal.begin('X',.30f,true,100));
    for(uint32_t t=120;t<=720;t+=20) {
        cal.step(goodFrame(2,1),.50f,50.0f,true,t);
    }
    assert(cal.state().accepted==0 && cal.state().rejected==31);
    assert(!cal.finish());
    cal.reset();
    assert(cal.begin('X',.30f,true,100));
    cal.step(goodFrame(10,0),.50f,0.0f,false,120);
    assert(cal.state().state==FlowCalState::FAILED);
    puts("PASS V6.3 flow axis calibration / duplicate / guards / authorization");
}
