#include "../include/flow_diagnostics.h"
#include <cassert>
#include <cstdio>
int main() {
    FlowDiagnostics f;
    FlowDiagInput i{}; i.now_ms=100;
    auto x=f.step(i);
    assert(!x.calibration_ready && x.reason==FlowDiagReason::NO_FRAME);
    i.frame_time_ms=100; i.frame_valid=true; i.quality=106;
    i.height_valid=true; i.motion_clear=true; i.height_m=.5f;
    i.x_raw=1; i.y_raw=-1;
    x=f.step(i);
    assert(x.calibration_ready && !x.xy_valid);
    assert(x.ready_samples==1);
    i.now_ms=110; x=f.step(i); assert(x.ready_samples==1);
    i.now_ms=250; x=f.step(i);
    assert(!x.calibration_ready && x.reason==FlowDiagReason::STALE);
    i.frame_time_ms=250; i.quality=45; x=f.step(i);
    assert(x.reason==FlowDiagReason::QUALITY_LOW);
    i.now_ms=270; i.frame_time_ms=270; i.quality=110; i.height_valid=false;
    x=f.step(i); assert(x.reason==FlowDiagReason::HEIGHT_BAD);
    i.now_ms=290; i.frame_time_ms=290; i.height_valid=true; i.motion_clear=false;
    x=f.step(i); assert(x.reason==FlowDiagReason::MOTION_BAD);
    i.now_ms=310; i.frame_time_ms=310; i.motion_clear=true; i.roll_deg=20;
    x=f.step(i); assert(x.reason==FlowDiagReason::TILT_BAD);
    i.now_ms=330; i.frame_time_ms=330; i.roll_deg=0; x=f.step(i);
    assert(x.calibration_ready && x.ready_samples==2 && !x.xy_valid);
    puts("PASS v6.2 raw flow diagnostic tests");
}
