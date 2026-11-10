#include "../include/flow_y_probe.h"
#include <cassert>
#include <cmath>
#include <cstdio>

int main() {
    FlowPacketCounters total{};
    YFlowProbe probe;
    assert(!probe.start(100, total, false));
    assert(probe.start(100, total, true));
    for (int i=0;i<200;i++) total.add(i%2 ? 1:-1, 0, 95);
    assert(probe.update(2100, total, true) == YProbeEvent::MOVE_NOW);
    assert(probe.still().accepted == 200 && probe.still().sum_x == 0);
    for (int i=0;i<30;i++) total.add(1, 0, 95);
    assert(probe.update(2400, total, true) == YProbeEvent::WINDOW);
    assert(probe.window().sum_x==30);
    for (int i=0;i<270;i++) total.add(0, -2, 95);
    assert(probe.update(5100, total, true) == YProbeEvent::COMPLETE);
    assert(probe.move().accepted==300);
    assert(probe.move().sum_y==-540);
    assert(probe.correctedY()==-540.0);
    assert(probe.coherence() > .9);
    // Avoid falsely treating raw absolute sums as net motion.
    FlowPacketCounters n{};
    for (int i=0;i<1000;i++) n.add(i%2?1:-1,0,95);
    assert(n.sum_x==0 && n.abs_x==1000);
    n.add(8, -3, 10); // quality gate
    n.add(999, 1, 95); // spike gate
    assert(n.low_quality==1 && n.spikes==1 && n.accepted==1000);
    // Auto abort when stream/safety gate fails.
    YFlowProbe abortProbe;
    assert(abortProbe.start(100,n,true));
    assert(abortProbe.update(200,n,false)==YProbeEvent::ABORT);
    assert(abortProbe.abortReason()==YProbeAbortReason::SAFETY_GATE);
    // Minimum baseline acceptance required.
    YFlowProbe tooFew;
    assert(tooFew.start(100,n,true));
    for(int i=0;i<10;i++) n.add(1,0,95);
    assert(tooFew.update(2200,n,true)==YProbeEvent::ABORT);
    assert(tooFew.abortReason()==YProbeAbortReason::TOO_FEW_STILL_PACKETS);
    assert(yProbeRangeObservationOk(true, 0, 100));
    assert(yProbeRangeObservationOk(false, 100, 350));
    assert(!yProbeRangeObservationOk(false, 100, 351));
    assert(!yProbeRangeObservationOk(false, 0, 100));
    FlowPacketCounters c{};
    for (int i=0;i<30;i++) c.add(1,0,90);
    YFlowProbe countRollback;
    assert(countRollback.start(10,c,true));
    c.frames=0; c.accepted=0;
    assert(countRollback.update(30,c,true)==YProbeEvent::ABORT);
    assert(countRollback.abortReason()==YProbeAbortReason::COUNTER_ROLLBACK);
    puts("PASS V6.3Y Y-axis packet capture / stationary/move separation / abort gates");
}
