#include "../include/flow_guided_cal.h"
#include <cassert>
#include <cmath>
#include <cstdio>
static void add(FlowPacketCounters& c,int n,int x,int y,int quality=95) {
  for(int i=0;i<n;i++)c.add(x,y,(uint8_t)quality,70,200);
}
static void baseline(FlowPacketCounters& c) {
  for(int i=0;i<200;i++)c.add(i%2?1:-1,0,95,70,200);
}
static void motion(FlowPacketCounters& c, GuidedFlowCal& g, uint32_t t, int x,int y) {
  for(int k=0;k<10;k++) {
    add(c,30,x,y);
    auto e=g.update(t+(k+1)*500,c,true);
    if(k<9)assert(e==GuidedEvent::WINDOW);
    else assert(e==GuidedEvent::COMPLETE);
  }
}
int main(){
  GuidedFlowCal g; FlowPacketCounters c;
  assert(!g.start('X',.3f,10,c,false));
  assert(g.start('X',.3f,10,c,true));
  baseline(c);
  assert(g.update(2010,c,true)==GuidedEvent::READY_FOR_GO);
  assert(g.readyForGo() && g.active()); assert(!g.go(2200,c,false)); assert(g.go(3000,c,true));
  motion(c,g,3000,3,0);
  assert(g.x().measured && !g.y().measured);
  assert(g.x().fx_per_m>0 && g.x().fy_per_m==0);
  assert(g.reason()==GuidedReason::AXIS_MEASURED);
  assert(g.start('Y',.3f,6000,c,true));
  baseline(c);
  assert(g.update(8000,c,true)==GuidedEvent::READY_FOR_GO);
  assert(g.go(9000,c,true));
  motion(c,g,9000,0,3);
  assert(g.y().measured && g.axesCandidate());
  assert(g.axisSin()>.99);
  // Re-measurement clears same axis so stale successful results cannot survive failure.
  assert(g.start('Y',.3f,12000,c,true));
  assert(!g.y().measured && !g.axesCandidate());
  baseline(c);g.update(14000,c,true);assert(g.go(15000,c,true));
  for(int k=0;k<10;k++) {add(c,30,0,0);g.update(15000+(k+1)*500,c,true);}
  assert(!g.y().measured && !g.axesCandidate());
  // Strong net flow on the SAME axis must not be accepted as 2D position measurement.
  GuidedFlowCal col; FlowPacketCounters b;
  assert(col.start('X',.3f,1,b,true));baseline(b);col.update(2001,b,true);assert(col.go(2100,b,true));motion(b,col,2100,3,0);
  assert(col.start('Y',.3f,6000,b,true));baseline(b);col.update(8000,b,true);assert(col.go(8100,b,true));motion(b,col,8100,2,0);
  assert(col.reason()==GuidedReason::COLLINEAR && !col.axesCandidate());
  // Zero-net jitter cannot count as motion despite accumulating many absolute counts.
  GuidedFlowCal jitter;FlowPacketCounters j;
  assert(jitter.start('Y',.3f,1,j,true));baseline(j);jitter.update(2001,j,true);assert(jitter.go(2100,j,true));
  for(int k=0;k<10;k++) {
    for(int i=0;i<30;i++)j.add(i%2 ? 2:-2,0,95,70,200);
    jitter.update(2600+k*500,j,true);
  }
  assert(!jitter.y().measured && !jitter.axesCandidate());
  // Reversal after a coherent window is rejected, not treated as extra movement.
  GuidedFlowCal reverse;FlowPacketCounters r;
  assert(reverse.start('Y',.3f,1,r,true));baseline(r);reverse.update(2001,r,true);assert(reverse.go(2100,r,true));
  add(r,30,4,0);reverse.update(2400,r,true);
  add(r,30,-4,0);reverse.update(2700,r,true);
  for(int k=2;k<10;k++){add(r,30,k%2?4:-4,0); reverse.update(2100+(k+1)*300,r,true);}
  assert(reverse.windowsReverse()>0 && !reverse.y().measured);
  // Quality 8/255 => cannot calibrate despite large raw values.
  GuidedFlowCal low;FlowPacketCounters l;
  assert(low.start('Y',.3f,1,l,true));baseline(l);low.update(2001,l,true);assert(low.go(2100,l,true));
  for(int k=0;k<10;k++){add(l,30,20,0,8);low.update(2100+(k+1)*500,l,true);}
  assert(low.reason()==GuidedReason::LOW_QUALITY && !low.y().measured);
  // Abort immediately on safety gate loss; test wraparound / counter rollback.
  GuidedFlowCal abort;FlowPacketCounters z;
  assert(abort.start('Y',.3f,1,z,true));
  assert(abort.update(10,z,false)==GuidedEvent::ABORT);
  assert(abort.reason()==GuidedReason::UNSAFE);
  GuidedFlowCal rollback;add(z,30,1,0);assert(rollback.start('Y',.3f,1,z,true));z={};
  assert(rollback.update(10,z,true)==GuidedEvent::ABORT);
  assert(rollback.reason()==GuidedReason::COUNTER_RESET);
  // READY phase does not collect motion until the manual go command.
  GuidedFlowCal manual;FlowPacketCounters m;
  assert(manual.start('Y',.30f,100,m,true));baseline(m);
  assert(manual.update(2100,m,true)==GuidedEvent::READY_FOR_GO);
  for (int t=3000;t<7000;t+=500) {add(m,30,4,0); assert(manual.update(t,m,true)==GuidedEvent::NONE);}
  assert(!manual.y().measured && manual.readyForGo());
  assert(manual.go(7000,m,true));
  motion(m,manual,7000,0,4);
  assert(manual.y().measured);
  GuidedFlowCal timeout;FlowPacketCounters tm;
  assert(timeout.start('Y',.3f,1,tm,true));baseline(tm);
  assert(timeout.update(2001,tm,true)==GuidedEvent::READY_FOR_GO);
  assert(timeout.update(33002,tm,true)==GuidedEvent::ABORT);
  assert(timeout.reason()==GuidedReason::GO_TIMEOUT);
  // Regression: replay the user's recorded 30 cm X movement over a textured
  // surface. A moderately coherent one-way trace must be captured in BENCH,
  // even when no individual 300 ms interval reaches the old 0.60 coherence.
  // This checks the diagnostic classifier only, NOT scale accuracy in flight.
  GuidedFlowCal weak; FlowPacketCounters wc;
  assert(weak.start('X',.30f,10,wc,true));
  wc.frames+=202;wc.accepted+=202;wc.sum_x-=4;wc.sum_y+=1;wc.abs_x+=78;wc.abs_y+=15;
  assert(weak.update(2010,wc,true)==GuidedEvent::READY_FOR_GO);
  assert(weak.go(2100,wc,true));
  const int wx[16]={-1,0,-1,1,-5,0,-3,1,1,0,2,4,-2,5,2,-3};
  const int wy[16]={0,2,-2,-2,-13,-7,-9,-3,-14,-13,-5,-14,-15,-12,-5,-14};
  const int wabsx[16]={3,12,15,15,13,12,7,3,7,10,6,10,4,11,6,9};
  const int wabsy[16]={4,12,6,4,13,7,15,3,16,13,5,16,15,12,9,16};
  for(int i=0;i<16;i++) {
    wc.frames+=30;wc.accepted+=30;wc.sum_x+=wx[i];wc.sum_y+=wy[i];
    wc.abs_x+=wabsx[i];wc.abs_y+=wabsy[i];
    assert(weak.update(2100+(i+1)*300,wc,true)==GuidedEvent::WINDOW);
  }
  // Extra 20 packets finish the 5-second path; keep final path coherent.
  wc.frames+=20;wc.accepted+=20;wc.sum_y-=9;wc.abs_x+=4;wc.abs_y+=14;
  assert(weak.update(7100,wc,true)==GuidedEvent::COMPLETE);
  assert(weak.x().measured && weak.windowsUsed()>=3);
  assert(weak.reason()==GuidedReason::AXIS_MEASURED);
  assert(!weak.axesCandidate());  // No Y-axis measurement yet.
  // 2026-11-09 real X log: 3 total qualifying windows separated by idle
  // intervals are NOT a continuous 30 cm translation, never calibrate from them.
  GuidedFlowCal realX; FlowPacketCounters cx;
  assert(realX.start('X',.30f,10,cx,true));
  cx.frames+=202;cx.accepted+=202;cx.abs_x+=64;cx.abs_y+=4;
  assert(realX.update(2010,cx,true)==GuidedEvent::READY_FOR_GO);
  assert(realX.go(2100,cx,true));
  const int rx[16]={-1,-1,1,-2,-3,-1,-5,0,0,1,-1,-1,-20,0,0,1};
  const int ry[16]={0,0,-1,-3,-1,-15,-15,2,1,4,0,-3,-18,4,-4,-1};
  const int rax[16]={5,5,9,6,7,7,13,8,4,9,5,11,26,8,10,9};
  const int ray[16]={4,2,1,7,1,15,15,4,3,8,2,3,18,8,6,1};
  for(int i=0;i<16;i++) {
    cx.frames+=30;cx.accepted+=30;
    cx.sum_x+=rx[i];cx.sum_y+=ry[i];cx.abs_x+=rax[i];cx.abs_y+=ray[i];
    assert(realX.update(2100+(i+1)*300,cx,true)==GuidedEvent::WINDOW);
  }
  cx.frames+=22;cx.accepted+=22;
  assert(realX.update(7100,cx,true)==GuidedEvent::COMPLETE);
  assert(!realX.x().measured && !realX.axesCandidate());
  assert(realX.windowsUsed()<3 && realX.segmentsFound()>=2);
  assert(realX.reason()==GuidedReason::FRAGMENTED_MOTION);

  // 2026-11-09 real Y log: a coherent consecutive last four-window burst.
  // It may produce a BENCH axis measurement, not verified physical scale.
  GuidedFlowCal realY; FlowPacketCounters cy;
  assert(realY.start('Y',.30f,10,cy,true));
  cy.frames+=200;cy.accepted+=200;cy.abs_x+=10;cy.abs_y+=29;cy.sum_y+=1;
  assert(realY.update(2010,cy,true)==GuidedEvent::READY_FOR_GO);
  assert(realY.go(2100,cy,true));
  const int yx[16]={-1,2,0,1,2,14,-6,23,4,4,-4,3,22,30,27,24};
  const int yy[16]={0,0,0,-1,0,0,0,0,0,-1,0,-1,0,-4,-2,-2};
  const int yax[16]={9,6,6,7,2,14,40,23,20,12,20,17,22,34,29,24};
  const int yay[16]={2,0,4,1,4,2,2,0,0,3,0,1,6,4,2,2};
  for(int i=0;i<16;i++) {
    cy.frames+=30;cy.accepted+=30;
    cy.sum_x+=yx[i];cy.sum_y+=yy[i];cy.abs_x+=yax[i];cy.abs_y+=yay[i];
    assert(realY.update(2100+(i+1)*300,cy,true)==GuidedEvent::WINDOW);
  }
  cy.frames+=20;cy.accepted+=20;
  assert(realY.update(7100,cy,true)==GuidedEvent::COMPLETE);
  assert(realY.y().measured && realY.windowsUsed()>=4);
  assert(realY.bestFirstWindow()>=12 && realY.bestLastWindow()<=17);
  assert(!realY.axesCandidate());

  // Deliberately fragmented 2-window bursts with a gap: reject even though
  // global net flow is strong and the movement directions all agree.
  GuidedFlowCal fragments;FlowPacketCounters f;
  assert(fragments.start('X',.3f,10,f,true)); baseline(f);
  assert(fragments.update(2010,f,true)==GuidedEvent::READY_FOR_GO);
  assert(fragments.go(2100,f,true));
  for(int i=0;i<16;i++) {
      int speed=(i==1 || i==2 || i==8 || i==9)?4:0;
      add(f,30,0,-speed);
      assert(fragments.update(2100+(i+1)*300,f,true)==GuidedEvent::WINDOW);
  }
  add(f,20,0,0);assert(fragments.update(7100,f,true)==GuidedEvent::COMPLETE);
  assert(!fragments.x().measured);
  assert(fragments.reason()==GuidedReason::FRAGMENTED_MOTION);
  assert(fragments.windowsUsed()==2);

  // A short coherent genuine move followed by a strong reversed move is
  // not a valid one-way calibration, even across a quiet gap.
  GuidedFlowCal back;FlowPacketCounters bc;
  assert(back.start('X',.3f,10,bc,true));baseline(bc);
  assert(back.update(2010,bc,true)==GuidedEvent::READY_FOR_GO);
  assert(back.go(2100,bc,true));
  for(int i=0;i<16;i++) {
    const int v = i>=1 && i<=6 ? 4 : (i>=11 && i<=15 ? -4 : 0);
    add(bc,30,v,0);
    assert(back.update(2100+(i+1)*300,bc,true)==GuidedEvent::WINDOW);
  }
  add(bc,20,0,0);assert(back.update(7100,bc,true)==GuidedEvent::COMPLETE);
  assert(!back.x().measured && back.windowsReverse()>0);
  puts("PASS V6.3H manual GO, counters reset, packet capture, window filtering, orthogonality and fail-closed gates");
}
