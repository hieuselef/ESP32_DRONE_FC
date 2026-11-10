#include "../include/flow_velocity.h"
#include "../include/hold_modes.h"
#include <cassert>
#include <cmath>
#include <cstdio>
int main() {
  using namespace v63flow;
  Estimator locked;
  Sample s{}; s.now_ms=1000;s.last_frame_ms=1000;s.good_frames=10;
  s.sum_fx=100;s.sum_fy=0;s.quality=100;s.height_m=.5f;
  s.height_valid=true;s.flow_fresh=true;
  assert(!locked.step(s).valid); // real firmware: remains locked
  Scale axis{}; axis.independently_verified=true;
  axis.fx_per_m_x=1000;axis.fy_per_m_x=0;
  axis.fx_per_m_y=0;axis.fy_per_m_y=1000;
  axis.reference_height_m=.5f;
  Estimator simulator(axis);
  assert(!simulator.step(s).valid); // first sample is baseline
  s.now_ms=1020;s.last_frame_ms=1020;s.good_frames=12;s.sum_fx=110;
  auto v=simulator.step(s);
  assert(v.valid && std::fabs(v.vx_mps-.5f)<.001f);
  assert(std::fabs(v.vy_mps)<.001f);
  // Out-of-date packets must immediately invalidate velocity.
  s.now_ms=1200; s.last_frame_ms=1020;
  assert(!simulator.step(s).valid);
  // A pure yaw rotation must be removed before interpreting flow as XY motion.
  Scale gyroScale=axis;
  gyroScale.rotation_compensation_verified=true;
  gyroScale.fx_per_rad_yaw=1000.0f;
  Estimator rotComp(gyroScale);
  Sample g{}; g.now_ms=2000; g.last_frame_ms=2000; g.good_frames=20;
  g.height_m=.5f; g.height_valid=true; g.flow_fresh=true; g.quality=100;
  assert(!rotComp.step(g).valid);
  g.now_ms=2020;g.last_frame_ms=2020;g.good_frames=22;
  g.sum_fx=3;g.yaw_rate_dps=10.0f;
  auto gr=rotComp.step(g);
  assert(gr.valid && std::fabs(gr.vx_mps)<0.03f);
  using namespace v63hold;
  Controller c;
  Input p{};p.now_ms=1000;p.authorized=true;p.requested_mode=2;
  p.height_valid=true;p.range_age_ms=5;p.h_m=.5f;
  p.flow_valid=true;p.flow_verified=false;
  assert(c.step(p).reason==Reason::CAPTURE_WAIT);
  p.now_ms=1700; c.step(p);
  p.now_ms=1720;
  assert(c.step(p).reason==Reason::FLOW_UNVERIFIED);
  p.flow_verified=true;p.flow_age_ms=5;p.vx_body_mps=.1f;p.vy_body_mps=0;
  p.now_ms=1740;
  assert(c.step(p).reason==Reason::POS_TRACK);
  p.now_ms=1760;
  auto o=c.step(p);
  assert(o.active_mode==2);
  assert(o.estimated_x_m>0);
  assert(std::fabs(o.roll_target_deg)<=7.01f);
  assert(std::fabs(o.pitch_target_deg)<=7.01f);
  p.flow_valid=false;p.now_ms=1780;
  assert(c.step(p).reason==Reason::FLOW_STALE);
  assert(c.state().roll_target_deg==0 && c.state().pitch_target_deg==0);
  p.authorized=false;p.now_ms=1800;
  assert(c.step(p).reason==Reason::RC_BLOCKED);
  assert(c.state().active_mode==0);
  puts("PASS unified POS estimator: independent scale gate, velocity, stale reset, target and safe fallback");
}
