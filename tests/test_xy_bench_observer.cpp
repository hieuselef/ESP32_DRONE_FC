#include "xy_bench_observer.h"
#include <cassert>
#include <cmath>
#include <cstdio>
using namespace v2xybench;
static Input sample(uint32_t ms=1000, uint32_t f=100, int64_t fx=0,int64_t fy=0) {
 Input s{};s.now_ms=ms;s.last_flow_ms=ms;s.good_frames=f;s.sum_fx=fx;s.sum_fy=fy;
 s.height_m=0.385f;s.quality=100;s.authorized=true;s.imu_valid=s.height_valid=s.flow_valid=true;
 return s;
}
int main(){
 Observer o;auto s=sample();assert(o.start(s));
 // 30 cm +X: +2 FX, -150 FY. 3 packets per tick over 6 ticks.
 for(int i=1;i<=30;i++) {s=sample(1000+i*20,100+i*3,2*i/30,-5*i);assert(o.step(s).active);}
 assert(std::fabs(o.state().x_m-.30f)<.015f);
 assert(std::fabs(o.state().y_m)<.02f);
 o.stop();assert(!o.state().active&&o.state().reason==Reason::STOP);
 s=sample(2000,200);assert(o.start(s));
 for(int i=1;i<=30;i++) {s=sample(2000+i*20,200+i*3,5*i,-i/30);assert(o.step(s).active);}
 assert(std::fabs(o.state().y_m-.304f)<.015f);
 assert(std::fabs(o.state().x_m)<.02f);
 s.authorized=false;assert(!o.step(s).active);assert(o.state().reason==Reason::BAD_AUTH);
 s=sample(3000,300);s.gyro_norm_dps=9.0f;assert(!o.start(s)&&o.state().reason==Reason::ROTATION);
 s=sample(4000,400);assert(o.start(s));s=sample(4020,405);s.height_m=.20f;
 assert(!o.step(s).active&&o.state().reason==Reason::SENSOR);
 s=sample(5000,500);assert(o.start(s));s=sample(5200,520);
 assert(!o.step(s).active&&o.state().reason==Reason::FRAME_GAP);
 puts("PASS: X/Y displacement, axis separation, RC gate, gyro, height and timeouts");
}
