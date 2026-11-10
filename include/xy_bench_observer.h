#pragma once
// Observation-only independent validation of a CANDIDATE planar flow matrix.
// Never connected to SharedNav.xy_valid, flight scale verification, PID or motors.
#include <cmath>
#include <cstdint>

namespace v2xybench {

struct Candidate {
    // X=(FX,FY) per m, Y=(FX,FY) per m; observed at ~0.385 m.
    float a_fx_x=6.7f, b_fx_y=493.3f;
    float c_fy_x=-500.0f, d_fy_y=-26.7f;
    float reference_height_m=0.385f;
};
struct Input {
    uint32_t now_ms=0, last_flow_ms=0, good_frames=0;
    int64_t sum_fx=0, sum_fy=0;
    float height_m=0, roll_deg=0, pitch_deg=0, gyro_norm_dps=0;
    uint8_t quality=0;
    bool authorized=false, imu_valid=false, height_valid=false, flow_valid=false;
};
enum class Reason : uint8_t { IDLE=0, TRACK=1, BAD_AUTH=2, SENSOR=3, ROTATION=4,
                               FRAME_GAP=5, MATRIX=6, SPEED=7, STOP=8 };
inline const char* reasonName(Reason r) {
    switch (r) {
      case Reason::TRACK: return "TRACK";
      case Reason::BAD_AUTH: return "BAD_AUTH";
      case Reason::SENSOR: return "SENSOR";
      case Reason::ROTATION: return "ROTATION";
      case Reason::FRAME_GAP: return "FRAME_GAP";
      case Reason::MATRIX: return "MATRIX";
      case Reason::SPEED: return "SPEED";
      case Reason::STOP: return "STOP";
      default: return "IDLE";
    }
}
struct Output {
    bool active=false;
    Reason reason=Reason::IDLE;
    float x_m=0,y_m=0,vx_mps=0,vy_mps=0;
    float h_last_m=0;
    uint32_t frames=0, intervals=0;
    int64_t delta_fx=0,delta_fy=0;
};
class Observer {
public:
    explicit Observer(Candidate c={}) : c_(c) {}
    bool start(const Input &s) {
        out_={};
        if (!matrixOk()) {out_.reason=Reason::MATRIX;return false;}
        if (!basicOk(s)) {out_.reason= !s.authorized ? Reason::BAD_AUTH : Reason::SENSOR;return false;}
        if (!tiltOk(s)) {out_.reason=Reason::ROTATION;return false;}
        out_.active=true;out_.reason=Reason::TRACK;out_.h_last_m=s.height_m;
        sync(s);return true;
    }
    void stop(){if(out_.active){out_.active=false;out_.reason=Reason::STOP;}}
    const Output& state() const {return out_;}
    const Candidate& candidate() const {return c_;}
    const Output& step(const Input &s) {
        if (!out_.active) return out_;
        if (!s.authorized) return abort(Reason::BAD_AUTH);
        if (!basicOk(s)) return abort(Reason::SENSOR);
        if (!tiltOk(s)) return abort(Reason::ROTATION);
        if (s.good_frames < frames_) return abort(Reason::FRAME_GAP);
        if (s.good_frames == frames_) return out_; // no new packet yet
        if (s.now_ms<=ms_) return abort(Reason::FRAME_GAP);
        const uint32_t dt_ms=s.now_ms-ms_;
        const uint32_t n=s.good_frames-frames_;
        if (dt_ms<5 || dt_ms>120 || n>30) return abort(Reason::FRAME_GAP);
        const float dt=dt_ms*0.001f;
        const int64_t fx=s.sum_fx-fx_, fy=s.sum_fy-fy_;
        // Cap increments per sample to catch counter corruption; not a clamp.
        if (fx>600 || fx< -600 || fy>600 || fy< -600) return abort(Reason::FRAME_GAP);
        const float det=c_.a_fx_x*c_.d_fy_y-c_.b_fx_y*c_.c_fy_x;
        const float hscale=s.height_m/c_.reference_height_m;
        const float dx=hscale*(c_.d_fy_y*static_cast<float>(fx)-c_.b_fx_y*static_cast<float>(fy))/det;
        const float dy=hscale*(-c_.c_fy_x*static_cast<float>(fx)+c_.a_fx_x*static_cast<float>(fy))/det;
        if (!std::isfinite(dx) || !std::isfinite(dy) ||
            std::fabs(dx/dt)>2.0f || std::fabs(dy/dt)>2.0f) return abort(Reason::SPEED);
        out_.x_m+=dx;out_.y_m+=dy;
        out_.vx_mps=dx/dt;out_.vy_mps=dy/dt;
        out_.frames+=n;out_.intervals++;
        out_.delta_fx+=fx;out_.delta_fy+=fy;
        out_.h_last_m=s.height_m;
        sync(s);return out_;
    }
private:
    bool matrixOk() const {
        const float a=c_.a_fx_x, b=c_.b_fx_y, cc=c_.c_fy_x, d=c_.d_fy_y;
        const float nx=std::hypot(a,cc),ny=std::hypot(b,d),det=a*d-b*cc;
        return std::isfinite(det)&&nx>50&&ny>50&&std::fabs(det)>0.70f*nx*ny&&
            c_.reference_height_m>0.25f&&c_.reference_height_m<0.9f;
    }
    bool basicOk(const Input&s) const {
        return s.authorized&&s.imu_valid&&s.height_valid&&s.flow_valid&&s.good_frames>0&&
            s.last_flow_ms!=0&&s.now_ms>=s.last_flow_ms&&s.now_ms-s.last_flow_ms<=100&&
            s.quality>=70&&std::isfinite(s.height_m)&&s.height_m>=0.30f&&s.height_m<=0.60f;
    }
    bool tiltOk(const Input&s) const {
        return std::isfinite(s.roll_deg)&&std::isfinite(s.pitch_deg)&&std::isfinite(s.gyro_norm_dps)&&
            std::fabs(s.roll_deg)<=8.0f&&std::fabs(s.pitch_deg)<=8.0f&&s.gyro_norm_dps<=5.0f;
    }
    void sync(const Input&s) {ms_=s.now_ms;frames_=s.good_frames;fx_=s.sum_fx;fy_=s.sum_fy;}
    const Output& abort(Reason r) {out_.active=false;out_.reason=r;return out_;}
    Candidate c_{};
    Output out_{};
    uint32_t ms_=0,frames_=0;
    int64_t fx_=0,fy_=0;
};
}
