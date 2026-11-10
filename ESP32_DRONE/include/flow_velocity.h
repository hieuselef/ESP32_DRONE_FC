#pragma once
// Receives EVERY valid MSP flow packet via accumulated sums in SharedMTF.
// This velocity cannot be used until a separately VERIFIED scale/axis matrix exists.
// A V6.3 flow-calibration "candidate" is NOT such verification.
#include <cmath>
#include <cstdint>
namespace v63flow {
struct Scale {
    bool independently_verified = false;
    float fx_per_m_x=0, fy_per_m_x=0, fx_per_m_y=0, fy_per_m_y=0;
    float reference_height_m=0.5f;
    // Gyro-induced optical-flow counts per radian of body rotation.
    // Must be independently estimated with no XY translation before actuation.
    bool rotation_compensation_verified=false;
    float fx_per_rad_roll=0, fx_per_rad_pitch=0, fx_per_rad_yaw=0;
    float fy_per_rad_roll=0, fy_per_rad_pitch=0, fy_per_rad_yaw=0;
};
struct Sample {
    uint32_t now_ms=0, last_frame_ms=0, good_frames=0;
    int64_t sum_fx=0, sum_fy=0;
    uint8_t quality=0;
    float height_m=0;
    bool height_valid=false, flow_fresh=false;
    float roll_rate_dps=0,pitch_rate_dps=0,yaw_rate_dps=0;
};
struct Velocity { bool valid=false; float vx_mps=0,vy_mps=0; uint32_t sample_age_ms=UINT32_MAX, frames=0; };
class Estimator {
public:
    explicit Estimator(Scale scale = {}):scale_(scale){}
    void reset(){last_ms_=0; last_frames_=0; last_fx_=last_fy_=0; out_={};}
    const Velocity &step(const Sample &s){
        out_={}; out_.sample_age_ms=s.last_frame_ms ? s.now_ms-s.last_frame_ms:UINT32_MAX;
        const auto fail=[&]()->const Velocity&{ sync(s);return out_; };
        if(!scale_.independently_verified || !s.height_valid || !s.flow_fresh ||
           s.last_frame_ms==0 || out_.sample_age_ms>110 ||
           s.quality<45 || !std::isfinite(s.height_m) || s.height_m<0.25f || s.height_m>1.8f ||
           !std::isfinite(s.roll_rate_dps) || !std::isfinite(s.pitch_rate_dps) ||
           !std::isfinite(s.yaw_rate_dps) ||
           std::fabs(s.roll_rate_dps)>15 || std::fabs(s.pitch_rate_dps)>15 || std::fabs(s.yaw_rate_dps)>15)
            return fail();
        const float a=scale_.fx_per_m_x,b=scale_.fy_per_m_x;
        const float c=scale_.fx_per_m_y,d=scale_.fy_per_m_y;
        const float det=a*d-b*c;
        const float norm1=std::hypot(a,b), norm2=std::hypot(c,d);
        if (scale_.rotation_compensation_verified &&
            (!std::isfinite(scale_.fx_per_rad_roll) ||
             !std::isfinite(scale_.fx_per_rad_pitch) ||
             !std::isfinite(scale_.fx_per_rad_yaw) ||
             !std::isfinite(scale_.fy_per_rad_roll) ||
             !std::isfinite(scale_.fy_per_rad_pitch) ||
             !std::isfinite(scale_.fy_per_rad_yaw))) return fail();
        if(!std::isfinite(det)||norm1<20||norm2<20||
           std::fabs(det)<0.35f*norm1*norm2 ||
           scale_.reference_height_m<0.2f||scale_.reference_height_m>2.0f)
            return fail();
        if(!last_ms_ || s.good_frames<=last_frames_ || s.now_ms<=last_ms_) return fail();
        uint32_t delta_ms=s.now_ms-last_ms_;
        uint32_t delta_frames=s.good_frames-last_frames_;
        if(delta_ms<5||delta_ms>110||delta_frames>30) return fail();
        const float dt=delta_ms*0.001f;
        float fx=static_cast<float>(s.sum_fx-last_fx_);
        float fy=static_cast<float>(s.sum_fy-last_fy_);
        if (scale_.rotation_compensation_verified) {
            // First-order short-window gyro compensation; rates are sampled
            // by NAV at 50Hz, so large/rapid rotation must still be rejected.
            const float rad=0.017453292519943295f*dt;
            fx-=rad*(scale_.fx_per_rad_roll*s.roll_rate_dps +
                     scale_.fx_per_rad_pitch*s.pitch_rate_dps +
                     scale_.fx_per_rad_yaw*s.yaw_rate_dps);
            fy-=rad*(scale_.fy_per_rad_roll*s.roll_rate_dps +
                     scale_.fy_per_rad_pitch*s.pitch_rate_dps +
                     scale_.fy_per_rad_yaw*s.yaw_rate_dps);
        }
        // Counts-per-meter were measured at reference_height_m.
        // Pinhole scaling approximation; must be independently verified in real sensor.
        const float h_ratio=s.height_m/scale_.reference_height_m;
        const float vx=h_ratio*(fx*d-fy*c)/det/dt;
        const float vy=h_ratio*(fy*a-fx*b)/det/dt;
        if(!std::isfinite(vx)||!std::isfinite(vy)||std::fabs(vx)>1.5f||std::fabs(vy)>1.5f)
            return fail();
        out_.valid=true; out_.vx_mps=vx; out_.vy_mps=vy; out_.frames=delta_frames;
        sync(s); return out_;
    }
private:
    void sync(const Sample&s){last_ms_=s.now_ms;last_frames_=s.good_frames;last_fx_=s.sum_fx;last_fy_=s.sum_fy;}
    Scale scale_{}; Velocity out_{};
    uint32_t last_ms_=0,last_frames_=0;
    int64_t last_fx_=0,last_fy_=0;
};
} // namespace v63flow
