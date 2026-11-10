#pragma once
// 2-state Kalman: height (m), vertical speed (m/s), with OPTIONAL calibrated
// MPU6050 Earth-Z acceleration as prediction input, plus corrected lidar frames.
// Motor authority remains a separate explicit compile-time opt-in and gated.
// Only UNIQUE, validated range frames correct the prediction. A stale repeated
// range value is NOT re-counted as a statistically independent measurement.
#include <cmath>
#include <cstdint>
#include <algorithm>
namespace vertical_estimator {
struct State {
    bool initialized=false, healthy=false, measurement_used=false;
    float height_m=0, velocity_mps=0;
    float sigma_height_m=0, sigma_velocity_mps=0;
    float last_innovation_m=0;
    uint32_t last_measurement_age_ms=UINT32_MAX;
    uint32_t corrections=0, outliers=0, resets=0;
    bool imu_accel_used=false;
    float prediction_accel_mps2=0.0f;
    uint32_t imu_accel_predictions=0;
};
class Kalman2 {
public:
    void reset() { s_ = {}; p00_=0.0025f; p01_=0; p11_=0.25f;
                   last_predict_ms_=0; last_accepted_ms_=0; last_frame_ms_=0; }
    Kalman2(){ reset(); }
    // Legacy 5-argument callers remain range-only; passing optional calibrated
    // Earth-Z LINEAR acceleration activates the inertial prediction step.
    const State& update(uint32_t now_ms, bool sample_valid,
                        bool new_sample, uint32_t range_frame_ms,
                        float range_height_m, float earth_accel_z_mps2=0.0f,
                        bool calibrated_imu_valid=false) {
        s_.measurement_used=false;
        s_.imu_accel_used=false;
        s_.prediction_accel_mps2=0.0f;
        const bool use_imu=calibrated_imu_valid &&
            std::isfinite(earth_accel_z_mps2) &&
            std::fabs(earth_accel_z_mps2)<=4.5f;
        if (s_.initialized && last_predict_ms_ != 0) {
            const uint32_t elapsed=now_ms-last_predict_ms_;
            if (elapsed>1000) { const uint32_t r=s_.resets+1; reset(); s_.resets=r; }
            else if (elapsed>0) predict(std::min(elapsed, 50U)*0.001f,
                                        use_imu ? earth_accel_z_mps2 : 0.0f,
                                        use_imu);
        }
        last_predict_ms_=now_ms;
        if (sample_valid && new_sample && range_frame_ms &&
            range_frame_ms != last_frame_ms_ &&
            (uint32_t)(now_ms-range_frame_ms) <= MAX_MEAS_AGE_MS &&
            std::isfinite(range_height_m) &&
            range_height_m >= 0.20f && range_height_m <= 1.80f) {
            last_frame_ms_=range_frame_ms;
            if (!s_.initialized) initState(range_height_m,range_frame_ms);
            else correct(range_height_m,range_frame_ms);
        }
        s_.last_measurement_age_ms=last_accepted_ms_ ?
            (uint32_t)(now_ms-last_accepted_ms_) : UINT32_MAX;
        // Unlike prediction, new valid range frames are necessary for health.
        // Actuation is separately gated and must also have RAW range validity.
        s_.healthy=s_.initialized && s_.last_measurement_age_ms<=MAX_MEAS_AGE_MS &&
                   std::isfinite(s_.height_m) && std::isfinite(s_.velocity_mps) &&
                   std::isfinite(p00_) && std::isfinite(p11_) && p00_>=0 && p11_>=0;
        if (s_.initialized && !s_.healthy && s_.last_measurement_age_ms>1000) {
            const uint32_t resets=s_.resets+1;
            reset(); s_.resets=resets;
        }
        return s_;
    }
    const State& state()const {return s_;}
private:
    static constexpr uint32_t MAX_MEAS_AGE_MS=140;
    static constexpr float RANGE_SIGMA_M=0.035f; // provisional; identify from logs
    static constexpr float PROCESS_SIGMA_ACCEL_MPS2=1.5f; // provisional
    static constexpr float OUTLIER_FLOOR_M=0.22f;
    void initState(float z,uint32_t stamp) {
        s_.initialized=true; s_.height_m=z; s_.velocity_mps=0;
        p00_=RANGE_SIGMA_M*RANGE_SIGMA_M; p01_=0; p11_=0.25f;
        last_accepted_ms_=stamp; ++s_.corrections; s_.measurement_used=true;
        updateSigmas();
    }
    void predict(float dt,float accel_z,bool using_imu){
        const float q=PROCESS_SIGMA_ACCEL_MPS2*PROCESS_SIGMA_ACCEL_MPS2 *
                      (using_imu ? 2.25f : 1.0f);
        s_.height_m += 0.5f*accel_z*dt*dt;
        s_.velocity_mps += accel_z*dt;
        s_.imu_accel_used=using_imu;
        s_.prediction_accel_mps2=using_imu ? accel_z : 0.0f;
        if (using_imu) ++s_.imu_accel_predictions;
        const float dt2=dt*dt, dt3=dt2*dt, dt4=dt2*dt2;
        s_.height_m += dt*s_.velocity_mps;
        const float p00=p00_+2*dt*p01_+dt2*p11_+0.25f*dt4*q;
        const float p01=p01_+dt*p11_+0.5f*dt3*q;
        const float p11=p11_+dt2*q;
        p00_=p00; p01_=p01; p11_=p11;
        updateSigmas();
    }
    void correct(float z,uint32_t stamp){
        const float r=RANGE_SIGMA_M*RANGE_SIGMA_M;
        const float innovation=z-s_.height_m;
        const float s=p00_+r;
        s_.last_innovation_m=innovation;
        if (!(s>1e-8f) || !std::isfinite(s) ||
            std::fabs(innovation)>std::max(OUTLIER_FLOOR_M,5.0f*std::sqrt(s))) {
            ++s_.outliers; return;
        }
        const float k0=p00_/s, k1=p01_/s;
        s_.height_m+=k0*innovation;
        s_.velocity_mps+=k1*innovation;
        // Joseph covariance update: (I-KH)P(I-KH)' + KRK'.
        // For symmetric 2x2 P and H=[1,0], simplified below.
        const float a=1.0f-k0, b=-k1;
        const float p00=p00_, p01=p01_, p11=p11_;
        p00_=a*a*p00 + k0*k0*r;
        p01_=a*(b*p00+p01)+k0*k1*r;
        p11_=b*b*p00+2*b*p01+p11+k1*k1*r;
        if (!std::isfinite(s_.height_m) || !std::isfinite(s_.velocity_mps)
            || !std::isfinite(p00_) || !std::isfinite(p11_)) {
            const uint32_t n=s_.resets+1; reset(); s_.resets=n; return;
        }
        last_accepted_ms_=stamp;
        ++s_.corrections; s_.measurement_used=true;
        updateSigmas();
    }
    void updateSigmas(){
        s_.sigma_height_m=std::sqrt(std::max(0.0f,p00_));
        s_.sigma_velocity_mps=std::sqrt(std::max(0.0f,p11_));
    }
    State s_{};
    float p00_=0.0025f,p01_=0,p11_=0.25f;
    uint32_t last_predict_ms_=0,last_accepted_ms_=0,last_frame_ms_=0;
};
} // namespace vertical_estimator
