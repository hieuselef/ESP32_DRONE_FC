#pragma once
#include <math.h>
#include <stdint.h>

// Cac ham khong phu thuoc Arduino: co the unit-test tren may tinh.
namespace controller_math {
inline float clampf(float x, float a, float b) {
    return x < a ? a : (x > b ? b : x);
}
inline float lpAlpha(float cutoffHz, float dt) {
    if (cutoffHz <= 0.0f || dt <= 0.0f) return 1.0f;
    const float rc = 1.0f / (6.28318530718f * cutoffHz);
    return dt / (dt + rc);
}

struct PidOutput {
    float p = 0.0f, i = 0.0f, d = 0.0f, total = 0.0f;
    float measured_filtered = 0.0f;
    bool saturated = false;
};

class RatePid {
public:
    RatePid(float kp, float ki, float kd, float gyroCutHz, float dCutHz,
            float maxTermUs, float maxITermUs)
      : kp_(kp), ki_(ki), kd_(kd), gyroCutHz_(gyroCutHz), dCutHz_(dCutHz),
        limit_(maxTermUs), maxI_(maxITermUs) {}

    void reset() { init_ = false; iTerm_ = 0.0f; gyroLP_ = 0.0f; lastGyroLP_ = 0.0f; dLP_ = 0.0f; }

    PidOutput step(float targetRate, float rawRate, float dt, bool canIntegrate) {
        PidOutput r;
        if (!isfinite(targetRate) || !isfinite(rawRate) || dt <= 0.0f || dt > 0.03f) {
            reset();
            return r;
        }
        if (!init_) {
            gyroLP_ = lastGyroLP_ = rawRate;
            dLP_ = 0.0f;
            init_ = true;
        }
        gyroLP_ += lpAlpha(gyroCutHz_, dt) * (rawRate - gyroLP_);
        const float derivative = (gyroLP_ - lastGyroLP_) / dt;
        lastGyroLP_ = gyroLP_;
        dLP_ += lpAlpha(dCutHz_, dt) * (derivative - dLP_);
        const float err = targetRate - gyroLP_;
        r.p = kp_ * err;
        r.d = -kd_ * dLP_;
        // Anti-windup: khong tich phan neu dau ra saturate theo chieu sai so.
        float outputNoNewI = r.p + iTerm_ + r.d;
        const bool pushesBeyondLimit =
            (outputNoNewI >= limit_ && err > 0.0f) ||
            (outputNoNewI <= -limit_ && err < 0.0f);
        if (canIntegrate && !pushesBeyondLimit) {
            iTerm_ = clampf(iTerm_ + ki_ * err * dt, -maxI_, maxI_);
        }
        r.i = iTerm_;
        const float sum = r.p + r.i + r.d;
        r.total = clampf(sum, -limit_, limit_);
        r.saturated = (fabsf(sum) > limit_);
        r.measured_filtered = gyroLP_;
        return r;
    }
private:
    float kp_, ki_, kd_, gyroCutHz_, dCutHz_, limit_, maxI_;
    bool init_ = false;
    float iTerm_ = 0.0f, gyroLP_ = 0.0f, lastGyroLP_ = 0.0f, dLP_ = 0.0f;
};

struct MotorMix {
    int us[4] = {1000,1000,1000,1000};
    bool clipped = false;
};
// Mixer goc du an: motor[0] rear-right, [1] front-right,
// [2] rear-left, [3] front-left. Chua xac minh dau phan hoi thuc te!
inline MotorMix mixX(int throttleUs, float roll, float pitch, float yaw,
                     int minUs = 1000, int maxUs = 1950) {
    MotorMix out;
    const float corrections[4] = {
        -roll + pitch + yaw,
        -roll - pitch - yaw,
         roll + pitch - yaw,
         roll - pitch + yaw
    };
    for (int i=0; i<4; ++i) {
        float raw = throttleUs + corrections[i];
        out.clipped = out.clipped || raw < minUs || raw > maxUs;
        raw = clampf(raw, static_cast<float>(minUs), static_cast<float>(maxUs));
        out.us[i] = static_cast<int>(lroundf(raw));
    }
    return out;
}
} // namespace controller_math
