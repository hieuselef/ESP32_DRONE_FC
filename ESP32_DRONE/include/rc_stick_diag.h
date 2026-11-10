#pragma once
// Read-only diagnostic math; NEVER used in control_task / mixer / ESC.
namespace rc_stick_diag {
inline float norm(int us) {
    const int delta = us - 1500;
    if (delta >= -20 && delta <= 20) return 0.0f;
    float v = static_cast<float>(delta) / 500.0f;
    if (v < -1.0f) v = -1.0f;
    if (v > +1.0f) v = +1.0f;
    return v;
}
inline float angle(int us, float sign, float maxAngleDeg) {
    return sign * norm(us) * maxAngleDeg;
}
inline float yawRate(int us, float sign, float maxRateDps) {
    return sign * norm(us) * maxRateDps;
}
}
