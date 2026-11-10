#pragma once
// ALT V2 authority gate. Hardware-independent and host-testable.
// RC / IMU / scheduler / DISARM have absolute priority. A low throttle stick
// is a descent request WHILE ALT is engaged. Never use the throttle-low
// condition to silently terminate the ALT controller mid-air.
// In ANGLE with no ALT handover, original throttle-low hard motor stop remains.
#include <cstdint>

namespace alt_motor_gate {
enum Phase : uint8_t {
    HARD_STOP = 0, MANUAL = 1, ALT_AUTHORITY = 2,
    ALT_LOW_STICK = 3, MANUAL_HANDOVER = 4
};
struct Decision {
    bool motor_active = false;
    bool low_stick_alt_authority = false;
    Phase phase = HARD_STOP;
};
inline Decision decide(bool armed, bool rc_ok, bool imu_ok, bool timing_ok,
                       bool esc_ready, bool alt_was_controlling,
                       bool handover_blending, int stick_us,
                       int throttle_run_min_us = 1100) {
    Decision result{};
    // Hard safety gates NEVER get blended or masked by the altitude controller.
    if (!armed || !rc_ok || !imu_ok || !timing_ok || !esc_ready) return result;
    const bool pilot_throttle_running = stick_us >= throttle_run_min_us;
    const bool prior_alt_authority = alt_was_controlling || handover_blending;
    result.motor_active = pilot_throttle_running || prior_alt_authority;
    if (!result.motor_active) return result;
    result.low_stick_alt_authority = !pilot_throttle_running && prior_alt_authority;
    result.phase = handover_blending ? MANUAL_HANDOVER :
                   (result.low_stick_alt_authority ? ALT_LOW_STICK :
                   (alt_was_controlling ? ALT_AUTHORITY : MANUAL));
    return result;
}
} // namespace alt_motor_gate
