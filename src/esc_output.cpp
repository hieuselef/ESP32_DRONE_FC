#include "esc_output.h"
#include "config.h"

namespace flight_esc {
namespace {
constexpr uint8_t pins[4] = {PIN_ESC_1, PIN_ESC_2, PIN_ESC_3, PIN_ESC_4};
constexpr uint32_t kRateHz = 480; // Matches archived esp-fc dump output_motor_rate=480.
constexpr uint8_t kResolutionBits = 16;
constexpr uint32_t kPeriodUs = 1000000UL/kRateHz;
bool ready = false;
uint32_t dutyFromPulse(uint16_t pulseUs) {
    return (uint32_t(pulseUs)*((1UL<<kResolutionBits)-1UL)+kPeriodUs/2UL)/kPeriodUs;
}
void channelPulse(int c, uint16_t pulseUs) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcWriteChannel(c, dutyFromPulse(pulseUs));
#else
    ledcWrite(c, dutyFromPulse(pulseUs));
#endif
}
}
bool begin() {
    if (!ENABLE_MOTOR_OUTPUT) return false;
    for (int i=0; i<4; ++i) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
        if (!ledcAttachChannel(pins[i], kRateHz, kResolutionBits, i)) return false;
#else
        if (ledcSetup(i, kRateHz, kResolutionBits) == 0) return false;
        ledcAttachPin(pins[i], i);
#endif
        channelPulse(i, 125);
    }
    ready = true;
    return true;
}
void write(const int logical[4], bool enabled) {
    if (!ready || !ENABLE_MOTOR_OUTPUT) return;
    for (int i=0; i<4; ++i) {
        // DISARM/FAILSAFE always sends minimum. Oneshot125 is NOT analog 1000us PWM.
        const uint16_t pulse = enabled ? logicalToOneshotUs(logical[i]) : 125;
        channelPulse(i, pulse);
    }
}
bool isReady() { return ready; }
}
