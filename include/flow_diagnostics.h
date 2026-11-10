#pragma once
// Raw flow readiness diagnostics only. NO pixel->m/s conversion, no XY hold.
#include <cstdint>
#include <cmath>

struct FlowDiagInput {
    uint32_t now_ms = 0;
    uint32_t frame_time_ms = 0;
    int32_t x_raw = 0, y_raw = 0;
    uint8_t quality = 0;
    bool frame_valid = false;
    bool height_valid = false;
    bool motion_clear = false;
    float height_m = 0.0f;
    float roll_deg = 0.0f, pitch_deg = 0.0f;
};
enum class FlowDiagReason : uint8_t {
    NO_FRAME = 0, STALE, QUALITY_LOW, HEIGHT_BAD, MOTION_BAD,
    TILT_BAD, CAL_READY
};
inline const char* flowDiagName(FlowDiagReason r) {
    switch (r) {
        case FlowDiagReason::NO_FRAME: return "NO_FRAME";
        case FlowDiagReason::STALE: return "STALE";
        case FlowDiagReason::QUALITY_LOW: return "QUALITY_LOW";
        case FlowDiagReason::HEIGHT_BAD: return "HEIGHT_BAD";
        case FlowDiagReason::MOTION_BAD: return "MOTION_BAD";
        case FlowDiagReason::TILT_BAD: return "TILT_BAD";
        default: return "CAL_READY";
    }
}
struct FlowDiagOutput {
    FlowDiagReason reason = FlowDiagReason::NO_FRAME;
    bool frame_fresh = false;
    bool calibration_ready = false; // only readiness; not XY velocity validity
    bool xy_valid = false; // permanently false in V6.2
    uint32_t frame_age_ms = UINT32_MAX;
    uint32_t ready_samples = 0, rejected_samples = 0;
    int32_t x_raw = 0, y_raw = 0;
    uint8_t quality = 0;
};
struct FlowDiagConfig {
    uint32_t max_age_ms = 120;
    uint8_t min_quality = 70; // for bench calibration, not flight tuning
    float min_height_m = 0.15f;
    float max_height_m = 2.5f;
    float max_tilt_deg = 15.0f;
};
class FlowDiagnostics {
public:
    explicit FlowDiagnostics(FlowDiagConfig cfg = {}) : cfg_(cfg) {}
    const FlowDiagOutput& state() const { return out_; }
    const FlowDiagOutput& step(const FlowDiagInput& in) {
        out_.xy_valid = false;
        out_.quality = in.quality;
        out_.x_raw = in.x_raw;
        out_.y_raw = in.y_raw;
        out_.frame_age_ms = in.frame_time_ms == 0 ? UINT32_MAX :
            (uint32_t)(in.now_ms - in.frame_time_ms);
        out_.frame_fresh = in.frame_time_ms != 0 &&
            out_.frame_age_ms <= cfg_.max_age_ms;
        if (in.frame_time_ms == 0) out_.reason = FlowDiagReason::NO_FRAME;
        else if (!out_.frame_fresh) out_.reason = FlowDiagReason::STALE;
        else if (!in.frame_valid || in.quality < cfg_.min_quality)
            out_.reason = FlowDiagReason::QUALITY_LOW;
        else if (!in.height_valid || !std::isfinite(in.height_m) ||
                 in.height_m < cfg_.min_height_m || in.height_m > cfg_.max_height_m)
            out_.reason = FlowDiagReason::HEIGHT_BAD;
        else if (!in.motion_clear) out_.reason = FlowDiagReason::MOTION_BAD;
        else if (!std::isfinite(in.roll_deg) || !std::isfinite(in.pitch_deg) ||
                 std::fabs(in.roll_deg) > cfg_.max_tilt_deg ||
                 std::fabs(in.pitch_deg) > cfg_.max_tilt_deg)
            out_.reason = FlowDiagReason::TILT_BAD;
        else out_.reason = FlowDiagReason::CAL_READY;
        out_.calibration_ready = out_.reason == FlowDiagReason::CAL_READY;
        // Count only fresh frames not 50 Hz rereads of a single MSP packet.
        if (in.frame_time_ms != 0 && in.frame_time_ms != last_seen_ms_) {
            last_seen_ms_ = in.frame_time_ms;
            if (out_.calibration_ready) ++out_.ready_samples;
            else ++out_.rejected_samples;
        }
        return out_;
    }
private:
    FlowDiagConfig cfg_{};
    FlowDiagOutput out_{};
    uint32_t last_seen_ms_ = 0;
};
