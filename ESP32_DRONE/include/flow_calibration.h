#pragma once
// V6.3: Optical-flow axis/scale *diagnostic* via manually measured translations.
// Raw MTF flow units / integration period remain unverified: candidate matrix is
// strictly a bench observation, NEVER a flight velocity estimator or POS controller.
#include <cstdint>
#include <cmath>
#include "flow_diagnostics.h"

enum class FlowCalState : uint8_t { IDLE=0, COLLECT_X=1, COLLECT_Y=2, RESULT=3, FAILED=4 };
inline const char* flowCalStateName(FlowCalState s) {
    switch (s) {
        case FlowCalState::COLLECT_X: return "COLLECT_X";
        case FlowCalState::COLLECT_Y: return "COLLECT_Y";
        case FlowCalState::RESULT: return "RESULT";
        case FlowCalState::FAILED: return "FAILED";
        default: return "IDLE";
    }
}
enum class FlowCalReason : uint8_t {
    NONE=0, COLLECTING=1, NEED_SAMPLES=2, LOW_SIGNAL=3,
    COLLINEAR=4, READY=5, WRONG_MODE=6, BAD_DISTANCE=7,
    FRAME_INVALID=8, ROTATION=9, SPIKE=10, AXIS_MEASURED=11
};
inline const char* flowCalReasonName(FlowCalReason r) {
    switch (r) {
        case FlowCalReason::COLLECTING: return "COLLECTING";
        case FlowCalReason::NEED_SAMPLES: return "NEED_SAMPLES";
        case FlowCalReason::LOW_SIGNAL: return "LOW_SIGNAL";
        case FlowCalReason::COLLINEAR: return "COLLINEAR";
        case FlowCalReason::READY: return "AXES_CANDIDATE";
        case FlowCalReason::AXIS_MEASURED: return "AXIS_MEASURED";
        case FlowCalReason::WRONG_MODE: return "WRONG_MODE";
        case FlowCalReason::BAD_DISTANCE: return "BAD_DISTANCE";
        case FlowCalReason::FRAME_INVALID: return "FRAME_INVALID";
        case FlowCalReason::ROTATION: return "ROTATION";
        case FlowCalReason::SPIKE: return "SPIKE";
        default: return "NONE";
    }
}
struct FlowCalOutput {
    FlowCalState state = FlowCalState::IDLE;
    FlowCalReason reason = FlowCalReason::NONE;
    bool x_measured = false, y_measured = false;
    bool axes_candidate = false; // NEVER xy_valid or m/s
    uint32_t accepted = 0, rejected = 0;
    float planned_m = 0.0f;
    float sum_fx = 0.0f, sum_fy = 0.0f;
    float x_counts_per_m_fx = 0.0f, x_counts_per_m_fy = 0.0f;
    float y_counts_per_m_fx = 0.0f, y_counts_per_m_fy = 0.0f;
    float determinant = 0.0f;
};
struct FlowCalConfig {
    uint32_t min_frames = 12;
    float min_net_counts = 8.0f;
    float min_axis_sin_angle = 0.25f;
    float max_gyro_dps = 15.0f;
    float max_raw_count_per_frame = 200.0f;
    float min_height_m = 0.30f, max_height_m = 1.20f;
};
class FlowCalibrator {
public:
    explicit FlowCalibrator(FlowCalConfig cfg = {}) : cfg_(cfg) {}
    const FlowCalOutput& state() const { return out_; }
    void reset() { out_ = {}; last_seen_ms_ = 0; }
    // Caller MUST gate on CH5 LOW, RC fresh, CH6 MODE 2.
    bool begin(char axis, float displacement_m, bool authorized, uint32_t current_flow_ms) {
        if (!authorized) { out_.reason = FlowCalReason::WRONG_MODE; return false; }
        if (!std::isfinite(displacement_m) || displacement_m < 0.10f || displacement_m > 0.60f) {
            out_.reason = FlowCalReason::BAD_DISTANCE; return false;
        }
        if (axis != 'X' && axis != 'Y') { out_.reason = FlowCalReason::BAD_DISTANCE; return false; }
        out_.state = axis == 'X' ? FlowCalState::COLLECT_X : FlowCalState::COLLECT_Y;
        out_.reason = FlowCalReason::COLLECTING;
        out_.planned_m = displacement_m;
        out_.accepted = out_.rejected = 0;
        out_.sum_fx = out_.sum_fy = 0.0f;
        last_seen_ms_ = current_flow_ms;
        out_.axes_candidate = false;
        return true;
    }
    void step(const FlowDiagOutput& flow, float height_m, float gyro_norm_dps,
              bool authorized, uint32_t frame_time_ms) {
        if (out_.state != FlowCalState::COLLECT_X && out_.state != FlowCalState::COLLECT_Y) return;
        if (!authorized) { out_.state = FlowCalState::FAILED; out_.reason = FlowCalReason::WRONG_MODE; return; }
        if (!frame_time_ms || frame_time_ms == last_seen_ms_) return;
        last_seen_ms_ = frame_time_ms;
        if (!flow.calibration_ready || !std::isfinite(height_m) ||
            height_m < cfg_.min_height_m || height_m > cfg_.max_height_m) {
            ++out_.rejected; out_.reason = FlowCalReason::FRAME_INVALID; return;
        }
        if (!std::isfinite(gyro_norm_dps) || gyro_norm_dps > cfg_.max_gyro_dps) {
            ++out_.rejected; out_.reason = FlowCalReason::ROTATION; return;
        }
        if (std::fabs((float)flow.x_raw) > cfg_.max_raw_count_per_frame ||
            std::fabs((float)flow.y_raw) > cfg_.max_raw_count_per_frame) {
            ++out_.rejected; out_.reason = FlowCalReason::SPIKE; return;
        }
        ++out_.accepted;
        out_.sum_fx += (float)flow.x_raw;
        out_.sum_fy += (float)flow.y_raw;
        out_.reason = FlowCalReason::COLLECTING;
    }
    bool finish() {
        if (out_.state != FlowCalState::COLLECT_X && out_.state != FlowCalState::COLLECT_Y) return false;
        const FlowCalState finished_axis = out_.state;
        if (out_.accepted < cfg_.min_frames) {
            out_.state = FlowCalState::FAILED; out_.reason = FlowCalReason::NEED_SAMPLES; return false;
        }
        const float norm = std::sqrt(out_.sum_fx * out_.sum_fx + out_.sum_fy * out_.sum_fy);
        if (norm < cfg_.min_net_counts) {
            out_.state = FlowCalState::FAILED; out_.reason = FlowCalReason::LOW_SIGNAL; return false;
        }
        if (finished_axis == FlowCalState::COLLECT_X) {
            out_.x_counts_per_m_fx = out_.sum_fx / out_.planned_m;
            out_.x_counts_per_m_fy = out_.sum_fy / out_.planned_m;
            out_.x_measured = true;
        } else {
            out_.y_counts_per_m_fx = out_.sum_fx / out_.planned_m;
            out_.y_counts_per_m_fy = out_.sum_fy / out_.planned_m;
            out_.y_measured = true;
        }
        out_.state = FlowCalState::RESULT;
        out_.reason = FlowCalReason::AXIS_MEASURED;
        if (out_.x_measured && out_.y_measured) {
            out_.determinant = out_.x_counts_per_m_fx * out_.y_counts_per_m_fy -
                               out_.y_counts_per_m_fx * out_.x_counts_per_m_fy;
            const float xnorm = std::sqrt(out_.x_counts_per_m_fx*out_.x_counts_per_m_fx +
                                          out_.x_counts_per_m_fy*out_.x_counts_per_m_fy);
            const float ynorm = std::sqrt(out_.y_counts_per_m_fx*out_.y_counts_per_m_fx +
                                          out_.y_counts_per_m_fy*out_.y_counts_per_m_fy);
            if (xnorm * ynorm == 0.0f ||
                std::fabs(out_.determinant) / (xnorm * ynorm) < cfg_.min_axis_sin_angle) {
                out_.reason = FlowCalReason::COLLINEAR;
            } else {
                out_.axes_candidate = true;
                out_.reason = FlowCalReason::READY;
            }
        }
        return out_.axes_candidate;
    }
private:
    FlowCalConfig cfg_{};
    FlowCalOutput out_{};
    uint32_t last_seen_ms_ = 0;
};
