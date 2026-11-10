#pragma once
// V6.3Y: independent BENCH-only Y-axis motion observation.
// Raw MTF-01P counts are NOT position, velocity or a flight calibration.
// Collect every decoded flow packet in the MTF task, then compare a stationary
// baseline to a bounded single-direction Y translation. No flight output.
#include <cstdint>
#include <cmath>

struct FlowPacketCounters {
    uint32_t frames = 0;
    uint32_t accepted = 0, low_quality = 0, spikes = 0;
    int64_t sum_x = 0, sum_y = 0;
    uint64_t abs_x = 0, abs_y = 0;
    uint32_t pos_x = 0, neg_x = 0, pos_y = 0, neg_y = 0;

    // Quality gate is for counting only, NOT proof of trustworthy XY motion.
    void add(int32_t x, int32_t y, uint8_t q, uint8_t min_q = 70, int32_t cap = 200) {
        ++frames;
        if (q < min_q) { ++low_quality; return; }
        if (static_cast<int64_t>(x) > cap || static_cast<int64_t>(x) < -cap ||
            static_cast<int64_t>(y) > cap || static_cast<int64_t>(y) < -cap) {
            ++spikes; return;
        }
        ++accepted;
        sum_x += x; sum_y += y;
        abs_x += x < 0 ? static_cast<uint64_t>(-static_cast<int64_t>(x)) : static_cast<uint64_t>(x);
        abs_y += y < 0 ? static_cast<uint64_t>(-static_cast<int64_t>(y)) : static_cast<uint64_t>(y);
        if (x > 0) ++pos_x; else if (x < 0) ++neg_x;
        if (y > 0) ++pos_y; else if (y < 0) ++neg_y;
    }
};

inline FlowPacketCounters subtractFlowCounters(const FlowPacketCounters& a,
                                                const FlowPacketCounters& b) {
    FlowPacketCounters d{};
    // Caller samples a and b within a single short (<5s) capture session.
    d.frames = a.frames - b.frames;
    d.accepted = a.accepted - b.accepted;
    d.low_quality = a.low_quality - b.low_quality;
    d.spikes = a.spikes - b.spikes;
    d.sum_x = a.sum_x - b.sum_x; d.sum_y = a.sum_y - b.sum_y;
    d.abs_x = a.abs_x - b.abs_x; d.abs_y = a.abs_y - b.abs_y;
    d.pos_x = a.pos_x - b.pos_x; d.neg_x = a.neg_x - b.neg_x;
    d.pos_y = a.pos_y - b.pos_y; d.neg_y = a.neg_y - b.neg_y;
    return d;
}

enum class YProbePhase : uint8_t { IDLE=0, STILL=1, MOVE=2, DONE=3, ABORTED=4 };
enum class YProbeEvent : uint8_t { NONE=0, MOVE_NOW=1, WINDOW=2, COMPLETE=3, ABORT=4 };
enum class YProbeAbortReason : uint8_t { NONE=0, SAFETY_GATE=1, COUNTER_ROLLBACK=2, TOO_FEW_STILL_PACKETS=3 };
inline const char* yProbeAbortReasonName(YProbeAbortReason r) {
    switch (r) {
        case YProbeAbortReason::SAFETY_GATE: return "SAFETY_GATE";
        case YProbeAbortReason::COUNTER_ROLLBACK: return "COUNTER_ROLLBACK";
        case YProbeAbortReason::TOO_FEW_STILL_PACKETS: return "TOO_FEW_STILL_PACKETS";
        default: return "NONE";
    }
}
// Tolerate only short range estimator glitches during an OBSERVATION, never
// at start and never as permission to enable flight, ARM, or PWM.
inline bool yProbeRangeObservationOk(bool height_valid, uint32_t last_height_valid_ms, uint32_t now) {
    return height_valid || (last_height_valid_ms != 0 &&
        static_cast<uint32_t>(now - last_height_valid_ms) <= 250);
}
class YFlowProbe {
public:
    static constexpr uint32_t kStillMs = 2000;
    static constexpr uint32_t kMoveMs = 3000;
    static constexpr uint32_t kWindowMs = 300;
    bool start(uint32_t now, const FlowPacketCounters& current, bool safe) {
        if (!safe) return false;
        abort_reason_ = YProbeAbortReason::NONE;
        phase_ = YProbePhase::STILL;
        started_ms_ = now;
        window_ms_ = now;
        start_counts_ = current;
        last_window_counts_ = current;
        still_counts_ = {}; move_counts_ = {}; last_window_ = {};
        return true;
    }
    YProbeEvent update(uint32_t now, const FlowPacketCounters& current, bool safe) {
        if (phase_ != YProbePhase::STILL && phase_ != YProbePhase::MOVE) return YProbeEvent::NONE;
        if (!safe) {
            abort_reason_ = YProbeAbortReason::SAFETY_GATE;
            phase_ = YProbePhase::ABORTED;
            return YProbeEvent::ABORT;
        }
        if (current.frames < start_counts_.frames ||
            current.accepted < start_counts_.accepted) {
            abort_reason_ = YProbeAbortReason::COUNTER_ROLLBACK;
            phase_ = YProbePhase::ABORTED;
            return YProbeEvent::ABORT;
        }
        if (phase_ == YProbePhase::STILL && static_cast<uint32_t>(now - started_ms_) >= kStillMs) {
            still_counts_ = subtractFlowCounters(current, start_counts_);
            if (still_counts_.accepted < 20) {
                abort_reason_ = YProbeAbortReason::TOO_FEW_STILL_PACKETS;
                phase_ = YProbePhase::ABORTED;
                return YProbeEvent::ABORT;
            }
            phase_ = YProbePhase::MOVE;
            started_ms_ = now;
            window_ms_ = now;
            start_counts_ = current;
            last_window_counts_ = current;
            return YProbeEvent::MOVE_NOW;
        }
        if (phase_ == YProbePhase::MOVE && static_cast<uint32_t>(now - started_ms_) >= kMoveMs) {
            move_counts_ = subtractFlowCounters(current, start_counts_);
            phase_ = YProbePhase::DONE;
            return YProbeEvent::COMPLETE;
        }
        if (phase_ == YProbePhase::MOVE && static_cast<uint32_t>(now - window_ms_) >= kWindowMs) {
            last_window_ = subtractFlowCounters(current, last_window_counts_);
            last_window_counts_ = current;
            window_ms_ = now;
            return YProbeEvent::WINDOW;
        }
        return YProbeEvent::NONE;
    }
    const FlowPacketCounters& still() const { return still_counts_; }
    const FlowPacketCounters& move() const { return move_counts_; }
    const FlowPacketCounters& window() const { return last_window_; }
    YProbePhase phase() const { return phase_; }
    YProbeAbortReason abortReason() const { return abort_reason_; }
    // Subtract stationary bias measured per valid sample, not raw absolute jitter.
    double correctedX() const {
        return static_cast<double>(move_counts_.sum_x) -
            (still_counts_.accepted ? static_cast<double>(still_counts_.sum_x) * move_counts_.accepted / still_counts_.accepted : 0.0);
    }
    double correctedY() const {
        return static_cast<double>(move_counts_.sum_y) -
            (still_counts_.accepted ? static_cast<double>(still_counts_.sum_y) * move_counts_.accepted / still_counts_.accepted : 0.0);
    }
    double coherence() const {
        const double denom = static_cast<double>(move_counts_.abs_x + move_counts_.abs_y);
        return denom > 0.0 ? std::sqrt(correctedX()*correctedX() + correctedY()*correctedY()) / denom : 0.0;
    }
private:
    YProbePhase phase_ = YProbePhase::IDLE;
    YProbeAbortReason abort_reason_ = YProbeAbortReason::NONE;
    uint32_t started_ms_ = 0, window_ms_ = 0;
    FlowPacketCounters start_counts_{}, last_window_counts_{};
    FlowPacketCounters still_counts_{}, move_counts_{}, last_window_{};
};
