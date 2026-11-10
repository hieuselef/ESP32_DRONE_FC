#pragma once
// V6.3H: guided optical-flow axis diagnostic with explicit user GO.
// Captures stationary baseline separately from a one-way movement; accepts only
// high-coherence motion windows. NO motor output or flight calibration enabled.
#include <cstdint>
#include <cmath>
#include "flow_y_probe.h"

enum class GuidedPhase : uint8_t { IDLE=0, STILL=1, READY=2, MOVE=3, DONE=4, FAILED=5 };
enum class GuidedEvent : uint8_t { NONE=0, READY_FOR_GO=1, WINDOW=2, COMPLETE=3, ABORT=4 };
enum class GuidedReason : uint8_t {
    NONE=0, UNSAFE=1, COUNTER_RESET=2, TOO_FEW_BASELINE=3,
    LOW_QUALITY=4, WEAK_OR_NO_MOTION=5, REVERSE_OR_JITTER=6,
    BACKGROUND_DRIFT=7, AXIS_MEASURED=8, COLLINEAR=9, AXES_CANDIDATE=10,
    GO_TIMEOUT=11
};
inline const char* guidedReasonName(GuidedReason r) {
    switch (r) {
        case GuidedReason::UNSAFE:return "UNSAFE";
        case GuidedReason::GO_TIMEOUT:return "GO_TIMEOUT";
        case GuidedReason::COUNTER_RESET:return "COUNTER_RESET";
        case GuidedReason::TOO_FEW_BASELINE:return "TOO_FEW_BASELINE";
        case GuidedReason::LOW_QUALITY:return "LOW_QUALITY";
        case GuidedReason::WEAK_OR_NO_MOTION:return "WEAK_OR_NO_MOTION";
        case GuidedReason::REVERSE_OR_JITTER:return "REVERSE_OR_JITTER";
        case GuidedReason::BACKGROUND_DRIFT:return "BACKGROUND_DRIFT";
        case GuidedReason::AXIS_MEASURED:return "AXIS_MEASURED";
        case GuidedReason::COLLINEAR:return "COLLINEAR";
        case GuidedReason::AXES_CANDIDATE:return "AXES_CANDIDATE";
        default:return "NONE";
    }
}
struct GuidedAxisResult {
    bool measured = false;
    double fx_per_m = 0, fy_per_m = 0;
    double coherence = 0;
    uint32_t windows = 0;
};
class GuidedFlowCal {
public:
    static constexpr uint32_t kStillMs = 2000, kMoveMs = 5000, kWindowMs = 300, kReadyTimeoutMs = 30000;
    // Restore all state in place (no temporary object on NAV task stack).
    void reset() {
        phase_ = GuidedPhase::IDLE;
        reason_ = GuidedReason::NONE;
        axis_ = '?'; meters_ = 0.0f;
        start_ms_ = window_ms_ = 0;
        first_counts_ = {}; window_counts_ = {};
        still_ = {}; all_move_ = {}; selected_ = {}; last_window_ = {};
        windows_used_ = windows_rejected_ = windows_reverse_ = 0;
        x_ = {}; y_ = {};
    }
    bool start(char axis, float meters, uint32_t now, const FlowPacketCounters& current, bool safe) {
        if (active() || !safe || (axis != 'X' && axis != 'Y') ||
            !std::isfinite(meters) || meters < .10f || meters > .60f) return false;
        // An unsuccessful retry must not leave an earlier result for this same axis.
        if (axis == 'X') x_ = {}; else y_ = {};
        axis_ = axis; meters_ = meters; phase_ = GuidedPhase::STILL;
        reason_ = GuidedReason::NONE; start_ms_ = window_ms_ = now;
        first_counts_ = window_counts_ = current;
        still_ = all_move_ = selected_ = last_window_ = {};
        windows_used_ = windows_rejected_ = windows_reverse_ = 0;
        return true;
    }
    bool active() const { return phase_==GuidedPhase::STILL || phase_==GuidedPhase::READY || phase_==GuidedPhase::MOVE; }
    bool readyForGo() const { return phase_==GuidedPhase::READY; }
    // Only an explicit command starts the MOVE capture. Counters reset here, so
    // seconds spent reading the terminal never contaminate the measurement.
    bool go(uint32_t now, const FlowPacketCounters& current, bool safe) {
        if (phase_!=GuidedPhase::READY || !safe || static_cast<uint32_t>(now-start_ms_)>=kReadyTimeoutMs) return false;
        phase_=GuidedPhase::MOVE;
        start_ms_=window_ms_=now;
        first_counts_=window_counts_=current;
        return true;
    }
    GuidedEvent update(uint32_t now, const FlowPacketCounters& current, bool safe) {
        if (!active()) return GuidedEvent::NONE;
        if (!safe) return abort(GuidedReason::UNSAFE);
        // Prevent unsigned wraparound on counters reset/reboot.
        if (current.frames < first_counts_.frames || current.accepted < first_counts_.accepted ||
            current.abs_x < first_counts_.abs_x || current.abs_y < first_counts_.abs_y ||
            current.low_quality < first_counts_.low_quality || current.spikes < first_counts_.spikes)
            return abort(GuidedReason::COUNTER_RESET);
        if (phase_==GuidedPhase::STILL && static_cast<uint32_t>(now-start_ms_)>=kStillMs) {
            still_ = subtractFlowCounters(current, first_counts_);
            if (still_.accepted < 80) return abort(GuidedReason::TOO_FEW_BASELINE);
            phase_=GuidedPhase::READY; start_ms_=now;
            return GuidedEvent::READY_FOR_GO;
        }
        if (phase_==GuidedPhase::READY && static_cast<uint32_t>(now-start_ms_)>=kReadyTimeoutMs)
            return abort(GuidedReason::GO_TIMEOUT);
        if (phase_==GuidedPhase::MOVE && static_cast<uint32_t>(now-start_ms_)>=kMoveMs) {
            last_window_ = subtractFlowCounters(current,window_counts_);
            considerWindow(last_window_);
            all_move_ = subtractFlowCounters(current,first_counts_);
            finish(); phase_=GuidedPhase::DONE;
            return GuidedEvent::COMPLETE;
        }
        if (phase_==GuidedPhase::MOVE && static_cast<uint32_t>(now-window_ms_)>=kWindowMs) {
            last_window_ = subtractFlowCounters(current,window_counts_);
            considerWindow(last_window_);
            window_counts_ = current; window_ms_=now;
            return GuidedEvent::WINDOW;
        }
        return GuidedEvent::NONE;
    }
    char axis() const {return axis_;}
    float plannedMeters() const {return meters_;}
    GuidedPhase phase() const {return phase_;}
    GuidedReason reason() const {return reason_;}
    const FlowPacketCounters& still() const {return still_;}
    const FlowPacketCounters& allMove() const {return all_move_;}
    const FlowPacketCounters& selected() const {return selected_;}
    const FlowPacketCounters& window() const {return last_window_;}
    uint32_t windowsUsed() const {return windows_used_;}
    uint32_t windowsRejected() const {return windows_rejected_;}
    uint32_t windowsReverse() const {return windows_reverse_;}
    double stillAbsRate() const {return still_.accepted ? double(still_.abs_x+still_.abs_y)/still_.accepted : 0;}
    double selectedCoherence() const {return coherence(selected_);}
    double rawCoherence() const {return coherence(all_move_);}
    const GuidedAxisResult& x() const {return x_;}
    const GuidedAxisResult& y() const {return y_;}
    double axisSin() const {
        if (!x_.measured || !y_.measured) return 0;
        const double nx=std::hypot(x_.fx_per_m,x_.fy_per_m), ny=std::hypot(y_.fx_per_m,y_.fy_per_m);
        if (nx <= 0 || ny <= 0) return 0;
        return std::fabs(x_.fx_per_m*y_.fy_per_m-x_.fy_per_m*y_.fx_per_m)/(nx*ny);
    }
    bool axesCandidate() const {return x_.measured && y_.measured && axisSin() >= .30;}
private:
    static double net(const FlowPacketCounters& c) {return std::hypot(double(c.sum_x),double(c.sum_y));}
    static double coherence(const FlowPacketCounters& c) {
        const double abs=double(c.abs_x+c.abs_y);
        return abs>0 ? net(c)/abs : 0;
    }
    static void combine(FlowPacketCounters &a, const FlowPacketCounters &b) {
        a.frames+=b.frames; a.accepted+=b.accepted; a.low_quality+=b.low_quality; a.spikes+=b.spikes;
        a.sum_x+=b.sum_x; a.sum_y+=b.sum_y; a.abs_x+=b.abs_x; a.abs_y+=b.abs_y;
        a.pos_x+=b.pos_x; a.neg_x+=b.neg_x; a.pos_y+=b.pos_y; a.neg_y+=b.neg_y;
    }
    void considerWindow(const FlowPacketCounters& w) {
        if (w.accepted<15) {++windows_rejected_;return;}
        const double still_rate=stillAbsRate();
        const double motion_net=net(w), motion_abs=double(w.abs_x+w.abs_y);
        // Guided bench-only capture: MPU/flow can show low-amplitude translation
        // over textured floors even when packet quality is high. Preserve the
        // direction check below; accept individual windows with moderate
        // coherence but require a MUCH stronger aggregate path in finish().
        // Do not use this diagnostic as automatic permission for POS actuation.
        const bool movement=motion_abs >= still_rate*w.accepted*1.05 &&
            motion_net >= fmax(8.0,still_rate*w.accepted*.45) && coherence(w)>=.50;
        if (!movement) {++windows_rejected_;return;}
        if (selected_.accepted && net(selected_) > 0) {
            const double dot=double(selected_.sum_x)*w.sum_x + double(selected_.sum_y)*w.sum_y;
            const double cos=dot/(net(selected_)*motion_net);
            if (cos < .55) {++windows_reverse_;return;}
        }
        combine(selected_,w); ++windows_used_;
    }
    void finish() {
        reason_=GuidedReason::WEAK_OR_NO_MOTION;
        if (all_move_.accepted < 90 || (all_move_.low_quality+all_move_.spikes) > all_move_.frames/3) {
            reason_=GuidedReason::LOW_QUALITY;return;
        }
        if (windows_used_ < 3 || selected_.accepted < 60) return;
        if (windows_reverse_ > 0 || selectedCoherence()<.55 || rawCoherence()<.30) {
            reason_=GuidedReason::REVERSE_OR_JITTER;return;
        }
        const double baseline_drift=still_.accepted ? net(still_)*selected_.accepted/still_.accepted : 0.0;
        // Require a substantial one-way displacement ABOVE stationary drift.
        // More permissive windows may NOT turn weak or reversing noise into
        // a valid axis measurement. The 60-count floor is for a 30 cm path.
        if (net(selected_) < fmax(60.0*meters_/.30,baseline_drift*2.5)) {
            reason_=GuidedReason::BACKGROUND_DRIFT;return;
        }
        GuidedAxisResult r{};
        r.measured=true; r.fx_per_m=double(selected_.sum_x)/meters_;
        r.fy_per_m=double(selected_.sum_y)/meters_;
        r.coherence=selectedCoherence(); r.windows=windows_used_;
        if (axis_=='X') x_=r; else y_=r;
        reason_=(!x_.measured || !y_.measured) ? GuidedReason::AXIS_MEASURED :
            (axesCandidate() ? GuidedReason::AXES_CANDIDATE : GuidedReason::COLLINEAR);
    }
    GuidedEvent abort(GuidedReason reason) { reason_=reason;phase_=GuidedPhase::FAILED;return GuidedEvent::ABORT; }
    GuidedPhase phase_=GuidedPhase::IDLE;
    GuidedReason reason_=GuidedReason::NONE;
    char axis_='?';float meters_=0;
    uint32_t start_ms_=0,window_ms_=0;
    FlowPacketCounters first_counts_{},window_counts_{},still_{},all_move_{},selected_{},last_window_{};
    uint32_t windows_used_=0,windows_rejected_=0,windows_reverse_=0;
    GuidedAxisResult x_{},y_{};
};
