#pragma once
#include <cstdint>

// BENCH calibration ONLY. This helper does not change RC failsafe in the
// motor/control task. A cached snapshot is valid only for a very short time.
inline bool guidedSnapshotFresh(uint32_t now, uint32_t read_ms,
                                uint32_t last_packet_ms, bool link_ok,
                                uint32_t rc_stale_ms, uint32_t max_cache_ms) {
    return read_ms != 0 && last_packet_ms != 0 && link_ok &&
           (uint32_t)(now - read_ms) <= max_cache_ms &&
           (uint32_t)(now - last_packet_ms) < rc_stale_ms;
}

inline bool guidedRcModeAuthorized(bool rc_fresh, bool armed_logic,
                                  int arm_channel, int mode_channel,
                                  int arm_low_max, int pos_mode_min,
                                  int channel_max) {
    return rc_fresh && !armed_logic && arm_channel < arm_low_max &&
           mode_channel >= pos_mode_min && mode_channel <= channel_max;
}
