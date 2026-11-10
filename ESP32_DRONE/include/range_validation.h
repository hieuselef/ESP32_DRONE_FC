#pragma once
#include <stdint.h>
namespace range_validation {
inline bool packetRangeOk(int32_t mm, uint8_t quality) {
    // Sensor reports -1/-3 on invalid returns. Reject before any cast.
    return quality > 0 && mm >= 20 && mm <= 12000;
}
inline bool navRangeEligible(int32_t mm, uint8_t quality) {
    // Conservative bench-only interval, NOT full spec of MTF-01P.
    return packetRangeOk(mm, quality) && mm >= 80 && mm <= 3000;
}
}
