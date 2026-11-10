#pragma once
#include <stdint.h>

// OneShot125 adapter only. Never map POS outputs to ESC.
namespace flight_esc {
// Input 1000..2000 (logical mixer units) => 125..250us pulse.
inline uint16_t logicalToOneshotUs(int logical) {
    if (logical < 1000) logical = 1000;
    if (logical > 2000) logical = 2000;
    return static_cast<uint16_t>(125 + (logical - 1000 + 4) / 8);
}
bool begin();
void write(const int logical[4], bool enabled);
bool isReady();
}
