#include "../include/range_validation.h"
#include <cassert>
#include <cstdio>
int main() {
    using namespace range_validation;
    assert(!packetRangeOk(-3, 255));
    assert(!packetRangeOk(-1, 255));
    assert(!packetRangeOk(0, 255));
    assert(!packetRangeOk(500, 0));
    assert(packetRangeOk(20, 1));
    assert(!navRangeEligible(20, 1));
    assert(navRangeEligible(300, 255));
    assert(!navRangeEligible(3001, 255));
    puts("PASS signed range validation tests");
}
