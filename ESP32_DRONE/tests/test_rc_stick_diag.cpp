#include <cassert>
#include <cmath>
#include "rc_stick_diag.h"
using namespace rc_stick_diag;
static void near(float a, float b) { assert(std::fabs(a-b) < 0.0001f); }
int main() {
    near(norm(1500), 0.0f);
    near(norm(1480), 0.0f); near(norm(1520), 0.0f);
    near(angle(1000, +1, 20), -20.0f);
    near(angle(2000, +1, 20), +20.0f);
    near(angle(1550, +1, 20), +2.0f);
    near(angle(1550, -1, 20), -2.0f);
    near(angle(950, +1, 20), -20.0f);
    near(angle(2050, +1, 20), +20.0f);
    near(yawRate(2000, +1, 100), +100.0f);
    near(yawRate(1000, -1, 100), +100.0f);
}
