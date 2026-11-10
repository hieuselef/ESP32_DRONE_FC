#include "guided_rc_gate.h"
#include <cassert>
#include <iostream>
int main() {
    // Ignore transient control_task mode fallbacks: physical CH6 remains 2000.
    assert(guidedRcModeAuthorized(true,false,1000,2000,1300,1700,2050));
    assert(!guidedRcModeAuthorized(true,false,1000,1000,1300,1700,2050));
    assert(!guidedRcModeAuthorized(true,false,1500,2000,1300,1700,2050));
    assert(!guidedRcModeAuthorized(true,true,1000,2000,1300,1700,2050));
    assert(!guidedRcModeAuthorized(false,false,1000,2000,1300,1700,2050));
    assert(!guidedRcModeAuthorized(true,false,1000,2070,1300,1700,2050));
    // Short missed mutex acquisition keeps last confirmed fresh RC snapshot.
    assert(guidedSnapshotFresh(1030,1000,1020,true,350,80));
    assert(guidedSnapshotFresh(1080,1000,1020,true,350,80));
    assert(!guidedSnapshotFresh(1081,1000,1020,true,350,80));
    assert(!guidedSnapshotFresh(1400,1000,1020,true,350,80));
    assert(!guidedSnapshotFresh(1030,1000,1020,false,350,80));
    assert(!guidedSnapshotFresh(1030,0,1020,true,350,80));
    assert(!guidedSnapshotFresh(1030,1000,0,true,350,80));
    std::cout << "V6.3H1 Guided RC gate checks passed\n";
}
