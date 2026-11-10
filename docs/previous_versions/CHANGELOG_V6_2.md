# V6.2 — NAV guard + optical-flow calibration-readiness diagnostics

Based on V6.1.2 LIB FIX. No motor output added.

- Added acceleration norm and angular-rate gates with settling period.
- Hardened range spike recovery (180ms cooldown + three-sample warmup).
- Hardened shadow ALT sensor loss, fresh recapture, mode-change reset.
- Added explicit disarmed-only shadow authorization from CH5 / RC state.
- Added raw flow quality/freshness/height/tilt/motion diagnostics.
- `xy_valid` remains hard-false. No optical-flow velocity scale invented.
- Integrated RC_CHECK directly into main logger (no manual code edit).
- Added test_nav_guard, test_flow_diag, test_replay_guard_hold; updated
  test_nav and test_shadow for V6.2 state-machine behavior.

**Safety scope:** BENCH ONLY, no propellers, ESC disconnected,
no flight controller validation or PlatformIO/hardware success claim.
