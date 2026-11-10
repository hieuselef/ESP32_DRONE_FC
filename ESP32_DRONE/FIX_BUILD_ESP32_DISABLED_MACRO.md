# Fix PlatformIO error: DISABLED macro collision (ESP32 FC V2)

**Cause:** Arduino ESP32 `esp32-hal-gpio.h` defines `#define DISABLED 0x00`.
`include/pos_authority.h` previously declared `enum class Reason { OFF=0, DISABLED, ... }`, so the preprocessor replaced enum token `DISABLED` with `0x00`, breaking the enum and triggering cascaded errors through the closing namespace brace.

**Fix:** Rename the enum member to `POS_DISABLED` and update all C++ references. The text returned by `reasonName` remains `DISABLED` so diagnostic log vocabulary does not change. Numeric enum values, control gates, PWM/ESC settings, Kalman, and PID are unchanged.

**Recommended test:** REMOVE PROPELLERS. Open the ZIP project root (the directory with `platformio.ini`), select `bench_kalman_z_imu`, run:

```bash
pio run -e bench_kalman_z_imu -t clean
pio run -e bench_kalman_z_imu
```

Your earlier log showed `Environment bench`: if the terminal still builds `bench`, explicitly choose environment `bench_kalman_z_imu` in PlatformIO. Do not move to experimental motor output environments until hardware safety validation is complete.

**Verification:** host C++ tests + a compile with `DISABLED=0x00` to reproduce Arduino ESP32 macro environment. Not a substitute for a real PlatformIO ESP32 build.
