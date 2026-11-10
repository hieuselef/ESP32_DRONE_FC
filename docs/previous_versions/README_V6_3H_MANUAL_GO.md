# V6.3H — Guided Flow Calibration (Manual GO, BENCH ONLY)

Fix: the previous `MOVE_NOW` announcement scrolled by in the noisy Serial Monitor.
The new version is **manual trigger**: no movement capture begins until an explicit `cal guided go`. Routine `[BENCH]/[NAV]/[RC]` prints are temporarily muted during calibration and for 20s after the result; sensor and safety tasks continue normally. The READY state times out after 30s if no GO is received. Capture after GO lasts **5 seconds**.

### Test trục Y

1. **Tháo cánh quạt và ngắt nguồn ESC.** Giữ CH5=1000, CH6=2000, cảm biến nhìn xuống nền có hoa văn tại 40–50cm, Flow Quality >=70 ổn định.
2. Mở PlatformIO Serial Monitor 115200 (`monitor_eol = LF`).
3. Nhập `cal guided y 0.30` + Enter. Giữ đứng yên ít nhất 2 giây.
4. Chờ dòng `[FLOW_GUIDED_V6_3H] READY` (log khác đã được tạm ẩn). **Lúc này chưa thu chuyển động.**
5. Chuẩn bị đường di chuyển sang phải 30cm, nhập `cal guided go` + Enter, và **dịch drone sang phải một lần trong 5 giây**, không quay về điểm đầu.
6. Gửi log `[FLOW_GUIDED_V6_3H] BASELINE`, `[FLOW_GUIDED_V6_3H] DONE`, `[FLOW_GUIDED_V6_3H] AXES`.

If no GO within 30s after READY -> `GO_TIMEOUT`. If a safety condition fails, calibration immediately ABORTs. `cal guided reset` can clear the session. The motor output remains hard locked (`PWM:0`), `xy_valid:0`, no flight/POS HOLD use.

### Scope

Only modified `include/flow_guided_cal.h`, `include/globals.h`, `src/globals.cpp`, `src/nav_task.cpp`, `src/main.cpp`, plus tests/docs. Did not modify PID, ESC code, MTF UART parser, RC handling, height estimator or shadow control.
