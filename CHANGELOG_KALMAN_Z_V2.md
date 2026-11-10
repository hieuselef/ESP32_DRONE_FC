# V2 Kalman Z ALT motor feedback — changelog

- Giữ nguyên các phần ARM, ANGLE, ALT, POS guarded, ELRS loss, OneShot125 motor mapping và các giới hạn ALT V2.
- `include/flight_altitude.h`: thêm `RangeState::measured_height_m` (frame mới, đã qua kiểm tra RangeTracker), giữ nguyên bộ điều khiển ALT cũ.
- `include/vertical_estimator.h`: KF hai trạng thái dùng range sample độc lập đã được bù nghiêng; không trộn MPU acceleration chưa xác minh.
- `include/alt_kalman_authority.h` (mới): gate confidence/innovation/σ, 8 frame warmup, blend 600ms, raw fallback latching, raw range invalid hard gate.
- `src/control_task.cpp`: chuyển đầu ra KF vào ALT theo compile flag; nếu không enable, đường ALT nhận RangeTracker như trước. Không đổi logic ESC, PID rate, RC và failsafe.
- `include/types.h`, `src/main.cpp`: log `[ALT_KF_Z]` gồm phase, applied, healthy, blend, fail latch, sigma và tuổi frame.
- `platformio.ini`: thêm build thử trên bàn `bench_kalman_z` và build opt-in ESC `flight_alt_kalman_z_candidate`; mặc định `bench`.
- `tests/test_alt_kalman_motor_path.cpp`: diễn tập điều khiển ga từ KF, giới hạn, fallback và lỗi range; các tests cũ giữ nguyên.
- Chưa xác nhận `pio run` thật, chưa flash ESP32, đo xung ESC hay bay thử.
