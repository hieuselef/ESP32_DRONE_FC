# CHANGELOG V6.3

- Dựa trên toàn bộ mã nguồn V6.2, giữ pinout, UART MSP V2, RC T8L/ELRS, thư viện PlatformIO đã chạy trên V6.2.
- Tách `NavEstimator::READY` khỏi quyền kích hoạt ALT_HOLD; thêm trạng thái `RECOVERY_PENDING`.
- Buộc `u=0`, `i=0` trong `SENSOR_LOSS`, `WAIT_HEIGHT`, `RECOVERY_PENDING`; chờ >=800ms, >=20 frame mới, ổn định chiều cao & tốc độ trước khi ghi target.
- Cấu hình ngưỡng riêng cho capture mới (0.30..1.80m) và tracking (0.25..2.00m) để chống rung trạng thái gần ngưỡng. Ngăn capture ở độ cao 8–10 cm.
- Thêm `capture_reason`, `capture_count`, `gate`, stable window & count, outlier diagnostic (last rejected range, delta, counter).
- Thêm thử nghiệm Optical Flow thu thập vectơ X+ và Y+ khi người dùng dịch chuyển thân drone có đo khoảng cách, điều khiển bằng USB Serial `cal x`, `cal y`, `cal stop`, `cal reset`, `cal status`.
- Mọi kết quả flow là ứng viên chẩn đoán trục, không suy ra m/s, không bật `xy_valid`/POS controller, không chạm motor.
- Khóa `ENABLE_MOTOR_OUTPUT=false` và `static_assert` giữ nguyên; không có PWM ESC.
