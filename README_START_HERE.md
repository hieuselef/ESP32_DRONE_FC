> **MỚI: KALMAN Z -> MOTOR ALT HOLD (opt-in, chưa kiểm định bay):** đọc `README_KALMAN_Z_MOTOR_V2.md` trước. Mặc định build `bench` không phát ESC; build `flight_alt_kalman_z_candidate` cho phép KF Z tham gia ALT motor qua gate và các giới hạn V2. POS HOLD vẫn khóa điều khiển motor. Các số liệu build/test của phần nội dung cũ dưới đây là tài liệu lịch sử của nhánh V2 trước khi nâng cấp.

> **Phát triển POS HOLD theo hướng dẫn MTF-01 + INAV6:** đọc `README_POS_HOLD_V2.md`. Đây là bản mở rộng V2, POS vẫn khóa motor mặc định vì chưa có hệ số X/Y đã kiểm định.

# BẢN NÂNG CẤP V2 — ĐỌC TRƯỚC KHI NẠP

**Xem `README_NANG_CAP_ALT_V2.md`** để biết khác biệt của bản này, 3 cấu hình PlatformIO, điều kiện an toàn và giới hạn. Bản mặc định `bench` **không phát xung ESC**.

---

**BẢN SỬA CHUYỂN TIẾP GA ALT — xem `README_ALT_HANDOVER_FIX.md` trước.**

# ESP32 FC 175mm — ANGLE + ALT HOLD + POS HOLD Shadow

**Một dự án PlatformIO, hai cấu hình Build**. Gói này hợp nhất mã V6.3U đang có với đường ESC OneShot125 thử nghiệm, ANGLE, ALT dùng MTF-01P và POS HOLD chỉ tính toán. **Đây KHÔNG phải firmware đã bay thành công, KHÔNG chứng nhận an toàn bay.**

## Tình trạng và sự thật quan trọng

- Firmware **esp-fc** trước đó của người dùng đã bay tay được nhưng **không có nguyên vẹn mã nguồn đang nạp** trong gói này. Các hệ số PID trong nhánh V6.3U vốn để chẩn đoán trên bàn, không tương đương PID đã tune từ esp-fc; không thể khẳng định ANGLE/ALT sẽ bay ổn định chỉ nhờ việc biên dịch.
- GitHub `https://github.com/KyThuatUAV/ESP32_FC` là nguồn tham khảo kiến trúc; code đó dùng **ICM20602, BMP388, SBUS**, không phải cấu hình phần cứng hiện tại **MPU6050, MTF-01P, CRSF**. Chưa sao chép driver/thuật toán firmware GitHub sang bản này.
- Các thử nghiệm ALT trước đây là **ALT shadow**, chưa phải ALT HOLD điều khiển motor thành công. Khẳng định “ALT đã hoàn thành để bay” hiện không được chứng minh.
- **POS HOLD:** `POS_HOLD_ACTUATION_ENABLED=false`, `xy_valid=false`, ma trận FX/FY chưa xác minh nên không có đường để POS tạo lệnh motor. Khi CH6 cao, ESC nhận **ANGLE + ALT** (nếu range hợp lệ); POS được quan sát qua log, **không giữ vị trí**.
- Không có battery ADC thực (cấu hình `ENABLE_BATTERY_ADC=false`), chưa có đồng hồ đo điện áp LiPo tích hợp. Không được coi máy có bảo vệ pin yếu.

## Hai cấu hình trong `platformio.ini`

| Build environment | ESC | Chế độ tính toán | Mục đích |
|---|---|---|---|
| `bench` (mặc định) | **Không phát PWM** | ANGLE mô phỏng, ALT/POS shadow | An toàn hơn để xem code, log |
| `flight_candidate` | **Có OneShot125**, 480Hz | ANGLE + ALT (độ cao qua MTF), POS shadow | **Mã ứng viên chưa được kiểm chứng**; không mặc nhiên dùng để bay |

Trong PlatformIO chọn đúng environment trước khi Build. **Không tự chuyển môi trường sang `flight_candidate` chỉ vì muốn xem log.**

## Dữ liệu phần cứng lấy từ cấu hình đã lưu

- ESC logical motor 1..4: `GPIO12, GPIO13, GPIO14, GPIO27`. **Phải kiểm lại thứ tự motor, chiều quay, hướng phản hồi** trên phần cứng; `GPIO12` là chân strap boot nhạy cảm.
- MPU6050 I²C: SDA18, SCL19. MTF-01P UART1: RX33, TX32, 115200, MSPv2. ELRS/CRSF UART2: RX16, TX17. GND chung.
- Điều khiển: CH1 Roll, CH2 Pitch, CH3 Throttle, CH4 Yaw, CH5 ARM, CH6 mode. CH6 thấp ANGLE, giữa ALT, cao ALT + POS telemetry. **Chỉ được ARM từ CH6 thấp**, CH5 chuyển thấp rồi cao, ga thấp, IMU hợp lệ. Khi CH5 thấp/mất RC/lỗi IMU thì xung về tối thiểu.
- ESC thực tế trước đó chạy `ONESHOT125` tại 480Hz (theo bản dump). Bản này chuyển logical 1000–2000 sang pulse **125–250μs**, không phải PWM analog 1000–2000μs. Cần xác nhận từng ESC hỗ trợ OneShot125.

## Điều khiển ALT trong nhánh thử nghiệm

- MTF đo khoảng cách: bù nghiêng, lấy mẫu mới, giới hạn 0.20–1.80m, lọc độ cao/vận tốc; bỏ dữ liệu lỗi, quá cũ, nghiêng lớn, và tạm ngừng 300ms sau bước nhảy bất hợp lý.
- Bật ALT sau khi đã ARM ANGLE. Giữ **ga tại mức đang có khi vào ALT**, ghi nhận độ cao hiện tại làm target và bù ga trong giới hạn **±70** đơn vị logical ESC. Đây là giới hạn thử nghiệm, **không phải hệ số đã tune**.
- Cần ga quanh điểm đã ghi nhận; lệch trên 300 sẽ chấm dứt ALT đến khi rời chế độ. Mất range / RC / IMU sẽ dừng ALT; RC hoặc IMU mất sẽ DISARM theo trạng thái an toàn. ALT mất range có thể trở về ga tay ngay, gây đổi lực nâng — rủi ro cần đánh giá trước bay.
- Điều khiển Roll/Pitch/Yaw vẫn là PID của nhánh V6.3U chưa tune, **không phải PID esp-fc đang bay được**. Không được dùng hệ số mặc định để kết luận bay ổn định.

## Trước khi có thể cân nhắc bay thực tế

**Không bay chỉ dựa trên Build PASS hoặc host tests.** Tối thiểu phải:

1. Sao lưu binary/dump của firmware **esp-fc** đã bay thành công; giữ khả năng khôi phục. Không tự ghi đè firmware đang bay nếu chưa có bản sao.
2. Không lắp cánh: xác minh chân ESC, giao thức xung bằng thiết bị đo phù hợp, thứ tự motor, chiều quay, mixer tăng giảm đúng khi nghiêng từng trục, fail-safe khi CH5 thấp/mất CRSF/IMU/cảm biến độ cao.
3. Đối chiếu **toàn bộ PID, hướng trục MPU, hướng lắp, mức ga hover** với firmware đã bay thực tế. Nếu khác, phải coi đây là controller mới, chưa thử bay.
4. Phải có phương án ngắt nguồn ESC vật lý; LEDC có thể tiếp tục giữ xung gần nhất khi MCU treo (chưa có watchdog ESC độc lập). Không tin một lời hứa “failsafe phần mềm” khi MCU bị kẹt.
5. Kiểm nghiệm ALT trong môi trường có kiểm soát và người hỗ trợ có kinh nghiệm, tránh người/vật ở gần. Phải biết trước cách trả về ANGLE và DISARM.

**Do các yêu cầu 2–4 chưa được xác nhận bằng phần cứng, bản `flight_candidate` hiện không thể được khuyến cáo để bay.**

## Cấu trúc mã

- `src/control_task.cpp`: đọc MPU6050/RC, ARM, ANGLE PID, ALT throttle mix, xuất ESC khi build `flight_candidate`.
- `include/flight_altitude.h`: bộ đo độ cao + điều khiển ALT riêng, không phụ thuộc motion gate của calibration.
- `src/esc_output.cpp`: driver OneShot125; `flight_candidate` mới attach PWM; fail-close khi disarm.
- `src/mtf_task.cpp`: giao thức MTF-01P MSPv2, range/flow theo packet.
- `src/nav_task.cpp`, `include/flow_velocity.h`, `include/hold_modes.h`: phân tích POS shadow, **không gọi ESC**.
- `tests/`: 15 bài kiểm thử C++ trên máy; gồm test map pulse OneShot125, ALT mất range, giảm ga, pilot override.

## Kiểm thử đã làm ở môi trường tạo mã

- **15/15 bài unit test C++ chạy thành công.** Các test mô phỏng tình huống toán học và không chứng nhận điều khiển bay.
- `control_task.cpp` và `esc_output.cpp` qua kiểm tra cú pháp bằng g++ với Arduino/FreeRTOS mocks ở **cả hai build flag**.
- **Chưa chạy `pio run`, chưa upload lên ESP32, chưa đo xung ESC trên oscilloscope, chưa bay thử.**

Để chạy host tests: `bash tests/run_host_tests.sh`. Để kiểm tra PlatformIO: `pio run -e bench` và `pio run -e flight_candidate`; **Build không phải kiểm tra phần cứng**.

## 2026-11-09 — Kalman Z MPU6050 + MTF-01P (optional)

Xem `README_KALMAN_Z_IMU_LIDAR_FUSION_V2.md`. Build bench đầu tiên là `bench_kalman_z_imu` (ESC OFF). Build `flight_alt_kalman_z_imu_candidate` cho phép Kalman hợp nhất tác động ga **chỉ sau khi xác minh an toàn trên phần cứng**; mặc định `bench` vẫn không quay motor. Các bản build cũ không thay đổi do cờ tính năng mới mặc định OFF.
