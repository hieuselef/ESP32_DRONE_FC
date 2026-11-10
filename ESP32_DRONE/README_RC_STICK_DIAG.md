# Patch CH1-CH4 & setpoint (read only) — RC_STICK_DIAG

Phiên bản này là bản sao của `ESP32_FC_175MM_ANGLE_ALT_POS_SHADOW_CANDIDATE`, chỉ bổ sung:
- `src/main.cpp`: log `[RC_STICKS]` CH1/CH2/CH3/CH4, `previewRoll`, `previewPitch`, `previewYawDps` ngay khi CH5=1000 (DISARM).
- `include/rc_stick_diag.h`: hàm chuyển xung sang góc **chỉ dành cho hiển thị**.
- `tests/test_rc_stick_diag.cpp`: thử nghiệm hàm chẩn đoán.

**Không sửa**: `src/control_task.cpp`, `src/rc_task.cpp`, `include/config.h`, `src/esc_output.cpp`, PID, ARM, failsafe, ALT HOLD, POS HOLD và `platformio.ini`.

**Lưu ý:** `previewRoll`/`previewPitch` là lệnh mong muốn đọc từ cần điều khiển, **không phải** setpoint PID đang hoạt động. Khi DISARM, `pidTargets:(0,0)` và `PWM:0` là đúng thiết kế.

## Cách nạp và kiểm tra

1. Mở thư mục chứa trực tiếp `platformio.ini` trong VS Code.
2. Tháo tất cả cánh quạt. Không cần cấp nguồn ESC để kiểm tra CH1-CH4.
3. Chọn `Project Tasks > bench > Upload` (mặc định), hoặc `pio run -e bench -t upload`.
4. Bật T8L, CH5=1000 và CH6=1000. Mở Serial Monitor 115200.
5. Không ARM. Chỉ di chuyển cần bên phải theo từng trục, xem `[RC_STICKS]`.

Khi gạt cần bên phải trái/phải, CH1_R hoặc CH2_P (tùy cấu hình radio) sẽ thay đổi khoảng 1000–2000; khi gạt lên/xuống, kênh còn lại thay đổi. Nếu cả hai không đổi, kiểm tra Channel Monitor/Mixes/Output của T8L, cấu hình ELRS receiver và mapping. Nếu thay đổi nhưng đảo trục, không vội đổi dấu trong firmware trước khi đối chiếu hướng lắp IMU.

Đọc log:
- `fresh:1`: gói CRSF mới hợp lệ; `fresh:0`: dữ liệu không dùng được (các CH hiển thị 0, preview = 0).
- `CH1_R`, `CH2_P`: giá trị nguyên từ CRSF, **không** phụ thuộc ARM.
- `previewRoll/Pitch`: góc tương ứng giả định mapping CH1=Roll, CH2=Pitch và deadband 20us, ±20°.
- `pidTargets`: setpoint **đang thật sự áp dụng** (bằng 0 khi DISARM).
- `PWM`: phản ánh motor enable như firmware gốc; bản BENCH luôn 0.

Không dùng `flight_candidate` để bay chỉ vì thấy cần bên phải có giá trị: giao thức ESC, chiều motor, IMU, PID, fail-safe, đo pin vẫn chưa được xác minh bay thực tế. `flight_candidate` nguyên trạng như phiên bản trước; chỉ thêm log.
