# ALT HOLD HANDOVER FIX — bản hoàn chỉnh cho PlatformIO

## Thay đổi giới hạn trong 4 khu vực

1. `include/throttle_handover.h` (mới): ALT -> ga tay đi qua bộ giới hạn thay đổi tập thể **1500 đơn vị ga/s**, tương đương tối đa khoảng **6 đơn vị trong chu kỳ 4 ms**. Chỉ tác động trong quá trình chuyển tiếp (không làm trễ thao tác ga tay ở ANGLE bình thường).
2. `include/flight_altitude.h`: khi **mất range hoặc PILOT OVERRIDE**, ALT bị khóa đến khi **CH6 về ANGLE** và chọn ALT lại. Không tự tái kích hoạt khi range phục hồi, và hạ ga không làm xóa khóa lỗi.
3. `src/control_task.cpp` dùng giá trị ga chuyển tiếp trong đường điều khiển **ANGLE/ALT**. Các điều kiện ARM, RC failsafe, IMU, PWM, PID, motor order và POS HOLD **không thay đổi**.
4. Log bổ sung `[ALT_TRANSFER] phase: ... collective: ... pilot: ... count: ... bypass: ...` để biết giai đoạn chuyển tiếp ga. `count` và `bypass` lưu số lần xảy ra nên không bị mất dấu khi giai đoạn BLEND ngắn hơn chu kỳ in log.

## Ưu tiên an toàn

- **DISARM / mất RC / lỗi IMU / lỗi timing / ga dưới `THROTTLE_RUN_MIN_US`**: ESC về mức dừng **ngay**, không chờ chuyển tiếp.
- **Cần ga lớn >= 1800** khi đang chuyển tiếp: người lái được quyền lấy điều khiển trực tiếp ngay (**FULL_BYPASS**). Trường hợp này có thể tạo bước nhảy ga lớn; ưu tiên thao tác chủ động của người lái thay vì làm trễ lệnh khẩn cấp.
- Khi cảm biến đo cao lỗi: ALT ngừng tác động, bắt đầu chuyển tiếp sang ga tay; **KHÔNG tự quay lại ALT** đến khi CH6 chuyển về ANGLE và quay lại ALT.
- Khi chủ động rời ALT qua CH6, cũng chuyển tiếp về ga tay, trừ khi DISARM hoặc dùng ga tối đa ưu tiên phi công.
- ALT chưa được kiểm chứng bay; `flight_candidate` vẫn là bản thử nghiệm. Không đảm bảo giữ độ cao ổn định trong thực tế. *Không lắp cánh để thử chỉ vì kiểm thử C++ đạt.*

## Log cần quan sát khi THÁO CÁNH

```text
[FLIGHT_ALT] requested:2 actual:1 active:1 reason:1 ...
[ALT_TRANSFER] phase:2 collective:... pilot:...
[FLIGHT_ALT] requested:2 actual:0 active:0 reason:3 ...
[ALT_TRANSFER] phase:3 collective:... pilot:...
[ALT_TRANSFER] phase:1 collective:... pilot:...
```

Các phase: **0 STOP, 1 MANUAL, 2 ALT, 3 BLEND, 4 FULL_BYPASS**.

## Cấu hình

Giữ `default_envs = bench` an toàn mặc định. `flight_candidate` vẫn là bản có OneShot125 cho ANGLE + ALT, **POS HOLD không được nối ESC**. Nếu đã có bản firmware bay ổn định, giữ file sao lưu của bản đó trước khi nạp bản thử nghiệm này. Mạch vẫn chưa có giám sát pin 2S thực tế (`ENABLE_BATTERY_ADC=false`).

## Kiểm thử trên máy

Chạy `bash tests/run_host_tests.sh`. Unit test mô phỏng giới hạn bước nhảy ga, thoát ALT, cảm biến mất / hồi phục, override, tắt output khẩn cấp. Chưa thay thế cho kiểm tra xung ESC trên oscilloscope, thứ tự motor và failsafe khi có lỗi thật.
