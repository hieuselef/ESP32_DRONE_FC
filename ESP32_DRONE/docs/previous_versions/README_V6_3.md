# ESP32 FC tự phát triển — V6.3 Stable Target & Optical Flow Axis Calibration

**CHỈ THỬ TRÊN BÀN, THÁO CÁNH QUẠT, KHÔNG CẤP NGUỒN ESC.** Bản này không phải flight controller đã được kiểm định. PWM motor được **khóa cứng = 0** bằng `ENABLE_MOTOR_OUTPUT=false`, `static_assert`, và không có lệnh `ledcWrite`/`servo.write`/gửi ESC. Các số `M:1400,...` là phép tính mixer giả lập, KHÔNG đi ra ESC.

## Phần cứng (không đổi so với V6.2)

- ESP32 DevKit, MPU6050 SDA 18 / SCL 19.
- RadioMaster T8L ELRS/CRSF RX 16 / TX 17. CH5 = ARM, CH6 = MODE.
- MTF-01P MSP V2 `$X<` UART RX 33 / TX 32 (không đổi sang Micolink).
- Chân motor giả lập: 12 (sau phải CCW), 13 (trước phải CW), 14 (sau trái CW), 27 (trước trái CCW).
- Khung X 175 mm, 4x 1503 2750KV, cánh 3.5", ESC 8A 2–3S, pin 2S 550mAh, logic 5V buck. Không giả định đo điện áp pin từ 5V buck.
- `platformio.ini`: `rfetick/MPU6050_light @ ^1.1.0` và AlfredoCRSF từ GitHub, giữ cấu hình chạy được trên V6.1.2/V6.2.

## Tính năng mới 1: bảo vệ việc bắt độ cao mục tiêu

V6.2 bắt lại `target` ngay khi NavEstimator chuyển `READY` sau ba mẫu. V6.3 thêm trạng thái **`RECOVERY_PENDING`**. Trong `WAIT_HEIGHT`, `SENSOR_LOSS` và `RECOVERY_PENDING`, các giá trị `u`, `i` và `vz_sp` đều bằng 0. Khi phát hiện một lần mất sensor, mục tiêu cũ chỉ được giữ trong log để tham khảo; **không được dùng để điều khiển**.

Giới hạn thử nghiệm mặc định:

| Điều kiện | Giá trị V6.3 (BENCH) |
| --- | --- |
| Dải độ cao **được bắt mục tiêu mới** | 0.30 – 1.80 m |
| Dải độ cao **duy trì TRACK** (hysteresis) | 0.25 – 2.00 m |
| Cửa sổ yên tĩnh liên tục | >= 800 ms |
| Số mẫu khoảng cách mới đã được NavEstimator chấp nhận | >= 20 |
| Vận tốc dọc Z trong cửa sổ | abs(vz) <= 0.08 m/s |
| Biên độ thay đổi độ cao lọc trong cửa sổ | <= 0.035 m |
| Tốc độ góc tổng hợp | <= 20 độ/s |
| Gia tốc tổng hợp | abs(norm - 1g) <= 0.10g |
| Tư thế gần nằm ngang | tilt_compensation >= cos(12°) |
| Mất khung đo mới quá lâu | > 120 ms -> đặt lại cửa sổ |

Những ngưỡng trên chỉ dùng để kiểm chứng phép tính trên bàn; chưa phải ngưỡng an toàn cho chuyến bay. Khi NavEstimator báo `READY` ở 9 cm, bộ điều khiển V6.3 vẫn **không cho phép bắt mục tiêu**. Sau mất dữ liệu, chỉ khi nhiều mẫu mới liên tiếp ổn định thì mới `CAPTURE`, tiếp đến `TRACK`.

Log:

- `[NAV_SHADOW_V6_3]` giữ các số `valid`, `reason`, `target`, `vz_sp`, `i`, `u`, trạng thái `shadow`.
- `[TARGET_GUARD_V6_3]` in `capture_reason:MODE_ENTRY | MODE_SWITCH | SENSOR_RECOVERY`, `capture_count`, `gate`, `stable_ms`, `stable_samples`, `last_capture_ms`, `last_capture_samples`, `rejected`.
- Cùng dòng log có `last_outlier_h`, `jump`, `outlier_count` để xem giá trị đo bị từ chối (kể cả khi log mỗi 500 ms không đúng lúc xuất hiện OUTLIER).

Kiểm tra: đặt MTF cách mặt bàn 0.50 m, CH5:1000, CH6:1500 (MODE1), giữ yên ít nhất 2–3 giây. Kỳ vọng `RECOVERY_PENDING -> CAPTURE -> TRACK`, `capture_count:1`, `last_capture_ms >= 800`. Hạ xuống khoảng 0.40 m, nâng lên 0.60 m để kiểm tra dấu `u`; **không cấp nguồn ESC**. Tạo mất dữ liệu rồi đưa lại 0.50 m, chờ 800 ms ổn định mới cho phép bắt `target` mới.

## Tính năng mới 2: hiệu chuẩn sơ bộ hướng trục Optical Flow X/Y

Không có đủ tài liệu đơn vị, thời gian tích phân và biến đổi trục MTF-01P để suy ra vận tốc m/s chính xác. Vì vậy **V6.3 chỉ thu thập các tổng số đếm FX/FY từ các mẫu 50Hz khi người dùng dịch chuyển drone bằng tay theo khoảng cách đo thực tế**. Tỷ số `counts/m` và định thức ma trận hai trục chỉ là giá trị *ứng viên* để đánh giá hướng trục; không được sử dụng cho điều khiển, định vị hay bay.

Điều kiện: drone tháo cánh quạt, ARM OFF (`CH5≈1000`), CH6≈2000 (`MODE:2`), `PWM:0`, mặt bàn có hoa văn, cảm biến nhìn xuống, độ cao khoảng **0.50 m** và giữ gần như ngang khi dịch chuyển, đủ sáng. `FLOW_DIAG_V6_3 reason:CAL_READY` là điều kiện đầu tiên; firmware vẫn **cưỡng bức `xy_valid:0`**.

Các lệnh nhập ở **Serial Monitor 115200 baud**, chọn `Newline` làm kết thúc dòng:

```
cal reset
cal x 0.30
cal stop
cal y 0.30
cal stop
cal status
```

Quy trình:

1. Xác định trục thân drone: **X+ = hướng mũi drone**, **Y+ = hướng bên phải drone** (đây là quy ước cho phép thử, không tự khẳng định trục MTF bên trong trùng với trục drone).
2. Giữ cao ~50 cm, ở MODE2, gõ `cal reset`, sau đó `cal x 0.30`. Dịch chuyển **chỉ một chiều** 30 cm theo X+ so với mặt bàn, giữ độ cao và hướng mũi cố định. **Không đưa drone trở lại vị trí ban đầu trong phiên thu này** vì hai chiều có thể triệt tiêu số đếm.
3. Khi kết thúc chuyển động, gõ `cal stop`; firmware lưu vector ứng viên cho X.
4. Đưa drone về vị trí thuận lợi **sau khi đã stop**, vẫn ngang cách bàn ~50 cm. Gõ `cal y 0.30`, dịch chuyển một chiều 30 cm theo Y+, sau đó `cal stop`.
5. Gửi log `[FLOW_CAL_V6_3]` và `[FLOW_CAL_CMD]` gồm hai cột `X:(fx,fy)`, `Y:(fx,fy)`, `det`, `candidate`. Nếu `reason:LOW_SIGNAL`, `NEED_SAMPLES`, `COLLINEAR`, hoặc `ROTATION`, làm lại chậm hơn và thẳng hướng hơn; không hạ ngưỡng để ép đạt.

Mỗi mẫu mới chỉ đếm **một lần theo `last_flow_ms`**; dữ liệu có góc quay >15°/s, độ cao ngoài 0.30–1.20 m, frame kém chất lượng, hoặc FX/FY nhảy vượt 200 được từ chối. Cần ít nhất 12 khung được nhận và vector tổng có biên độ tối thiểu 8. Hai vector X/Y không được gần song song. Các giới hạn này là heuristic phục vụ chẩn đoán, chưa hiệu chuẩn động học. **Dữ liệu MTF được đọc ở 50Hz nên có thể bỏ qua các khung xen giữa: các tỷ lệ counts/m không được coi là hiệu chuẩn m/s chuẩn.**

`FLOW_CAL_V6_3 candidate:1` chỉ có nghĩa cả hai hướng đã có vector không song song. `xy_valid` vẫn 0 trong mọi trường hợp và POS_HOLD ngang chưa tồn tại.

## Kiểm chứng & hạn chế

- Mã thuật toán đã có 10 test host C++11, có replay chuyển động, outlier, mất RC, low-height, chống capture khung trùng và flow-cal định hướng.
- Các file `src/*.cpp` đã vượt qua syntax check trên host với **mock headers** của Arduino/MPU6050/CRSF. Đây **không thay thế** Build/Upload ESP32 bằng PlatformIO và chưa được kiểm tra với phần cứng thực.
- Chưa xác định được chiều chính xác IMU so với frame, protocol ESC thực (ESP-FC cũ dùng ONESHOT125), phép đảo chiều motor, PID đã tune, battery ADC, fail-safe bay thực tế. V6.3 không sửa các phần này; **không lắp cánh quạt hay thử bay**.
