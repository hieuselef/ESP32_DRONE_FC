# PHIÊN BẢN HIỆN TẠI: V6.3 STABLE TARGET & FLOW AXIS CAL — BENCH ONLY

**Hãy đọc [`README_V6_3.md`](README_V6_3.md) trước.** Tài liệu phía dưới là lịch sử từ V6.2. V6.3 khóa PWM motor và POS XY.

---

# TAI LIEU CU (V6.1.x) — Xem README_V6_2.md cho ban moi

# V6.1.2 - SỬA PHỤ THUỘC THƯ VIỆN PLATFORMIO

Bản này sửa `UnknownPackageError: rfetick/MPU6050_light @ 1.2.1`.
Đã dùng `rfetick/MPU6050_light @ ^1.1.0` như các bản chạy được trước đây.
Giữ nguyên khóa motor/ESC và code diagnostic V6.1.1.

Đọc `HUONG_DAN_SUA_LIB_V6_1_2.md` để Build.

---

# V6.1.1 BUILD FIX - Thay the ban V6.1 build loi

Xem `HUONG_DAN_SUA_BUILD_V6_1_1.md`. Ban nay khong xuat PWM cho ESC.

# V6 — ESP32 FC tự phát triển: ALT_HOLD NAV_SHADOW (KHÔNG điều khiển ESC)

**Thay thế V5 hoàn chỉnh** bằng thư mục V6 trong VS Code/PlatformIO. Không chép file V6 lên ESP-FC/LacWing. Đây là firmware tự viết cho phần cứng đã khai báo.

## Phần cứng giữ nguyên

- ESP32 DevKit, MPU6050 I2C SDA18/SCL19, ELRS CRSF RadioMaster T8L UART2 GPIO16/17.
- MTF-01P UART1 RX33/TX32 ở 115200, MSP V2 (`0x1F01`, `0x1F02`).
- 4× motor 1503 2750KV, cánh 3.5 inch, 4× ESC 8A, LiPo 2S 550mAh; nguồn logic qua buck 5V.
- **Không gắn cánh quạt khi kiểm thử. Ngắt nguồn ESC.**

## Thay đổi V5 → V6

1. Parser dùng kiểm tra khoảng cách signed: `-1`, `-3`, `0`, quality=0 **không hợp lệ** ngay cả khi nhận đúng CRC.
2. Tách **rPacketOK** (dải đo sơ bộ 20..12000mm) với **valid** của NAV (chỉ 80..3000mm, độ nghiêng ≤25°, dữ liệu ≤120ms, qua 3 frame warm-up), tránh hiểu nhầm `Dist:20 rValid:1`.
3. Bù nghiêng dựa trên `cos(roll)*cos(pitch)` — chỉ đúng khi trục laser gần trùng trục thẳng xuống; không tự suy ra góc lệch lắp đặt.
4. Lọc h (PT1) và vz (sai phân phép đo liên tiếp + PT1), loại bước nhảy bất thường và **thu nhận lại 3 mẫu ổn định** sau lỗi; dữ liệu cũ không giữ `valid`.
5. `ALT_HOLD` **SHADOW hai tầng**: sai lệch chiều cao -> vận tốc lên/xuống đặt -> hiệu chỉnh *chuẩn hóa* (dimensionless). Có chặn biên, I chống bão hòa, capture target khi vào mode và reset khi mất cảm biến.
6. **Không cần ARM** khi thử: giữ CH5 thấp, đặt CH6 ở giữa (`MODE:1`). `shadow` chuyển CAPTURE → TRACK (log 2 Hz có thể chỉ thấy TRACK vì CAPTURE diễn ra rất nhanh). Khi `MODE:0`, shadow OFF; khi cảm biến sai, shadow 0; `MODE:2` chỉ làm ALT shadow, **không phải POS_HOLD thực**.
7. Không còn thư viện `ESP32Servo` và không có hàm attach/write PWM. Lệnh motor chỉ tính toán nội bộ để chẩn đoán, không được phát ra GPIO.
8. Bộ lọc vận tốc ngang/`xy_valid` vẫn chưa kích hoạt vì chưa hiệu chuẩn đơn vị và hướng Optical Flow.

## Cách thử

1. Giải nén, mở chính thư mục V6 chứa `platformio.ini`, nhấn **Build → Upload → Monitor** ở 115200. **Không dùng script `apply_to_espfc.py`.**
2. Đặt drone tĩnh trên mặt phẳng trước khi bật để `mpu.calcOffsets` thực hiện hiệu chuẩn. Cảm biến hướng xuống mặt bàn có họa tiết, cách 30–50 cm và đủ sáng.
3. Đặt tay RadioMaster T8L: **CH5 ARM OFF**, **CH6 MODE giữa** (MODE:1 trong log). Nếu CH6 không đúng, cần đối chiếu mapping thay vì bật ARM.
4. Xem dòng `[NAV_SHADOW_V6]`:
    - `valid:1 reason:READY`; `range_age < 120ms`; `warm` đủ 3; `accepted` tăng dần.
    - `shadow:CAPTURE` rồi `shadow:TRACK`; `target` được chụp khi vào mode.
    - Hạ cảm biến nhẹ nhàng: `h` giảm, `vz` âm và `u` thường dương; nâng cao: `u` thường âm.
    - `u` là giá trị **giả lập -0.20..0.20, KHÔNG phải PWM hoặc phần trăm ga**.
5. Thử mất/không hợp lệ phép đo bằng cách che sensor hoặc đặt sát bàn 20mm: `valid:0`, reason có thể `RANGE_BAD`, `shadow:...`, `u:0`. Trả lại 30–50 cm để theo dõi reacquire.
6. Dòng `[BENCH] ... PWM:0` phải duy trì. Nếu khác 0, **ngừng kiểm tra**.

Ví dụ log **minh họa** (không phải kết quả đo của bạn):

```
[NAV_SHADOW_V6] valid:1 reason:READY range_age:17ms raw_h:0.500 tilt:1.000 filt_h:0.492 vz:0.012 | mode:1 shadow:TRACK target:0.500 vz_sp:0.008 vel_err:-0.004 i:0.001 u:0.000 | warm:3 accepted:34 rejects:0 reacquire:1 xy_valid:0
```

## Vấn đề chưa làm — quan trọng

- Chưa có bộ điều khiển giữ độ cao thực tế: **không ghép giá trị `u` vào ga ESC**. Muốn thử bay cần kiểm chứng lực nâng/ga hover, cấp điện, dấu điều khiển, lọc nhiễu rung, cảm biến mất dữ liệu, failsafe, và bộ điều khiển bay trong test stand an toàn.
- Không có GPS, chưa có tích phân tọa độ vị trí, gyro-compensation Flow chưa hiệu chuẩn. **POS_HOLD không hoạt động**.
- Không biên dịch được trực tiếp với PlatformIO ở môi trường tạo file vì không có toolchain `pio`; xem bước Build trên máy bạn là kiểm chứng cuối.
- Các hệ số controller hiện chỉ dành cho thử nghiệm trên bàn, KHÔNG phải PID đã tune để bay.

## Kiểm thử độc lập (máy tính có g++)

```bash
for test in tests/test_nav.cpp tests/test_shadow.cpp tests/test_range_validation.cpp tests/test_msp.cpp tests/test_control_math.cpp; do
    g++ -std=c++17 -Wall -Wextra -Werror -pedantic -Iinclude "$test" -o /tmp/test_v6 && /tmp/test_v6
done
```


## V6.1 IMU diagnostics
Doc `HUONG_DAN_IMU_V6_1.md` sau khi phat hien `reason:IMU_BAD`.
