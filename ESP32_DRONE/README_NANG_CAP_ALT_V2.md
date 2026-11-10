# ESP32 FC 175 mm — ALT SAFETY / KALMAN SHADOW V2

**Nguồn gốc:** Copy nguyên dự án `ESP32_FC_175MM_FLIGHT_CANDIDATE_ALT_HANDOVER_FIX.zip` đã được người dùng nạp kiểm tra. Không thay chân GPIO, protocol MSP V2, CRSF, IMU, motor mixer/driver, PID ANGLE hoặc POS Shadow.

**Trạng thái:** *BẢN THỬ NGHIỆM V2, CHƯA XÁC NHẬN AN TOÀN BAY THỰC TẾ.* 19/19 host tests chạy thành công; kiểm tra cú pháp control task với Arduino/MPU mô phỏng. **Không có PlatformIO ESP32 toolchain và bộ ESC thật trong môi trường này nên chưa thể khẳng định build và bay được.**

## Các environment

| Cấu hình | Có PWM ra ESC? | Low-stick trong ALT | Vai trò |
|---|---|---|---|
| `bench` (mặc định) | **Không** | Thuật toán V2 được chọn, nhưng ALT không điều khiển motor | Kiểm tra IMU, RC, range, Kalman Shadow |
| `flight_candidate` | Có | Logic cũ (ga <1100 dừng motor) | Đối chiếu với bản đã dùng, **không phải firmware bay đã được chứng thực** |
| `flight_alt_v2_candidate` | Có | **Logic mới**: khi ALT đang ACTIVE, ga thấp là yêu cầu hạ độ cao | Chỉ kiểm tra mặt đất **THÁO CÁNH**; chưa được bay |

Build/nạp bằng PlatformIO: chọn đúng environment trong Project Tasks. Ví dụ dòng lệnh:

```sh
pio run -e bench
pio run -e bench -t upload
# KHONG nạp flight_alt_v2_candidate cho drone có cánh trước khi kiểm chứng!
```

## Các thay đổi có ảnh hưởng tới logic điều khiển

### 1. `include/alt_motor_gate.h` — quyền điều khiển ga ALT V2

- Nguyên tắc không thay đổi: DISARM, mất RC, lỗi IMU, lỗi timing hoặc ESC chưa sẵn sàng -> motor STOP ngay.
- ANGLE và ALT **chưa được kích hoạt**: khi CH3 dưới 1100, motor STOP như bản cũ.
- Sau khi ALT đã thật sự `active:1`: nếu CH3 hạ dưới 1100, **không tự dừng**; ALT tiếp tục điều khiển độ cao, coi ga thấp là lệnh hạ xuống.
- Khi thoát ALT hoặc mất range, `throttle_handover` tiếp quản để chuyển về ga tay với giới hạn bước điều chỉnh.
- Khi quá trình chuyển về ga tay hoàn tất và cần ga vẫn dưới 1100, hệ thống có thể quay lại mức motor STOP. **Đây vẫn là giới hạn an toàn chưa được chứng minh trong bay thật; cần xử lý bằng bài kiểm tra mất sensor**.

### 2. `flight_altitude.h` — giữ quyền hạ độ cao trong ALT

- Với V2, cần ga thấp hơn mức ga bắt đầu ALT quá 300 us **không còn bị hiểu nhầm là override**; nó là lệnh hạ.
- Cần ga cao hơn mốc ALT quá 300 us vẫn là Pilot Override và khóa ALT; CH6 về ANGLE hoặc CH5 DISARM luôn có tác dụng.
- Mất range -> latch reason 2, chỉ vào ALT lại sau khi thoát chế độ và chọn lại. Không tự kích hoạt lại.
- Trong cấu hình `flight_candidate` cũ giữ nguyên semantics để đối chiếu.

### 3. `throttle_handover.h` — đo bước chuyển ga

- Giữ cơ chế chuyển tiếp có giới hạn 1500 logical us/s trong các trường hợp chuyển mềm thông thường.
- Thêm `last_step_us`, `max_blend_step_us` để ghi nhận, thay vì chỉ nhìn log 2Hz có thể bỏ qua thời điểm chuyển tiếp.
- Kênh ga cao cố ý >=1800 trong lúc chuyển vẫn có quyền bypass; DISARM không bị làm chậm.

## Nâng cấp estimator (chưa tác động motor)

`include/vertical_estimator.h`: bộ lọc Kalman trạng thái `height [m]`, `vz [m/s]`.

- Predict tại nhịp control task dựa trên `dt` thực (từ `millis`, 4ms danh định).
- **Chỉ correct mỗi frame range MTF mới và hợp lệ**. Không dùng một phép đo lặp nhiều lần như các phép đo độc lập.
- Joseph covariance update; innovation gate; timeout và reset/reacquire.
- `sigma_range = 0.035m`, `sigma_accel = 1.5m/s²` là giả thiết khởi tạo để kiểm tra **không phải tham số đã hiệu chuẩn**.
- Gia tốc Z sau bù nghiêng được hiển thị để kiểm tra dấu trục, nhưng **KHÔNG đưa vào Kalman / PID**. Chỉ khi đo bias và rung trên cấu hình thật mới xem xét.
- **Không chuyển tín hiệu estimator mới vào ALT controller**. Chương 4 khuyên hiệu chỉnh bộ lọc và dữ liệu trước khi kết nối điều khiển.

## Đọc log mới

```
[BOOT_ALT_V2] low_stick_alt_authority:1 KF:SHADOW_ONLY hardware_build_unverified:1
[ALT_SAFETY_V2] enabled:1 phase:3 low_stick_authority:1 step:+0 max_blend_step:6 ...
[ALT_KF_SHADOW] healthy:1 fresh_frame:0 z:0.780 vz:+0.002 sigma_z:... age:... updates:... outliers:... az_earth(LOG_ONLY):... MOTOR_FEEDBACK:0
```

- `phase`: `0=HARD_STOP`, `1=MANUAL`, `2=ALT_AUTHORITY`, `3=ALT_LOW_STICK`, `4=MANUAL_HANDOVER`.
- `fresh_frame` ở dòng 2Hz thường là 0 vì control task chạy 250Hz. **Xem `updates` có tăng**, không suy luận chỉ từ cờ một thời điểm.
- `max_blend_step_us` chỉ đo bước lúc BLEND; không phản ánh cú bypass ga cao hay xung tắt khẩn cấp.
- `MOTOR_FEEDBACK:0` nghĩa là Kalman mới **chưa** kết nối đến motor.
- Các mã `reason:2` (range fault) và `reason:3` (pilot override) vẫn giữ nguyên.

## Checklist kiểm tra tối thiểu (không cánh quạt)

1. `bench`: `RC_CHECK`, `IMU_DIAG`, `NAV_SHADOW` và `ALT_KF_SHADOW` báo range hợp lệ, không có outlier tăng liên tục khi đứng yên; `PWM:0`.
2. Đảm bảo chân ESC, đúng thứ tự motor, chiều tự cân bằng ANGLE được kiểm chứng trên chính bản build này, không chỉ dựa vào kết quả bay firmware khác.
3. `flight_alt_v2_candidate`, tháo cánh: khởi động từ ANGLE với CH3 thấp, ARM; ga >1100 và chuyển ALT CH6. Xác nhận `FLIGHT_ALT active:1` trước khi thử logic ga thấp.
4. Khi ALT ACTIVE, giảm CH3 xuống 990: kỳ vọng `[ALT_SAFETY_V2] phase:3`, `PWM:1`, không có `FLIGHT_ALT reason:3`; CH5 DISARM luôn `PWM:0`.
5. Chuyển CH6 về ANGLE hoặc làm mất range trong lúc ALT ACTIVE: log `ALT_TRANSFER phase:3` và `max_blend_step_us` không vượt mức có chủ đích; ALT latch phải ngăn tự khởi động lại. **Không thử làm mất RC khi có cánh.**
6. Test RC failsafe/ESC với props off, giám sát xem library AlfredoCRSF có thực sự báo `isLinkUp=false` khi tắt máy phát; một số receiver có thể giữ kênh cũ. Chưa có bảo đảm từ phần mềm khi receiver vẫn phát dữ liệu như thường.
7. Trước bay: cần đo điện áp pin 2S bằng đường chia áp đã hiệu chuẩn (hiện `ENABLE_BATTERY_ADC=false`), đo lực đẩy/mức ga treo, xác thực fallback mất sensor/mất RC trên phần cứng. **Bản này chưa đủ điều kiện để bay chỉ dựa vào các host test**.

## Những việc CHƯA hoàn thành (không được ngầm coi là đã xong)

- Chưa thay PID ALT bằng estimator Kalman mới; chưa tinh chỉnh PID theo drone thật.
- Chưa hiệu chuẩn gia tốc Z, độ rung, hover throttle hay lực nâng theo mức pin.
- Chưa thay cơ chế failsafe ELRS bằng trạng thái link quality / failsafe của receiver ở cấp protocol; chưa có chính sách hạ cánh tự động an toàn khi mất sóng.
- Chưa thay mixer bằng thuật toán ưu tiên mô-men; vẫn dùng mixer ANGLE mà bạn đã kiểm tra.
- Chưa có phát hiện tiếp đất/không phận; không chứng minh được điều kiện ngừng motor trên không an toàn trong mọi hoàn cảnh.
- Chưa có full PlatformIO/Arduino-ESP32 real-library build và **chưa được bay thử**.

## Thử nghiệm tự động

```
bash tests/run_host_tests.sh
```

Toàn bộ mã cũ đã giữ lại, chỉ cập nhật module ALT, bổ sung estimator và logging. Lịch sử bản cũ ở `docs/previous_versions/`.
