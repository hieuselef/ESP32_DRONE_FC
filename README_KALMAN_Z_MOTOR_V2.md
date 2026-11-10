# ESP32 FC V2 — Kalman Z thành nguồn phản hồi ALT HOLD (thử nghiệm)

## Trạng thái

Đây là **một nâng cấp vào chính mã nguồn V2 + POS HOLD guarded**. Chưa chạy firmware trên ESP32 hoặc đo xung ESC thực tế, nên **KHÔNG được xem là đã thử bay thành công**.

Ở mã V2 trước, `vertical_estimator::Kalman2` chỉ ghi log, còn `flight_alt::AltController` dùng height/vz của `RangeTracker`. Bản này cho phép chọn Kalman Z làm phản hồi điều khiển độ cao sau khi có 8 frame range mới được nhận và chuyển tiếp mượt ~600 ms.

**Đường điều khiển thật** (chỉ ở build opt-in):

`MTF-01P MSPv2 -> RangeTracker kiểm tra mẫu thô + bù nghiêng -> Kalman2 (z,vz) -> alt_kalman_authority::Gate -> AltController (giới hạn ±70 us) -> throttle_handover -> mixer X -> ESC OneShot125`.

**Quan trọng:** Kalman Z chỉ dùng rangefinder MTF-01P, **chưa fusion gia tốc Z MPU6050** (gia tốc dùng log). Vì lệch trục/bias/rung chưa được xác minh, không được gọi đây là IMU+range fused Kalman.

## Môi trường PlatformIO

| env | KF Z | ESC vật lý | Mục đích |
|---|---|---|---|
| `bench` (mặc định) | Không tác động ALT | **OFF** | So sánh với firmware V2 cũ |
| `bench_kalman_z` | Bật selector để ghi log | **OFF** | Kiểm tra thuật toán với cánh tháo rời |
| `flight_alt_kalman_z_candidate` | **ON**: feedback từ KF có thể tạo correction ga | **ON** | Thử nghiệm phần cứng được giám sát; chưa đủ cơ sở bay |

Các env có sẵn `flight_candidate`, `flight_alt_v2_candidate` vẫn được giữ nguyên. Ở các env này `FC_ALT_KALMAN_ACTUATION=0` nên đường ga ALT cũ không đổi. POS HOLD vẫn khóa điều khiển động cơ (`FC_POS_ACTUATION=0`).

Cấu hình này KHÔNG tự nạp firmware. Phải chủ động chọn environment trong PlatformIO. Không bật `flight_alt_kalman_z_candidate` chỉ để xem log vì env này có phát xung ESC.

## Mức giới hạn (giữ đúng từ V2)

- Control task: 4 ms danh định (~250 Hz; cần đo jitter ESP32).
- ALT correction: ±70 microseconds logical; ALT collective: 1100..1850 logical.
- Tốc độ leo/xuống lệnh: ±0.30 m/s; giới hạn target 0.25..1.70m.
- Ngưỡng mất range: >140 ms, không dùng dự đoán Kalman để tiếp tục giữ ALT.
- `Kalman2` chỉ cập nhật khi có frame mới đã hợp lệ. Sigma/độ chênh của KF so với range phải nằm trong cổng riêng; các ngưỡng này là sơ bộ, **chưa tune bằng flight logs**.

## Gate KF / trạng thái logger `[ALT_KF_Z]`

- `phase:0 RAW_ONLY`: build chưa bật KF, phản hồi ALT dùng RangeTracker nguyên trạng.
- `phase:1 KF_WARMUP`: chờ ít nhất 8 frame mới được KF chấp nhận, nguồn ga còn là RangeTracker.
- `phase:2 KF_BLEND`: feedback đang nội suy dần từ RangeTracker sang KF, thời gian khoảng 600 ms.
- `phase:3 KF_LIVE`: độ cao/vận tốc KF đang làm phản hồi cho ALT (nếu `applied:1` và `FLIGHT_ALT active:1`).
- `phase:4 RAW_FALLBACK_LATCHED`: KF mất độ tin cậy; về RangeTracker và **không tự bật lại KF** đến khi phi công rời ALT/POS trên CH6.
- `phase:5 RANGE_INVALID`: range thô không hợp lệ; ALT controller theo logic V2 khóa lỗi, chuyển tiếp về ga tay nếu còn RC, **không tự hạ cánh an toàn**.

`enabled:1` chỉ có nghĩa bộ chọn KF được bật khi build. `applied:1` đồng thời với `phase:2` hoặc `phase:3` cho thấy KF đã góp phần tính ga; kiểm tra thêm `[FLIGHT_ALT]` và `[ALT_TRANSFER]` để biết ALT và xuất xung ESC có đang hoạt động không. Kalman sigma và innovation là chẩn đoán, không phải chứng nhận chất lượng đo.

## Những điều phải kiểm tra TRƯỚC KHI thử quay motor

1. Sao lưu binary của bản esp-fc thực tế đã bay được. Phiên bản V2 này có PID không được chứng minh giống bản đó.
2. **Tháo toàn bộ 4 cánh quạt**, ngắt nguồn ESC khi nạp/cắm nối USB, kiểm tra mass chung, giới hạn nguồn, kiểm tra motor output và kill switch CH5.
3. Chạy build `bench_kalman_z`, đổi CH6 ANGLE -> ALT, di chuyển drone bằng tay trong vùng range hợp lệ. Ở trạng thái ổn định, logger có thể tiến đến `phase:3` nhưng **motor_output vẫn OFF**.
4. Với cánh tháo rời và có thiết bị đo xung, mới kiểm tra trên `flight_alt_kalman_z_candidate`: thử lỗi mất MTF (ALT fault), mất ELRS/IMU (hard stop), rời ALT về ANGLE, ga cao chủ đích (override), ga thấp trong ALT, tình huống Kalman không tin cậy (fallback). Chú ý: nếu LiPo đo từ buck 5 V thì **không phải điện áp pin thật**, ADC pin chưa được tích hợp.
5. Chỉ đánh giá khả năng bay sau khi đã xác minh mapping motor, chiều quay, hướng IMU và bộ PID góc/tốc độ, giới hạn động cơ 8A, thời gian ổn định và phản ứng failover trên phần cứng. **Không dùng test phần mềm làm cơ sở bay ngay**.

## Test phần mềm

Chạy `bash tests/run_host_tests.sh` tại gốc dự án. Test mới `tests/test_alt_kalman_motor_path.cpp` diễn tập 250Hz vòng ALT có dữ liệu range 50Hz, chứng minh lệnh ga lấy từ feedback đã chọn và bị chặn theo giới hạn; kiểm tra KF mất độ tin cậy, range lỗi, latch và không tự tái kích hoạt. Đây là phép mô phỏng host-only, chưa xác nhận PlatformIO, đo xung ESC hay kiểm định bay.
