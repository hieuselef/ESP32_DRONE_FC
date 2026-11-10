# ESP32 FC V2 — Kalman Z hợp nhất gia tốc MPU6050 và LiDAR MTF-01P

## Những gì đã thay đổi

Dựa trên ESP32_FC_V2_KALMAN_Z_ALT_MOTOR.zip (V2 + POS guarded), không làm lại FC mới.

Trước đó Kalman Z chỉ ước lượng `z` và `vz` từ LiDAR; gia tốc `az_earth` chỉ được in log. Bản này bổ sung:

1. `include/vertical_accel_fusion.h`: chuyển gia tốc body MPU6050 đơn vị **g** sang linear acceleration **m/s²** theo trục Z thế giới (hướng **lên**), trừ 1g, lọc nhiễu và bù sai lệch zero-g.
2. `include/vertical_estimator.h`: Kalman 2 trạng thái `z` và `vz`, dự đoán theo gia tốc tại mỗi tick ~4 ms, hiệu chỉnh bằng **frame LiDAR mới, đã được RangeTracker kiểm tra**. Không tái sử dụng bản đo cũ làm quan sát mới.
3. `include/alt_kalman_authority.h`: chỉ chấp nhận phản hồi Kalman hợp nhất khi IMU đang cung cấp gia tốc hợp lệ. Khi mất gia tốc/lidar/độ tin cậy sau khi Kalman đã tham gia, latch về phản hồi range; mất range tiếp tục kích hoạt fault ALT của V2.
4. `src/control_task.cpp`: truyền gia tốc đã xử lý vào bộ Kalman và truyền kết quả từ Kalman qua ALT controller, throttle handover và motor mixer trong **build thử nghiệm opt-in**.
5. `[KF_IMU_Z]` log để phân biệt hiệu chuẩn, raw, bias, linear acceleration, số lần dự đoán, tình trạng sử dụng Kalman.

## Vì sao phải hiệu chuẩn trước

`getAccZ()` là gia tốc riêng theo trục **của chip**, không phải gia tốc thẳng đứng của drone. Nếu chỉ dùng `getAccZ()-1` khi drone nghiêng sẽ sinh sai lệch gia tốc giả. Chuyển sang world-Z dựa trên `getAngleX()/getAngleY()` trực tiếp của chip (không nhân thêm dấu stick):

    a_Z_up = 9.80665 * (
       -sin(theta) * ax_g + cos(theta)*sin(phi)*ay_g
       +cos(theta)*cos(phi)*az_g - 1.0)

`phi` = `mpu.getAngleX`, `theta` = `mpu.getAngleY`, đơn vị radian. Quy ước này đòi hỏi chip được lắp mặt thuận (+Z hướng lên) và trục gốc khớp với mã. **Phải kiểm tra log khi nghiêng bằng tay trước khi cho phép quay động cơ.**

Bộ tiền xử lý chỉ đánh dấu `calibrated:1` sau ít nhất 300 mẫu gần như nằm ngang và đứng yên, **khi DISARM**, gyro thấp, độ lớn tổng gia tốc gần 1g. Khoảng 1.2 giây tại 250Hz nhưng phụ thuộc jitter thực tế. Bias không tự cập nhật khi ARM. Nếu kiểm tra gia tốc không qua, không dùng IMU trong Kalman và không dùng Kalman cho ga.

## PlatformIO: lựa chọn đúng môi trường

- `bench` (mặc định): chức năng cũ, xuất motor OFF.
- `bench_kalman_z`: Kalman dùng LiDAR, motor OFF (như bản trước).
- **`bench_kalman_z_imu`**: Kalman **MPU6050 + LiDAR** có kiểm tra source, motor **OFF** — bắt đầu từ đây.
- `flight_alt_kalman_z_candidate`: Kalman dùng LiDAR, có đường motor theo bản trước, IMU fusion OFF.
- **`flight_alt_kalman_z_imu_candidate`**: IMU+LiDAR -> Kalman -> ALT -> mixer -> ESC, chỉ trong build opt-in. **CHƯA ĐƯỢC XÁC NHẬN BAY AN TOÀN, KHÔNG DÙNG BAY NGAY**.

Cấu hình motor mới cần tất cả `FC_FLIGHT_CANDIDATE=1`, `FC_ALT_V2_AUTHORITY=1`, `FC_ALT_KALMAN_ACTUATION=1`, `FC_ALT_KALMAN_IMU_FUSION=1`. POS_HOLD vẫn chưa được phép điều khiển motor nếu chưa hiệu chuẩn Optical Flow X/Y riêng.

## Log Serial 115200

- `[BOOT_ALT_V2] ... KF_IMU_FUSION:1` xác nhận chọn đúng build.
- `[KF_IMU_Z] fusion:1 calibrated:1 valid:1 predicted:1 stable:300 bias:... az_linear:... predictions:...` xác nhận tích hợp gia tốc trong Kalman.
- `[ALT_KF_Z] phase:3 applied:1` và `[FLIGHT_ALT] active:1` (CH6 = ALT, có ARM) mới cho biết **Kalman thực sự được chọn làm phản hồi ga**, không phải chỉ shadow. Trong bench, `FLIGHT_ALT` không chạy điều khiển motor dù log KF vẫn cập nhật.
- `[ALT_KF_Z] phase:4` nghĩa là lỗi đã latch về feedback RangeTracker; thoát ALT/POS qua CH6 để xóa latch theo logic V2.
- `[ALT_KF_Z] phase:5` nghĩa là range thô không còn hợp lệ; **không** được tiếp tục giữ cao độ dựa vào dự đoán IMU. V2 có latch lỗi độ cao và chuyển quyền ga, **không có autoland an toàn**.

## Quy trình xác minh tại bàn (tháo cánh quạt)

1. Giữ bản binary FC đã bay thành công riêng biệt. Chọn env `bench_kalman_z_imu`, **tháo cánh quạt, không cấp nguồn motor**. Đặt trên bàn phẳng, bật nguồn và giữ yên ít nhất 2-3 giây.
2. Kiểm tra `[IMU_POSTCAL] ax~0 ay~0 az~1`, roll/pitch gần 0; `[KF_IMU_Z] calibrated:1`, `az_linear` trung bình gần 0 m/s², log `bias` có giá trị hợp lý. Nếu không đạt, kiểm tra cảm biến, offset, trục gắn.
3. Nghiêng drone ~10–15 độ theo Roll rồi Pitch, giữ yên ở góc đó; `az_linear` phải dần quay về gần 0, không tạo gia tốc ảo rõ rệt. Khớp chiều trục MPU bằng thao tác tay.
4. Nâng drone chậm lên/xuống theo Z khi hướng trục giữ ổn định. Quan sát `az_linear` đổi dấu đúng chiều; `z,vz` thay đổi cùng hướng LiDAR, `frame_used` tăng theo frame thật, không tăng 250 lần/giây.
5. Thử che cảm biến LiDAR hoặc tạo khung đo cũ >140 ms: ALT không dùng dự đoán IMU để tiếp tục điều khiển. Thử mất IMU hoặc sample sai -> gate KF về raw và latch.
6. Chỉ sau khi đo bằng thiết bị chuyên dụng và kiểm tra hướng mixer/ESC/failsafe/điện áp pin và PID thực tế, mới cân nhắc thử nghiệm `flight_alt_kalman_z_imu_candidate` ở giá thử chắc chắn với **4 cánh tháo rời**. Không thử bay ngay.

## Giới hạn giữ nguyên V2

Vòng điều khiển 4 ms danh định (250Hz). ALT ±70 µs; collective 1100–1850 µs; tốc độ mục tiêu ±0.30 m/s; khoảng đặt cao 0.25–1.70 m; dữ liệu range quá 140 ms không hợp lệ.

Chưa có dữ liệu thực tế về độ rung, body->earth alignment trên quad 175 mm, vị trí lắp LiDAR so với tâm quay, offset gia tốc khi motor chạy, hay định chuẩn phương sai Kalman/PID. `PROCESS_SIGMA_ACCEL_MPS2`, giới hạn acceleration 4.5 m/s², 18 Hz LPF và ngưỡng trong Gate chỉ là thông số khởi tạo để nghiên cứu, **không chứng minh máy bay an toàn**. Máy chỉ có MPU6050 (không có từ kế/baro), nên yaw và quan trắc Z có thể trôi; nhờ LiDAR mà Z có neo độ cao nhưng không đảm bảo giữ vị trí ngang.

## Test code

    bash tests/run_host_tests.sh

Kiểm thử host-only bằng g++ và kiểm tra cú pháp `src/control_task.cpp` với Arduino mocks; chưa thể biên dịch PlatformIO đầy đủ hay đo motor/IMU/MTF thật trong môi trường này.
