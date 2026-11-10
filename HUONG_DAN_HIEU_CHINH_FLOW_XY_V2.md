# V2 — sửa bộ thu mẫu Guided Optical Flow, giữ nguyên điều khiển motor

## Thay đổi

- Chỉ `include/flow_guided_cal.h`: chấp nhận các cửa sổ chuyển động X/Y biên độ vừa, trong khi vẫn giữ kiểm tra chiều và bổ sung điều kiện tổng hành trình mạnh hơn: tối thiểu 60 counts cho 0,30 m, >2,5 lần trôi baseline, raw coherence >= 0,30, selected coherence >= 0,55, ít nhất 3 cửa sổ, tối thiểu 60 khung, không đảo chiều.
- `tests/test_v63g_guided_flow.cpp`: thêm phép phát lại một đường di chuyển có nhiễu (từ log người dùng) và kiểm thử hồi quy của các tình huống nhiễu/đảo chiều/mất dữ liệu trước đây.
- Không thay đổi `vertical_accel_fusion.h`, PID, mixer, ALT HOLD, RC, ESC. **POS actuator vẫn bị khóa `FC_POS_ACTUATION=0`**.
- Đây chỉ là thay đổi để đo và tạo *ứng viên* hiệu chuẩn X/Y. Ngay cả khi `candidate:1`, KHÔNG mở đầu ra POS motor cho đến khi đã xác minh riêng hai chiều X/Y, hiệu chuẩn chuyển động quay và chiều phản hồi.

## Vấn đề IMU trong log

Một phiên ghi khác cho thấy `[KF_IMU_Z] calibrated:0 valid:0 predicted:0` và `gz` khoảng 3 độ/giây khi drone gần đứng yên. Bộ lọc V2 yêu cầu vận tốc góc tổng hợp < 3 độ/giây trong 300 mẫu liên tiếp; vì thế điều kiện hiệu chuẩn không đạt. Đây là vấn đề độc lập với khâu phân tích cửa sổ Flow. Đừng tăng ngưỡng gyro để ép PASS. Thử kiểm tra cấu hình cảm biến, hiệu chuẩn gyro MPU6050 trên mặt phẳng chắc, chờ đứng yên và kiểm tra `gz`; nếu vẫn lệch quanh 3 độ/giây, kiểm tra bias gyro/calibration trong MPU6050_light trước khi làm tiếp.

## Chạy kiểm tra

1. Backup thư mục V2 đang dùng. Giải nén bản ZIP này; chạy `pio run -e bench_kalman_z_imu`. Không gắn cánh, DISARM CH5=1000, không cấp nguồn ESC.
2. Đặt MTF-01P hướng xuống nền nhiều hoa văn tại 35-50 cm, đầu drone hướng về màn hình. Chuyển CH6=2000.
3. Chỉ tiến hành khi `[KF_IMU_Z] calibrated:1 valid:1`; chưa đạt thì dừng, kiểm tra gyro.
4. `cal guided x 0.30` -> READY -> `cal guided go` -> đi thẳng 30cm trong khoảng 2-3s.
5. `cal guided y 0.30` -> READY -> `cal guided go` -> đi sang phải thân drone 30cm trong khoảng 2-3s.
6. Gửi `cal guided status` và toàn bộ `WINDOW/DONE` để xác minh.

**Chưa chạy build PlatformIO trên ESP32 thật ở môi trường của trợ lý.** Chỉ đã biên dịch và thực thi các bài kiểm thử C++ trên máy chủ. Không coi đây là xác nhận an toàn bay.
