# V2 — sửa nhận diện đoạn chuyển động liên tục Optical Flow X

## Vấn đề trong log

Lần đo X gần nhất có 3 cửa sổ chuyển động được nhận, nhưng số 6–7 và 13 cách nhau bằng nhiều cửa sổ gần như đứng yên. Bộ chọn cũ cộng các cửa sổ này vào cùng `selected_` dù không phải một đoạn liên tục, khiến `coh` thấp và có nguy cơ ước lượng tỷ lệ trên một phần hành trình 30 cm.

## Sửa đúng trong V2 (không tạo V3)

- `include/flow_guided_cal.h`: thu tối đa 20 cửa sổ trong vùng GO 5 giây, tách các đợt chuyển động thành các đoạn liên tục theo ngưỡng Flow và góc hướng; chỉ nối qua tối đa một cửa sổ yếu khi đoạn tiếp theo tiếp tục cùng hướng.
- Chọn một đoạn đủ liên tục thay vì cộng các đoạn rời nhau; kiểm tra thêm đảo chiều giữa các đoạn, độ nhất quán cả đoạn, drift baseline, cường độ tối thiểu và độ nhất quán toàn hành trình.
- Mã trạng thái mới `FRAGMENTED_MOTION` khi có hoạt động Flow nhưng không có đủ một đoạn liên tục 3 cửa sổ. Trạng thái này KHÔNG có nghĩa là mất UART hay hỏng IMU.
- `src/nav_task.cpp`: thêm dòng `[FLOW_SEGMENT_V2]` hiển thị số cửa sổ có chuyển động, số đoạn, khoảng số thứ tự cửa sổ tốt nhất, độ nhất quán; giữ các dòng `[FLOW_GUIDED_V6_3H1]` cũ.
- `tests/test_v63g_guided_flow.cpp`: phát lại log X/Y 2026-11-09, thử phân mảnh và đảo chiều, bổ sung kiểm thử an toàn.

## Giới hạn an toàn

- Tất cả vẫn là **BENCH / OBSERVE_ONLY**. `FC_POS_ACTUATION=0` vẫn khóa motor POS; không sửa PID/mixer/ALT HOLD/RC/ESC/Kalman Z.
- Mốc 0.30 m được nhập bằng tay; firmware KHÔNG xác minh độc lập quãng đường 30 cm. `AXIS_MEASURED` là kết quả chẩn đoán, không chứng nhận đúng tỷ lệ khi bay.
- Không hạ tùy tiện ngưỡng để ép cho X đạt. Nếu chuyển động đứt quãng, firmware phải báo `FRAGMENTED_MOTION` thay vì cộng các nhịp lại.

## Triển khai

1. Sao lưu thư mục V2 đang chạy. Nếu Y đã `measured:1` trong RAM: ghi lại `(546.7,-23.3)` từ log; **khi nạp firmware mới, RAM Guided bị xóa**, Y phải được xác minh/đo lại trước khi đủ ứng viên ma trận X/Y. Không reset trên phiên cũ khi chưa cần.
2. Nạp bản V2 này bằng môi trường **bench_kalman_z_imu**, tháo toàn bộ cánh, CH5=1000, CH6=2000, ngắt nguồn ESC.
3. Chỉ thu mẫu khi `NAV ready:1`, `FLOW_DIAG CAL_READY`, MTF-01P nhìn xuống nền có họa tiết ở 35–50 cm, drone gần nằm ngang.
4. `cal guided x 0.30` -> READY -> `cal guided go`. Tiến theo hướng mũi 30cm liền mạch trong 2–3 giây, sau đó giữ yên đến DONE. Không đưa drone lùi lại trong cửa sổ 5s.
5. Xem `DONE` và `[FLOW_SEGMENT_V2]`. Nếu `FRAGMENTED_MOTION`, các cửa sổ thật sự không liên tục, không cố ép đạt. Nếu `AXIS_MEASURED`, vẫn phải xác minh thêm tỷ lệ/dấu/gyro và đo lại Y để hình thành ma trận bench.

## Kiểm thử đã chạy

- `bash tests/run_host_tests.sh`: **22/22 chương trình kiểm thử C++ đều đạt** trên máy chủ.
- **Chưa chạy build PlatformIO ESP32 hoặc kiểm chứng trên thiết bị thật trong môi trường trợ lý**. Không xem các phép thử này là chứng nhận an toàn bay.
