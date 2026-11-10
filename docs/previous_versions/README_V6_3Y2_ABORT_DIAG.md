# V6.3Y2 — sửa công cụ quan sát Y bị ABORT sớm (BENCH ONLY)

Tạo trực tiếp từ bản V6.3Y; giữ nguyên V6.3 calibration X/Y, PID, mixer và khóa ESC.

## Vì sao cần sửa?
Log 2026-11-08 có `START phase:STILL`, tiếp theo `ABORT` trước `MOVE_NOW`. Đây không phải kết quả hiệu chuẩn Y: phép đo chưa hoàn tất. Ở V6.3Y, một chu kỳ `nav.height_valid=false` khiến probe bị hủy ngay. Điều này có thể xảy ra vì bộ ước lượng độ cao kiểm tra gói range chặt hơn yêu cầu quan sát FX/FY trên bàn. Cũng có thể do IMU/RC/Flow hoặc bộ đếm; bản cũ chưa ghi lý do.

## Sửa thay đổi gì?
1. Probe quan sát Y chịu được `nav.height_valid=false` tối đa **250ms** sau lần xác nhận gần nhất. Nếu tình trạng kéo dài, ABORT. Chỉ áp dụng **cho phép đo quan sát**, không thay estimator, PID, motor hoặc yêu cầu bắt đầu đo.
2. ARM/disarm/RC/mode, IMU fresh, và Flow fresh vẫn được kiểm tra nghiêm ngặt mỗi chu kỳ.
3. Dòng `ABORT reason:... gate:...` phân biệt `SAFETY_GATE`, `COUNTER_ROLLBACK`, `TOO_FEW_STILL_PACKETS`, và nêu rõ thành phần gate.
4. Giữ nguyên thu **từng gói** trên MTF task và hai pha `STILL` 2s → `MOVE_NOW` → `MOVE` 3s.
5. `PWM:0` vẫn khóa cứng compile-time, `xy_valid:0`. KHÔNG dùng bản này để bay.

## Test
1. Tháo cánh quạt, ngắt nguồn ESC. Giữ CH5=1000, CH6=2000. Nạp bằng PlatformIO, Serial 115200.
2. Giữ drone đứng yên, cách mặt giấy có hoa văn khoảng 50cm. Nhập `flow ytest`.
3. Chờ chính xác dòng `MOVE_NOW` rồi mới dịch drone ngang sang **phải theo trục thân** 30cm trong 3 giây, không xoay mũi, không kéo ngược. Không cần `cal stop`.
4. Gửi tất cả dòng `[Y_PROBE_V6_3Y2]`. Nếu vẫn `ABORT` thì không cần lặp lại; dòng `reason/gate` đã cho biết bước cần sửa.

## Giới hạn
Chưa có xác nhận Build/Upload trên ESP32 thực. V6.3Y2 chỉ làm phép đo quan sát, không tạo hệ số hiệu chuẩn chính thức và không cho phép ALT/POS HOLD bay.
