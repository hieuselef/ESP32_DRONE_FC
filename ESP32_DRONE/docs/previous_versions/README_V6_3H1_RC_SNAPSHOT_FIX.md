# V6.3H1 – sửa lỗi huỷ hiệu chuẩn do đọc RC tức thời (BENCH ONLY)

## Từ log thực tế

V6.3H in `START`, rồi `ABORT reason:UNSAFE rc_mode:0` dù các log RC lân cận cho thấy `CH5:1000 CH6:2000`. Sau `ABORT`, lệnh `go` bị từ chối. Quality 108–114 là hợp lệ.

## Thay đổi

- Chỉ cho quy trình **guided calibration**: xác nhận CH5, CH6 từ mẫu RC thực đọc được, thay vì phụ thuộc vào `control.requested_mode` có thể thoáng về 0 khi control_task không lấy được RC mutex.
- Nếu nav_task hụt đọc RC mutex một vòng, cho phép dùng mẫu RC cuối chỉ **tối đa 80 ms**, đồng thời yêu cầu CRSF frame còn mới theo `RC_STALE_MS`, link hợp lệ, disarmed và CH6 >= 1700. Nếu lấy được mẫu RC không hợp lệ, **không dùng cache**. Đây là chống rung trạng thái quan sát, không phải bỏ kiểm tra an toàn.
- Khi hủy sẽ có thêm `[FLOW_GUIDED_V6_3H1] RC_DIAG` để xác định lần sau thực sự do liên kết, kênh hoặc mode.
- Không đổi `control_task`, `rc_task`, UART MTF, PID, ngưỡng Quality, ước lượng độ cao, ESC. PWM vẫn khóa cứng và `xy_valid=0`.

## Test một lượt Y

**Gỡ cánh quạt, ESC ngắt nguồn**. Giữ CH5=1000 và CH6=2000, mắt MTF cách mặt phẳng có hoa văn 40–50 cm. Serial 115200, `monitor_eol = LF` đã có.

1. Nhập `cal guided y 0.30`, chờ `READY` (~2 giây).
2. Nhập `cal guided go`, xác nhận `GO ACCEPTED`.
3. Di chuyển drone theo chiều sang phải 30 cm trong 5 giây, giữ nguyên cao độ và không quay về trong cửa sổ thu.
4. Gửi `BASELINE`, `WINDOW`, `DONE`, `AXES`, hoặc `ABORT` cùng `RC_DIAG` nếu có.

Không dùng bản BENCH để bay. Chưa xác nhận build trên bo ESP32 thật.
