# V2: Tích hợp POS HOLD theo Optical Flow + LiDAR (mặc định khóa điều khiển motor)

- Giữ nguyên V2 làm nền, không tạo V3, giữ nguyên cả ba PlatformIO environments.
- Bổ sung `include/pos_authority.h` để tách quyền điều khiển XY và failsafe khỏi PID ANGLE/ALT, có kiểm tra tuổi NAV+flow+range, cổng hiệu chuẩn, 300ms xác nhận, giới hạn góc 5°, slew 12°/s, fail latch, override bằng cần lái.
- Nối target X/Y tính toán từ `nav_task` vào `control_task` qua mutex và freshness gate. Chưa kiểm định nên mặc định POS chỉ là Shadow và không can thiệp motor.
- `flow_velocity.h` bổ sung mô hình bù quay gyro bằng 6 hệ số counts/radian (mặc định 0, chưa xác minh); yêu cầu đủ ba cờ hiệu chuẩn trước khi bật POS output.
- NAV armed POS dùng nguồn độ cao ALT đang chạy; giữ nguyên chế độ guided calibration đã có, logs thêm `[POS_AUTH_V2]`.
- 20/20 host tests pass; cú pháp `control_task.cpp`, `nav_task.cpp` được kiểm với stub Arduino cho bench và flight_alt_v2_candidate. Không có xác nhận PlatformIO ESP32/hardware/flight.

**POS_ACTUATION=0 / verified_xy=0 / verified_signs=0 / verified_rotation=0** là trạng thái xuất xưởng đúng; không được tùy tiện sửa sang 1.
