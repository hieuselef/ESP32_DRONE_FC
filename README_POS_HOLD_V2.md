# POS HOLD – Tích hợp có khóa an toàn trên nền ESP32 FC V2

## Nguồn tham khảo

Video: https://www.youtube.com/watch?v=bEKm-PGRnks — MicoAir MTF-01, thiết lập Optical Flow + LiDAR để drone giữ vị trí và độ cao trong INAV6.

- Tài liệu MicoAir: https://github.com/micoair/MTF-01_USER_MANUAL
- Hướng dẫn INAV: https://oscarliang.com/setup-optical-flow-rangefinder-inav/
- **Không dùng firmware INAV** hoặc nhập PID INAV sang ESP32; đường MSPv2/MTF hiện tại vẫn giữ nguyên.

## Bổ sung chức năng gì?

1. `include/pos_authority.h`: cầu nối POS -> ANGLE targets, chỉ dùng khi CH6 cao, đang ARM, RC/IMU/timing hợp lệ, ALT thật sự đang ACTIVE, XY đã hiệu chuẩn, dữ liệu NAV/flow/range mới và lệnh nghiêng hữu hạn. Đợi liên tục 300 ms đủ điều kiện trước khi cho quyền can thiệp Roll/Pitch.
2. Ngay khi flow/range/NAV hỏng sau khi POS đã active, **hủy hiệu chỉnh ngang, khóa POS** tới khi CH6 rời mức cao, giữ ALT HOLD cũ nếu ALT còn điều kiện. DISARM vẫn có ưu tiên cao nhất. Cần Roll/Pitch/Yaw lệch tâm >80 us lấy lại điều khiển ANGLE và khóa POS cho đến khi rời CH6 high.
3. Tối đa ±5° Roll/Pitch cho POS và giới hạn tốc độ thay đổi target 12°/s tại nhịp điều khiển; không thay đổi ANGLE PID, ALT controller, throttle handover hay motor driver.
4. NAV dùng **độ cao ALT thực tế** khi đang ARM; thêm tuổi dữ liệu flow cộng cả độ trễ NAV->CTRL. Telemetry `[POS_AUTH_V2]` cho biết yêu cầu, chế độ thật, nguyên nhân khóa, tuổi dữ liệu, góc tác động.
5. `tests/test_pos_authority_v2.cpp` kiểm thử các cổng, timeout, latch, override, slew và fallback; toàn bộ host tests cũ được giữ.

## Hiện trạng đặc biệt quan trọng

- Mặc định `FC_POS_ACTUATION=0`, `POS_FLOW_SCALE_INDEPENDENTLY_VERIFIED=false`, `POS_CONTROL_SIGNS_INDEPENDENTLY_VERIFIED=false`, `POS_ROTATION_COMPENSATION_VERIFIED=false`. **POS chưa điều khiển động cơ**. `bench` và hai môi trường flight candidate vẫn giữ tính năng V2 như trước.
- Cấu hình ma trận 2D `POS_FLOW_FX_PER_M_X`, `POS_FLOW_FY_PER_M_X`, `POS_FLOW_FX_PER_M_Y`, `POS_FLOW_FY_PER_M_Y` đang bằng 0 trong `include/config.h` vì chưa có số đo thật. Firmware chủ động chặn build có POS motor nếu chưa có cả hai xác nhận hiệu chuẩn.
- Không có magnetometer/GPS: yaw tham chiếu chỉ tích phân gyro và trôi dần. Optical Flow không đo vị trí tuyệt đối. Thuật toán bù quay đã bổ sung nhưng 6 hệ số chưa được hiệu chuẩn, phép bù dùng gyro NAV 50Hz nên chỉ là xấp xỉ cho chuyển động quay chậm; cần đánh giá trên phần cứng trước khi bật điều khiển.
- Các giá trị PID và dấu phản hồi vẫn chưa được chứng minh trên đúng airframe. Không nhận POS là đã sẵn sàng bay sau khi unit-test hoặc build thành công.

## Hoàn thiện hiệu chuẩn XY (không lắp cánh)

Dữ liệu V2 hiện có hỗ trợ lệnh Guided Calibration, **không cần tạo firmware thử nghiệm khác**:

1. Giữ máy **DISARM** (CH5 thấp), CH6 cao, độ cao ~0.5 m so với mặt sàn có hoa văn rõ, ánh sáng đủ; Serial Monitor 115200, Newline. Không dùng mặt sàn trơn, phản quang.
2. Thực hiện `cal guided x 0.30`; giữ đứng yên đến READY, gõ `cal guided go` rồi dịch thẳng 30 cm theo hướng **mũi drone**. Ghi nhận `AXES X:(fx,fy)` và log quality. Có thể dịch ngược chiều để đối chiếu **dấu raw phải đảo**; không dùng hệ số của bước ngược như hệ số trục dương (`cal guided` chỉ nhận độ dài dương).
3. Thực hiện `cal guided y 0.30`; giữ đứng yên đến READY, gõ `cal guided go` rồi dịch sang **bên phải drone** 30 cm. Ghi nhận `AXES Y:(fx,fy)`. Lặp lại quan sát dấu đảo khi dịch ngược chiều (không đưa hệ số ngược chiều vào ma trận trục dương). Cần đo độc lập nhiều lượt, đảm bảo ma trận không gần suy biến.
4. Kiểm tra bổ sung **quay roll/pitch/yaw khi không tịnh tiến**: flow do chuyển động quay KHÔNG được phép bị hiểu nhầm là vận tốc đất. `flow_velocity.h` đã hỗ trợ bù quay theo 6 hệ số counts/radian, nhưng tất cả đang bằng 0 và khóa tính năng. Cần đo riêng, đối chiếu chiều trục gyro và xác nhận `POS_ROTATION_COMPENSATION_VERIFIED`, không thể suy ra chỉ từ `cal guided`.
5. Đối chiếu tốc độ suy ra với chuyển động thật ở độ cao khác nhau và xác minh riêng chiều **roll/pitch correction**. Không suy diễn dấu servo/mixer từ dấu raw flow.
6. Chỉ sau khi dữ liệu đủ tin cậy, điền ma trận theo đơn vị **raw counts trên mỗi mét di chuyển ở độ cao tham chiếu**, điền hai hệ số dấu, xác nhận các cờ verified, rồi xem xét kích hoạt `-DFC_POS_ACTUATION=1` trong một build được kiểm tra riêng (không được tự động bật tại đây). Cần đánh giá rotor/ESC/failsafe trên giá thử có bảo vệ trước mọi thử nghiệm bay.

## Log quan trọng

```
[BOOT] ... POS_ACTUATION:0 verified_xy:0 verified_signs:0 verified_rotation:0
[POS_UNIFIED] ... reason:FLOW_UNVERIFIED ... actuator_allowed:0
[POS_AUTH_V2] enabled:0 requested:2 active:1 applied:0 latch:0 reason:DISABLED ...
```

`active:1` trên POS_AUTH = **ALT HOLD**. `active:2` chỉ được dùng khi POS thực sự được phép và đang tác động Roll/Pitch. `POS_UNIFIED active:2` chỉ là toán học Shadow; không đồng nghĩa động cơ có POS.

## Đảm bảo không chạm các tính năng đã kiểm tra

Không thay: GPIO, RC channel, MSPv2 parser, CRSF, MPU6050/ANGLE PID, motor mixer, OneShot125 driver, ALT V2 low-stick, ALT->ANGLE throttle handover, MTF/ELRS failsafe hay Kalman Z shadow. POS chỉ ghi đè **target angle** nếu tất cả cổng an toàn đều đạt, còn không trả lại đúng ANGLE/ALT V2.

**Lưu ý:** không có PlatformIO/Arduino ESP32 toolchain hay phần cứng ESC thật để xác nhận build/upload/flight tại đây. Unit tests host chỉ xác thực logic thuần C++; chưa chứng nhận bay. Bay trong nhà cần khoảng trống, bảo vệ người xung quanh và công tắc ngắt khẩn cấp vật lý.
