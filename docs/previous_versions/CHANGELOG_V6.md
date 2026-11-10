# Thay đổi V6 so với V5

- NAV tách hiệu lực CRC+range (`rPacketOK`) với hiệu lực sử dụng cho ước lượng cao độ (`valid`/`reason`).
- Khoảng cách `-1`, `-3`, 0 và quality 0 không bao giờ hợp lệ.
- Vùng NAV 80..3000mm; kiểm tra trễ mới ≤120ms; kiểm tra độ nghiêng ≤25°.
- Bù nghiêng cos(roll)cos(pitch), giữ chiều cao raw để so sánh đo lọc.
- Warm-up 3 mẫu, reject đột biến, reset và nhận lại sau khi mất tín hiệu.
- Vận tốc thẳng đứng từ sai phân đo liên tiếp, có lọc và giới hạn.
- ALT_HOLD SHADOW xếp tầng: giữ độ cao target lúc chuyển CH6, điều khiển vận tốc đặt, P+I vận tốc và anti-windup. Output giới hạn [-0.2, 0.2] chỉ để xem log.
- Shadow chạy khi CH5 ARM OFF và CH6 ở MID (không cần ARM). Mất range/mất RC => correction về 0.
- POS_HOLD chưa tính velocity XY vì Flow MSP raw chưa có scale/hướng đã hiệu chuẩn.
- Loại bỏ thư viện ESP32Servo; code không có lời gọi xuất PWM ESC.
