# V6.3Y - CHI KIEM TRA TRUC Y (BENCH, khong bay)

Bản này được sửa trực tiếp từ file ZIP V6.3 gốc của người dùng.

## Thay đổi

- `src/mtf_task.cpp`: Cộng số liệu của **từng gói MSP2 Optical Flow** khi nhận, không chỉ dựa vào bản sao mới nhất ở tác vụ điều hướng 50 Hz.
- `include/flow_y_probe.h`: Bộ quan sát độc lập cho Y: lấy nền đứng yên 2 giây, sau đó cửa sổ di chuyển 3 giây, in số liệu từng 300 ms. Lọc gói có quality < `MIN_FLOW_QUALITY_DIAG` hoặc biên độ >200 counts/gói; không giả định dữ liệu chắc chắn đúng.
- `src/nav_task.cpp`: Thêm duy nhất lệnh `flow ytest`. Kết thúc tự động. Không thay thuật toán hiệu chuẩn X/Y và không bật `xy_valid`.
- `tests/test_y_flow_probe.cpp`: Bài kiểm thử máy tính.

## Cách dùng

1. Tháo **toàn bộ cánh quạt**, **ngắt nguồn ESC/motor**. Chỉ cấp nguồn kiểm tra cho ESP32/cảm biến.
2. Mở dự án bằng PlatformIO và Build/Upload. Serial Monitor: 115200 baud, Newline.
3. CH5 1000, CH6 2000, kiểm tra `ARM:0 PWM:0 MODE:2` và range ~0.5 m; dùng nền có hoa văn và ánh sáng tốt.
4. Giữ drone đứng yên, gửi lệnh **`flow ytest`**. Giữ đứng yên 2 giây.
5. Khi hiện **`MOVE_NOW`**, dịch drone sang **phải đúng 30 cm** (theo trục phải của drone; mũi giữ nguyên) trong **3 giây**, không quay hay di chuyển ngược; giữ nguyên khoảng cách tới mặt sàn.
6. Tự động dừng, không cần `cal stop` hay `cal reset`. Gửi các dòng `[Y_PROBE_V6_3Y]` về để phân tích. Nếu `ABORT`, gửi nguyên nhân log.

Các pha vẫn sẽ có chút thời gian trễ do Serial Monitor; di chuyển ngay sau thông báo MOVE_NOW. **Không cần làm lại hiệu chuẩn trục X** vì kết quả X từ log trước được dùng để đối chiếu thủ công, không đưa vào firmware mới sau nạp.

## Cách đọc kết quả

- `STILL`: nhiễu nền đứng yên (nhìn net và abs cùng nhau).
- `WINDOW`: 300 ms riêng lẻ, hỗ trợ phát hiện lúc nào tín hiệu đổi dấu.
- `DONE`: chỉ dữ liệu trong 3 giây dịch sang phải.
- `COMPARE corrected`: trừ độ lệch trung bình của nền đứng yên; không phải hệ số hiệu chuẩn hay vận tốc m/s.
- Nếu `corrected Y` gần 0 và `corrected X` lớn, không được tự sửa dấu/tráo trục bừa; cần xác minh hướng gắn sensor và chuyển động vật lý.
- Không có kết quả nào ở bản này cho phép bật POS HOLD. `xy_valid` vẫn luôn `false`.

**Ghi chú quan trọng:** `cal x`, `cal y`, `cal stop` vẫn hoạt động như V6.3 gốc, chưa dùng bộ tích lũy mới. `flow ytest` là phép quan sát phụ để tìm đúng nguyên nhân Y trước khi sửa hiệu chuẩn. Bản này chưa được Build/Upload xác nhận trên ESP32 thật.
