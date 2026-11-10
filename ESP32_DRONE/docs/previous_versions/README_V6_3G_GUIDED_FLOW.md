# V6.3G — Hướng dẫn kiểm tra Optical Flow X/Y (BENCH ONLY)

**Bản này kế thừa V6.3Y2 (bản phát triển từ V6.3), không sửa PID, không xuất PWM ESC và không cho POS HOLD hoạt động.**

## Lý do sửa

- Trước đây, `cal x/y` chỉ đọc gói Flow mới nhất ở vòng NAV 50 Hz; MTF truyền nhanh hơn, nên có thể bỏ mẫu.
- Trong V6.3Y2 đã có bộ cộng gói trực tiếp ở tác vụ MTF; V6.3G thêm bộ cộng riêng chỉ chấp nhận Quality >= 70 và giới hạn spike <= 200 counts/gói.
- Lượt thử Y gần đây vừa có nhiễu lúc đứng yên, vừa có các cửa sổ FX trái/phải đổi dấu. Trừ trung bình nhiễu một cách mù quáng có thể tạo ra tín hiệu FY giả. V6.3G **KHÔNG trừ nền để ép đạt**, chỉ dùng nền để quyết định đoạn nào đủ khác với đứng yên.
- Trong 3 giây di chuyển, chỉ chọn các cửa sổ 300 ms có tín hiệu và hướng đủ nhất quán. Nếu đổi chiều hoặc hai trục đo được gần song song, chương trình thông báo rõ ràng và không chấp nhận hệ số.

## Cách chạy

1. Tháo cánh quạt, ngắt nguồn ESC. Chỉ dùng USB và nguồn 5V logic phù hợp.
2. Mở ZIP trong PlatformIO, Build -> Upload, Serial 115200, Newline.
3. CH5=1000 (DISARM), CH6=2000 (MODE2), `ARM:0 PWM:0`.
4. Đặt drone nằm ngang, MTF nhìn xuống mặt nền rộng, đủ sáng, có hoa văn, khoảng 40–55 cm; đảm bảo chất lượng Flow tốt.
5. Nhập `cal guided y 0.30`. **Đứng yên 2 giây**, chỉ khi thấy `MOVE_NOW` mới dịch drone sang phải (theo thân drone) 30cm trong 3 giây. **Không quay lại vị trí cũ trong 3 giây này**. Tự kết thúc, không cần `cal stop`.
6. Gửi dòng `[FLOW_GUIDED_V6_3G] BASELINE`, `DONE`, `AXES`. Không thử lặp lại liên tục nếu không đạt.
7. Nếu Y có tín hiệu hợp lý và muốn đánh giá khả năng phân biệt X/Y, dùng thêm `cal guided x 0.30` tiến theo hướng mũi drone. Sau đó `cal guided status`. **Sau khi nạp lại firmware, kết quả cũ trong RAM sẽ mất**.

## Lệnh

- `cal guided x 0.30` — hướng dẫn thử X (đứng yên 2s + di chuyển 3s)
- `cal guided y 0.30` — hướng dẫn thử Y (đứng yên 2s + di chuyển 3s)
- `cal guided status` — hiển thị đo X/Y đã được chấp nhận trong phiên hiện tại
- `cal guided reset` — xóa hai kết quả guided
- Lệnh cũ `cal x`, `cal y`, `cal stop`, `cal reset`, `cal status` và `flow ytest` vẫn được giữ để đối chiếu; **kết quả guided và lệnh cũ không dùng chung bộ hiệu chuẩn**.

## Đọc kết quả

- `LOW_QUALITY`: không đủ gói Flow đạt Quality >=70 hoặc tín hiệu bị loại.
- `WEAK_OR_NO_MOTION`: không đủ cửa sổ có chuyển động rõ hơn nền.
- `REVERSE_OR_JITTER`: đổi chiều mạnh hoặc dữ liệu thiếu nhất quán.
- `BACKGROUND_DRIFT`: chuyển động tích lũy chưa tách được nhiễu lúc đứng yên.
- `AXIS_MEASURED`: mới đo sơ bộ được một trục.
- `COLLINEAR`: X và Y quá gần cùng hướng.
- `AXES_CANDIDATE`: sơ bộ đo được hai hướng phân biệt, **KHÔNG** có nghĩa POS HOLD an toàn.

**Không dùng số đếm Flow này làm vận tốc m/s. `xy_valid` bị khóa `false`; PWM và ARM thật không được bật.** Cần xác minh hướng/tỷ lệ, lấy mẫu với rung thực tế, kiểm chứng bộ ước lượng và tích hợp firmware thực sự đã bay được trước khi cho HOLD điều khiển motor.

Đã kiểm tra thuật toán C++ trên máy tính; chưa có xác nhận build PlatformIO hoặc thử phần cứng ESP32.
