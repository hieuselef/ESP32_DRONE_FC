# T8L CRSF monitor (PlatformIO standalone)

Dùng để tìm đúng CH1..CH10 cho T8L và kiểm tra LINK. Chỉ nạp khi đã **tháo cánh quạt**. Phần mềm thay firmware đang có; **sao lưu trước**. Mở riêng thư mục này bằng VS Code/PlatformIO (không mở root project). Nạp và mở Serial Monitor 115200. Mỗi lần chuyển một stick/switch, xem kênh nào có dấu `*` và thay đổi về khoảng 1000/1500/2000. Nếu LINK=0, kiểm tra RX đang giao tiếp CRSF và đường TX->GPIO16. Không xuất PWM ra ESC.
