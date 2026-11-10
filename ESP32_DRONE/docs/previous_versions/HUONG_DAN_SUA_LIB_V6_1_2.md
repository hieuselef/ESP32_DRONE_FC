# Hướng dẫn V6.1.2 — Sửa UnknownPackageError trong PlatformIO

## Lỗi gốc

```
UnknownPackageError: Could not find the package with 'rfetick/MPU6050_light @ 1.2.1' requirements
```

Trong `platformio.ini` đã thay `rfetick/MPU6050_light @ 1.2.1` bằng `rfetick/MPU6050_light @ ^1.1.0`. Bản này KHÔNG sử dụng thuộc tính `upsideDownMounting` (chỉ đo chẩn đoán IMU).

## Cách chạy trên Windows

1. Giải nén ZIP vào **thư mục mới** và mở đúng thư mục có `platformio.ini`.
2. Giữ drone **không có cánh quạt**, ngắt nguồn ESC khi kiểm tra.
3. VS Code -> Terminal -> New Terminal, tại thư mục dự án:

```powershell
& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" run -e esp32dev -t clean
& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" run -e esp32dev
```

4. Khi Build `SUCCESS` mới Upload và xem `[IMU_PRECAL]`, `[IMU_POSTCAL]`, `[IMU_DIAG]`.
5. Nếu còn lỗi thư viện, thay dòng lib_deps bằng URL GitHub chính thức:

```ini
lib_deps =
    https://github.com/rfetick/MPU6050_light.git#v1.2.1
    https://github.com/AlfredoSystems/AlfredoCRSF.git
```

Lựa chọn GitHub này cần Git và kết nối Internet. Chỉ thay 1 dòng thư viện MPU6050, không thay code. Nếu vẫn lỗi, gửi toàn bộ log từ dòng `Error` đầu tiên. KHÔNG xóa toàn bộ `.platformio`.

## Giới hạn an toàn

V6.1.2 chỉ dùng chẩn đoán trên bàn, không phát PWM điều khiển ESC. Không bay với firmware này. Chưa Build được trên bộ toolchain PlatformIO Windows của người dùng; kiểm thử C++ đơn vị không thay cho Build firmware.
