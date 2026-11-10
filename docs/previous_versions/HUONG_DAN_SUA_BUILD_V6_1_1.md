# V6.1.1 - Sua loi Build MPU6050 / C++11

## Da sua
- Loai bo truy cap `mpu.upsideDownMounting` (gay loi voi MPU6050_light cu/cached).
- Khong tu dong dao truc IMU; `IMU_UPSIDE_DOWN_MOUNTING` phai `false` trong ban chuan doan nay (static_assert ngan bat nham).
- Doi `if constexpr` thanh `if` voi cau hinh const bool; tuong thich C++11.
- V6.1.2: dung `rfetick/MPU6050_light @ ^1.1.0` de tranh UnknownPackageError tren PlatformIO; V6.1.1 khong can upsideDownMounting.
- Khong co code xuat PWM den ESC, chi bench test.

## Tren Windows, thuc hien
1. Thao het canh quat, ngat nguon ESC.
2. Giai nen ZIP; trong VS Code mo **thu muc moi** chua `platformio.ini`. Khong chep le file `main.cpp` vao project cu.
3. PlatformIO: Clean, Build. Hoac trong terminal `pio run -t clean` va `pio run`.
4. Neu PlatformIO cache ban MPU6050_light cu, thoat VS Code, xoa thu muc `.pio/libdeps/esp32dev/MPU6050_light` trong project, sau do mo lai va Build. Chi xoa thu muc libdeps nay neu can, KHONG xoa backup firmware goc.
5. Chi sau khi Build SUCCESS moi Upload test, giu PWM=0. Serial Monitor 115200.
6. Kiem tra `[IMU_PRECAL]`, `[IMU_POSTCAL]`, `[IMU_DIAG]`; gia tri roll/pitch nen hop ly khi nam ngang. Neu goc ~180 do, do lai huong lap MPU va cac truc truoc khi chinh sua.

## Gioi han
- Chua duoc build tren ESP32 toolchain thuc (PlatformIO khong co trong moi truong tao file).
- Ban sua chi sua loi tuong thich thu vien/chuẩn C++; khong cam ket loai bo moi loi o moi may.
- Khong bay bang ban bench, khong bat motor output.
