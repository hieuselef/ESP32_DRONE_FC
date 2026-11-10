# V6.1 — Chan doan IMU_BAD (BENCH, khong xuat PWM)

Ban nay giu nguyen bo loc NAV va che do khoa motor cua V6. Bo sung log chi tiet MPU6050; KHONG tu dong dao truc, bo dieu kien IMU_BAD hoac cap PID cho motor.

## Log nguoi dung
- `Roll:174.1 Pitch:63.1`, `reason:IMU_BAD`: goc khong nam trong mien bo dieu huong/imu cho phep.
- `Dist:...` thay doi, `FQ` >0: MTF van gui du lieu.
- `raw_h:0.474`, `filt_h:0.524` khi invalid la gia tri CU, khong phai do moi.
- `PWM:0`: van khoa output motor.

## Cach test
1. Thao toan bo canh quat, ngat nguon ESC; giu USB, MPU va MTF co nguon 5V dung thong so.
2. Nhan reset ESP32 khi dat drone DUNG PHUONG (canh day huong xuong), de im tren mat phang ngang trong thoi gian `calcOffsets`.
3. Tai Serial Monitor 115200, luu lai 3 loai dong: `[IMU_PRECAL]`, `[IMU_POSTCAL]`, `[IMU_DIAG]`, va mot dong `[NAV_SHADOW_V6]`.
4. Nhan drone nghieng nhe tung truc mot sau khi khoi dong; quan sat do va dau cua `accAngleX/Y` va `filtRoll/Pitch`.
5. Khi nam ngang, accelerometer phai co norm gan 1 g, raw az thuong duong neu MPU lap xuoi (trong he truc mac dinh thu vien). Neu raw az am khi nam ngang, co the lap nguoc: XAC MINH HUONG LAP VA TRUC truoc khi thay `IMU_UPSIDE_DOWN_MOUNTING`. Doi flag khong thay the buoc kiem tra mixer.
6. Neu norm sai nhieu khi dung im: kiem tra nguon/I2C/moi noi/sensor; khong mo dieu khien.
7. Neu sau khoi dong `accAngleX/Y` gan 0 nhung `filtRoll/Pitch` van lon: xem lai thoi diem tinh offset, nhieu I2C, bo loc/gyro hoac van de trong thu vien.

## Tham so config.h
- `IMU_UPSIDE_DOWN_MOUNTING = false` mac dinh. CHI doi sau khi kiem tra chinh xac huong lap.
- `IMU_CALIBRATE_ACCEL = true` mac dinh giu tu V6. Neu MPU dong co hoc lap cheo, khong the giai quyet chi bang auto offset. Kiem tra he truc IMU, khong dao truc bang cach nguoc dau PIDs.
- `ENABLE_MOTOR_OUTPUT = false` va `static_assert(!ENABLE_MOTOR_OUTPUT)` bat buoc.

## Chua co
Chua Build bang PlatformIO tren ESP32 that. Khong ket luan imu loi phan cung neu chua co `[IMU_PRECAL]` va `[IMU_DIAG]`.


**V6.1.1**: `upsideDownMounting` khong duoc goi de dam bao build voi thu vien cach ly. Khong bat `IMU_UPSIDE_DOWN_MOUNTING = true`. Neu IMU nam nguoc, can xac minh truc va chon thu vien/thuật toán phu hop trong ban sau, khong sua dai.
