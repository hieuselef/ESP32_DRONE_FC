# ESP32 Flight Controller tu phat trien — V6.2 NAV GUARD (BENCH ONLY)

## Phan mem nay lam gi?

V6.2 duoc phat trien tiep truc tiep tu du an **V6.1.2 LIB FIX** da nhan
MPU6050, CRSF T8L CH5/CH6, va MTF-01P MSP V2 tren phan cung cua ban.
Ban nay **KHONG DUOC PHEP BAY**. Khong co ESC driver, PWM output bi khoa
`ENABLE_MOTOR_OUTPUT = false` va `static_assert` trong `include/config.h`.
Danh sach `M:1400,...` trong log chi la **phep tinh mixer ao**, khong phai
PWM gui den ESC. Giữ thao canh quat, ESC ngat nguon khi bench.

## Nang cap so voi V6.1.2

1. `MOTION_BAD`: neu `|acc_norm_g - 1g| > 0.20g` hoac do lon gyro
   `> 180 deg/s`, danh dau do cao khong dang tin. `imuOK:1` co the van xuat
   hien (IMU con ket noi) nhung ALT shadow phai khong su dung do cao do.
2. `SETTLING`: sau khi chuyen dong manh ket thuc, can **300ms lien tuc**
   trang thai yen truoc khi thu lay lai mau do cao.
3. `OUTLIER`: buoc nhay do cao lon se bi cach ly **180ms**; sau do can
   them **3 mau lien tuc** trong vung on dinh de khoi phuc. Khong gan lai
   target ALT chi vi mot goi do dot bien.
4. `SENSOR_LOSS`: xoa I-term, `u=0` khi do cao khong hop le. Khi
   do cao on dinh lai, `CAPTURE` lay do cao **moi**; khong dung muc tieu cu.
   Khi chuyen MODE 1 <-> 2 cung lay lai target moi.
5. Shadow chi chay khi **CH5 ARM OFF** (CH5 <1300), RC con tin hieu,
   che do CH6 = 1 hoac 2. Bat ARM se cho shadow ve `OFF` va `u=0`.
6. `[FLOW_DIAG_V6_2]`: kiem tra tuoi goi Flow, chat luong, tu the,
   do cao va trang thai chuyen dong, de danh gia **du lieu du dieu kien
   cho qua trinh thu thap hieu chuan**. Khong chuyen counts sang m/s.
   `xy_valid` van **KHOA = 0**. CH6 = 2000 KHONG phai POS_HOLD bay that.
7. `[RC_CHECK] CH5:... CH6:...` da co san, khong can them log bang tay.

Tat ca nguong tren chi de **kiem thu tren ban**, chua phai quy dinh
an toan bay, chua duoc xac nhan trong chuyen dong va rung cua drone.

## Chan pin giu nguyen

- MPU6050 I2C: SDA18, SCL19.
- CRSF / ELRS: RX16, TX17 (UART2).
- MTF-01P MSP V2: RX33, TX32 (UART1), baud 115200.
- ESC pins: 12,13,14,27 — **chi duoc set LOW, khong co PWM**.
- CH1 Roll, CH2 Pitch, CH3 Throttle, CH4 Yaw, CH5 ARM, CH6 MODE.
- ADC pin = OFF; KHONG doc 2S thong qua 5V buck.

## Cau hinh PlatformIO

Mo dung thu muc chua `platformio.ini` va chon `esp32dev`.
Phu thuoc da tra lai khai bao **tung chay duoc tren may ban**:

```ini
lib_deps =
    rfetick/MPU6050_light @ ^1.1.0
    https://github.com/AlfredoSystems/AlfredoCRSF.git
```

Trong VS Code / PlatformIO: **Clean -> Build -> Upload**. Chỉ upload
sau khi **thao canh quat** va ngat nguon ESC, neu Build thanh cong.
Serial Monitor = 115200.

Luu y: chua co kha nang build ESP32/PlatformIO that trong moi truong
kiem thu tu dong o day. Cac bai test C++11 va cu phap ESP32 mock da PASS;
nguoi dung can xac nhan BUILD SUCCESS tren thiet bi.

## Kich ban test V6.2

1. **Tay cam va nguon:** bat T8L ELRS, CH5=1000 (ARM OFF), CH6=1000;
   thao canh quat, ngat nguon ESC, cap nguon logic USB/5V buck an toan.
2. **Tua cam bien:** dat MTF-01P huong xuong ban, cach 40–70 cm, giu on
   dinh cho `valid:1 reason:READY` va `motion_clear:1`.
3. **ALT shadow:** CH6=1500, xem `mode:1 shadow:CAPTURE` roi `TRACK`.
   Ha/nang nhe, doi chieu dau `u`; `PWM:0` BAT BUOC.
4. **Kich dong sensor:** di chuyen drone nhanh bang tay (giu chac,
   KHONG co canh quat), tim `reason:MOTION_BAD` khi acc_norm lon;
   `shadow:SENSOR_LOSS u:0.000 i:0.000`.
5. **Doi khi on dinh:** giu yen, thay `SETTLING` -> `WARMUP` -> `READY`,
   `CAPTURE` target moi sau khi lay du 3 mau. Neu buoc nhay range
   nhanh nhung norm~1, co the thay `OUTLIER` 180ms.
6. **POS diag:** CH6=2000, do cao khoang 50cm, mat san co hoa tiet;
   xem `[FLOW_DIAG_V6_2] reason:CAL_READY ready:1` neu diem chat luong
   Flow >=70 va cam bien du do cao/on dinh. `xy_valid:0` van la DUNG.
7. **Test ARM inhibit:** CH5=2000 khi dang MODE 1/2 (khong cap nguon ESC);
   `shadow:OFF`, `u:0`; sau do CH5 ve 1000. KHONG TEST motor.

## Y nghia log

```
[RC_CHECK] CH5:1000 CH6:1503
[BENCH] RC:1 ARM:0 MIX:1 PWM:0 MODE:1 | ...
[NAV_SHADOW_V6_2] valid:1 reason:READY ... shadow:TRACK ...
    motion_clear:1 acc_g:1.003 gyro_dps:0.4
[FLOW_DIAG_V6_2] mode:1 ready:1 reason:CAL_READY age:4ms
    q:106 fx:1 fy:-1 ready_samples:125 rejected:7 xy_valid:0
```

Day **chi la minh hoa**, khong phai log thuc tu ESP32. Neu `CAL_READY` thi
Flow moi du dieu kien **thu thap du lieu thô**, khong co nghia do duoc
van toc ngang hoac da hieu chuan. Khi thay `RANGE_BAD` voi `Dist:20-44`,
nang cam bien len >=80mm, thu o 400-700mm.

## Nhung gi CHUA co / KHONG duoc thuc hien

- Chua xac thuc chieu truc IMU theo frame va motor/ESC protocol.
- Chua tune PID bay, mixer/Oneshot125, failsafe cho motor.
- Chua chuyen pixel/flow counts thanh m/s; chua bo loc XY/POS HOLD.
- Chua test trong chuyen dong co dong co rung hoac bay that.
- Khong dung `u` (hieu chinh ga ao) lam PWM hoac map sang ESC.
- `VBAT:not-connected/0.00V` vi khong co ADC; khong suy doan pin 2S.

Cac files `HUONG_DAN_*V6_1*` va `TEST_RESULTS.txt` goc la lich su
V6.1.2. Ket qua moi nam trong `TEST_RESULTS_V6_2.txt`.
