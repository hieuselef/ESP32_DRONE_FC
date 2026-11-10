# Các cài đặt từ dump gốc cần GIỮ NGUYÊN

```
ESPF ESP32 v0.3.0-dev 0000000 Oct 21 2026
GYRO: MPU6050 I2C; gyro 8000Hz; loop 2000Hz; accel 500Hz
set mixer_type QUADX
set output_motor_protocol ONESHOT125
set output_motor_rate 480
set pin_output_0 12
set pin_output_1 13
set pin_output_2 14
set pin_output_3 27
set pin_i2c_scl 19
set pin_i2c_sda 18
set serial_0 1 115200 0
set serial_1 1 115200 0   # Thay thành 0 để giải phóng UART1
set serial_2 64 115200 0
set pin_serial_1_tx 33  # Config trước đây, task mới thiết lập RX33/TX32 trực tiếp
set pin_serial_1_rx 32
set pin_serial_2_tx 17
set pin_serial_2_rx 16
set mode_0 0 4 1750 2075 0 0  # ARM trên input index 4
set mode_1 2 6 1325 1650 0 0  # AIRMODE (không phải ALTHOLD)
set pid_roll_p 45
set pid_roll_i 80
set pid_roll_d 24
set pid_roll_f 88
set pid_pitch_p 47
set pid_pitch_i 84
set pid_pitch_d 27
set pid_pitch_f 92
set pid_yaw_p 45
set pid_yaw_i 80
set pid_yaw_d 0
set pid_yaw_f 88
set blackbox_dev NONE
```

Các hệ số / cài đặt còn lại đã được bảo toàn vì patch v4 chỉ sửa `src/main.cpp` và `Cli.cpp` trong source, không ghi trực tiếp vào NVS firmware; `set serial_1 0 115200 0` là thao tác CLI duy nhất cần có trước khi sử dụng UART1 riêng.
