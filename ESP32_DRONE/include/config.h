#pragma once
#include <Arduino.h>

// Hardware nguoi dung: frame X 175 mm, 4x 1503 2750KV, propeller 3.5 inch,
// 4x ESC 8 A (2-3S), LiPo 2S 550mAh 90C; 5V buck cho logic.
// CHUA BIET: motor maker/thrust curve, prop pitch/blade count, AUW,
// ESC input protocol, voltage regulator output current, actual IMU alignment.
// Do NOT infer flyable PID from these specifications.
// Pin mapping: giu theo du an goc. Can kiem tra lai truc IMU va motor!
#define PIN_ESC_1 12   // Rear right, CCW (theo mo ta nguoi dung)
#define PIN_ESC_2 13   // Front right, CW
#define PIN_ESC_3 14   // Rear left, CW
#define PIN_ESC_4 27   // Front left, CCW
#define PIN_I2C_SDA 18
#define PIN_I2C_SCL 19
#define PIN_CRSF_RX 16
#define PIN_CRSF_TX 17
#define PIN_MTF_RX 33
#define PIN_MTF_TX 32
#define PIN_LED 2
#define LED_ON HIGH
#define LED_OFF LOW

#define CH_ROLL 1
#define CH_PITCH 2
#define CH_THROTTLE 3
#define CH_YAW 4
#define CH_ARM 5
#define CH_MODE 6

// T8L: CH1-CH6 duoc gia dinh theo du an goc, PHẢI KIEM TRA qua Receiver / T8L_CRFS_MONITOR.
// Safety: BENCH is default. Explicit flight_candidate env adds FC_FLIGHT_CANDIDATE=1.
#ifndef FC_FLIGHT_CANDIDATE
#define FC_FLIGHT_CANDIDATE 0
#endif
// Opt-in ALT Kalman Z: only modifies altitude feedback in the existing
// AltController. Default OFF. ESC cannot be enabled in bench builds.
#ifndef FC_ALT_KALMAN_ACTUATION
#define FC_ALT_KALMAN_ACTUATION 0
#endif
static_assert(FC_ALT_KALMAN_ACTUATION == 0 || FC_ALT_KALMAN_ACTUATION == 1,
              "FC_ALT_KALMAN_ACTUATION must be 0 or 1");
constexpr bool ENABLE_ALT_KALMAN_ACTUATION = (FC_ALT_KALMAN_ACTUATION == 1);
// OPTIONAL calibrated MPU6050 + LiDAR fusion for Kalman prediction.
// Explicit opt-in: disabled in legacy builds, including prior motor candidate.
#ifndef FC_ALT_KALMAN_IMU_FUSION
#define FC_ALT_KALMAN_IMU_FUSION 0
#endif
static_assert(FC_ALT_KALMAN_IMU_FUSION == 0 || FC_ALT_KALMAN_IMU_FUSION == 1,
              "FC_ALT_KALMAN_IMU_FUSION must be 0 or 1");
static_assert(FC_ALT_KALMAN_IMU_FUSION == 0 || FC_ALT_KALMAN_ACTUATION == 1,
              "IMU fusion to ALT requires explicit KF authority opt-in");
constexpr bool ENABLE_ALT_KALMAN_IMU_FUSION = FC_ALT_KALMAN_IMU_FUSION == 1;
// Chi xem xet thay doi sau khi kiem chung tren gia thu (bo thao canh quat).
constexpr bool ENABLE_MOTOR_OUTPUT = (FC_FLIGHT_CANDIDATE == 1);
#ifndef FC_ALT_V2_AUTHORITY
#define FC_ALT_V2_AUTHORITY 0
#endif
static_assert(FC_ALT_V2_AUTHORITY == 0 || FC_ALT_V2_AUTHORITY == 1,
              "ALT V2 authority requires explicit compile-time selection");
// V2 makes low stick a descent request while ALT is already engaged.
// NEVER enables motors by itself: independent ARM/RC/IMU/timing gates apply.
constexpr bool ENABLE_ALT_V2_AUTHORITY = (FC_ALT_V2_AUTHORITY == 1);
static_assert(FC_FLIGHT_CANDIDATE == 0 || FC_FLIGHT_CANDIDATE == 1, "Invalid build mode");
// CHI CHO BENCH: khi DISARM va ESC output dang khoa, tinh motor mix gia lap
// de kiem tra chieu phan hoi khi nghieng drone (khong quay motor).
constexpr bool BENCH_MIX_WHILE_DISARMED = !ENABLE_MOTOR_OUTPUT;
constexpr int BENCH_FAKE_THROTTLE_US = 1400;
// 1: xuat CSV 20 Hz de plot setpoint/gyro/PID; 0: log tom tat 2 Hz.
constexpr bool ENABLE_PID_CSV_LOG = false;

// Khung 175 mm: chi de luu metadata, KHONG dung de tu dong tinh PID.
constexpr uint16_t FRAME_DIAGONAL_MM = 175;
constexpr uint16_t MOTOR_STATOR_DIAMETER_MM = 15; // 1503
constexpr uint16_t MOTOR_STATOR_HEIGHT_MM = 3;
constexpr uint16_t MOTOR_KV = 2750;
constexpr float PROPELLER_DIAMETER_IN = 3.5f;
constexpr uint8_t MOTOR_COUNT = 4;
constexpr uint8_t ESC_CONTINUOUS_CURRENT_A = 8;
constexpr uint8_t LIPO_CELL_COUNT = 2;
constexpr uint16_t LIPO_CAPACITY_MAH = 550;
constexpr uint8_t LIPO_C_RATING_ADVERTISED = 90;
constexpr bool LOGIC_POWERED_BY_5V_BUCK = true;

// LUU Y: Do dien ap SAU BUCK 5V khong cho biet dien ap pin 2S.
// DO PIN TUY CHON - phai co cau chia ap VAT LY tu PIN 2S vao GPIO34 (ADC1).
// KHONG CAP 8.4V TRUC TIEP VAO GPIO34!
// Chi bat true sau khi dau dung va do doi chieu bang dong ho volt.
constexpr bool ENABLE_BATTERY_ADC = false;
constexpr uint8_t PIN_BATTERY_ADC = 34; // GPIO34 input-only
constexpr float VBAT_R_HIGH_OHM = 100000.0f; // BAT+ -> GPIO34
constexpr float VBAT_R_LOW_OHM  = 47000.0f;  // GPIO34 -> GND
constexpr float VBAT_CALIBRATION = 1.0f;     // dieu chinh theo dong ho
// Threshold tham khao de HIEN CANH BAO/NGAN ARM, KHONG tu dung motor giua khong trung
constexpr float VBAT_ARM_MIN_V = 7.0f;       // preflight only (2S)
constexpr float VBAT_WARNING_V = 7.0f;       // load dependent; can tinh chinh


// Flight candidate: ANGLE+ALT can drive motors in explicit flight_candidate build.
// POS requested: ALT only on motors; XY controller is strictly shadow.
constexpr int MODE_ANGLE = 0;
constexpr int MODE_ALT_HOLD = 1;
constexpr int MODE_POS_HOLD = 2;

constexpr int ESC_STOP_US = 1000;
constexpr int ESC_MAX_US = 1950;
constexpr int THROTTLE_ARM_MAX_US = 1050;
constexpr int THROTTLE_RUN_MIN_US = 1100;
// KHONG CO throttle limit duoc tu dong tinh tu KV: can do dong/tinh toan thrust.
constexpr int CH_ARM_LOW_MAX_US = 1300;
constexpr int CH_ARM_HIGH_MIN_US = 1700;

constexpr uint32_t CONTROL_PERIOD_MS = 4;   // 250 Hz yeu cau; phai do jitter thuc te
constexpr uint32_t RC_PERIOD_MS = 4;
constexpr uint32_t SENSOR_PERIOD_MS = 2;
constexpr uint32_t RC_STALE_MS = 350;      // AlfredoCRSF tu timeout link sau 300ms
constexpr uint32_t MTF_STALE_MS = 120;

constexpr float MAX_ARM_TILT_DEG = 20.0f;
constexpr float MAX_VALID_TILT_DEG = 75.0f;
constexpr float MAX_COMMAND_ANGLE_DEG = 20.0f;
constexpr float MAX_COMMAND_YAW_RATE_DPS = 100.0f;
constexpr float MAX_COMMAND_ROLLPITCH_RATE_DPS = 160.0f;

// DAU TRUC: TAM THOI +1, CHUA DUOC KIEM CHUNG VOI HUONG LAP IMU.
// Chan doan: CHI chuyen true neu cam bien duoc lap up-side-down VA da xac minh bang ax/ay/az.
// Khong tu dong sua goc vi co the che giau loi truc khi bay.
constexpr bool IMU_UPSIDE_DOWN_MOUNTING = false;
// MPU6050_light yeu cau dung tren mat phang khi tinh accel offsets.
// Mac dinh giu hanh vi V6, duoc phep tat de chan doan.
constexpr bool IMU_CALIBRATE_ACCEL = true;
constexpr float IMU_ROLL_SIGN = 1.0f;
constexpr float IMU_PITCH_SIGN = 1.0f;
constexpr float IMU_YAW_SIGN = 1.0f;
constexpr float STICK_ROLL_SIGN = 1.0f;
constexpr float STICK_PITCH_SIGN = 1.0f;
constexpr float STICK_YAW_SIGN = 1.0f;

// V6 PID IS NOT FLIGHT-VALIDATED. NEVER assume tuned to user previous esp-fc.
// KHONG PHAI PID DA TUNE CHO FRAME/MOTOR/ESC/PIN CUA BAN.
constexpr float ANGLE_KP = 4.0f;
// PID duoi day la gia tri CHO GIA LAP TREN BAN, khong phai PID da bay.
// Khong chep thang he so Betaflight/INAV sang code nay (khac don vi mixer).
constexpr float GYRO_LPF_CUTOFF_HZ = 60.0f;
constexpr float DTERM_LPF_CUTOFF_HZ = 25.0f;
constexpr float PID_I_TERM_LIMIT_US = 80.0f;
constexpr float ROLL_RATE_KP = 0.80f;
constexpr float ROLL_RATE_KI = 0.10f;
constexpr float ROLL_RATE_KD = 0.002f;
constexpr float PITCH_RATE_KP = 0.80f;
constexpr float PITCH_RATE_KI = 0.10f;
constexpr float PITCH_RATE_KD = 0.002f;
constexpr float YAW_RATE_KP = 0.60f;
constexpr float YAW_RATE_KI = 0.05f;
constexpr float YAW_RATE_KD = 0.0f;

constexpr float MAX_AXIS_CORRECTION_US = 200.0f;
constexpr uint8_t MIN_FLOW_QUALITY_DIAG = 30;
constexpr int32_t MIN_RANGE_MM = 20; // parser validation only; NAV requires >=80mm
constexpr int32_t MAX_RANGE_MM = 12000;

// Shadow navigation continues separately; live ALT is handled only by flight_altitude.h.
constexpr bool ENABLE_DISARMED_SHADOW_TEST = true;

// POS HOLD integration is compiled but OFF by default. Do not switch the flag
// until 2 independent X/Y translations, tilt/rotation isolation, output signs
// and complete props-off failover have been verified on this actual airframe.
#ifndef FC_POS_ACTUATION
#define FC_POS_ACTUATION 0
#endif
static_assert(FC_POS_ACTUATION == 0 || FC_POS_ACTUATION == 1,
              "FC_POS_ACTUATION must be 0 or 1");
// V6.3U: POS HOLD math is present, but actuation remains disabled by default.
// MTF MSPv2 values are RAW counts, NOT meters. A single cal candidate is not
// independent confirmation of physical axis signs / counts per meter.
constexpr bool POS_FLOW_SCALE_INDEPENDENTLY_VERIFIED = false;
constexpr bool POS_CONTROL_SIGNS_INDEPENDENTLY_VERIFIED = false;
constexpr bool POS_ROTATION_COMPENSATION_VERIFIED = false;
// MTF-01P optical-flow raw counts per radian of pure sensor rotation.
// All six numbers MUST come from measured, independently checked data.
constexpr float POS_FX_PER_RAD_ROLL = 0.0f;
constexpr float POS_FX_PER_RAD_PITCH = 0.0f;
constexpr float POS_FX_PER_RAD_YAW = 0.0f;
constexpr float POS_FY_PER_RAD_ROLL = 0.0f;
constexpr float POS_FY_PER_RAD_PITCH = 0.0f;
constexpr float POS_FY_PER_RAD_YAW = 0.0f;
constexpr float POS_ROLL_CORRECTION_SIGN = 1.0f;
constexpr float POS_PITCH_CORRECTION_SIGN = 1.0f;
constexpr bool POS_HOLD_ACTUATION_ENABLED = (FC_POS_ACTUATION == 1) && ENABLE_MOTOR_OUTPUT;
static_assert(FC_POS_ACTUATION == 0 ||
              (POS_FLOW_SCALE_INDEPENDENTLY_VERIFIED &&
               POS_CONTROL_SIGNS_INDEPENDENTLY_VERIFIED &&
               POS_ROTATION_COMPENSATION_VERIFIED &&
               FC_FLIGHT_CANDIDATE == 1),
              "POS requires verified XY matrix, gyro flow compensation, control signs and ESC build");
static_assert(POS_ROLL_CORRECTION_SIGN == 1.0f || POS_ROLL_CORRECTION_SIGN == -1.0f,
              "Invalid POS roll correction sign");
static_assert(POS_PITCH_CORRECTION_SIGN == 1.0f || POS_PITCH_CORRECTION_SIGN == -1.0f,
              "Invalid POS pitch correction sign");
// DO NOT populate or enable these constants from unverified calibration logs.
constexpr float POS_FLOW_FX_PER_M_X = 0.0f;
constexpr float POS_FLOW_FY_PER_M_X = 0.0f;
constexpr float POS_FLOW_FX_PER_M_Y = 0.0f;
constexpr float POS_FLOW_FY_PER_M_Y = 0.0f;
constexpr float POS_FLOW_REFERENCE_HEIGHT_M = 0.50f;
