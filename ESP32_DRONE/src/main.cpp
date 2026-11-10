#include <Arduino.h>
#include "hold_modes.h"
#include "pos_authority.h"
#include <cmath>
#include <Wire.h>
#include <MPU6050_light.h>
#include <HardwareSerial.h>
#include <AlfredoCRSF.h>
#include "config.h"
#include "globals.h"
#include "rc_task.h"
#include "mtf_task.h"
#include "nav_task.h"
#include "control_task.h"
#include "esc_output.h"
#include "nav_estimator.h"
#include "flow_diagnostics.h"
#include "flow_calibration.h"
#include "rc_stick_diag.h"

static_assert(!IMU_UPSIDE_DOWN_MOUNTING,
    "V6.1.1 diagnostic: upside-down auto correction is disabled; verify IMU orientation first");

MPU6050 mpu(Wire);
AlfredoCRSF crsf;
HardwareSerial crsfSerial(2);
HardwareSerial mtfSerial(1);

void setup() {
    Serial.begin(115200);
    delay(1000);
    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, LED_OFF);

    // BENCH defaults to GPIO low. In flight_candidate, attach OneShot125 after init.
    const int motorPins[] = {PIN_ESC_1, PIN_ESC_2, PIN_ESC_3, PIN_ESC_4};
    for (int pin : motorPins) { pinMode(pin, OUTPUT); digitalWrite(pin, LOW); }

    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    Wire.setClock(400000);
    if (mpu.begin() != 0) {
        Serial.println("[FATAL] MPU6050 khong ket noi - khong khoi dong controller");
        while (true) delay(1000);
    }
    // Diagnostics MUST be taken BEFORE offsets; calibration can hide a bad mounting orientation.
    mpu.update();
    delay(10);
    const float preAx = mpu.getAccX();
    const float preAy = mpu.getAccY();
    const float preAz = mpu.getAccZ();
    const float preNorm = sqrtf(preAx * preAx + preAy * preAy + preAz * preAz);
    Serial.printf("[IMU_PRECAL] ax:%+.3f ay:%+.3f az:%+.3f norm:%.3fg\n",
                  preAx, preAy, preAz, preNorm);
    if (!std::isfinite(preNorm) || preNorm < 0.7f || preNorm > 1.3f) {
        Serial.println("[IMU_WARNING] Precal norm not near 1g. Check power/I2C/motion!");
    }
    if (preAz < -0.5f && !IMU_UPSIDE_DOWN_MOUNTING) {
        Serial.println("[IMU_WARNING] Raw Z negative: maybe upside down. CHECK sensor orientation; do NOT enable motors.");
    }
    // V6.1.1: Khong truy cap upsideDownMounting trong thu vien phien ban cu.
    // Chi do chan doan huong lap IMU; khong tu dong dao truc.
    Serial.printf("[IMU] Keep sensor FLAT + STATIONARY. accOffset:%u upsideDown:%u\n",
                  IMU_CALIBRATE_ACCEL ? 1u : 0u, IMU_UPSIDE_DOWN_MOUNTING ? 1u : 0u);
    mpu.calcOffsets(true, IMU_CALIBRATE_ACCEL);
    for (int n = 0; n < 15; ++n) { mpu.update(); delay(10); }
    Serial.printf("[IMU_POSTCAL] ax:%+.3f ay:%+.3f az:%+.3f accXAngle:%+.1f accYAngle:%+.1f filtX:%+.1f filtY:%+.1f\n",
                  mpu.getAccX(), mpu.getAccY(), mpu.getAccZ(),
                  mpu.getAccAngleX(), mpu.getAccAngleY(), mpu.getAngleX(), mpu.getAngleY());

    crsfSerial.begin(CRSF_BAUDRATE, SERIAL_8N1, PIN_CRSF_RX, PIN_CRSF_TX);
    crsf.begin(crsfSerial);
    mtfSerial.setRxBufferSize(1024);
    mtfSerial.begin(115200, SERIAL_8N1, PIN_MTF_RX, PIN_MTF_TX);

    // ESC is attached only by explicit flight_candidate PlatformIO environment.
    // If initialization fails, HALT before starting any control tasks.
    if (ENABLE_MOTOR_OUTPUT && !flight_esc::begin()) {
        Serial.println("[FATAL] OneShot125 ESC initialization failed; refusing to arm.");
        while (true) delay(1000);
    }

    mtx_rc = xSemaphoreCreateMutex();
    mtx_sensor = xSemaphoreCreateMutex();
    mtx_control = xSemaphoreCreateMutex();
    mtx_battery = xSemaphoreCreateMutex();
    mtx_nav = xSemaphoreCreateMutex();
    if (!mtx_rc || !mtx_sensor || !mtx_control || !mtx_battery || !mtx_nav) {
        Serial.println("[FATAL] Mutex alloc failed");
        while (true) delay(1000);
    }

    BaseType_t t1 = xTaskCreatePinnedToCore(rc_task, "RC", 4096, nullptr, 4, nullptr, 0);
    BaseType_t t2 = xTaskCreatePinnedToCore(mtf_task, "MTF", 4096, nullptr, 3, nullptr, 0);
    BaseType_t t3 = xTaskCreatePinnedToCore(control_task, "CTRL", 8192, nullptr, 5, nullptr, 1);
    BaseType_t t4 = xTaskCreatePinnedToCore(nav_task, "NAV", 6144, nullptr, 2, nullptr, 0);
    if (t1 != pdPASS || t2 != pdPASS || t3 != pdPASS || t4 != pdPASS) {
        Serial.println("[FATAL] Cannot start all tasks");
        while (true) delay(1000);
    }
    if (ENABLE_BATTERY_ADC) {
        analogReadResolution(12);
        analogSetPinAttenuation(PIN_BATTERY_ADC, ADC_11db);
    }
    Serial.printf("[BOOT] FRAME:%umm | 4x1503 %uKV | PROP:%.1fin | ESC:%uA x4 | LiPo:%uS %umAh | 5V buck:%u | BatteryADC:%d\n",
        (unsigned)FRAME_DIAGONAL_MM, (unsigned)MOTOR_KV, PROPELLER_DIAMETER_IN,
        (unsigned)ESC_CONTINUOUS_CURRENT_A, (unsigned)LIPO_CELL_COUNT,
        (unsigned)LIPO_CAPACITY_MAH, LOGIC_POWERED_BY_5V_BUCK ? 1u : 0u,
        ENABLE_BATTERY_ADC ? 1 : 0);
    Serial.printf("[BOOT] V2 ANGLE+ALT + POS guarded | build:%s | Oneshot125:%d | POS_ACTUATION:%d verified_xy:%d verified_signs:%d verified_rotation:%d\n",
         ENABLE_MOTOR_OUTPUT ? "FLIGHT_CANDIDATE_UNVALIDATED" : "BENCH", ENABLE_MOTOR_OUTPUT ? 1 : 0,
         POS_HOLD_ACTUATION_ENABLED ? 1 : 0, POS_FLOW_SCALE_INDEPENDENTLY_VERIFIED ? 1 : 0,
         POS_CONTROL_SIGNS_INDEPENDENTLY_VERIFIED ? 1 : 0,
         POS_ROTATION_COMPENSATION_VERIFIED ? 1 : 0);
    Serial.println("[BOOT] CH6 low=ANGLE, mid=ALT, high=ALT+POS(GUARDED); arm only from ANGLE");
    Serial.printf("[BOOT_ALT_V2] low_stick_alt_authority:%d KF_Z_FEEDBACK:%d KF_IMU_FUSION:%d motor_hw_unverified:1\n",
                  ENABLE_ALT_V2_AUTHORITY ? 1 : 0,
                  ENABLE_ALT_KALMAN_ACTUATION ? 1 : 0,
                  ENABLE_ALT_KALMAN_IMU_FUSION ? 1 : 0);
}

void loop() {
    if (ENABLE_BATTERY_ADC) {
        // Divided ADC must be checked against multimeter BEFORE enabling!
        // Raw pin is limited to ESP32 ADC voltage range.
        const float adcV = analogReadMilliVolts(PIN_BATTERY_ADC) / 1000.0f;
        const float packV = adcV * ((VBAT_R_HIGH_OHM + VBAT_R_LOW_OHM) / VBAT_R_LOW_OHM)
                            * VBAT_CALIBRATION;
        SharedBattery bat{};
        bat.enabled = true;
        bat.valid = std::isfinite(packV) && packV >= 5.0f && packV <= 8.8f;
        bat.warning = bat.valid && packV <= VBAT_WARNING_V;
        bat.voltage_v = packV;
        bat.last_update_ms = millis();
        if (mtx_battery && xSemaphoreTake(mtx_battery, pdMS_TO_TICKS(5)) == pdTRUE) {
            shared_battery = bat;
            xSemaphoreGive(mtx_battery);
        }
    }
    SharedMTF sensor{};
    SharedRC rc{};
    SharedControl ctrl{};
    SharedBattery battery{};
    SharedNav nav{};
    bool sOk = false, rOk = false, cOk = false, bOk = false, nOk = false;
    if (xSemaphoreTake(mtx_sensor, pdMS_TO_TICKS(5)) == pdTRUE) {
        sensor = shared_mtf; sOk = true;
        xSemaphoreGive(mtx_sensor);
    }
    if (xSemaphoreTake(mtx_rc, pdMS_TO_TICKS(5)) == pdTRUE) {
        rc = shared_rc; rOk = true;
        xSemaphoreGive(mtx_rc);
    }
    if (xSemaphoreTake(mtx_control, pdMS_TO_TICKS(5)) == pdTRUE) {
        ctrl = shared_control; cOk = true;
        xSemaphoreGive(mtx_control);
    }
    if (xSemaphoreTake(mtx_battery, pdMS_TO_TICKS(5)) == pdTRUE) {
        battery = shared_battery; bOk = true;
        xSemaphoreGive(mtx_battery);
    }
    if (xSemaphoreTake(mtx_nav, pdMS_TO_TICKS(5)) == pdTRUE) {
        nav = shared_nav; nOk = true; xSemaphoreGive(mtx_nav);
    }
    uint32_t now = millis();
    // Quiet display only: leave sensor, RC, navigation and safety tasks running.
    if (static_cast<int32_t>(guided_quiet_until_ms - now) > 0) {
        delay(100);
        return;
    }
    // Keep CH5/CH6 diagnostic integrated: no manual rc_task edit needed.
    if (!ENABLE_PID_CSV_LOG) {
        Serial.printf("[RC_CHECK] CH5:%d CH6:%d\n",
                      rOk && rc.link_ok ? rc.ch[CH_ARM] : 0,
                      rOk && rc.link_ok ? rc.ch[CH_MODE] : 0);
        // CH1..CH4 read-only preview, even when DISARM. This does NOT set
        // ctrl.target_roll/pitch, PID, mixer or ESC pulses.
        const bool rcStickFresh = rOk && rc.link_ok && rc.last_ok_ms != 0 &&
                                  (uint32_t)(now - rc.last_ok_ms) < RC_STALE_MS;
        const int c1 = rcStickFresh ? rc.ch[CH_ROLL] : 0;
        const int c2 = rcStickFresh ? rc.ch[CH_PITCH] : 0;
        const int c3 = rcStickFresh ? rc.ch[CH_THROTTLE] : 0;
        const int c4 = rcStickFresh ? rc.ch[CH_YAW] : 0;
        const float previewRoll = rcStickFresh ?
            rc_stick_diag::angle(c1, STICK_ROLL_SIGN, MAX_COMMAND_ANGLE_DEG) : 0.0f;
        const float previewPitch = rcStickFresh ?
            rc_stick_diag::angle(c2, STICK_PITCH_SIGN, MAX_COMMAND_ANGLE_DEG) : 0.0f;
        const float previewYaw = rcStickFresh ?
            rc_stick_diag::yawRate(c4, STICK_YAW_SIGN, MAX_COMMAND_YAW_RATE_DPS) : 0.0f;
        Serial.printf("[RC_STICKS] fresh:%d CH1_R:%d CH2_P:%d CH3_T:%d CH4_Y:%d | "
                      "previewRoll:%+.1f previewPitch:%+.1f previewYawDps:%+.1f "
                      "(LOG_ONLY) | pidTargets:(%+.1f,%+.1f) ARM:%d PWM:%d\n",
                      rcStickFresh ? 1 : 0, c1, c2, c3, c4,
                      previewRoll, previewPitch, previewYaw,
                      ctrl.target_roll_deg, ctrl.target_pitch_deg,
                      cOk && ctrl.armed_logic ? 1 : 0,
                      cOk && ctrl.motor_output_enabled ? 1 : 0);
    }
    bool rangeFresh = sOk && sensor.last_range_ms != 0 && now - sensor.last_range_ms < MTF_STALE_MS;
    bool flowFresh = sOk && sensor.last_flow_ms != 0 && now - sensor.last_flow_ms < MTF_STALE_MS;
    if (ENABLE_PID_CSV_LOG) {
        // CSV de import Excel/Python: 20 Hz, chi gia tri da giam sat trong bench.
        static bool headerPrinted = false;
        if (!headerPrinted) {
            Serial.println("ms,rc,arm,bench,pwm,mode,roll_deg,pitch_deg,roll_rate_target,roll_rate_filtered,pitch_rate_target,pitch_rate_filtered,yaw_rate_target,yaw_rate_filtered,rP,rI,rD,pP,pI,pD,yP,yI,yD,m1,m2,m3,m4,clipped,dt_ms,loop_faults,dist_mm,flowx,flowy,flow_quality,flow_fresh,range_fresh,crc_errors");
            headerPrinted = true;
        }
        Serial.printf(
            "%lu,%d,%d,%d,%d,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,"
            "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,"
            "%d,%d,%d,%d,%d,%.2f,%lu,%ld,%ld,%ld,%u,%d,%d,%lu\n",
            (unsigned long)now, rc.link_ok ? 1 : 0, ctrl.armed_logic ? 1 : 0,
            ctrl.bench_simulated ? 1 : 0, ctrl.motor_output_enabled ? 1 : 0,
            ctrl.requested_mode, ctrl.roll_deg, ctrl.pitch_deg,
            ctrl.target_rate[0], ctrl.filtered_rate[0],
            ctrl.target_rate[1], ctrl.filtered_rate[1],
            ctrl.target_rate[2], ctrl.filtered_rate[2],
            ctrl.pid_p[0], ctrl.pid_i[0], ctrl.pid_d[0],
            ctrl.pid_p[1], ctrl.pid_i[1], ctrl.pid_d[1],
            ctrl.pid_p[2], ctrl.pid_i[2], ctrl.pid_d[2],
            ctrl.motor_us[0], ctrl.motor_us[1], ctrl.motor_us[2], ctrl.motor_us[3],
            ctrl.mixer_clipped ? 1 : 0, ctrl.loop_dt_ms,
            (unsigned long)ctrl.loop_fault_count, (long)sensor.distance_mm,
            (long)sensor.flow_x_raw, (long)sensor.flow_y_raw,
            (unsigned)sensor.flow_quality, flowFresh ? 1 : 0,
            rangeFresh ? 1 : 0, (unsigned long)sensor.crc_errors
        );
    } else {
        Serial.printf(
          "[FC] RC:%d ARM:%d MIX:%d PWM:%d MODE:%d | "
          "Roll:%.1f Pitch:%.1f gx:%.1f gy:%.1f gz:%.1f dt:%.1f | "
          "Dist:%ld rPacketOK:%d rFresh:%d | FX:%ld FY:%ld FQ:%u fFresh:%d | "
          "M:%d,%d,%d,%d clip:%d faults:%lu | VBAT:%s%.2fV warning:%d\n",
          rOk && rc.link_ok ? 1 : 0, cOk && ctrl.armed_logic ? 1 : 0,
          ctrl.bench_simulated ? 1 : 0, ctrl.motor_output_enabled ? 1 : 0,
          ctrl.requested_mode, ctrl.roll_deg, ctrl.pitch_deg,
          ctrl.gyro_x_dps, ctrl.gyro_y_dps, ctrl.gyro_z_dps, ctrl.loop_dt_ms,
          (long)sensor.distance_mm, sensor.range_valid ? 1 : 0, rangeFresh ? 1 : 0,
          (long)sensor.flow_x_raw, (long)sensor.flow_y_raw,
          (unsigned)sensor.flow_quality, flowFresh ? 1 : 0,
          ctrl.motor_us[0], ctrl.motor_us[1], ctrl.motor_us[2], ctrl.motor_us[3],
          ctrl.mixer_clipped ? 1 : 0, (unsigned long)ctrl.loop_fault_count,
          ENABLE_BATTERY_ADC && bOk ? "" : "not-connected/",
          ENABLE_BATTERY_ADC && bOk ? battery.voltage_v : 0.0f,
          ENABLE_BATTERY_ADC && bOk && battery.warning ? 1 : 0
        );
    }
    // A fixed 2 Hz IMU diagnostic uses the snapshot copied from control_task.
    // DO NOT call mpu.update() here: the MPU object belongs to control_task on core 1.
    static uint32_t lastImuLog = 0;
    if (cOk && now - lastImuLog >= 500) {
        lastImuLog = now;
        if (ENABLE_MOTOR_OUTPUT) {
            Serial.printf("[FLIGHT_ALT] requested:%d actual:%d active:%d reason:%u h:%.3f target:%.3f vz:%+.3f hover:%d corr:%+.1f | POS_MOTOR:0\n",
                ctrl.requested_mode, ctrl.active_mode, ctrl.alt_live ? 1 : 0,
                (unsigned)ctrl.alt_reason, ctrl.alt_height_m, ctrl.alt_target_m,
                ctrl.alt_vz_mps, ctrl.alt_hover_us, ctrl.alt_correction_us);
            Serial.printf("[ALT_TRANSFER] phase:%u collective:%d pilot:%d "
                          "count:%lu bypass:%lu last_ms:%lu | "
                          "0:STOP 1:MANUAL 2:ALT 3:BLEND 4:FULL_BYPASS\n",
                (unsigned)ctrl.throttle_transfer_phase,
                ctrl.throttle_collective_us, ctrl.throttle_pilot_us,
                (unsigned long)ctrl.throttle_transitions,
                (unsigned long)ctrl.throttle_bypasses,
                (unsigned long)ctrl.throttle_last_transition_ms);
            Serial.printf("[ALT_SAFETY_V2] enabled:%d phase:%u low_stick_authority:%d "
                          "step:%+d max_blend_step:%lu | rc:%d arm:%d pwm:%d\n",
                          ctrl.alt_v2_authority?1:0, (unsigned)ctrl.alt_authority_phase,
                          ctrl.alt_low_stick_authority?1:0,
                          ctrl.throttle_last_step_us,
                          (unsigned long)ctrl.throttle_max_blend_step_us,
                          ctrl.rc_ok?1:0, ctrl.armed_logic?1:0,
                          ctrl.motor_output_enabled?1:0);
        }
        Serial.printf("[ALT_KF_Z] enabled:%d applied:%d phase:%u latched:%d blend:%.2f "
                      "warm_frames:%u healthy:%d frame_used:%d z:%.3f vz:%+.3f "
                      "sigma_z:%.3f sigma_v:%.3f age:%lums corrections:%lu outliers:%lu "
                      "az_earth_raw:%+.3f\n",
                      ctrl.alt_kf_enabled?1:0, ctrl.alt_kf_active?1:0,
                      (unsigned)ctrl.alt_kf_phase, ctrl.alt_kf_latched?1:0,
                      ctrl.alt_kf_blend, (unsigned)ctrl.alt_kf_good_frames,
                      ctrl.est_z_healthy?1:0, ctrl.est_z_frame_used?1:0,
                      ctrl.est_z_height_m, ctrl.est_z_vz_mps,
                      ctrl.est_z_sigma_h, ctrl.est_z_sigma_vz,
                      (unsigned long)ctrl.est_z_age_ms,
                      (unsigned long)ctrl.est_z_used_count,
                      (unsigned long)ctrl.est_z_outliers,
                      ctrl.est_z_acc_earth_mps2);
        Serial.printf("[KF_IMU_Z] fusion:%d calibrated:%d valid:%d predicted:%d "
                      "stable:%u bias:%+.3f az_linear:%+.3f predictions:%lu rejected:%lu "
                      "| FLIGHT_ALT_active:%d KF_applied:%d\n",
                      ctrl.est_z_imu_fusion_enabled?1:0,
                      ctrl.est_z_imu_calibrated?1:0,
                      ctrl.est_z_imu_valid?1:0,ctrl.est_z_imu_used?1:0,
                      (unsigned)ctrl.est_z_imu_still_samples,
                      ctrl.est_z_imu_bias_mps2,ctrl.est_z_imu_linear_mps2,
                      (unsigned long)ctrl.est_z_imu_predict_count,
                      (unsigned long)ctrl.est_z_imu_rejected,
                      ctrl.alt_live?1:0,ctrl.alt_kf_active?1:0);
        Serial.printf("[IMU_DIAG] ax:%+.3f ay:%+.3f az:%+.3f norm:%.3fg | "
                      "accAngleX:%+.1f accAngleY:%+.1f | "
                      "filtRoll:%+.1f filtPitch:%+.1f | "
                      "imuOK:%u age:%lums\n",
                      ctrl.acc_x_g, ctrl.acc_y_g, ctrl.acc_z_g, ctrl.acc_norm_g,
                      ctrl.acc_angle_x_deg, ctrl.acc_angle_y_deg,
                      ctrl.roll_deg, ctrl.pitch_deg, ctrl.imu_ok ? 1u : 0u,
                      (unsigned long)(now - ctrl.updated_ms));
    }
    static uint32_t lastNavLog = 0;
    if (nOk && now - lastNavLog >= 500) {
        lastNavLog = now;
        Serial.printf("[NAV_SHADOW_V6_3] valid:%d reason:%s range_age:%lums "
                      "raw_h:%.3f tilt:%.3f filt_h:%.3f vz:%.3f | "
                      "mode:%d shadow:%s target:%.3f vz_sp:%.3f vel_err:%.3f i:%.3f u:%.3f | "
                      "warm:%u accepted:%lu rejects:%lu reacquire:%lu xy_valid:%d "
                      "motion_clear:%d acc_g:%.3f gyro_dps:%.1f\n",
             nav.height_valid ? 1 : 0,
             heightReasonName(static_cast<HeightReason>(nav.height_reason)), (unsigned long)nav.range_age_ms,
             nav.raw_vertical_m, nav.tilt_compensation, nav.height_m, nav.climb_mps,
             ctrl.requested_mode, shadowStateName(static_cast<ShadowState>(nav.shadow_state)),
             nav.altitude_target_m, nav.shadow_target_climb_mps, nav.shadow_velocity_error_mps,
             nav.shadow_i_term, nav.shadow_alt_correction, (unsigned)nav.warmup_count,
             (unsigned long)nav.range_samples, (unsigned long)nav.rejects,
             (unsigned long)nav.reacquisitions, nav.xy_valid ? 1 : 0,
             nav.motion_clear ? 1 : 0, nav.acc_norm_g, nav.gyro_norm_dps);
        Serial.printf("[TARGET_GUARD_V6_3] capture_reason:%s capture_count:%lu "
                      "gate:%s stable_ms:%lu stable_samples:%u last_capture_ms:%lu last_capture_samples:%u rejected:%lu "
                      "last_outlier_h:%.3f jump:%.3f outlier_count:%lu\n",
             captureReasonName(static_cast<TargetCaptureReason>(nav.target_capture_reason)),
             (unsigned long)nav.target_capture_count,
             captureGateName(static_cast<CaptureGate>(nav.target_capture_gate)),
             (unsigned long)nav.target_stable_ms, (unsigned)nav.target_stable_samples,
             (unsigned long)nav.target_last_capture_window_ms, (unsigned)nav.target_last_capture_window_samples,
             (unsigned long)nav.target_rejected_count, nav.last_outlier_h_m,
             nav.last_outlier_delta_m, (unsigned long)nav.outlier_count);
        // POS is default shadow-only; actual applied state is logged separately.
        Serial.printf("[POS_UNIFIED] request:%d active:%u reason:%s "
                      "scale_verified:%d flow_valid:%d frames:%lu "
                      "xy:(%.3f,%.3f) target:(%.3f,%.3f) "
                      "angles:(%.2f,%.2f) actuator_allowed:%d\n",
                      ctrl.requested_mode, (unsigned)nav.pos_active_mode,
                      v63hold::reasonName(static_cast<v63hold::Reason>(nav.pos_reason)),
                      nav.pos_flow_verified ? 1 : 0, nav.pos_flow_valid ? 1 : 0,
                      (unsigned long)nav.pos_flow_frames,
                      nav.pos_x_m, nav.pos_y_m,
                      nav.pos_target_x_m, nav.pos_target_y_m,
                      nav.pos_roll_target_deg, nav.pos_pitch_target_deg,
                      POS_HOLD_ACTUATION_ENABLED ? 1 : 0);
        Serial.printf("[POS_AUTH_V2] enabled:%d requested:%d active:%d applied:%d latch:%d "
                      "reason:%s nav_age:%lums flow_age:%lums xy:%d "
                      "cmd:(%+.2f,%+.2f)\n",
                      POS_HOLD_ACTUATION_ENABLED ? 1 : 0,
                      ctrl.requested_mode, ctrl.active_mode,
                      ctrl.pos_live ? 1 : 0, ctrl.pos_fault_latched ? 1 : 0,
                      pos_authority::reasonName(static_cast<pos_authority::Reason>(ctrl.pos_authority_reason)),
                      (unsigned long)(nav.last_calc_ms ? (uint32_t)(now-nav.last_calc_ms) : UINT32_MAX),
                      (unsigned long)nav.pos_flow_age_ms,
                      nav.xy_valid ? 1 : 0,
                      ctrl.pos_applied_roll_deg, ctrl.pos_applied_pitch_deg);
        Serial.printf("[FLOW_DIAG_V6_3] mode:%d ready:%d reason:%s age:%lums q:%u fx:%ld fy:%ld "
                      "ready_samples:%lu rejected:%lu xy_valid:%d\n",
             ctrl.requested_mode, nav.flow_cal_ready ? 1 : 0,
             flowDiagName(static_cast<FlowDiagReason>(nav.flow_diag_reason)),
             (unsigned long)nav.flow_age_ms, (unsigned)nav.flow_diag_quality,
             (long)nav.flow_diag_x_raw, (long)nav.flow_diag_y_raw,
             (unsigned long)nav.flow_ready_samples, (unsigned long)nav.flow_rejects,
             nav.xy_valid ? 1 : 0);
        Serial.printf("[FLOW_CAL_V6_3] state:%s reason:%s accepted:%lu rejected:%lu "
                      "planned:%.2fm sums:(%.0f,%.0f) candidate:%d "
                      "X:(%.1f,%.1f) Y:(%.1f,%.1f) det:%.1f xy_valid:0\n",
             flowCalStateName(static_cast<FlowCalState>(nav.flow_cal_state)),
             flowCalReasonName(static_cast<FlowCalReason>(nav.flow_cal_reason)),
             (unsigned long)nav.flow_cal_accepted, (unsigned long)nav.flow_cal_rejected,
             nav.flow_cal_planned_m, nav.flow_cal_sum_x, nav.flow_cal_sum_y,
             nav.flow_axes_candidate ? 1 : 0,
             nav.flow_cal_x_fx, nav.flow_cal_x_fy, nav.flow_cal_y_fx,
             nav.flow_cal_y_fy, nav.flow_cal_determinant);
    }
    delay(ENABLE_PID_CSV_LOG ? 50 : 500);
}
