#include "control_task.h"
#include "config.h"
#include "globals.h"
#include "controller_math.h"
#include "flight_altitude.h"
#include "throttle_handover.h"
#include "alt_motor_gate.h"
#include "vertical_estimator.h"
#include "vertical_accel_fusion.h"
#include "alt_kalman_authority.h"
#include "esc_output.h"
#include "pos_authority.h"
#include <MPU6050_light.h>
#include <math.h>

extern MPU6050 mpu;

namespace {
using controller_math::RatePid;
using controller_math::PidOutput;
using controller_math::clampf;

float stickNorm(int us, int deadband = 20) {
    int diff = us - 1500;
    if (abs(diff) <= deadband) return 0.0f;
    return clampf((float)diff / 500.0f, -1.0f, 1.0f);
}
}

void control_task(void *parameter) {
    (void)parameter;
    RatePid pid[3] = {
        RatePid(ROLL_RATE_KP, ROLL_RATE_KI, ROLL_RATE_KD,
                GYRO_LPF_CUTOFF_HZ, DTERM_LPF_CUTOFF_HZ,
                MAX_AXIS_CORRECTION_US, PID_I_TERM_LIMIT_US),
        RatePid(PITCH_RATE_KP, PITCH_RATE_KI, PITCH_RATE_KD,
                GYRO_LPF_CUTOFF_HZ, DTERM_LPF_CUTOFF_HZ,
                MAX_AXIS_CORRECTION_US, PID_I_TERM_LIMIT_US),
        RatePid(YAW_RATE_KP, YAW_RATE_KI, YAW_RATE_KD,
                GYRO_LPF_CUTOFF_HZ, DTERM_LPF_CUTOFF_HZ,
                MAX_AXIS_CORRECTION_US, PID_I_TERM_LIMIT_US)
    };
    bool armed = false;
    bool armLowSeen = false;
    bool lastMixerClipped = false;
    flight_alt::RangeTracker liveRange;
    vertical_estimator::Kalman2 altKalmanZ;
    vertical_accel_fusion::Processor accelZ;
    alt_kalman_authority::Gate altKalmanGate;
    flight_alt::AltController liveAlt;
    throttle_handover::Controller throttleTransfer;
    pos_authority::Gate posGate;
    SharedNav cachedNav{};
    SharedRC cachedRc{};
    uint32_t loopFaultCount = 0;
    TickType_t wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(CONTROL_PERIOD_MS) > 0 ?
                              pdMS_TO_TICKS(CONTROL_PERIOD_MS) : 1;
    uint32_t lastUs = micros();

    for (;;) {
        const uint32_t nowUs = micros();
        float dt = static_cast<uint32_t>(nowUs - lastUs) * 1.0e-6f;
        lastUs = nowUs;
        const bool timingOk = dt >= 0.001f && dt <= 0.030f;
        if (!timingOk) {
            ++loopFaultCount;
            dt = 0.004f; // chi cho chuan hoa log, KHONG cho phép arm
        }

        mpu.update();
        float roll = IMU_ROLL_SIGN * mpu.getAngleX();
        float pitch = IMU_PITCH_SIGN * mpu.getAngleY();
        const float accX = mpu.getAccX();
        const float accY = mpu.getAccY();
        const float accZ = mpu.getAccZ();
        const float accNorm = sqrtf(accX*accX + accY*accY + accZ*accZ);
        const float accAngleX = mpu.getAccAngleX();
        const float accAngleY = mpu.getAccAngleY();
        const float rawRate[3] = {
            IMU_ROLL_SIGN * mpu.getGyroX(),
            IMU_PITCH_SIGN * mpu.getGyroY(),
            IMU_YAW_SIGN * mpu.getGyroZ()
        };
        bool imuOk = isfinite(roll) && isfinite(pitch) &&
                     fabsf(roll) < MAX_VALID_TILT_DEG &&
                     fabsf(pitch) < MAX_VALID_TILT_DEG;
        for (float r : rawRate) imuOk = imuOk && isfinite(r) && fabsf(r) < 1000.0f;

        SharedRC rc = cachedRc;
        if (mtx_rc && xSemaphoreTake(mtx_rc, 0) == pdTRUE) {
            rc = shared_rc;
            cachedRc = rc;
            xSemaphoreGive(mtx_rc);
        }
        // If a mutex is busy for one control tick, use the last valid snapshot
        // but NEVER extend its actual radio freshness.
        bool rcOk = rc.link_ok && rc.last_ok_ms != 0 &&
                    millis() - rc.last_ok_ms < RC_STALE_MS;
        SharedBattery bat{};
        bool batCopied = false;
        if (mtx_battery && xSemaphoreTake(mtx_battery, 0) == pdTRUE) {
            bat = shared_battery;
            batCopied = true;
            xSemaphoreGive(mtx_battery);
        }
        bool batteryReadyToArm = !ENABLE_BATTERY_ADC ||
            (batCopied && bat.enabled && bat.valid &&
             millis() - bat.last_update_ms < 2000 && bat.voltage_v >= VBAT_ARM_MIN_V);

        const int requestedMode = rcOk ?
            (rc.ch[CH_MODE] < 1300 ? MODE_ANGLE :
             (rc.ch[CH_MODE] < 1700 ? MODE_ALT_HOLD : MODE_POS_HOLD)) : MODE_ANGLE;
        const bool armLow = rcOk && rc.ch[CH_ARM] < CH_ARM_LOW_MAX_US;
        const bool armHigh = rcOk && rc.ch[CH_ARM] > CH_ARM_HIGH_MIN_US;
        const bool throttleLow = rcOk && rc.ch[CH_THROTTLE] <= THROTTLE_ARM_MAX_US;
        const bool level = fabsf(roll) <= MAX_ARM_TILT_DEG &&
                           fabsf(pitch) <= MAX_ARM_TILT_DEG;

        if (!rcOk || !imuOk || !timingOk) {
            armed = false;
            armLowSeen = false;
        } else if (!armHigh) {
            armed = false;
            armLowSeen = armLow && throttleLow && level && batteryReadyToArm &&
                         requestedMode == MODE_ANGLE;
        } else if (!armed && armLowSeen && throttleLow && level &&
                   batteryReadyToArm && requestedMode == MODE_ANGLE &&
                   (!ENABLE_MOTOR_OUTPUT || flight_esc::isReady())) {
            // ARM transition permitted ONLY from ANGLE, with throttle low.
            armed = true;
            armLowSeen = false;
        }

        // Hard safety failures always stop immediately. Under V2, after ALT has
        // engaged, low stick means controlled descent rather than motor cutoff.
        // The original flight_candidate environment keeps legacy behavior.
        const bool escReady = !ENABLE_MOTOR_OUTPUT || flight_esc::isReady();
        const auto authority = alt_motor_gate::decide(
            armed, rcOk, imuOk, timingOk, escReady,
            liveAlt.state().controlling,
            throttleTransfer.state().blending,
            rc.ch[CH_THROTTLE], THROTTLE_RUN_MIN_US);
        const bool active = ENABLE_ALT_V2_AUTHORITY ? authority.motor_active :
            (armed && rcOk && imuOk && timingOk &&
             rc.ch[CH_THROTTLE] >= THROTTLE_RUN_MIN_US);
        // Range is observed directly to avoid misinterpreting NAV's motion gate
        // (which is designed for guided calibration) as loss of raw range data.
        SharedMTF mtf{};
        bool mtfRead = false;
        if (mtx_sensor && xSemaphoreTake(mtx_sensor, 0) == pdTRUE) {
            mtf = shared_mtf;
            mtfRead = true;
            xSemaphoreGive(mtx_sensor);
        }
        static SharedMTF cachedMtf{};
        if (mtfRead) cachedMtf = mtf;
        else mtf = cachedMtf;
        const uint32_t sensorNowMs=millis();
        const auto &range = liveRange.update(sensorNowMs, mtf.last_range_ms,
            mtf.distance_mm, mtf.range_valid, roll, pitch);
        // Use SENSOR frame tilt angles: stick-command sign settings do not
        // belong in a body->earth acceleration transformation.
        const float gyroNorm=sqrtf(rawRate[0]*rawRate[0] +
                                   rawRate[1]*rawRate[1] +
                                   rawRate[2]*rawRate[2]);
        const auto &az=accelZ.update(sensorNowMs,accX,accY,accZ,
            mpu.getAngleX(),mpu.getAngleY(),gyroNorm,!armed,timingOk && imuOk);
        // Inertial prediction at control-tick rate; UNIQUE valid lidar frames
        // only correct the Kalman Z innovation. No reliance on IMU-only stale
        // position when lidar fails: RangeTracker and ALT latches remain.
        const auto &kf=altKalmanZ.update(sensorNowMs,range.valid,
            range.new_sample,range.frame_ms,range.measured_height_m,
            az.earth_z_linear_mps2,
            ENABLE_ALT_KALMAN_IMU_FUSION && az.valid);
        const bool altitudeModeSelected =
            requestedMode == MODE_ALT_HOLD || requestedMode == MODE_POS_HOLD;
        const bool wantsAltitude = ENABLE_MOTOR_OUTPUT && altitudeModeSelected;
        const auto &kalmanFeedback = altKalmanGate.update(
            sensorNowMs, altitudeModeSelected, ENABLE_ALT_KALMAN_ACTUATION,
            range, kf, ENABLE_ALT_KALMAN_IMU_FUSION);
        // Real actuator chain, when explicitly enabled:
        // KF height/vz -> guarded feedback -> AltController -> throttle
        // handover -> X mixer -> ESC. Raw range faults still latch live ALT.
        const auto &altitude = liveAlt.update(sensorNowMs, wantsAltitude, active,
            kalmanFeedback.feedback, rc.ch[CH_THROTTLE], ENABLE_ALT_V2_AUTHORITY);
        // Do not route ALT->MANUAL through the raw stick in one tick.
        // Immediate disarm/RC/IMU/timing faults still stop ESC.
        // V2 permits low stick in ALT and bounds exit-to-manual transition.
        const auto &throttleTransferState = throttleTransfer.update(millis(),
            active, altitude.controlling, altitude.throttle_us, rc.ch[CH_THROTTLE]);
        // Read NAV as one timestamped snapshot. Missing mutex does not extend
        // freshness, stale/invalid flow always drops POS back to existing ALT.
        SharedNav nav = cachedNav;
        if (mtx_nav && xSemaphoreTake(mtx_nav, 0) == pdTRUE) {
            nav = shared_nav;
            cachedNav = nav;
            xSemaphoreGive(mtx_nav);
        }
        pos_authority::Input pi{};
        pi.now_ms=millis();
        pi.nav_updated_ms=nav.last_calc_ms;
        pi.enabled=POS_HOLD_ACTUATION_ENABLED;
        pi.armed=armed; pi.rc_ok=rcOk; pi.imu_ok=imuOk; pi.timing_ok=timingOk;
        pi.alt_active=altitude.controlling;
        pi.scale_verified=POS_FLOW_SCALE_INDEPENDENTLY_VERIFIED;
        pi.signs_verified=POS_CONTROL_SIGNS_INDEPENDENTLY_VERIFIED;
        pi.rotation_compensation_verified=POS_ROTATION_COMPENSATION_VERIFIED;
        pi.requested_mode=requestedMode;
        pi.roll_us=rc.ch[CH_ROLL]; pi.pitch_us=rc.ch[CH_PITCH]; pi.yaw_us=rc.ch[CH_YAW];
        pi.xy_valid=nav.xy_valid;
        pi.flow_valid=nav.pos_flow_valid;
        pi.pos_active_mode=nav.pos_active_mode;
        const uint32_t navLag = nav.last_calc_ms ?
            (uint32_t)(pi.now_ms-nav.last_calc_ms) : UINT32_MAX;
        pi.flow_age_ms=pos_authority::effectiveAge(nav.pos_flow_age_ms, navLag);
        pi.range_age_ms=pos_authority::effectiveAge(nav.range_age_ms, navLag);
        pi.roll_cmd_deg=nav.pos_roll_target_deg;
        pi.pitch_cmd_deg=nav.pos_pitch_target_deg;
        const auto &posCommand=posGate.step(pi, dt);
        const bool bench = !ENABLE_MOTOR_OUTPUT && BENCH_MIX_WHILE_DISARMED &&
                           !active && imuOk && timingOk;
        const bool runPid = active || bench;
        float targetRoll = 0, targetPitch = 0;
        float targetRates[3] = {};
        PidOutput pidOut[3] = {};
        int motor[4] = {ESC_STOP_US, ESC_STOP_US, ESC_STOP_US, ESC_STOP_US};
        bool mixerClipped = false;

        if (runPid) {
            if (active) {
                targetRoll = STICK_ROLL_SIGN * stickNorm(rc.ch[CH_ROLL]) * MAX_COMMAND_ANGLE_DEG;
                targetPitch = STICK_PITCH_SIGN * stickNorm(rc.ch[CH_PITCH]) * MAX_COMMAND_ANGLE_DEG;
                // POS is permitted ONLY when independent calibration and an
                // explicit compile-time flag have authorized its output.
                // Otherwise this remains byte-for-byte ANGLE stick control.
                if (posCommand.applied) {
                    targetRoll = posCommand.roll_deg;
                    targetPitch = posCommand.pitch_deg;
                }
                targetRates[2] = STICK_YAW_SIGN * stickNorm(rc.ch[CH_YAW]) * MAX_COMMAND_YAW_RATE_DPS;
            }
            targetRates[0] = clampf(ANGLE_KP * (targetRoll - roll),
                                    -MAX_COMMAND_ROLLPITCH_RATE_DPS,
                                    MAX_COMMAND_ROLLPITCH_RATE_DPS);
            targetRates[1] = clampf(ANGLE_KP * (targetPitch - pitch),
                                    -MAX_COMMAND_ROLLPITCH_RATE_DPS,
                                    MAX_COMMAND_ROLLPITCH_RATE_DPS);
            // Khong tich phan trong bench (tranh day keo dong ho khi cam drone).
            for (int i=0; i<3; ++i) {
                pidOut[i] = pid[i].step(targetRates[i], rawRate[i], dt,
                                        active && !lastMixerClipped);
            }
            const int throttleUs = bench ? BENCH_FAKE_THROTTLE_US :
                throttleTransferState.output_us;
            auto mix = controller_math::mixX(throttleUs,
                                             pidOut[0].total,
                                             pidOut[1].total,
                                             pidOut[2].total,
                                             ESC_STOP_US, ESC_MAX_US);
            for (int i=0; i<4; ++i) motor[i] = mix.us[i];
            mixerClipped = mix.clipped;
            lastMixerClipped = mixerClipped;
            // Default: ANGLE targets + optional ALT. POS reaches the mixer
            // only after the strict verified, enabled, fresh-data gate.
        } else {
            lastMixerClipped = false;
            for (auto &p : pid) p.reset();
        }

        // Flight candidate sends OneShot125. Default BENCH never attaches ESC.
        // Link/disarm/IMU faults yield minimum pulse. In ANGLE, low stick
        // still stops; V2 ALT-low stick remains a descent command.
        flight_esc::write(motor, ENABLE_MOTOR_OUTPUT && active);
        digitalWrite(PIN_LED,
                     rcOk ? (armed ? LED_ON :
                             ((millis() / 250) % 2 ? LED_ON : LED_OFF)) : LED_OFF);
        if (mtx_control && xSemaphoreTake(mtx_control, 0) == pdTRUE) {
            shared_control.imu_ok = imuOk && timingOk;
            shared_control.acc_x_g = accX;
            shared_control.acc_y_g = accY;
            shared_control.acc_z_g = accZ;
            shared_control.acc_norm_g = accNorm;
            shared_control.acc_angle_x_deg = accAngleX;
            shared_control.acc_angle_y_deg = accAngleY;
            shared_control.rc_ok = rcOk;
            shared_control.armed_logic = armed;
            shared_control.motor_output_enabled = ENABLE_MOTOR_OUTPUT && active && flight_esc::isReady();
            shared_control.arm_low_seen = armLowSeen;
            shared_control.requested_mode = requestedMode;
            shared_control.active_mode = posCommand.applied ? MODE_POS_HOLD :
                (altitude.controlling ? MODE_ALT_HOLD : MODE_ANGLE);
            shared_control.roll_deg = roll;
            shared_control.pitch_deg = pitch;
            shared_control.gyro_x_dps = rawRate[0];
            shared_control.gyro_y_dps = rawRate[1];
            shared_control.gyro_z_dps = rawRate[2];
            shared_control.loop_dt_ms = dt * 1000.0f;
            shared_control.target_roll_deg = targetRoll;
            shared_control.target_pitch_deg = targetPitch;
            shared_control.alt_live = altitude.controlling;
            shared_control.alt_reason = altitude.reason;
            shared_control.alt_target_m = altitude.target_m;
            shared_control.alt_height_m = altitude.height_m;
            shared_control.alt_vz_mps = altitude.vz_mps;
            shared_control.alt_correction_us = altitude.correction_us;
            shared_control.alt_hover_us = altitude.hover_us;
            shared_control.alt_v2_authority = ENABLE_ALT_V2_AUTHORITY;
            shared_control.alt_authority_phase = ENABLE_ALT_V2_AUTHORITY ?
                static_cast<uint8_t>(authority.phase) : 0;
            shared_control.alt_low_stick_authority = ENABLE_ALT_V2_AUTHORITY &&
                authority.low_stick_alt_authority;
            shared_control.alt_kf_enabled = ENABLE_ALT_KALMAN_ACTUATION;
            shared_control.est_z_imu_fusion_enabled=ENABLE_ALT_KALMAN_IMU_FUSION;
            shared_control.est_z_imu_calibrated=az.calibrated;
            shared_control.est_z_imu_valid=az.valid;
            shared_control.est_z_imu_used=kf.imu_accel_used;
            shared_control.est_z_imu_still_samples=az.stationary_samples;
            shared_control.est_z_imu_rejected=az.rejected_samples;
            shared_control.est_z_imu_predict_count=kf.imu_accel_predictions;
            shared_control.est_z_imu_bias_mps2=az.bias_mps2;
            shared_control.est_z_imu_linear_mps2=az.earth_z_linear_mps2;
            shared_control.alt_kf_active = kalmanFeedback.kalman_used && altitude.controlling;
            shared_control.alt_kf_phase = static_cast<uint8_t>(kalmanFeedback.phase);
            shared_control.alt_kf_latched = kalmanFeedback.fallback_latched;
            shared_control.alt_kf_blend = kalmanFeedback.blend;
            shared_control.alt_kf_good_frames = kalmanFeedback.good_frames;
            shared_control.est_z_healthy = kf.healthy;
            shared_control.est_z_frame_used = kf.measurement_used;
            shared_control.est_z_height_m = kf.height_m;
            shared_control.est_z_vz_mps = kf.velocity_mps;
            shared_control.est_z_sigma_h = kf.sigma_height_m;
            shared_control.est_z_sigma_vz = kf.sigma_velocity_mps;
            shared_control.est_z_acc_earth_mps2 = az.earth_z_raw_mps2;
            shared_control.est_z_age_ms = kf.last_measurement_age_ms;
            shared_control.est_z_used_count = kf.corrections;
            shared_control.est_z_outliers = kf.outliers;
            shared_control.throttle_transfer_phase = throttleTransferState.phase;
            shared_control.throttle_collective_us = throttleTransferState.output_us;
            shared_control.throttle_pilot_us = throttleTransferState.pilot_us;
            shared_control.throttle_transitions = throttleTransferState.transitions;
            shared_control.throttle_bypasses = throttleTransferState.emergency_bypasses;
            shared_control.throttle_last_transition_ms = throttleTransferState.last_transition_ms;
            shared_control.throttle_last_step_us = throttleTransferState.last_step_us;
            shared_control.throttle_max_blend_step_us = throttleTransferState.max_blend_step_us;
            shared_control.pos_live = posCommand.applied;
            shared_control.pos_fault_latched = posCommand.latched;
            shared_control.pos_authority_reason = static_cast<uint8_t>(posCommand.reason);
            shared_control.pos_applied_roll_deg = posCommand.roll_deg;
            shared_control.pos_applied_pitch_deg = posCommand.pitch_deg;
            shared_control.mixer_clipped = mixerClipped;
            shared_control.bench_simulated = bench;
            shared_control.loop_fault_count = loopFaultCount;
            for (int i=0; i<3; ++i) {
                shared_control.target_rate[i] = targetRates[i];
                shared_control.filtered_rate[i] = pidOut[i].measured_filtered;
                shared_control.pid_p[i] = pidOut[i].p;
                shared_control.pid_i[i] = pidOut[i].i;
                shared_control.pid_d[i] = pidOut[i].d;
                shared_control.pid_total[i] = pidOut[i].total;
            }
            for (int i=0; i<4; ++i) shared_control.motor_us[i] = motor[i];
            shared_control.updated_ms = millis();
            xSemaphoreGive(mtx_control);
        }
        vTaskDelayUntil(&wake, period);
    }
}
