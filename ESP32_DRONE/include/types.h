#pragma once
#include <Arduino.h>
#include "flow_y_probe.h"

struct SharedMTF {
    int32_t distance_mm = -1;          // -1: invalid/no ranging
    int32_t flow_x_raw = 0;           // counts, NOT m/s
    int32_t flow_y_raw = 0;
    uint8_t range_quality = 0;
    uint8_t flow_quality = 0;
    bool range_valid = false;
    bool flow_valid = false;          // so bo ve quality; chua duoc kiem dinh cho POS_HOLD
    uint32_t last_range_ms = 0;
    uint32_t last_flow_ms = 0;
    uint32_t range_frames = 0;
    uint32_t flow_frames = 0;
    uint32_t crc_errors = 0;
    // Packet-rate counters for Y-only bench observation. Never fed to flight control.
    FlowPacketCounters y_probe_counts{};
    FlowPacketCounters guided_cal_counts{}; // per-packet diagnostic, q>=70 only; never flight output
    // Aggregate EVERY good decoded flow frame. Required for velocity estimation;
    // NEVER convert counts to meters without independently verified 2D scale.
    int64_t flow_sum_x = 0, flow_sum_y = 0;
    uint32_t flow_good_frames = 0;
};

struct SharedRC {
    int ch[11] = {};
    bool link_ok = false;
    uint32_t last_ok_ms = 0;
};

struct SharedControl {
    bool imu_ok = false;
    float acc_x_g = 0, acc_y_g = 0, acc_z_g = 0;
    float acc_norm_g = 0;
    float acc_angle_x_deg = 0, acc_angle_y_deg = 0;

    bool rc_ok = false;
    bool armed_logic = false;
    bool motor_output_enabled = false;
    bool arm_low_seen = false;
    int requested_mode = 0;
    int active_mode = 0;
    float roll_deg = 0;
    float pitch_deg = 0;
    float gyro_x_dps = 0;
    float gyro_y_dps = 0;
    float gyro_z_dps = 0;
    float loop_dt_ms = 0;
    // Bang do de PID tuning (setpoint/gyro va P-I-D goc/toc do)
    float target_roll_deg = 0, target_pitch_deg = 0;
    float target_rate[3] = {};    // Roll Pitch Yaw (deg/s)
    float filtered_rate[3] = {};
    float pid_p[3] = {}, pid_i[3] = {}, pid_d[3] = {}, pid_total[3] = {};
    bool alt_live = false;
    uint8_t alt_reason = 0;
    float alt_target_m = 0, alt_height_m = 0, alt_vz_mps = 0, alt_correction_us = 0;
    int alt_hover_us = 0;
    bool alt_v2_authority = false;
    bool alt_low_stick_authority = false;
    uint8_t alt_authority_phase = 0;
    // KF motor feedback path; selected only in FC_ALT_KALMAN_ACTUATION build.
    bool alt_kf_enabled = false, alt_kf_active = false, alt_kf_latched = false;
    uint8_t alt_kf_phase = 0;
    uint16_t alt_kf_good_frames = 0;
    float alt_kf_blend = 0.0f;
    bool est_z_healthy = false, est_z_frame_used = false;
    float est_z_height_m = 0, est_z_vz_mps = 0;
    float est_z_sigma_h = 0, est_z_sigma_vz = 0;
    float est_z_acc_earth_mps2 = 0;
    bool est_z_imu_fusion_enabled=false;
    bool est_z_imu_calibrated=false, est_z_imu_valid=false, est_z_imu_used=false;
    uint16_t est_z_imu_still_samples=0;
    uint32_t est_z_imu_rejected=0, est_z_imu_predict_count=0;
    float est_z_imu_bias_mps2=0, est_z_imu_linear_mps2=0;
    uint32_t est_z_age_ms = UINT32_MAX;
    uint32_t est_z_used_count = 0, est_z_outliers = 0;
    // ALT->MANUAL output handover, for diagnosis (no POS motor actuation).
    uint8_t throttle_transfer_phase = 0;
    int throttle_collective_us = 1000, throttle_pilot_us = 1000;
    uint32_t throttle_transitions = 0, throttle_bypasses = 0;
    uint32_t throttle_last_transition_ms = 0;
    int throttle_last_step_us = 0;
    uint32_t throttle_max_blend_step_us = 0;
    bool pos_live = false;
    bool pos_fault_latched = false;
    uint8_t pos_authority_reason = 0;
    float pos_applied_roll_deg = 0, pos_applied_pitch_deg = 0;
    bool mixer_clipped = false;
    bool bench_simulated = false;
    uint32_t loop_fault_count = 0;
    int motor_us[4] = {1000, 1000, 1000, 1000};  // outputs calculated, may be blocked
    uint32_t updated_ms = 0;
};

// Battery monitor: measured only if ENABLE_BATTERY_ADC and divider exists.
struct SharedBattery {
    bool enabled = false;
    bool valid = false;
    bool warning = false;
    float voltage_v = 0.0f;
    uint32_t last_update_ms = 0;
};

// Nav shadow v6: observe-only altitude cascade. Never sent to ESC.
struct SharedNav {
    bool height_valid = false, xy_valid = false, flow_fresh = false;
    bool motion_clear = false, flow_cal_ready = false;
    uint8_t flow_diag_reason = 0, flow_diag_quality = 0;
    int32_t flow_diag_x_raw = 0, flow_diag_y_raw = 0;
    uint32_t flow_age_ms = 0, flow_ready_samples = 0, flow_rejects = 0;
    float acc_norm_g = 0.0f, gyro_norm_dps = 0.0f;
    float height_m = 0.0f, climb_mps = 0.0f;
    float vx_body_mps = 0.0f, vy_body_mps = 0.0f; // LOCKED until verified
    // Separate POS HOLD shadow controller; never enters motor mixer.
    uint8_t pos_reason = 0;
    uint8_t pos_active_mode = 0;
    bool pos_flow_verified = false, pos_flow_valid = false;
    float pos_x_m = 0.0f, pos_y_m = 0.0f;
    float pos_target_x_m = 0.0f, pos_target_y_m = 0.0f;
    float pos_roll_target_deg = 0.0f, pos_pitch_target_deg = 0.0f;
    uint32_t pos_flow_frames = 0;
    uint32_t pos_flow_age_ms = UINT32_MAX;
    float raw_vertical_m = 0.0f, tilt_compensation = 1.0f;
    float altitude_target_m = 0.0f, shadow_alt_correction = 0.0f;
    float shadow_target_climb_mps = 0.0f, shadow_velocity_error_mps = 0.0f;
    float shadow_i_term = 0.0f;
    uint8_t height_reason = 0, shadow_state = 0, warmup_count = 0;
    // V6.3: stable ALT target capture (diagnostics only).
    uint8_t target_capture_reason = 0, target_capture_gate = 0;
    uint32_t target_capture_count = 0, target_rejected_count = 0;
    uint32_t target_stable_ms = 0, target_last_capture_window_ms = 0;
    uint16_t target_stable_samples = 0, target_last_capture_window_samples = 0;
    // V6.3: guided flow axis calibration (bench-only candidate).
    uint8_t flow_cal_state = 0, flow_cal_reason = 0;
    bool flow_axes_candidate = false;
    uint32_t flow_cal_accepted = 0, flow_cal_rejected = 0;
    float flow_cal_planned_m = 0.0f;
    float flow_cal_sum_x = 0.0f, flow_cal_sum_y = 0.0f;
    float flow_cal_x_fx = 0.0f, flow_cal_x_fy = 0.0f;
    float flow_cal_y_fx = 0.0f, flow_cal_y_fy = 0.0f;
    float flow_cal_determinant = 0.0f;
    float last_outlier_h_m = 0.0f, last_outlier_delta_m = 0.0f;
    uint32_t outlier_count = 0;
    uint32_t range_age_ms = 0;
    uint32_t range_samples = 0, rejects = 0, reacquisitions = 0, last_calc_ms = 0;
};
