#include "nav_task.h"
#include "config.h"
#include "globals.h"
#include "nav_estimator.h"
#include "flow_diagnostics.h"
#include "flow_calibration.h"
#include "flow_y_probe.h"
#include "flow_guided_cal.h"
#include "guided_rc_gate.h"
#include "flow_velocity.h"
#include "xy_bench_observer.h"
#include "hold_modes.h"
#include <cstdio>
#include <cstring>
#include <cstdint>
#include <cctype>
#include <cmath>

void nav_task(void *parameter) {
    (void)parameter;
    NavEstimator estimator;
    ShadowAltHold hold;
    FlowDiagnostics flow_diag;
    FlowCalibrator flow_cal;
    YFlowProbe y_probe;
    // Static allocation prevents GuidedFlowCal from consuming NAV task stack.
    static GuidedFlowCal guided_cal;
    // Candidate XY matrix only for independent disarmed displacement observation.
    // Never assigned to POS flow verification, SharedNav.xy_valid, or motor control.
    static v2xybench::Observer xy_observer;
    // Packet-level audit is independent of XY Observer auto-abort.
    // It captures the WHOLE physical trial until explicit `xy stop`.
    // Only telemetry; it never affects flight control or failsafes.
    FlowPacketCounters xy_audit_begin{}, xy_audit_end{};
    bool xy_audit_valid=false;
    bool xy_audit_tracking=false;
    uint32_t xy_audit_start_ms=0, xy_audit_end_ms=0;
    uint32_t last_xy_print_ms=0;
    // POS HOLD shadow is separate from the EXISTING altitude controller.
    // Its outputs are telemetry only; never sent to control_task/motor mixer.
    v63flow::Scale pos_scale{};
    pos_scale.independently_verified = POS_FLOW_SCALE_INDEPENDENTLY_VERIFIED;
    pos_scale.fx_per_m_x = POS_FLOW_FX_PER_M_X;
    pos_scale.fy_per_m_x = POS_FLOW_FY_PER_M_X;
    pos_scale.fx_per_m_y = POS_FLOW_FX_PER_M_Y;
    pos_scale.fy_per_m_y = POS_FLOW_FY_PER_M_Y;
    pos_scale.reference_height_m = POS_FLOW_REFERENCE_HEIGHT_M;
    pos_scale.rotation_compensation_verified = POS_ROTATION_COMPENSATION_VERIFIED;
    pos_scale.fx_per_rad_roll = POS_FX_PER_RAD_ROLL;
    pos_scale.fx_per_rad_pitch = POS_FX_PER_RAD_PITCH;
    pos_scale.fx_per_rad_yaw = POS_FX_PER_RAD_YAW;
    pos_scale.fy_per_rad_roll = POS_FY_PER_RAD_ROLL;
    pos_scale.fy_per_rad_pitch = POS_FY_PER_RAD_PITCH;
    pos_scale.fy_per_rad_yaw = POS_FY_PER_RAD_YAW;
    v63flow::Estimator pos_velocity(pos_scale);
    v63hold::Config pos_config{};
    pos_config.pos_roll_sign = POS_ROLL_CORRECTION_SIGN;
    pos_config.pos_pitch_sign = POS_PITCH_CORRECTION_SIGN;
    v63hold::Controller pos_hold(pos_config);
    // V6.3H1: last confirmed RC read for guided BENCH calibration ONLY.
    // A single zero-timeout mutex miss must not abort an otherwise healthy trial.
    SharedRC guided_last_rc{};
    uint32_t guided_last_rc_read_ms = 0;
    uint32_t last_y_height_valid_ms = 0; // observation-only, no effect on flight logic
    char serial_line[64] = {};
    size_t serial_len = 0;
    TickType_t wake = xTaskGetTickCount();
    for (;;) {
        SharedMTF sensor{};
        SharedControl control{};
        SharedRC rc{};
        bool sensor_read = false, ctrl_read = false, rc_read = false;
        if (mtx_sensor && xSemaphoreTake(mtx_sensor, 0) == pdTRUE) {
            sensor = shared_mtf; sensor_read = true; xSemaphoreGive(mtx_sensor);
        }
        if (mtx_control && xSemaphoreTake(mtx_control, 0) == pdTRUE) {
            control = shared_control; ctrl_read = true; xSemaphoreGive(mtx_control);
        }
        if (mtx_rc && xSemaphoreTake(mtx_rc, 0) == pdTRUE) {
            rc = shared_rc; rc_read = true; xSemaphoreGive(mtx_rc);
            guided_last_rc = rc;
            guided_last_rc_read_ms = millis();
        }
        if (sensor_read && ctrl_read) {
            const uint32_t now = millis();
            NavInput in{};
            in.now_ms = now;
            in.range_time_ms = sensor.last_range_ms;
            in.flow_time_ms = sensor.last_flow_ms;
            in.distance_mm = sensor.distance_mm;
            in.flow_x_raw = sensor.flow_x_raw;
            in.flow_y_raw = sensor.flow_y_raw;
            in.range_quality = sensor.range_quality;
            in.flow_quality = sensor.flow_quality;
            in.range_valid = sensor.range_valid;
            in.flow_valid = sensor.flow_valid;
            // A stale IMU snapshot must not qualify for navigation.
            in.imu_valid = control.imu_ok && control.updated_ms != 0 &&
                (uint32_t)(now - control.updated_ms) <= 100;
            in.roll_deg = control.roll_deg;
            in.pitch_deg = control.pitch_deg;
            in.acc_norm_g = control.acc_norm_g;
            in.gyro_norm_dps = sqrtf(control.gyro_x_dps * control.gyro_x_dps +
                                      control.gyro_y_dps * control.gyro_y_dps +
                                      control.gyro_z_dps * control.gyro_z_dps);
            const NavOutput &nav = estimator.step(in);
            if (nav.height_valid) last_y_height_valid_ms = now;
            FlowDiagInput fi{};
            fi.now_ms = now;
            fi.frame_time_ms = sensor.last_flow_ms;
            fi.x_raw = sensor.flow_x_raw;
            fi.y_raw = sensor.flow_y_raw;
            fi.quality = sensor.flow_quality;
            fi.frame_valid = sensor.flow_valid;
            fi.height_valid = nav.height_valid;
            fi.motion_clear = nav.motion_clear;
            fi.height_m = nav.height_m;
            fi.roll_deg = in.roll_deg;
            fi.pitch_deg = in.pitch_deg;
            const FlowDiagOutput &flow = flow_diag.step(fi);
            const bool rc_fresh = rc_read && rc.link_ok && rc.last_ok_ms != 0 &&
                (uint32_t)(now - rc.last_ok_ms) < RC_STALE_MS;
            // Bench shadow is intentionally enabled only while explicitly DISARMED.
            // A high ARM switch (CH5) immediately resets target and I-term.
            const bool disarmed_bench = rc_fresh && !control.armed_logic &&
                rc.ch[CH_ARM] < CH_ARM_LOW_MAX_US;
            const int mode = (ENABLE_DISARMED_SHADOW_TEST && disarmed_bench) ?
                control.requested_mode : MODE_ANGLE;
            const ShadowOutput &sh = hold.step(now, mode, rc_fresh, nav);
            const bool authorized_cal = disarmed_bench && mode == MODE_POS_HOLD;
            // V6.3H1: dedicated RC authorization for guided BENCH acquisition.
            // Use physical CH5/CH6 rather than control_task.requested_mode, which
            // can briefly fall back to ANGLE on an RC mutex miss. For this
            // diagnostic ONLY, tolerate a missed RC snapshot for <=80ms; a
            // genuinely lost link, stale packet, ARM-high or CH6-low still stops it.
            const uint32_t rc_snapshot_age = guided_last_rc_read_ms ?
                (uint32_t)(now - guided_last_rc_read_ms) : UINT32_MAX;
            const bool guided_cached_rc_ok = !rc_read && guidedSnapshotFresh(
                now, guided_last_rc_read_ms, guided_last_rc.last_ok_ms,
                guided_last_rc.link_ok, RC_STALE_MS, 80);
            const SharedRC &guided_rc = rc_read ? rc : guided_last_rc;
            const bool guided_rc_fresh = rc_fresh || guided_cached_rc_ok;
            const bool guided_authorized = guidedRcModeAuthorized(
                guided_rc_fresh, control.armed_logic, guided_rc.ch[CH_ARM],
                guided_rc.ch[CH_MODE], CH_ARM_LOW_MAX_US, 1700, 2050);
            auto xy_bench_input = [&]() {
                v2xybench::Input b{};
                b.now_ms=now;b.last_flow_ms=sensor.last_flow_ms;
                b.good_frames=sensor.flow_good_frames;
                b.sum_fx=sensor.flow_sum_x;b.sum_fy=sensor.flow_sum_y;
                b.height_m=nav.height_m;b.height_valid=nav.height_valid;
                b.quality=sensor.flow_quality;b.flow_valid=sensor.flow_valid;
                b.roll_deg=control.roll_deg;b.pitch_deg=control.pitch_deg;
                b.gyro_norm_dps=nav.gyro_norm_dps;b.imu_valid=in.imu_valid;
                b.authorized=guided_authorized && !guided_cal.active() &&
                    y_probe.phase()!=YProbePhase::MOVE &&
                    y_probe.phase()!=YProbePhase::STILL;
                return b;
            };
            // Commands are read ONLY by this task; no Serial.read race with loop().
            // Use Serial Monitor send line ending "Newline" and 115200 baud.
            while (Serial.available() > 0) {
                const int b = Serial.read();
                if (b < 0) break;
                const char c = static_cast<char>(b);
                if (c == '\r') continue;
                if (c != '\n' && serial_len + 1 < sizeof(serial_line)) {
                    serial_line[serial_len++] = c;
                    continue;
                }
                serial_line[serial_len] = 0;
                for (size_t j=0; j<serial_len; ++j)
                    serial_line[j] = static_cast<char>(tolower(static_cast<unsigned char>(serial_line[j])));
                char axis = 0;
                float dist = 0.0f;
                if (sscanf(serial_line, "cal guided %c %f", &axis, &dist) == 2 &&
                    (axis == 'x' || axis == 'y')) {
                    const bool safe_start = guided_authorized && in.imu_valid && nav.height_valid &&
                        sensor.last_flow_ms && (uint32_t)(now - sensor.last_flow_ms) < 150;
                    if (guided_cal.start(axis == 'x' ? 'X' : 'Y', dist, now,
                                         sensor.guided_cal_counts, safe_start &&
                                         y_probe.phase() != YProbePhase::STILL && y_probe.phase() != YProbePhase::MOVE)) {
                        guided_quiet_until_ms = now + 180000UL; // mute routine spam during session
                        Serial.printf("\n[FLOW_GUIDED_V6_3H1] START axis:%c %.2fm | STILL for 2 seconds. DO NOT MOVE.\n",
                            axis == 'x' ? 'X' : 'Y',dist);
                        Serial.println("[FLOW_GUIDED_V6_3H1] After READY, type: cal guided go  (30s window)");
                    } else {
                        Serial.println("[FLOW_GUIDED_V6_3H1] REJECT | CH5 low/CH6 high, sensor fresh, valid distance, no active session");
                    }
                } else if (strcmp(serial_line, "cal guided go") == 0) {
                    const bool safe_go = guided_authorized && in.imu_valid &&
                        yProbeRangeObservationOk(nav.height_valid, last_y_height_valid_ms, now) &&
                        sensor.last_flow_ms != 0 &&
                        (uint32_t)(now - sensor.last_flow_ms) < 150;
                    if (guided_cal.go(now, sensor.guided_cal_counts, safe_go)) {
                        Serial.printf("\n[FLOW_GUIDED_V6_3H1] GO ACCEPTED axis:%c | MOVE RIGHT for Y / FORWARD for X, %.2fm ONCE within 5 seconds; NO RETURN.\n",
                            guided_cal.axis(), guided_cal.plannedMeters());
                    } else {
                        Serial.println("[FLOW_GUIDED_V6_3H1] GO REJECTED: wait for READY, keep CH5 low/CH6 mode2 and sensors valid.");
                    }
                } else if (strcmp(serial_line, "xy zero") == 0) {
                    const auto b=xy_bench_input();
                    if (xy_observer.start(b)) {
                        xy_audit_begin = xy_audit_end = sensor.guided_cal_counts;
                        xy_audit_start_ms=xy_audit_end_ms=now;
                        xy_audit_valid=true;
                        xy_audit_tracking=true;
                        last_xy_print_ms=now;
                        Serial.println("[XY_BENCH_V2] ZERO OK | candidate matrix ONLY; move X/Y with PROP OFF; no motor authority");
                    } else {
                        xy_audit_valid=false;
                        xy_audit_tracking=false;
                        Serial.printf("[XY_BENCH_V2] ZERO REJECT reason:%s | CH5 low CH6 high, stable height 0.30..0.60m, no rotation\n",
                            v2xybench::reasonName(xy_observer.state().reason));
                    }
                } else if (strcmp(serial_line, "xy stop") == 0) {
                    // Finish full-trial audit even if XY Observer aborted earlier.
                    // Keep Observer failure state intact, never turn motors on.
                    if (xy_audit_valid && xy_audit_tracking) {
                        xy_audit_end=sensor.guided_cal_counts;
                        xy_audit_end_ms=now;
                        xy_audit_tracking=false;
                    }
                    xy_observer.stop();
                    const auto &r=xy_observer.state();
                    Serial.printf("[XY_BENCH_V2] STOP reason:%s x:%.3fm y:%.3fm frames:%lu raw:(%lld,%lld) | OBSERVE_ONLY\n",
                        v2xybench::reasonName(r.reason),r.x_m,r.y_m,(unsigned long)r.frames,
                        (long long)r.delta_fx,(long long)r.delta_fy);
                } else if (strcmp(serial_line, "xy status") == 0) {
                    const auto &r=xy_observer.state();
                    Serial.printf("[XY_BENCH_V2] STATUS active:%d reason:%s x:%.3fm y:%.3fm vx:%+.3fmps vy:%+.3fmps frames:%lu h:%.3fm raw:(%lld,%lld) | OBSERVE_ONLY\n",
                        r.active?1:0,v2xybench::reasonName(r.reason),r.x_m,r.y_m,
                        r.vx_mps,r.vy_mps,(unsigned long)r.frames,r.h_last_m,
                        (long long)r.delta_fx,(long long)r.delta_fy);
                    if (xy_audit_valid) {
                        const auto &last = xy_audit_tracking ? sensor.guided_cal_counts : xy_audit_end;
                        const uint32_t end_ms = xy_audit_tracking ? now : xy_audit_end_ms;
                        if (last.frames >= xy_audit_begin.frames &&
                            last.accepted >= xy_audit_begin.accepted &&
                            last.abs_x >= xy_audit_begin.abs_x &&
                            last.abs_y >= xy_audit_begin.abs_y &&
                            last.pos_x >= xy_audit_begin.pos_x &&
                            last.neg_x >= xy_audit_begin.neg_x &&
                            last.pos_y >= xy_audit_begin.pos_y &&
                            last.neg_y >= xy_audit_begin.neg_y) {
                            const auto d=subtractFlowCounters(last,xy_audit_begin);
                            Serial.printf("[XY_AUDIT_V2] capture:%s ms:%lu packets:%lu valid:%lu lowq:%lu spikes:%lu net:(%lld,%lld) abs:(%llu,%llu) X(+%lu,-%lu) Y(+%lu,-%lu) observer_raw:(%lld,%lld) | BENCH_ONLY\n",
                                xy_audit_tracking ? "LIVE" : "MANUAL_STOP",
                                (unsigned long)(end_ms-xy_audit_start_ms),
                                (unsigned long)d.frames,(unsigned long)d.accepted,
                                (unsigned long)d.low_quality,(unsigned long)d.spikes,
                                (long long)d.sum_x,(long long)d.sum_y,
                                (unsigned long long)d.abs_x,(unsigned long long)d.abs_y,
                                (unsigned long)d.pos_x,(unsigned long)d.neg_x,
                                (unsigned long)d.pos_y,(unsigned long)d.neg_y,
                                (long long)r.delta_fx,(long long)r.delta_fy);
                        } else {
                            Serial.println("[XY_AUDIT_V2] COUNTER_RESET | BENCH_ONLY");
                        }
                    }
                } else if (strcmp(serial_line, "cal guided reset") == 0) {
                    guided_cal.reset(); guided_quiet_until_ms = now + 2000UL;
                    Serial.println("[FLOW_GUIDED_V6_3H1] RESET | both guided axes cleared");
                } else if (strcmp(serial_line, "cal guided status") == 0) {
                    Serial.printf("[FLOW_GUIDED_V6_3H1] STATUS reason:%s X:(%.1f,%.1f) measured:%d Y:(%.1f,%.1f) measured:%d axis_sin:%.3f candidate:%d | BENCH_ONLY\n",
                        guidedReasonName(guided_cal.reason()),
                        guided_cal.x().fx_per_m,guided_cal.x().fy_per_m,guided_cal.x().measured?1:0,
                        guided_cal.y().fx_per_m,guided_cal.y().fy_per_m,guided_cal.y().measured?1:0,
                        guided_cal.axisSin(),guided_cal.axesCandidate()?1:0);
                } else if (sscanf(serial_line, "cal %c %f", &axis, &dist) == 2 &&
                    (axis == 'x' || axis == 'y')) {
                    if (flow_cal.begin(axis == 'x' ? 'X' : 'Y', dist,
                                       authorized_cal, sensor.last_flow_ms)) {
                        Serial.printf("[FLOW_CAL_CMD] START %c %.2fm | translate parallel to floor, keep height level\n",
                                      axis == 'x' ? 'X' : 'Y', dist);
                    } else {
                        Serial.printf("[FLOW_CAL_CMD] REJECT reason:%s | CH5 low, CH6 MODE2, distance 0.10..0.60m\n",
                            flowCalReasonName(flow_cal.state().reason));
                    }
                } else if (strcmp(serial_line, "flow ytest") == 0) {
                    const bool y_safe = authorized_cal && in.imu_valid && nav.height_valid &&
                        sensor.last_flow_ms != 0 && (uint32_t)(now - sensor.last_flow_ms) < 150;
                    if (y_probe.start(now, sensor.y_probe_counts, y_safe && !guided_cal.active())) {
                        Serial.println("[Y_PROBE_V6_3Y2] START phase:STILL 2000ms | DO NOT MOVE drone");
                        Serial.println("[Y_PROBE_V6_3Y2] Wait for MOVE_NOW; then move RIGHT 30cm in 3 seconds, no reverse");
                    } else {
                        Serial.printf("[Y_PROBE_V6_3Y2] REJECT rc_mode:%d imu:%d height:%d flow_age:%lums | CH5 low, CH6 MODE2, fresh IMU/MTF/range required\n",
                            authorized_cal ? 1 : 0, in.imu_valid ? 1 : 0, nav.height_valid ? 1 : 0,
                            (unsigned long)(sensor.last_flow_ms ? now - sensor.last_flow_ms : 0xffffffffUL));
                    }
                } else if (strcmp(serial_line, "cal stop") == 0) {
                    flow_cal.finish();
                    Serial.printf("[FLOW_CAL_CMD] STOP reason:%s samples:%lu sum_x:%.0f sum_y:%.0f candidate:%d\n",
                        flowCalReasonName(flow_cal.state().reason),
                        (unsigned long)flow_cal.state().accepted,
                        flow_cal.state().sum_fx, flow_cal.state().sum_fy,
                        flow_cal.state().axes_candidate ? 1 : 0);
                } else if (strcmp(serial_line, "cal reset") == 0) {
                    flow_cal.reset(); Serial.println("[FLOW_CAL_CMD] RESET");
                } else if (strcmp(serial_line, "cal status") == 0) {
                    Serial.printf("[FLOW_CAL_CMD] STATUS state:%s reason:%s\n",
                        flowCalStateName(flow_cal.state().state), flowCalReasonName(flow_cal.state().reason));
                } else if (serial_len > 0) {
                    Serial.println("[FLOW_CAL_CMD] Unknown. Use: xy zero/status/stop | cal guided x/y 0.30 | cal guided go | cal guided status/reset | flow ytest | cal x/y 0.30 | cal stop/status/reset");
                }
                serial_len = 0;
            }
            flow_cal.step(flow, nav.height_m, nav.gyro_norm_dps,
                          authorized_cal, sensor.last_flow_ms);
            // Y-only packet-rate observer; does NOT alter FlowCalibrator or X result.
            // A short fixed window avoids mixing long stationary dwell into a Y trial.
            // Do not abort an observation because of one short height-estimator
            // dropout (max 250ms). RC/disarm/mode, IMU and fresh Flow remain strict.
            const bool y_height_ok = yProbeRangeObservationOk(nav.height_valid, last_y_height_valid_ms, now);
            const bool y_flow_fresh = sensor.last_flow_ms != 0 &&
                (uint32_t)(now - sensor.last_flow_ms) < 150;
            const bool y_safe = authorized_cal && in.imu_valid && y_height_ok && y_flow_fresh;
            const YProbeEvent y_event = y_probe.update(now, sensor.y_probe_counts, y_safe);
            if (y_event == YProbeEvent::MOVE_NOW) {
                const auto &b = y_probe.still();
                Serial.printf("[Y_PROBE_V6_3Y2] STILL frames:%lu valid:%lu lowq:%lu spikes:%lu net:(%lld,%lld) abs:(%llu,%llu)\n",
                    (unsigned long)b.frames, (unsigned long)b.accepted,
                    (unsigned long)b.low_quality, (unsigned long)b.spikes,
                    (long long)b.sum_x, (long long)b.sum_y,
                    (unsigned long long)b.abs_x, (unsigned long long)b.abs_y);
                Serial.println("[Y_PROBE_V6_3Y2] MOVE_NOW | move drone RIGHT 30cm (Y+) in 3s, no turn, no return");
            } else if (y_event == YProbeEvent::WINDOW) {
                const auto &w = y_probe.window();
                Serial.printf("[Y_PROBE_V6_3Y2] WINDOW frames:%lu valid:%lu net:(%lld,%lld) abs:(%llu,%llu) +X:%lu -X:%lu +Y:%lu -Y:%lu\n",
                    (unsigned long)w.frames, (unsigned long)w.accepted,
                    (long long)w.sum_x, (long long)w.sum_y,
                    (unsigned long long)w.abs_x, (unsigned long long)w.abs_y,
                    (unsigned long)w.pos_x, (unsigned long)w.neg_x,
                    (unsigned long)w.pos_y, (unsigned long)w.neg_y);
            } else if (y_event == YProbeEvent::COMPLETE) {
                const auto &b = y_probe.still();
                const auto &m = y_probe.move();
                Serial.printf("[Y_PROBE_V6_3Y2] DONE motion_frames:%lu valid:%lu lowq:%lu spikes:%lu raw:(%lld,%lld) abs:(%llu,%llu) +X:%lu -X:%lu +Y:%lu -Y:%lu\n",
                    (unsigned long)m.frames, (unsigned long)m.accepted,
                    (unsigned long)m.low_quality, (unsigned long)m.spikes,
                    (long long)m.sum_x, (long long)m.sum_y,
                    (unsigned long long)m.abs_x, (unsigned long long)m.abs_y,
                    (unsigned long)m.pos_x, (unsigned long)m.neg_x,
                    (unsigned long)m.pos_y, (unsigned long)m.neg_y);
                Serial.printf("[Y_PROBE_V6_3Y2] COMPARE baseline:(%lld,%lld)/%lu corrected:(%.1f,%.1f) coherence:%.3f | OBSERVE_ONLY no XY calibration/flight\n",
                    (long long)b.sum_x, (long long)b.sum_y, (unsigned long)b.accepted,
                    y_probe.correctedX(), y_probe.correctedY(), y_probe.coherence());
            } else if (y_event == YProbeEvent::ABORT) {
                const char* gate = !authorized_cal ? "RC_DISARM_MODE" :
                    (!in.imu_valid ? "IMU_STALE" :
                    (!y_height_ok ? "HEIGHT_INVALID_OVER_250MS" :
                    (!y_flow_fresh ? "FLOW_STALE" : "OK")));
                Serial.printf("[Y_PROBE_V6_3Y2] ABORT reason:%s gate:%s rc_mode:%d imu:%d height:%d height_grace:%d flow_age:%lums frames:%lu valid:%lu | still<20 only checked after 2s\n",
                    yProbeAbortReasonName(y_probe.abortReason()), gate,
                    authorized_cal ? 1 : 0, in.imu_valid ? 1 : 0,
                    nav.height_valid ? 1 : 0, y_height_ok ? 1 : 0,
                    (unsigned long)(sensor.last_flow_ms ? now - sensor.last_flow_ms : 0xffffffffUL),
                    (unsigned long)sensor.y_probe_counts.frames,
                    (unsigned long)sensor.y_probe_counts.accepted);
            }
            // V6.3G: guided axis diagnostics use ALL decoded flow packets, not NAV's
            // last sampled packet. Fails closed, never enables xy_valid / PWM.
            const bool guided_safe = guided_authorized && in.imu_valid && y_height_ok && y_flow_fresh;
            const GuidedEvent ge = guided_cal.update(now, sensor.guided_cal_counts, guided_safe);
            if (ge == GuidedEvent::READY_FOR_GO) {
                const auto &b = guided_cal.still();
                Serial.printf("[FLOW_GUIDED_V6_3H1] BASELINE valid:%lu/%lu lowq:%lu abs:(%llu,%llu) net:(%lld,%lld) avg_abs:%.2f\n",
                    (unsigned long)b.accepted,(unsigned long)b.frames,(unsigned long)b.low_quality,
                    (unsigned long long)b.abs_x,(unsigned long long)b.abs_y,
                    (long long)b.sum_x,(long long)b.sum_y,guided_cal.stillAbsRate());
                Serial.printf("\n[FLOW_GUIDED_V6_3H1] READY axis:%c | baseline captured. NO timer running!\n",
                    guided_cal.axis());
                Serial.println("[FLOW_GUIDED_V6_3H1] Keep drone STILL, prepare 30cm path; then type: cal guided go");
                Serial.println("[FLOW_GUIDED_V6_3H1] You have 30 seconds to type GO, then 5 seconds to move.\n");
            } else if (ge == GuidedEvent::WINDOW) {
                const auto &w = guided_cal.window();
                Serial.printf("[FLOW_GUIDED_V6_3H1] WINDOW axis:%c valid:%lu net:(%lld,%lld) abs:(%llu,%llu) used:%lu reverse:%lu\n",
                    guided_cal.axis(),(unsigned long)w.accepted,(long long)w.sum_x,(long long)w.sum_y,
                    (unsigned long long)w.abs_x,(unsigned long long)w.abs_y,
                    (unsigned long)guided_cal.windowsUsed(),(unsigned long)guided_cal.windowsReverse());
            } else if (ge == GuidedEvent::COMPLETE) {
                guided_quiet_until_ms = now + 20000UL;
                const auto &m = guided_cal.allMove(), &u = guided_cal.selected();
                Serial.printf("[FLOW_GUIDED_V6_3H1] DONE axis:%c reason:%s packets:%lu valid:%lu lowq:%lu spikes:%lu all_net:(%lld,%lld) raw_coh:%.3f selected:%lu windows:%lu reject:%lu reverse:%lu net:(%lld,%lld) coh:%.3f candidate:%d xy_valid:0 PWM:0\n",
                    guided_cal.axis(),guidedReasonName(guided_cal.reason()),
                    (unsigned long)m.frames,(unsigned long)m.accepted,
                    (unsigned long)m.low_quality,(unsigned long)m.spikes,
                    (long long)m.sum_x,(long long)m.sum_y,guided_cal.rawCoherence(),
                    (unsigned long)u.accepted,(unsigned long)guided_cal.windowsUsed(),
                    (unsigned long)guided_cal.windowsRejected(),(unsigned long)guided_cal.windowsReverse(),
                    (long long)u.sum_x,(long long)u.sum_y,guided_cal.selectedCoherence(),
                    guided_cal.axesCandidate()?1:0);
                Serial.printf("[FLOW_GUIDED_V6_3H1] AXES X:(%.1f,%.1f) ok:%d Y:(%.1f,%.1f) ok:%d sin:%.3f | OBSERVE_ONLY\n",
                    guided_cal.x().fx_per_m,guided_cal.x().fy_per_m,guided_cal.x().measured?1:0,
                    guided_cal.y().fx_per_m,guided_cal.y().fy_per_m,guided_cal.y().measured?1:0,
                    guided_cal.axisSin());
            } else if (ge == GuidedEvent::ABORT) {
                guided_quiet_until_ms = now + 20000UL;
                Serial.printf("[FLOW_GUIDED_V6_3H1] ABORT axis:%c reason:%s rc_mode:%d imu:%d height:%d flow_fresh:%d | BENCH_ONLY\n",
                    guided_cal.axis(),guidedReasonName(guided_cal.reason()),
                    guided_authorized?1:0,in.imu_valid?1:0,y_height_ok?1:0,y_flow_fresh?1:0);
                Serial.printf("[FLOW_GUIDED_V6_3H1] RC_DIAG live:%d cached:%d age:%lums ch5:%d ch6:%d ctrl_mode:%d link:%d\n",
                    rc_read ? 1 : 0, guided_cached_rc_ok ? 1 : 0,
                    (unsigned long)rc_snapshot_age, guided_rc.ch[CH_ARM],
                    guided_rc.ch[CH_MODE], control.requested_mode,
                    guided_rc.link_ok ? 1 : 0);
            }
            // Candidate-matrix displacement path: separate from V2 POS estimator.
            const bool xy_was_active=xy_observer.state().active;
            const auto &xy_state=xy_observer.step(xy_bench_input());
            if (xy_was_active && !xy_state.active) {
                // Do not stop the packet audit here: user may still be sliding.
                Serial.printf("[XY_BENCH_V2] STOPPED reason:%s x:%.3fm y:%.3fm gyro:%.2fdps roll:%+.1f pitch:%+.1f | OBSERVE_ONLY\n",
                    v2xybench::reasonName(xy_state.reason),xy_state.x_m,xy_state.y_m,
                    nav.gyro_norm_dps,control.roll_deg,control.pitch_deg);
            } else if (xy_state.active && (uint32_t)(now-last_xy_print_ms)>=250) {
                last_xy_print_ms=now;
                Serial.printf("[XY_BENCH_V2] x:%+.3fm y:%+.3fm vx:%+.3fmps vy:%+.3fmps h:%.3fm frames:%lu | CANDIDATE_ONLY\n",
                    xy_state.x_m,xy_state.y_m,xy_state.vx_mps,xy_state.vy_mps,
                    xy_state.h_last_m,(unsigned long)xy_state.frames);
            }
            // Convert accumulated per-packet counts only with a separately
            // VERIFIED matrix. For the user's present setup this is fail-closed.
            v63flow::Sample pos_sample{};
            pos_sample.now_ms = now;
            pos_sample.last_frame_ms = sensor.last_flow_ms;
            pos_sample.good_frames = sensor.flow_good_frames;
            pos_sample.sum_fx = sensor.flow_sum_x;
            pos_sample.sum_fy = sensor.flow_sum_y;
            pos_sample.quality = sensor.flow_quality;
            pos_sample.height_m = nav.height_m;
            pos_sample.height_valid = nav.height_valid;
            pos_sample.flow_fresh = sensor.flow_valid && sensor.last_flow_ms &&
                (uint32_t)(now - sensor.last_flow_ms) <= MTF_STALE_MS;
            pos_sample.roll_rate_dps = control.gyro_x_dps;
            pos_sample.pitch_rate_dps = control.gyro_y_dps;
            pos_sample.yaw_rate_dps = control.gyro_z_dps;
            const v63flow::Velocity &velocity = pos_velocity.step(pos_sample);

            v63hold::Input pos_input{};
            pos_input.now_ms = now;
            // While armed in flight candidate, observe POS calculations without
            // ever writing its corrections into motor mixer.
            const bool shadowWhileArmed = ENABLE_MOTOR_OUTPUT &&
                rc_fresh && control.armed_logic && in.imu_valid;
            pos_input.requested_mode = shadowWhileArmed ? control.requested_mode : mode;
            pos_input.authorized = disarmed_bench || shadowWhileArmed;
            // In flight, POS must share the proven live-ALT height authority.
            // Disarmed guided calibration retains NAV shadow source unchanged.
            pos_input.height_valid = shadowWhileArmed ? control.alt_live : nav.height_valid;
            pos_input.range_age_ms = shadowWhileArmed ?
                (sensor.last_range_ms ? (uint32_t)(now-sensor.last_range_ms) : UINT32_MAX) : nav.range_age_ms;
            pos_input.h_m = shadowWhileArmed ? control.alt_height_m : nav.height_m;
            pos_input.vz_mps = shadowWhileArmed ? control.alt_vz_mps : nav.climb_mps;
            pos_input.roll_deg = control.roll_deg;
            pos_input.pitch_deg = control.pitch_deg;
            pos_input.yaw_rate_dps = control.gyro_z_dps;
            if (rc_fresh) {
                pos_input.throttle_us = rc.ch[CH_THROTTLE];
                pos_input.roll_us = rc.ch[CH_ROLL];
                pos_input.pitch_us = rc.ch[CH_PITCH];
            }
            pos_input.flow_verified = POS_FLOW_SCALE_INDEPENDENTLY_VERIFIED;
            pos_input.flow_valid = velocity.valid;
            pos_input.flow_age_ms = velocity.sample_age_ms;
            pos_input.vx_body_mps = velocity.vx_mps;
            pos_input.vy_body_mps = velocity.vy_mps;
            const v63hold::Output &pos = pos_hold.step(pos_input);
            const FlowCalOutput &fc = flow_cal.state();
            if (mtx_nav && xSemaphoreTake(mtx_nav, 0) == pdTRUE) {
                shared_nav.height_valid = nav.height_valid;
                // XY telemetry requires an independently verified matrix.
                // Without it, stays 0 and can NEVER command the motors.
                shared_nav.xy_valid = POS_FLOW_SCALE_INDEPENDENTLY_VERIFIED &&
                    velocity.valid && pos.active_mode == MODE_POS_HOLD;
                shared_nav.flow_fresh = flow.frame_fresh;
                shared_nav.motion_clear = nav.motion_clear;
                shared_nav.acc_norm_g = nav.acc_norm_g;
                shared_nav.gyro_norm_dps = nav.gyro_norm_dps;
                shared_nav.flow_cal_ready = flow.calibration_ready;
                shared_nav.flow_diag_reason = static_cast<uint8_t>(flow.reason);
                shared_nav.flow_age_ms = flow.frame_age_ms;
                shared_nav.flow_diag_quality = flow.quality;
                shared_nav.flow_diag_x_raw = flow.x_raw;
                shared_nav.flow_diag_y_raw = flow.y_raw;
                shared_nav.flow_ready_samples = flow.ready_samples;
                shared_nav.flow_rejects = flow.rejected_samples;
                shared_nav.height_m = nav.height_m;
                shared_nav.climb_mps = nav.climb_mps;
                shared_nav.raw_vertical_m = nav.range_vertical_m;
                shared_nav.tilt_compensation = nav.tilt_compensation;
                shared_nav.vx_body_mps = shared_nav.xy_valid ? velocity.vx_mps : 0.0f;
                shared_nav.vy_body_mps = shared_nav.xy_valid ? velocity.vy_mps : 0.0f;
                // Never publish unverified flow as real ground velocity.
                shared_nav.pos_reason = static_cast<uint8_t>(pos.reason);
                shared_nav.pos_active_mode = static_cast<uint8_t>(pos.active_mode);
                shared_nav.pos_flow_verified = POS_FLOW_SCALE_INDEPENDENTLY_VERIFIED;
                shared_nav.pos_flow_valid = velocity.valid;
                shared_nav.pos_flow_frames = velocity.frames;
                shared_nav.pos_flow_age_ms = velocity.sample_age_ms;
                shared_nav.pos_x_m = pos.estimated_x_m;
                shared_nav.pos_y_m = pos.estimated_y_m;
                shared_nav.pos_target_x_m = pos.target_x_m;
                shared_nav.pos_target_y_m = pos.target_y_m;
                shared_nav.pos_roll_target_deg = pos.roll_target_deg;
                shared_nav.pos_pitch_target_deg = pos.pitch_target_deg;
                shared_nav.height_reason = static_cast<uint8_t>(nav.reason);
                shared_nav.range_age_ms = nav.range_age_ms;
                shared_nav.warmup_count = nav.warmup_count;
                shared_nav.altitude_target_m = sh.target_m;
                shared_nav.shadow_alt_correction = sh.normalized_correction;
                shared_nav.shadow_target_climb_mps = sh.desired_climb_mps;
                shared_nav.shadow_velocity_error_mps = sh.velocity_error_mps;
                shared_nav.shadow_i_term = sh.i_term;
                shared_nav.shadow_state = static_cast<uint8_t>(sh.state);
                shared_nav.target_capture_reason = static_cast<uint8_t>(sh.last_capture_reason);
                shared_nav.target_capture_gate = static_cast<uint8_t>(sh.gate);
                shared_nav.target_capture_count = sh.capture_count;
                shared_nav.target_rejected_count = sh.capture_reject_count;
                shared_nav.target_stable_ms = sh.stable_ms;
                shared_nav.target_stable_samples = sh.stable_samples;
                shared_nav.target_last_capture_window_ms = sh.last_capture_window_ms;
                shared_nav.target_last_capture_window_samples = sh.last_capture_window_samples;
                shared_nav.flow_cal_state = static_cast<uint8_t>(fc.state);
                shared_nav.flow_cal_reason = static_cast<uint8_t>(fc.reason);
                shared_nav.flow_axes_candidate = fc.axes_candidate;
                shared_nav.flow_cal_accepted = fc.accepted;
                shared_nav.flow_cal_rejected = fc.rejected;
                shared_nav.flow_cal_planned_m = fc.planned_m;
                shared_nav.flow_cal_sum_x = fc.sum_fx;
                shared_nav.flow_cal_sum_y = fc.sum_fy;
                shared_nav.flow_cal_x_fx = fc.x_counts_per_m_fx;
                shared_nav.flow_cal_x_fy = fc.x_counts_per_m_fy;
                shared_nav.flow_cal_y_fx = fc.y_counts_per_m_fx;
                shared_nav.flow_cal_y_fy = fc.y_counts_per_m_fy;
                shared_nav.flow_cal_determinant = fc.determinant;
                shared_nav.last_outlier_h_m = nav.last_outlier_h_m;
                shared_nav.last_outlier_delta_m = nav.last_outlier_delta_m;
                shared_nav.outlier_count = nav.outlier_count;
                shared_nav.range_samples = nav.accepted_samples;
                shared_nav.rejects = nav.rejected_samples;
                shared_nav.reacquisitions = nav.warmups_completed;
                shared_nav.last_calc_ms = now;
                xSemaphoreGive(mtx_nav);
            }
        }
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(20)); // 50 Hz; no motor output
    }
}
