/*
 * ryung_pid.c -- faithful port of the balance controller in
 *   Ryung-coding/ICT2024_surveillance_balancing_robot/src/controller/src/controller_main.cpp (+ controller_main.h/_function.h)
 * Structure (verbatim from the source):
 *   PID0: ref_theta     = PID(0, ref[1](=stick), 0)            <- "velocity loop" fed by the stick, no wheel feedback
 *   PID1: ref_theta_dot = PID(ref_theta + 0.1164, imu_theta, imu_theta_dot)
 *   PID2: balancing_CMD = PID(ref_theta_dot, imu_theta_dot, imu_theta_ddot)   (theta_ddot = LPF(0.9) of numeric diff)
 *   Motor cmd = -balancing_CMD (CW->CCW), clamp +-Lim_INPUT (=20 rev/s), sent to ODrive in VELOCITY mode.
 * Gains Kp/Kd/Ki, loop_hz=100, anti-windup 2, LPF weights are the committed values.
 * Output type = wheel velocity reference [rad/s]  (20 rev/s * 2pi).  The driver's velocity loop is simulated by the harness.
 * Only the "stick" input differs: we map v_ref [m/s] -> stick % with 100 % = 1 m/s  (their ref[1] is a stick percentage).
 */
#include <math.h>
#include "../ctrl_api.h"

static float Kp[4] = {-0.002f, 16.50f, 4.380f, 0.0000f};
static float Kd[4] = {0.0000f, 0.330f, 0.100f, 0.0000f};
static float Ki[4] = {-0.000005f, 0.0080f, 0.0000f, 0.0000f};
#define anti_windup_gain 2.0f
#define Lim_INPUT 20.0f
#define RevPerS2RadPerS 6.283185307f
#ifndef THETA_OFFSET
#define THETA_OFFSET 0.0f
#endif
static float I[4], dt, imu_theta_dot_past, imu_theta_ddot, imu_theta_f, imu_theta_dot_f;

static float lowpassfilter(float filter, float data, float past_weight) { return data*(1-past_weight) + filter*past_weight; }

static float computePID(float r, float y, float y_dot, float dtl, int c) {
    float error = r - y;
    float Pq = Kp[c]*error;
    I[c] += Ki[c]*error*dtl;
    if (fabsf(I[c]) > anti_windup_gain) I[c] = I[c] > 0 ? anti_windup_gain : -anti_windup_gain;
    float D = Kd[c]*(0 - y_dot);
    return Pq + I[c] + D;
}

void ctrl_init(float dtl, const ctrl_params_t* p) { (void)p; dt = dtl; for (int i = 0; i < 4; i++) I[i] = 0; imu_theta_dot_past = imu_theta_ddot = imu_theta_f = imu_theta_dot_f = 0; }
int ctrl_output_type(void) { return CTRL_OUT_WHEELVEL; }
const char* ctrl_name(void) { return "Ryung ICT2024: 3-loop PID -> ODrive velocity cmd (no wheel feedback)"; }

void ctrl_step(const ctrl_in_t* in, float out[2]) {
    /* imu_callback: lowpass with past_weight 0.001 (i.e. none) */
    imu_theta_f = lowpassfilter(imu_theta_f, in->theta, 0.001f);
    imu_theta_dot_f = lowpassfilter(imu_theta_dot_f, in->theta_dot, 0.001f);
    float stick = in->v_ref * 100.0f;                        /* ref[1] is a stick percentage */
    float ref_theta = computePID(0, stick, 0, dt, 0);
    /* +0.1164 rad in the original = calibration of THEIR mechanical COM offset; the sim's equilibrium is theta = 0 */
    float ref_theta_dot = computePID(ref_theta + THETA_OFFSET, imu_theta_f, imu_theta_dot_f, dt, 1);
    imu_theta_ddot = lowpassfilter(imu_theta_ddot, (imu_theta_dot_f - imu_theta_dot_past)/dt, 0.9f);
    float balancing_CMD = computePID(ref_theta_dot, imu_theta_dot_f, imu_theta_ddot, dt, 2);
    imu_theta_dot_past = imu_theta_dot_f;
    float heading_CMD = 0;                                   /* heading loop uses yaw angle; ignored (yaw_rate_ref=0) */
    float L = balancing_CMD - heading_CMD, R = balancing_CMD + heading_CMD;
    L = -L; R = -R;                                          /* CW->CCW */
    if (fabsf(L) > Lim_INPUT) L = L > 0 ? Lim_INPUT : -Lim_INPUT;
    if (fabsf(R) > Lim_INPUT) R = R > 0 ? Lim_INPUT : -Lim_INPUT;
    /* NOTE: their sign convention for theta/motor is unknown; the harness tries both signs and keeps the stable one */
    out[0] = L * RevPerS2RadPerS; out[1] = R * RevPerS2RadPerS;
}
