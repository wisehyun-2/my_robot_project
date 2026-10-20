/*
 * zephyr_lqr4.c -- port of 123zhengzz/zephyr_ascento_f407_wheel_leg  src/app/control/ascento_balance.c  (~1 kHz)
 *   Hardware: 2 M3508 wheels via VESC (current) + 2 DM4340 hips, four-bar -> 4 motors.  Robot 2.315 kg, r = 0.03 m,
 *   leg 0.064..0.205 m (docs/ROBOT_PARAMETERS_ZH.md).
 *   Law (verbatim): balance_torque = -(k_pitch*(pitch - theta_eq - target) + k_pitch_rate*pitch_rate
 *                                      + k_position*pos_err + k_velocity*(v - v_target))
 *   k_pitch, k_pitch_rate, k_velocity quadratic in leg length (recompute_lqr_gains.m), k_position constant:
 *     K0 = -59.40269692 L^2 + 17.93409573 L - 8.962238054      (stand: -7.67)
 *     K1 = -54.51745738 L^2 + 18.40135485 L - 2.812553242      (stand: -1.40)
 *     K2 = -3.464101615 (code) / +3.464102 (stand table)        K3 = -39.73239567 L^2 + 14.45755025 L - 4.034081544 (stand: +2.88 in table)
 *   velocity LPF tau 0.012 s (13 Hz); pitch-rate LPF same; stiction compensation (800 mA ramp 0.5..3 deg) omitted
 *   (its current->torque constant was not fetched).  NOTE the docs' stand-gain table has K_position/K_velocity POSITIVE
 *   while the schedule polynomials give them NEGATIVE; we use the polynomials (what the code evaluates) and the
 *   harness also runs a variant with the velocity/position sign flipped.  Output scale searched for our robot.
 */
#include <math.h>
#include <string.h>
#include "../ctrl_api.h"
static float DT, SCALE = 1.0f, x = 0, x_set = 0, v_f = 0, rate_f = 0; static int first = 1; static float VSIGN = 1.0f;
static float quad(float a, float b, float c, float L) { return a*L*L + b*L + c; }
void ctrl_init(float dt, const ctrl_params_t* p) { (void)p; DT = dt; x = x_set = v_f = rate_f = 0; first = 1; }
void ctrl_set_scale(float s) { SCALE = s; }
void ctrl_set_param(const char* n, float v) { if (!strcmp(n, "vsign")) VSIGN = v; }
int ctrl_output_type(void) { return CTRL_OUT_TORQUE; }
const char* ctrl_name(void) { return "zephyr_ascento_f407: 1 kHz 4-state LQR, gains quadratic in leg length, 13 Hz velocity LPF"; }
void ctrl_step(const ctrl_in_t* in, float out[2]) {
    float a = DT / (0.012f + DT);
    if (first) { v_f = in->x_dot; rate_f = in->theta_dot; first = 0; }
    v_f += a * (in->x_dot - v_f); rate_f += a * (in->theta_dot - rate_f);
    x += v_f * DT; if (fabsf(in->v_ref) > 1e-3f) x_set = x;
    float L = in->leg_len; if (L < 0.064f) L = 0.064f; if (L > 0.25f) L = 0.25f;
    float k_pitch = quad(-59.40269692f, 17.93409573f, -8.962238054f, L);
    float k_rate  = quad(-54.51745738f, 18.40135485f, -2.812553242f, L);
    float k_pos   = -3.464101615f * VSIGN;
    float k_vel   = quad(-39.73239567f, 14.45755025f, -4.034081544f, L) * VSIGN;
    float tau = -(k_pitch * in->theta + k_rate * rate_f + k_pos * (x - x_set) + k_vel * (v_f - in->v_ref));
    out[0] = 0.5f * SCALE * tau; out[1] = 0.5f * SCALE * tau;
}
