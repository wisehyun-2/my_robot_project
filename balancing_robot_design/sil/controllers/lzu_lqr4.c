/*
 * lzu_lqr4.c -- port of hpzhaozhao/LZU_ASCENTO  lzu_ascento/Core/Src/task/chassis_task.c  (STM32H723, 1 kHz)
 *   Hardware: 2 DM-H6215 hub wheels (MIT torque) + 2 Stackforce 5 N m joints, Ascento four-bar -> 4 motors.
 *   Law (verbatim): per wheel  T = k0*(x_set-x) + k1*(v_set-v) + k2*(phi_set-pitch) + k3*(dphi_set-gyro)
 *   with k0..k3 = cubic polynomials of the leg length (Poly_Coefficient[][4], from LQR_k_calc.m), saturate +-1.2 N m,
 *   v = 10-sample moving average of wheel speed, x = integral of v (1 ms), phi_set = -0.7833*L + 0.3011 (their COM offset).
 *   Their robot: wheel r = 0.06 m, leg 0.07..0.30 m.  To run on another robot the output scale is searched; the gain
 *   ratios and the leg-length scheduling are theirs.  phi_set is set to 0 (our model's equilibrium is theta = 0).
 */
#include <math.h>
#include "../ctrl_api.h"
static float DT, SCALE = 1.0f, x = 0, x_set = 0, averr[10] = {0};
static const float Poly[4][4] = {
    {1.7157e-12f, -9.9069e-13f, 1.8819e-13f, -0.10541f},
    {5.8329f, -4.5708f, 1.2674f, -0.37987f},
    {-30.1102f, 27.0942f, -10.3735f, -1.4992f},
    {2.227f, -1.8923f, -0.084791f, -0.33836f}};
static float Kcalc(const float* c, float L) { return c[0]*L*L*L + c[1]*L*L + c[2]*L + c[3]; }
void ctrl_init(float dt, const ctrl_params_t* p) { (void)p; DT = dt; x = x_set = 0; for (int i = 0; i < 10; i++) averr[i] = 0; }
void ctrl_set_scale(float s) { SCALE = s; }
int ctrl_output_type(void) { return CTRL_OUT_TORQUE; }
const char* ctrl_name(void) { return "LZU_ASCENTO: 1 kHz 4-state LQR per wheel, gains cubic in leg length, 10-sample speed MA"; }
void ctrl_step(const ctrl_in_t* in, float out[2]) {
    for (int i = 0; i < 9; i++) averr[i] = averr[i+1];
    averr[9] = in->x_dot; float v = 0; for (int i = 0; i < 10; i++) v += 0.1f * averr[i];
    x += v * DT;
    if (fabsf(in->v_ref) > 1e-3f) x_set = x;                     /* moving: no position hold (their x_set follows) */
    float L = in->leg_len; if (L < 0.07f) L = 0.07f; if (L > 0.30f) L = 0.30f;
    float k0 = Kcalc(Poly[0], L), k1 = Kcalc(Poly[1], L), k2 = Kcalc(Poly[2], L), k3 = Kcalc(Poly[3], L);
    float T = k0*(x_set - x) + k1*(in->v_ref - v) + k2*(0.0f - in->theta) + k3*(0.0f - in->theta_dot);
    if (T > 1.2f) T = 1.2f; if (T < -1.2f) T = -1.2f;             /* their per-wheel saturation */
    out[0] = SCALE * T; out[1] = SCALE * T;
}
