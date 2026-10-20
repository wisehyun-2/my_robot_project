/*
 * pow_lqr4.c -- port of SeungbinOh/Pow_WBR_Project  ESP32/WBR_Control/VYBController.h + WBR_Control.ino (79 stars)
 *   Hardware: 2 RS485 torque motors (LK/MG 0xA1 current command) + 2 hip servos -> 4 motors.  dt = 8 ms (125 Hz).
 *   Law (verbatim): u = K(h) * (x_d - x),  x = [theta, theta_dot, v, yaw_rate], K(h) 2x4 interpolated from a table of 14
 *   gain matrices every 10 mm of height from HEIGHT_MIN = 0.07 m, u saturated to +-MAX_TORQUE = 0.75 N m per wheel.
 *   Row 2 (left wheel) is the negative of row 1 for the first three states (reversed wiring); we use row 1 for both
 *   wheels (symmetric torque) and drop the yaw term (yaw_rate_ref = 0).  Output scale searched for our robot.
 */
#include <math.h>
#include "../ctrl_api.h"
static const float Ks[14][4] = {
    {1.35316904f, 0.14009245f, 0.23358589f, -0.12161902f}, {1.40047216f, 0.14638508f, 0.23369175f, -0.12160757f},
    {1.44602994f, 0.15279748f, 0.23382116f, -0.12158976f}, {1.48968650f, 0.15931475f, 0.23397350f, -0.12156787f},
    {1.53149557f, 0.16592279f, 0.23414424f, -0.12154222f}, {1.57156576f, 0.17260877f, 0.23432853f, -0.12151281f},
    {1.61001973f, 0.17936153f, 0.23452203f, -0.12147964f}, {1.64697959f, 0.18617179f, 0.23472118f, -0.12144289f},
    {1.68256117f, 0.19303219f, 0.23492316f, -0.12140290f}, {1.71687199f, 0.19993741f, 0.23512588f, -0.12136017f},
    {1.75001117f, 0.20688435f, 0.23532789f, -0.12131532f}, {1.78207150f, 0.21387275f, 0.23552830f, -0.12126915f},
    {1.81314599f, 0.22090653f, 0.23572687f, -0.12122289f}, {1.84334794f, 0.22799757f, 0.23592427f, -0.12117958f}};
static float SCALE = 1.0f;
void ctrl_init(float dt, const ctrl_params_t* p) { (void)dt; (void)p; }
void ctrl_set_scale(float s) { SCALE = s; }
int ctrl_output_type(void) { return CTRL_OUT_TORQUE; }
const char* ctrl_name(void) { return "Pow_WBR (SeungbinOh): 125 Hz 4-state LQR u=K(h)(x_d-x), K table vs height, +-0.75 N m"; }
void ctrl_step(const ctrl_in_t* in, float out[2]) {
    float h = in->leg_len; float t = (h - 0.07f) / 0.01f; int idx = (int)floorf(t); float K[4];
    if (idx >= 0 && idx < 13) { float r = t - idx; for (int i = 0; i < 4; i++) K[i] = Ks[idx][i]*(1-r) + Ks[idx+1][i]*r; }
    else if (idx < 0) { for (int i = 0; i < 4; i++) K[i] = Ks[0][i]; } else { for (int i = 0; i < 4; i++) K[i] = Ks[13][i]; }
    float u = K[0]*(0.0f - in->theta) + K[1]*(0.0f - in->theta_dot) + K[2]*(in->v_ref - in->x_dot) + K[3]*(0.0f - in->psi_dot);
    if (u > 0.75f) u = 0.75f; if (u < -0.75f) u = -0.75f;
    out[0] = SCALE * u; out[1] = SCALE * u;
}
