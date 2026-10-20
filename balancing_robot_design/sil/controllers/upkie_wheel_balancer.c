/*
 * upkie_wheel_balancer.c -- port of upkie/upkie  upkie/cpp/controllers/WheelBalancer.cpp  (v13.1.0, Apache-2.0)
 *   The "wheel_balancer" controller pipeline that runs INSIDE the spine at 1 kHz on the real robot (Raspberry Pi +
 *   pi3hat, moteus mj5208 wheels in velocity mode, hip/knee position-controlled).  Robot: 6 motors (hip+knee+wheel
 *   per leg), ~5.4 kg, wheel radius 0.06 m, COM height ~0.58 m.
 *   Law (verbatim):
 *     error = [x_target - x, 0 - pitch]
 *     integral_velocity += (position_stiffness*e_x + pitch_stiffness*e_pitch) * dt,  |.| <= 10 m/s   (floor contact)
 *     x_target += v_target*dt, clamped within +-1 m of x
 *     ground_velocity = -v_target - (position_damping*e_x + pitch_damping*e_pitch) - integral_velocity, |.| <= 2 m/s
 *     wheel velocity = ground_velocity / r   (sent to the moteus velocity loops)
 *   gains: pitch_damping 1.8, pitch_stiffness 20, position_damping 0.7, position_stiffness 1.6  (m/s per rad, m/s^2 per rad ...)
 *   The "non-minimum phase trick": the commanded velocity is -v_target so that the integral has to lean the body first.
 *   Because the law is written in ground-velocity units it is robot-agnostic -> run AS-IS, no scale search.
 *   Output type: wheel velocity reference (harness simulates the moteus velocity loop).
 */
#include <math.h>
#include "../ctrl_api.h"
static const float pitch_damping = 1.8f, pitch_stiffness = 20.0f, position_damping = 0.7f, position_stiffness = 1.6f;
static const float max_ground_velocity = 2.0f, kMaxIntegralVelocity = 10.0f, kMaxTargetDistance = 1.0f, fall_pitch = 1.0f;
static float DT, wheel_r, x = 0, x_target = 0, integral_velocity = 0;
void ctrl_init(float dt, const ctrl_params_t* p) { DT = dt; wheel_r = p->r; x = x_target = integral_velocity = 0; }
int ctrl_output_type(void) { return CTRL_OUT_WHEELVEL; }
const char* ctrl_name(void) { return "upkie WheelBalancer (spine, 1 kHz): velocity PI on pitch+position -> moteus wheel velocity"; }
void ctrl_step(const ctrl_in_t* in, float out[2]) {
    x += in->x_dot * DT;                                   /* wheel odometry position */
    float v_target = in->v_ref;
    if (fabsf(in->theta) > fall_pitch) { out[0] = out[1] = 0; return; }
    float e_x = x_target - x, e_p = 0.0f - in->theta;
    integral_velocity += (position_stiffness * e_x + pitch_stiffness * e_p) * DT;
    if (integral_velocity > kMaxIntegralVelocity) integral_velocity = kMaxIntegralVelocity;
    if (integral_velocity < -kMaxIntegralVelocity) integral_velocity = -kMaxIntegralVelocity;
    x_target += v_target * DT;
    if (x_target > x + kMaxTargetDistance) x_target = x + kMaxTargetDistance;
    if (x_target < x - kMaxTargetDistance) x_target = x - kMaxTargetDistance;
    float trick_velocity = -v_target;
    float gv = trick_velocity - (position_damping * e_x + pitch_damping * e_p) - integral_velocity;
    if (gv > max_ground_velocity) gv = max_ground_velocity; if (gv < -max_ground_velocity) gv = -max_ground_velocity;
    float w = gv / wheel_r;
    out[0] = w; out[1] = w;
}
