/*
 * upkie_torque_p.c -- port of upkie/upkie  examples/pybullet/torque_balancing.py  (Upkie-PyBullet-Servos, 200 Hz)
 *   feedforward_torque = +GAIN * pitch per wheel, GAIN = 10 N m/rad, wheel kd_scale = 0 (pure torque).  No damping,
 *   no wheel feedback.  Torque units -> run AS-IS and also with the harness scale search.
 */
#include "../ctrl_api.h"
static float SCALE = 1.0f;
void ctrl_init(float dt, const ctrl_params_t* p) { (void)dt; (void)p; }
void ctrl_set_scale(float s) { SCALE = s; }
int ctrl_output_type(void) { return CTRL_OUT_TORQUE; }
const char* ctrl_name(void) { return "upkie torque_balancing.py (200 Hz): tau_wheel = 10 Nm/rad * pitch (P only)"; }
void ctrl_step(const ctrl_in_t* in, float out[2]) { float t = SCALE * 10.0f * in->theta; out[0] = t; out[1] = t; }
