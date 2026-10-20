/*
 * upkie_pd.c -- port of upkie/upkie  examples/pybullet/pd_balancing.py  (Upkie-PyBullet-Pendulum env, frequency = 1000)
 *   action (commanded ground velocity, m/s) = clip(10.0*pitch + 1.0*ground_position + 0.1*ground_velocity, +-0.99)
 *   No integral.  Robot-agnostic velocity units -> run AS-IS.  Output: wheel velocity reference = action / r.
 */
#include <math.h>
#include "../ctrl_api.h"
static float DT, wheel_r, x = 0;
void ctrl_init(float dt, const ctrl_params_t* p) { DT = dt; wheel_r = p->r; x = 0; }
int ctrl_output_type(void) { return CTRL_OUT_WHEELVEL; }
const char* ctrl_name(void) { return "upkie pd_balancing.py (1 kHz): v_cmd = 10*pitch + 1*x + 0.1*xdot, |v| <= 0.99 m/s"; }
void ctrl_step(const ctrl_in_t* in, float out[2]) {
    x += in->x_dot * DT;
    float a = 10.0f * in->theta + 1.0f * x + 0.1f * in->x_dot;
    if (a > 0.99f) a = 0.99f; if (a < -0.99f) a = -0.99f;
    float w = (a + in->v_ref) / wheel_r;      /* example has no velocity command; add v_ref as feed-forward for the step test */
    out[0] = w; out[1] = w;
}
