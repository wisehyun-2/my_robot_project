/*
 * Common software-in-the-loop (SIL) interface for balance controllers.
 * Every controller is compiled to its own shared library exposing these three functions.
 * The Python harness (sil_harness.py) loads the .so with ctypes and calls ctrl_step() every Ts.
 *
 * Units: SI.  Sign conventions (same as sim/twip_model.py and the Teensy firmware):
 *   theta      body pitch, + = leaning forward            [rad]
 *   theta_dot  pitch rate (gyro)                           [rad/s]
 *   x_dot      forward speed of the wheel axle (avg of both wheels)   [m/s]
 *   psi_dot    yaw rate (gyro)                             [rad/s]
 *   leg_len    axle -> body COM distance (changes with hip angle)      [m]
 *   wheel_r    wheel radius                                [m]
 *
 * Output: out[0] = left wheel command, out[1] = right wheel command.
 *   CTRL_OUT_TORQUE   : commands are wheel torques [N m] (direct-drive torque mode)
 *   CTRL_OUT_WHEELVEL : commands are wheel angular velocity references [rad/s]
 *                       (projects that drive a velocity-mode motor driver, e.g. ODrive input_vel)
 */
#ifndef CTRL_API_H
#define CTRL_API_H
#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float theta, theta_dot, x_dot, psi_dot, leg_len, wheel_r;
    float v_ref, yaw_rate_ref;
    float t;            /* time [s] */
} ctrl_in_t;

enum { CTRL_OUT_TORQUE = 0, CTRL_OUT_WHEELVEL = 1 };

/* robot parameters some controllers need (LQR designed online, gain scheduling) */
typedef struct {
    float M_b, m_w, r, l, I_b, I_w, tau_max;
} ctrl_params_t;

void ctrl_init(float dt, const ctrl_params_t* p);
int  ctrl_output_type(void);
void ctrl_step(const ctrl_in_t* in, float out[2]);
const char* ctrl_name(void);

#ifdef __cplusplus
}
#endif
#endif
