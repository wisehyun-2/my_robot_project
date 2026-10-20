/*
 * wl1_cascade.c -- port of lk888l/wheeled-legged_Robot-WL1  car_firmware/.../MotionControlTask.cpp
 *   Hardware: 2 DC wheels (TB6612, PWM +-1000) + 2 PWM servos (one per leg, four-bar) -> 4 motors.
 *   Structure (verbatim):
 *     10 ms attitude loop:  even_pwm = angle_pid.update(angle_target, pitch_deg + angle_bias)   PID(70, 0, 51)
 *     50 ms velocity loop:  angle_target = velocity_pid.update(velocity_target, average_rpm)   PID(0.04, 0.006, 0), out +-10 deg
 *     angle_bias = pitchBias(height): quadratic in leg height (balance-point compensation vs. leg geometry)
 *     left/right pwm = even_pwm +- difference_pwm, clamp +-1000
 *   The repo's PID.hpp body was not in the dump; assumed classic positional form with per-tick difference:
 *     out = Kp*e + Ki*sum(e) + Kd*(e - e_prev), e = target - measured, integral clamp +-100, output clamp.
 *   PWM -> torque scale is the only free parameter (ctrl_set_scale); gains/ratios/rates are theirs.
 */
#include <math.h>
#include "../ctrl_api.h"

typedef struct { float kp, ki, kd, omin, omax, imin, imax, isum, eprev; } pid_t;
static pid_t angle_pid = {70.0f, 0, 51.0f, -1000, 1000, -100, 100, 0, 0};
static pid_t velocity_pid = {0.04f, 0.006f, 0, -10, 10, -100, 100, 0, 0};
static float DT, SCALE = 1.0f, wheel_r, angle_target = 0, even_pwm = 0; static int tick = 0;

static float pid_update(pid_t* p, float target, float meas) {
    float e = target - meas;
    p->isum += e; if (p->isum > p->imax) p->isum = p->imax; if (p->isum < p->imin) p->isum = p->imin;
    float out = p->kp * e + p->ki * p->isum + p->kd * (e - p->eprev);
    p->eprev = e;
    if (out > p->omax) out = p->omax; if (out < p->omin) out = p->omin;
    return out;
}
void ctrl_init(float dt, const ctrl_params_t* p) { DT = dt; wheel_r = p->r; tick = 0; angle_target = 0; even_pwm = 0;
    angle_pid.isum = angle_pid.eprev = velocity_pid.isum = velocity_pid.eprev = 0; }
void ctrl_set_scale(float s) { SCALE = s; }
int ctrl_output_type(void) { return CTRL_OUT_TORQUE; }
const char* ctrl_name(void) { return "WL1 (lk888l): cascade PID, 10 ms angle PD(70,51) / 50 ms velocity PI(0.04,0.006), PWM out"; }

void ctrl_step(const ctrl_in_t* in, float out[2]) {       /* called every 10 ms */
    if (++tick >= 5) {                                     /* 50 ms velocity loop */
        tick = 0;
        float rpm = in->x_dot / wheel_r * 60.0f / 6.2831853f;
        float rpm_target = in->v_ref / wheel_r * 60.0f / 6.2831853f;
        angle_target = pid_update(&velocity_pid, rpm_target, rpm);     /* deg */
    }
    float pitch_deg = in->theta * 57.29578f;
    even_pwm = pid_update(&angle_pid, angle_target, pitch_deg);        /* angle_bias = 0 in the sim (COM calibration) */
    float tau_total = SCALE * even_pwm;                                /* PWM (+-1000) -> torque */
    out[0] = 0.5f * tau_total; out[1] = 0.5f * tau_total;
}
