/*
 * sonic_pid.c -- port of XRobots/SonicRobot (James Bruton)  ARobot08/ARobot08.ino  (Teensy 3.6, 100 Hz, ODrive)
 *   Hardware: 2 ODrive hoverboard wheels (VELOCITY mode, SetVelocity) + 2 ODrive ball-screw knees -> 4 motors.
 *   Law (verbatim): PID1(Kp 7500, Ki 75000, Kd 180) from the Arduino PID_v1 library on pitch [deg],
 *   Setpoint = RFB (stick) + trim, SampleTime 10 ms, OutputLimits +-360000 [encoder counts/s] -> odrive.SetVelocity.
 *   PID_v1 semantics: ki_eff = Ki*Ts, kd_eff = Kd/Ts, derivative on measurement, integral clamped to output limits.
 *   No wheel-speed feedback.  counts/s -> rad/s with 8192 CPR (ODrive v3 AMT102 era); the harness scale search covers
 *   the unit uncertainty.  Output type: wheel velocity reference (driver speed loop simulated by the harness).
 */
#include <math.h>
#include "../ctrl_api.h"
static float Ts, SCALE = 1.0f, ITerm = 0, lastInput = 0; static int first = 1;
static const float Kp = 7500.0f, Ki = 75000.0f, Kd = 180.0f, OUT_LIM = 360000.0f, CPR = 8192.0f;
void ctrl_init(float dt, const ctrl_params_t* p) { (void)p; Ts = dt; ITerm = 0; lastInput = 0; first = 1; }
void ctrl_set_scale(float s) { SCALE = s; }
int ctrl_output_type(void) { return CTRL_OUT_WHEELVEL; }
const char* ctrl_name(void) { return "SonicRobot (James Bruton): 100 Hz PID_v1(7500,75000,180) on pitch -> ODrive wheel velocity, no wheel feedback"; }
void ctrl_step(const ctrl_in_t* in, float out[2]) {
    float Input = in->theta * 57.29578f;
    float Setpoint = in->v_ref * 10.0f;                 /* RFB stick: a few degrees of lean per unit command */
    if (first) { lastInput = Input; first = 0; }
    float error = Setpoint - Input;
    ITerm += (Ki * Ts) * error; if (ITerm > OUT_LIM) ITerm = OUT_LIM; if (ITerm < -OUT_LIM) ITerm = -OUT_LIM;
    float dInput = Input - lastInput;
    float Output = Kp * error + ITerm - (Kd / Ts) * dInput;
    if (Output > OUT_LIM) Output = OUT_LIM; if (Output < -OUT_LIM) Output = -OUT_LIM;
    lastInput = Input;
    float w = Output / CPR * 6.2831853f;                /* rad/s */
    out[0] = SCALE * w; out[1] = SCALE * w;             /* drive1 = Output1, drive2 = -Output1 (reversed wiring) */
}
