/*
 * venus_cascade.c -- port of track-tech/Venus-1__Four-link-wheel-legged-robot  .../Application/TaskLayer/chassis_task.c
 *   + Device_dev.c (gains) + AlgorithmLayer/pid.c (formula).  STM32H723, 1 ms loop.  69 stars.
 *   Hardware: 2 M2006 wheels (C610 current command via "MOTOR_Speed_Contorl", +-16000) + 2 CyberGear joints (position)
 *   -> 4 motors, four-link legs.
 *   Law (verbatim, Venus_BlanceCal):
 *     Venus_pitch_Speed (PI kp 0.6, ki 0.003, out +-16000) on wheel rpm/36  ->  Venus_pitch.target  [deg]
 *     Venus_pitch (PD kp 1800, kd 18000 per-tick difference, out +-16000) on IMU pitch [deg]  ->  motor current command
 *     (position loop Venus_Position gains not in the fetched files -> omitted; it only sets the speed target)
 *   pid.c: err = target - measure; pout = kp*err; iout += ki*err (clamped); dout = kd*(err - last_err); output clamped.
 *   Current-command -> torque scale searched for our robot; gains/ratios/rates are theirs.
 */
#include <math.h>
#include "../ctrl_api.h"
typedef struct { float kp, ki, kd, maxout, ilim, err, last_err, iout, target; } pid_t;
static pid_t sp = {0.6f, 0.003f, 0, 16000, 5000, 0, 0, 0, 0}, pitch = {1800.0f, 0, 18000.0f, 16000, 5000, 0, 0, 0, 0};
static float SCALE = 1.0f, wheel_r;
static float calc(pid_t* p, float measure) {
    p->last_err = p->err; p->err = p->target - measure;
    float pout = p->kp * p->err; p->iout += p->ki * p->err;
    if (p->iout > p->ilim) p->iout = p->ilim; if (p->iout < -p->ilim) p->iout = -p->ilim;
    float dout = p->kd * (p->err - p->last_err);
    float o = pout + p->iout + dout; if (o > p->maxout) o = p->maxout; if (o < -p->maxout) o = -p->maxout;
    return o;
}
void ctrl_init(float dt, const ctrl_params_t* p) { (void)dt; wheel_r = p->r; sp.err = sp.last_err = sp.iout = 0; pitch.err = pitch.last_err = pitch.iout = 0; }
void ctrl_set_scale(float s) { SCALE = s; }
int ctrl_output_type(void) { return CTRL_OUT_TORQUE; }
const char* ctrl_name(void) { return "Venus-1 (track-tech): 1 kHz cascade, wheel-rpm PI(0.6,0.003) -> pitch target, pitch PD(1800,18000) -> current"; }
void ctrl_step(const ctrl_in_t* in, float out[2]) {
    float rpm = in->x_dot / wheel_r * 60.0f / 6.2831853f;        /* output-shaft rpm (their speed_rpm/36) */
    sp.target = in->v_ref / wheel_r * 60.0f / 6.2831853f;
    pitch.target = calc(&sp, rpm);                                 /* deg */
    float cmd = calc(&pitch, in->theta * 57.29578f);               /* current units +-16000 */
    out[0] = 0.5f * SCALE * cmd; out[1] = 0.5f * SCALE * cmd;
}
