/*
 * mushibo_lqr4.c -- port of the balance loop of MuShibo/Micro-Wheeled_leg-Robot (3.7k stars, ESP32 + SimpleFOC)
 *   source: 3.Software/wl_pro_robot/wl_pro_robot.ino  lqr_balance_loop()   (identical in luoyx257's STM32 C port
 *   balanced_robots/Control/controller.c Controller_LqrBalanceLoop()).  Hardware: 2 BLDC wheels (2804 gimbal, torque
 *   = voltage mode) + 2 STS3032 bus servos (one per leg, linkage) -> 4 motors, Ascento-like.
 *
 * Law (their comment):  u = k1*(angle - angle_zero) + k2*gyro + k3*(distance - distance_zero) + k4*speed,
 * implemented as four P "PIDControllers" each clipped to +-8:
 *   pid_angle P=1.0  on pitch [deg]           pid_gyro P=0.06 on gyro [deg/s]
 *   pid_distance P=0.5 on wheel angle [rad]   pid_speed P=0.7 on wheel speed [rad/s] (0.7/0.6/0.5 vs leg height)
 * plus: small-torque PI (P=1, I=15, limit 8) applied to u when |u|<5 and |distance_control|<4,
 *       balance-point adaptation angle_zero -= 0.002 * LPF(distance_control, Tf=0.1 s),
 *       distance zero reset when |speed| > 15 rad/s, wheel-off-ground detection (skipped here).
 * Output: motor.target = -0.5*(u +- yaw) in VOLTS for a 2804 gimbal motor.  To run on another robot the only free
 * parameter is the volt->torque scale (ctrl_set_scale), which the harness searches; the gain RATIOS are theirs.
 * Loop: the ESP32 loop runs as fast as SimpleFOC allows (~1 kHz); we use Ts = 1 ms.
 */
#include <math.h>
#include "../ctrl_api.h"

typedef struct { float P, I, D, limit, integral_prev, error_prev; } pidc_t;
static pidc_t pid_angle = {1.0f, 0, 0, 8}, pid_gyro = {0.06f, 0, 0, 8}, pid_distance = {0.5f, 0, 0, 8},
              pid_speed = {0.7f, 0, 0, 8}, pid_lqr_u = {1.0f, 15.0f, 0, 8}, pid_zeropoint = {0.002f, 0, 0, 4};
static float DT, SCALE = 1.0f, angle_zeropoint = 0.0f, distance_zeropoint = 0.0f, lpf_zp = 0, lpf_joy = 0;
static float wheel_r; static int first = 1;
static float height_mm = 40.0f;   /* their height units (32..80 mm); we map leg_len -> height */

static float pid(pidc_t* p, float e) {     /* SimpleFOC PIDController: P + I (Tustin) + D, output limited */
    float prop = p->P * e;
    float integ = p->integral_prev + p->I * DT * 0.5f * (e + p->error_prev);
    if (integ > p->limit) integ = p->limit; if (integ < -p->limit) integ = -p->limit;
    float der = p->D * (e - p->error_prev) / DT;
    float out = prop + integ + der;
    if (out > p->limit) out = p->limit; if (out < -p->limit) out = -p->limit;
    p->integral_prev = integ; p->error_prev = e;
    return out;
}
static float lpf(float* y, float x, float Tf) { float a = Tf / (Tf + DT); *y = a * (*y) + (1 - a) * x; return *y; }

void ctrl_init(float dt, const ctrl_params_t* p) {
    DT = dt; wheel_r = p->r; first = 1; angle_zeropoint = 0; lpf_zp = lpf_joy = 0;
    pid_angle.integral_prev = pid_gyro.integral_prev = pid_distance.integral_prev = pid_speed.integral_prev = pid_lqr_u.integral_prev = pid_zeropoint.integral_prev = 0;
    pid_angle.error_prev = pid_gyro.error_prev = pid_distance.error_prev = pid_speed.error_prev = pid_lqr_u.error_prev = pid_zeropoint.error_prev = 0;
}
void ctrl_set_scale(float s) { SCALE = s; }
int ctrl_output_type(void) { return CTRL_OUT_TORQUE; }
const char* ctrl_name(void) { return "MuShibo Micro-Wheeled_leg-Robot: 4-term P 'LQR' (deg, rad) + small-torque PI + zero adaptation"; }

static float dist_rad = 0;   /* integrated wheel angle (their shaft_angle) */
void ctrl_step(const ctrl_in_t* in, float out[2]) {
    float LQR_angle = in->theta * 57.29578f;            /* deg */
    float LQR_gyro  = in->theta_dot * 57.29578f;        /* deg/s */
    float LQR_speed = in->x_dot / wheel_r;              /* rad/s (wheel) */
    dist_rad += LQR_speed * DT;                         /* rad */
    float LQR_distance = dist_rad;
    if (first) { distance_zeropoint = LQR_distance; first = 0; }
    /* leg height -> their speed-gain schedule (height<50: 0.7, <64: 0.6, else 0.5) */
    height_mm = 32.0f + 48.0f * (in->leg_len / 0.16f - 0.75f) / 0.65f;   /* 0.12 m -> 32, 0.224 m -> 80 */
    pid_speed.P = height_mm < 50 ? 0.7f : (height_mm < 64 ? 0.6f : 0.5f);

    float angle_control = pid(&pid_angle, LQR_angle - angle_zeropoint);
    float gyro_control  = pid(&pid_gyro, LQR_gyro);
    int joy = (fabsf(in->v_ref) > 1e-3f);
    if (joy) { distance_zeropoint = LQR_distance; pid_lqr_u.integral_prev = 0; }      /* moving: no position hold */
    if (fabsf(LQR_speed) > 15.0f) distance_zeropoint = LQR_distance;
    float distance_control = pid(&pid_distance, LQR_distance - distance_zeropoint);
    /* their joystick scale: speed_control = P*(speed - 0.1*lpf(joyy)); joyy in [-100,100] -> ~10 rad/s max.
       map v_ref [m/s] -> joyy so that 1 m/s corresponds to the wheel speed 1/r rad/s */
    float joyy = in->v_ref / wheel_r * 10.0f;
    float speed_control = pid(&pid_speed, LQR_speed - 0.1f * lpf(&lpf_joy, joyy, 0.2f));
    float LQR_u = angle_control + gyro_control + distance_control + speed_control;
    if (fabsf(LQR_u) < 5.0f && !joy && fabsf(distance_control) < 4.0f) {
        LQR_u = pid(&pid_lqr_u, LQR_u);
        angle_zeropoint -= pid(&pid_zeropoint, lpf(&lpf_zp, distance_control, 0.1f));
    } else {
        pid_lqr_u.integral_prev = 0;
    }
    /* motor.target = (-0.5)*(LQR_u +- YAW): negative because of their wiring; sign chosen by harness autosign */
    float tau_total = SCALE * LQR_u;
    out[0] = 0.5f * tau_total; out[1] = 0.5f * tau_total;
}
