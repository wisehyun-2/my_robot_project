/*
 * fern_lqr3.c -- port of fernandomierhicks/wheeled-leg-robot  firmware/robot_teensy/teensy/src/control_loop.cpp
 *   Hardware: 2 AK45-10 hip actuators + 2 ODrive wheels (torque mode), four-bar legs -> 4 motors (Ascento-like).
 *   Structure (verbatim, Phase 5/6):
 *     500 Hz.  x0 = pitch - theta_ref - trim,  x1 = pitch_rate,  x2 = wheel velocity
 *     tau_sym = -(k_pitch*x0 + k_rate*x1 + k_vel*x2), k_pitch/k_rate interpolated linearly in leg height alpha
 *     between RET and EXT values from an LQR gain table (lqr.py: Q_pitch=0.01, Q_pitch_rate=0.1884,
 *     Q_vel=0.00508442, R=100), theta_ref = PI(velocity error) with anti-windup, then rate-limited,
 *     FF2 = ff2_alpha * M_BODY*g*l_eff*sin(pitch) (gravity feed-forward), torque limit, hip-coupling FF1 (0 here).
 *   Their numeric gain defaults live in param_registry.cpp (not dumped), so the gains are regenerated here for the
 *   target robot with THEIR Q/R weights by the same Riccati iteration as mine_lqi.c (continuous-time 3-state model,
 *   discretised at 2 ms).  Velocity-PI gains and ff2_alpha are free (meta.json); theta_ref rate limit 30 deg/s.
 */
#include <math.h>
#include <string.h>
#include "../ctrl_api.h"
#define N 3
static float DT, K[N], vel_integral = 0, theta_ref_rlt = 0, l_last = -1;
static ctrl_params_t P;
static float VEL_KP = 0.15f, VEL_KI = 0.3f, FF2 = 0.0f, THETA_REF_MAX = 0.26f, RATE_LIM = 0.52f; /* rad, rad/s */

static void build_model(float l, double Ad[N*N], double Bd[N]) {      /* states [theta, theta_dot, v] */
    const double g = 9.81; double I_b = P.I_b * (l/P.l)*(l/P.l);
    double a11 = P.M_b + 2.0*P.m_w + 2.0*P.I_w/(P.r*P.r), a12 = P.M_b*l, a22 = I_b + P.M_b*l*l, det = a11*a22 - a12*a12;
    double A[N*N] = {0}, B[N] = {0};
    A[0*N+1] = 1.0; A[1*N+0] = (a11*P.M_b*g*l)/det; A[2*N+0] = (-a12*P.M_b*g*l)/det;
    B[1] = (-a12/P.r - a11)/det; B[2] = (a22/P.r + a12)/det;
    double Pw[N*N], T[N*N], Bint[N*N];
    for (int i = 0; i < N*N; i++) { Ad[i] = (i % (N+1) == 0); Pw[i] = Ad[i]; Bint[i] = Ad[i]*DT; }
    double fact = 1.0;
    for (int k = 1; k <= 8; k++) {
        for (int i = 0; i < N; i++) for (int j = 0; j < N; j++) { double s = 0; for (int m = 0; m < N; m++) s += Pw[i*N+m]*A[m*N+j]*DT; T[i*N+j] = s; }
        memcpy(Pw, T, sizeof(T)); fact *= k;
        for (int i = 0; i < N*N; i++) { Ad[i] += Pw[i]/fact; Bint[i] += Pw[i]/fact*DT/(k+1); }
    }
    for (int i = 0; i < N; i++) { Bd[i] = 0; for (int j = 0; j < N; j++) Bd[i] += Bint[i*N+j]*B[j]; }
}
static void dlqr(const double Ad[N*N], const double Bd[N], const double Q[N], double R, float Kout[N]) {
    double Pm[N*N] = {0}, Kd[N] = {0};
    for (int i = 0; i < N; i++) Pm[i*N+i] = Q[i];
    for (int it = 0; it < 400000; it++) {
        double PB[N], BPB = R, Acl[N*N], AP[N*N], Pn[N*N];
        for (int i = 0; i < N; i++) { PB[i] = 0; for (int j = 0; j < N; j++) PB[i] += Pm[i*N+j]*Bd[j]; }
        for (int i = 0; i < N; i++) BPB += Bd[i]*PB[i];
        for (int j = 0; j < N; j++) { double s = 0; for (int i = 0; i < N; i++) s += PB[i]*Ad[i*N+j]; Kd[j] = s/BPB; }
        for (int i = 0; i < N; i++) for (int j = 0; j < N; j++) Acl[i*N+j] = Ad[i*N+j] - Bd[i]*Kd[j];
        for (int i = 0; i < N; i++) for (int j = 0; j < N; j++) { double s = 0; for (int k = 0; k < N; k++) s += Acl[k*N+i]*Pm[k*N+j]; AP[i*N+j] = s; }
        double diff = 0, mag = 0;
        for (int i = 0; i < N; i++) for (int j = 0; j < N; j++) { double s = 0; for (int k = 0; k < N; k++) s += AP[i*N+k]*Acl[k*N+j]; Pn[i*N+j] = (i == j ? Q[i] : 0) + s + Kd[i]*R*Kd[j]; }
        for (int i = 0; i < N; i++) for (int j = i; j < N; j++) { double v = 0.5*(Pn[i*N+j]+Pn[j*N+i]); Pn[i*N+j] = Pn[j*N+i] = v; }
        for (int i = 0; i < N*N; i++) { diff += fabs(Pn[i]-Pm[i]); mag += fabs(Pn[i]); }
        memcpy(Pm, Pn, sizeof(Pm)); if (diff < 1e-12*mag) break;
    }
    for (int j = 0; j < N; j++) Kout[j] = (float)Kd[j];
}
static void design(float l) { double Ad[N*N], Bd[N]; build_model(l, Ad, Bd);
    double Q[N] = {0.01, 0.1884, 0.00508442}; double R = 100.0;       /* THEIR weights (lqr.py self-test) */
    dlqr(Ad, Bd, Q, R, K); }

void ctrl_init(float dt, const ctrl_params_t* p) { DT = dt; P = *p; vel_integral = theta_ref_rlt = 0; l_last = P.l; design(P.l); }
void ctrl_set_scale(float s) { (void)s; }
void ctrl_set_param(const char* n, float v) { if (!strcmp(n, "vel_kp")) VEL_KP = v; else if (!strcmp(n, "vel_ki")) VEL_KI = v; else if (!strcmp(n, "ff2")) FF2 = v; }
void ctrl_get_gains(float* k) { for (int i = 0; i < N; i++) k[i] = K[i]; }
int ctrl_output_type(void) { return CTRL_OUT_TORQUE; }
const char* ctrl_name(void) { return "fernandomierhicks wheeled-leg-robot: 500 Hz 3-state LQR (their Q/R), gain-scheduled, vel PI->theta_ref, FF2"; }

void ctrl_step(const ctrl_in_t* in, float out[2]) {
    if (fabsf(in->leg_len - l_last) > 0.01f) { design(in->leg_len); l_last = in->leg_len; }  /* gain schedule vs leg height */
    /* velocity PI -> lean target (anti-windup by clamping), then rate limit */
    float ev = in->v_ref - in->x_dot;
    vel_integral += ev * DT; float imax = THETA_REF_MAX / (VEL_KI > 1e-6f ? VEL_KI : 1e-6f);
    if (vel_integral > imax) vel_integral = imax; if (vel_integral < -imax) vel_integral = -imax;
    float theta_raw = VEL_KP * ev + VEL_KI * vel_integral;
    if (theta_raw > THETA_REF_MAX) theta_raw = THETA_REF_MAX; if (theta_raw < -THETA_REF_MAX) theta_raw = -THETA_REF_MAX;
    float d = theta_raw - theta_ref_rlt, lim = RATE_LIM * DT; if (d > lim) d = lim; if (d < -lim) d = -lim; theta_ref_rlt += d;
    float x0 = in->theta - theta_ref_rlt, x1 = in->theta_dot, x2 = in->x_dot;
    float tau_sym = -(K[0]*x0 + K[1]*x1 + K[2]*x2);
    tau_sym += FF2 * P.M_b * 9.81f * in->leg_len * sinf(in->theta);
    float lim_t = 2*P.tau_max; if (tau_sym > lim_t) tau_sym = lim_t; if (tau_sym < -lim_t) tau_sym = -lim_t;
    out[0] = 0.5f*tau_sym; out[1] = 0.5f*tau_sym;
}
