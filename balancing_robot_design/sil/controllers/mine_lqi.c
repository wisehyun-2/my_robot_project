/*
 * mine_lqi.c -- the controller proposed in this study (same law as firmware/teensy_balance/teensy_balance.ino):
 *   discrete LQI on [x_dot - v_ref, theta, theta_dot, int(x_dot - v_ref)], torque output, reference ramp (1 m/s^2),
 *   anti-windup, yaw-rate P.  Gains are computed ONLINE from the robot parameters by solving the discrete Riccati
 *   equation, so the same .so works for any robot and re-designs itself when the leg length (COM height) changes
 *   (gain scheduling).  On the MCU, run the design once per leg-length grid point at boot and interpolate.
 *
 * Model (4 states, position dropped because it carries no cost):
 *   xs = [x_dot, theta, theta_dot, z],  z_dot = x_dot - v_ref
 *   a11 = M_b + 2 m_w + 2 I_w/r^2,  a12 = M_b l,  a22 = I_b + M_b l^2,  det = a11 a22 - a12^2
 *   x_ddot     = (-a12 M_b g l theta + (a22/r + a12) tau) / det
 *   theta_ddot = ( a11 M_b g l theta - (a12/r + a11) tau) / det
 */
#include <math.h>
#include <string.h>
#include "../ctrl_api.h"

#define N 4
static float DT, K[N], z, v_ramp, tau_max_total, Kp_yaw = 0.5f, l_last = -1.0f;
static ctrl_params_t P;

static void build_model(float l, double Ad[N*N], double Bd[N]) {
    const double g = 9.81;
    double I_b = P.I_b * (l / P.l) * (l / P.l);
    double a11 = P.M_b + 2.0*P.m_w + 2.0*P.I_w/(P.r*P.r), a12 = P.M_b*l, a22 = I_b + P.M_b*l*l;
    double det = a11*a22 - a12*a12;
    double A[N*N] = {0}, B[N] = {0};
    A[0*N+1] = (-a12 * P.M_b*g*l) / det;      /* x_ddot / theta */
    A[2*N+1] = ( a11 * P.M_b*g*l) / det;      /* theta_ddot / theta */
    A[1*N+2] = 1.0;                           /* theta_dot */
    A[3*N+0] = 1.0;                           /* z_dot = x_dot */
    B[0] = (a22/P.r + a12) / det;
    B[2] = (-a12/P.r - a11) / det;
    /* ZOH: Ad = exp(A T) = sum (A T)^k/k!,  Bd = sum A^k T^(k+1)/(k+1)! B */
    double Pw[N*N], T[N*N], Bint[N*N];
    for (int i = 0; i < N*N; i++) { Ad[i] = (i % (N+1) == 0); Pw[i] = Ad[i]; Bint[i] = Ad[i]*DT; }
    double fact = 1.0;
    for (int k = 1; k <= 8; k++) {
        for (int i = 0; i < N; i++) for (int j = 0; j < N; j++) { double s = 0; for (int m = 0; m < N; m++) s += Pw[i*N+m]*A[m*N+j]*DT; T[i*N+j] = s; }
        memcpy(Pw, T, sizeof(T)); fact *= k;
        for (int i = 0; i < N*N; i++) { Ad[i] += Pw[i]/fact; Bint[i] += Pw[i]/fact * DT/(k+1); }
    }
    for (int i = 0; i < N; i++) { Bd[i] = 0; for (int j = 0; j < N; j++) Bd[i] += Bint[i*N+j]*B[j]; }
}

/* backward Riccati iteration in Joseph form  P = Q + (A-BK)'P(A-BK) + K'RK  (numerically stable; the plain
   form Q + A'PA - A'PB(R+B'PB)^-1 B'PA cancels catastrophically for this plant).  Slow velocity/integral modes
   need ~1e4-1e5 steps at Ts = 1 ms: milliseconds on a PC, ~0.5 s on a Teensy 4.1 -> do it at boot per leg-length grid point. */
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
        for (int i = 0; i < N; i++) for (int j = 0; j < N; j++) {
            double s = 0; for (int k = 0; k < N; k++) s += AP[i*N+k]*Acl[k*N+j];
            Pn[i*N+j] = (i == j ? Q[i] : 0) + s + Kd[i]*R*Kd[j];
        }
        for (int i = 0; i < N; i++) for (int j = i; j < N; j++) { double v = 0.5*(Pn[i*N+j] + Pn[j*N+i]); Pn[i*N+j] = Pn[j*N+i] = v; }
        for (int i = 0; i < N*N; i++) { diff += fabs(Pn[i] - Pm[i]); mag += fabs(Pn[i]); }
        memcpy(Pm, Pn, sizeof(Pm));
        if (diff < 1e-12 * mag) break;
    }
    for (int j = 0; j < N; j++) Kout[j] = (float)Kd[j];
}

static void design(float l) {
    double Ad[N*N], Bd[N];
    build_model(l, Ad, Bd);
    double Q[N] = {1.0, 25.0, 0.25, 1.0};           /* x_dot 1 m/s, theta 0.2 rad, theta_dot 2 rad/s, int 1 m */
    double R = 1.0 / ((0.5*P.tau_max) * (0.5*P.tau_max));
    dlqr(Ad, Bd, Q, R, K);
}

void ctrl_init(float dt, const ctrl_params_t* p) {
    DT = dt; P = *p; z = 0; v_ramp = 0; tau_max_total = 2*P.tau_max; l_last = P.l;
    design(P.l);
}
int ctrl_output_type(void) { return CTRL_OUT_TORQUE; }
void ctrl_get_gains(float* k4) { for (int i = 0; i < N; i++) k4[i] = K[i]; }
const char* ctrl_name(void) { return "mine: LQI torque, 1 kHz, online Riccati, gain-scheduled on leg length"; }

void ctrl_step(const ctrl_in_t* in, float out[2]) {
    if (fabsf(in->leg_len - l_last) > 0.01f) { design(in->leg_len); l_last = in->leg_len; }
    float dv = in->v_ref - v_ramp, lim = 1.0f*DT; if (dv > lim) dv = lim; if (dv < -lim) dv = -lim; v_ramp += dv;
    float ev = in->x_dot - v_ramp;
    z += ev*DT; if (z > 1) z = 1; if (z < -1) z = -1;
    float u = -(K[0]*ev + K[1]*in->theta + K[2]*in->theta_dot + K[3]*z);
    if (u > tau_max_total) u = tau_max_total; if (u < -tau_max_total) u = -tau_max_total;
    float td = Kp_yaw * (in->yaw_rate_ref - in->psi_dot);
    out[0] = 0.5f*u - 0.5f*td; out[1] = 0.5f*u + 0.5f*td;
}
void ctrl_debug_model(double* Ad_out, double* Bd_out) { double Ad[N*N], Bd[N]; build_model(P.l, Ad, Bd); memcpy(Ad_out, Ad, sizeof(Ad)); memcpy(Bd_out, Bd, sizeof(Bd)); }
