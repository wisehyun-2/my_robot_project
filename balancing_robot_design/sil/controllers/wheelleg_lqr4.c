/*
 * wheelleg_lqr4.c -- port of 1772631792/WheelLeg_LQR (C chassis algorithm + Python plant, MuJoCo), the only
 *   "wheel-leg" project in the survey whose balance law is a 4-state cart-pole LQR (no leg angle in the state):
 *     states  s = [p, v, theta, omega]      (config/robot_params.py, simulation/model.py: theta + = forward tilt)
 *     plant   H(theta) [p.., th..] = [u - b v, m g l sin th],  H = [[mb+m, m l],[m l, I + m l^2]]   (u = horizontal force)
 *     design  Q = diag(20, 2, 300, 5), R = 0.1, Ts = 1 ms, exact ZOH (matrix exponential of [[A,B],[0,0]] Ts by
 *             scaling & squaring + Taylor) and Riccati value iteration P <- Q + Ad'PAd - Ad'PBd (R+Bd'PBd)^-1 Bd'PAd
 *             from P = Q to 1e-12 (controller/lqr_design.c).  Their robot: base 1 kg, body 5 kg, COM 0.25 m, I 0.1.
 *     law     drive = -sum_i k_i (s_i - ref_i)  with ref only on p (integrated velocity command), |drive| <= 40 N,
 *             wheel torque = r * drive / 2 per wheel  (chassis.c: tl = 0.06 * drive/2).  Legs: length PID + VMC
 *             only (support force), no hip pitch torque -> on a four-bar/five-bar robot the hips just hold the height.
 *   Two ways to run it (meta.json):
 *     wheelleg_lqr4         their Q/R and design procedure, model built from THIS robot's parameters (like fern_lqr3)
 *     wheelleg_lqr4_native  their published robot parameters -> their exact K, applied as-is (output scale searched)
 *   The solver below is a re-implementation of the same algorithm (the repository carries no licence file).
 */
#include <math.h>
#include <string.h>
#include "../ctrl_api.h"
#define N 4
static float DT; static ctrl_params_t P;
static double K[N], pos = 0, pos_ref = 0, l_last = -1;
static int NATIVE = 0;                       /* 1: use their robot parameters (base 1, body 5, com 0.25, I 0.1) */
static double Q[N] = {20.0, 2.0, 300.0, 5.0}, R = 0.1, FRICTION = 0.1, OUT_LIMIT = 40.0;
static float SCALE = 1.0f;

static void model(double l, double A[N*N], double B[N]) {
    double mb, m, I, g = 9.81;
    if (NATIVE) { mb = 1.0; m = 5.0; l = 0.25; I = 0.10; }
    else { mb = 2.0*P.m_w + 2.0*P.I_w/(P.r*P.r); m = P.M_b; I = P.I_b*(l/P.l)*(l/P.l); }
    double h11 = mb + m, h12 = m*l, h22 = I + m*l*l, det = h11*h22 - h12*h12;
    double i11 = h22/det, i12 = -h12/det, i22 = h11/det;        /* H^-1 */
    memset(A, 0, sizeof(double)*N*N); memset(B, 0, sizeof(double)*N);
    A[0*N+1] = 1.0; A[2*N+3] = 1.0;
    A[1*N+1] = i11*(-FRICTION);          A[3*N+1] = i12*(-FRICTION);
    A[1*N+2] = i12*(m*g*l);              A[3*N+2] = i22*(m*g*l);
    B[1] = i11; B[3] = i12;
}
/* exact ZOH: exp([[A,B],[0,0]] Ts) by scaling & squaring with a 24-term Taylor series (same scheme as theirs) */
static void zoh(const double A[N*N], const double B[N], double Ad[N*N], double Bd[N]) {
    const int M = N+1; double Z[M*M], E[M*M], T[M*M], W[M*M]; double norm = 0, scale = 1; int sq = 0;
    memset(Z, 0, sizeof(Z));
    for (int i = 0; i < N; i++) { for (int j = 0; j < N; j++) Z[i*M+j] = A[i*N+j]*DT; Z[i*M+N] = B[i]*DT; }
    for (int i = 0; i < M; i++) { double row = 0; for (int j = 0; j < M; j++) row += fabs(Z[i*M+j]); if (row > norm) norm = row; }
    while (norm/scale > 0.5) { scale *= 2; sq++; }
    for (int i = 0; i < M*M; i++) Z[i] /= scale;
    memset(E, 0, sizeof(E)); memset(T, 0, sizeof(T));
    for (int i = 0; i < M; i++) E[i*M+i] = T[i*M+i] = 1.0;
    for (int n = 1; n <= 24; n++) {
        for (int i = 0; i < M; i++) for (int j = 0; j < M; j++) { double v = 0; for (int l = 0; l < M; l++) v += T[i*M+l]*Z[l*M+j]; W[i*M+j] = v/n; }
        for (int i = 0; i < M*M; i++) { T[i] = W[i]; E[i] += W[i]; }
    }
    while (sq-- > 0) {
        for (int i = 0; i < M; i++) for (int j = 0; j < M; j++) { double v = 0; for (int l = 0; l < M; l++) v += E[i*M+l]*E[l*M+j]; W[i*M+j] = v; }
        memcpy(E, W, sizeof(E));
    }
    for (int i = 0; i < N; i++) { Bd[i] = E[i*M+N]; for (int j = 0; j < N; j++) Ad[i*N+j] = E[i*M+j]; }
}
static void dlqr(const double Ad[N*N], const double Bd[N]) {
    double Pm[N*N] = {0}, PB[N], PA[N*N], Pn[N*N], k[N];
    for (int i = 0; i < N; i++) Pm[i*N+i] = Q[i];
    for (int it = 0; it < 300000; it++) {
        double den = R, err = 0, mag = 0;
        for (int i = 0; i < N; i++) { PB[i] = 0; for (int l = 0; l < N; l++) PB[i] += Pm[i*N+l]*Bd[l];
            for (int j = 0; j < N; j++) { double v = 0; for (int l = 0; l < N; l++) v += Pm[i*N+l]*Ad[l*N+j]; PA[i*N+j] = v; } }
        for (int i = 0; i < N; i++) den += Bd[i]*PB[i];
        for (int j = 0; j < N; j++) { k[j] = 0; for (int i = 0; i < N; i++) k[j] += Bd[i]*PA[i*N+j]; k[j] /= den; }
        for (int i = 0; i < N; i++) for (int j = 0; j < N; j++) { double v = (i == j) ? Q[i] : 0.0;
            for (int l = 0; l < N; l++) v += Ad[l*N+i]*PA[l*N+j]; Pn[i*N+j] = v - den*k[i]*k[j]; }
        for (int i = 0; i < N; i++) for (int j = i+1; j < N; j++) { double a = 0.5*(Pn[i*N+j]+Pn[j*N+i]); Pn[i*N+j] = Pn[j*N+i] = a; }
        for (int i = 0; i < N*N; i++) { double d = Pn[i]-Pm[i]; err += d*d; mag += Pn[i]*Pn[i]; Pm[i] = Pn[i]; }
        if (sqrt(err/fmax(mag, 1.0)) < 1e-12) break;
    }
    double den = R; for (int i = 0; i < N; i++) { PB[i] = 0; for (int l = 0; l < N; l++) PB[i] += Pm[i*N+l]*Bd[l]; den += Bd[i]*PB[i]; }
    for (int j = 0; j < N; j++) { K[j] = 0; for (int i = 0; i < N; i++) K[j] += PB[i]*Ad[i*N+j]; K[j] /= den; }
}
static void design(double l) { double A[N*N], B[N], Ad[N*N], Bd[N]; model(l, A, B); zoh(A, B, Ad, Bd); dlqr(Ad, Bd); l_last = l; }

void ctrl_init(float dt, const ctrl_params_t* p) { DT = dt; P = *p; pos = 0; pos_ref = 0; design(P.l); }
int  ctrl_output_type(void) { return CTRL_OUT_TORQUE; }
void ctrl_set_scale(float s) { SCALE = s; }
void ctrl_set_param(const char* name, float v) {
    if (!strcmp(name, "native")) { NATIVE = (int)v; design(P.l); }
    else if (!strcmp(name, "r")) { R = v; design(P.l); }
    else if (!strcmp(name, "q_pos")) { Q[0] = v; design(P.l); } else if (!strcmp(name, "q_vel")) { Q[1] = v; design(P.l); }
    else if (!strcmp(name, "q_pitch")) { Q[2] = v; design(P.l); } else if (!strcmp(name, "q_rate")) { Q[3] = v; design(P.l); }
}
void ctrl_get_gains(float* g) { for (int i = 0; i < N; i++) g[i] = (float)K[i]; }
void ctrl_step(const ctrl_in_t* in, float out[2]) {
    if (!NATIVE && fabs(in->leg_len - l_last) > 0.01) design(in->leg_len);          /* their 3-node height schedule */
    pos += in->x_dot*DT; pos_ref += in->v_ref*DT;
    double drive = -(K[0]*(pos - pos_ref) + K[1]*in->x_dot + K[2]*in->theta + K[3]*in->theta_dot);
    drive *= SCALE;
    if (drive > OUT_LIMIT) drive = OUT_LIMIT; if (drive < -OUT_LIMIT) drive = -OUT_LIMIT;
    float tau = (float)(in->wheel_r*drive*0.5);
    out[0] = tau; out[1] = tau;
}
const char* ctrl_name(void) { return NATIVE ? "WheelLeg_LQR (1772631792): their 4-state cart-pole dLQR K as published (base 1 + body 5 kg, COM 0.25 m), force -> wheel torque, 1 kHz"
                                            : "WheelLeg_LQR (1772631792): 4-state cart-pole dLQR, THEIR Q=diag(20,2,300,5) R=0.1 designed on this robot, 1 kHz"; }
