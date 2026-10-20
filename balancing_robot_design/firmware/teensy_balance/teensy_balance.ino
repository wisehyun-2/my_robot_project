/*
 * teensy_balance.ino  --  Balance loop for a direct-drive two-wheeled robot (Teensy 4.1)
 *
 * Architecture (DIABLO-style split):
 *   Teensy 4.1 (this file)                     Companion PC (ROS 2)
 *   - 1 kHz hardware-timer ISR               - sends velocity / yaw-rate setpoints (50-100 Hz)
 *   - IMU over SPI (ICM-42688-P or BMI088)   - receives state telemetry (100 Hz) and republishes
 *   - complementary filter -> pitch           - vision, navigation, logging
 *   - LQI pitch/velocity + PD yaw             - NEVER in the balance loop
 *   - torque commands to both wheel drivers over CAN (FOC torque / current mode)
 *   - watchdog on the setpoint link, tilt cut-off, torque limits
 *
 * The wheel driver adapter (sendTorqueCAN / parseFeedbackCAN) is the only part that depends on the
 * motor you buy. Three adapters are sketched at the bottom:  DDT M15-series (CAN current mode),
 * mjbots moteus (CAN-FD, torque mode), ODrive S1/Pro (CAN 2.0B, Set_Input_Torque).
 *
 * Gains come from sim/gen_gains.py (results/lqr_design_baseline.json). Units: SI, rad, N m.
 *
 * Sign conventions (must match the model in sim/twip_model.py):
 *   pitch  theta  > 0  : body leaning FORWARD
 *   wheel  x_dot  > 0  : robot moving FORWARD
 *   torque tau    > 0  : motor drives the wheel FORWARD (reaction pitches the body backward)
 *   The LQR law then produces  tau = +k_th*theta + k_thd*theta_dot + k_v*(x_dot - v_ref) + k_i*int(...)
 *   with all k's positive: lean forward -> drive forward under the body; moving forward -> drive MORE
 *   forward (this positive wheel-speed feedback is correct and is what hand-tuned cascades get wrong).
 */
#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <SPI.h>

// ------------------------------------------------------------------------------------ parameters
constexpr float LOOP_HZ      = 1000.0f;
constexpr float DT           = 1.0f / LOOP_HZ;
constexpr float WHEEL_R      = 0.0825f;        // m
constexpr float TRACK_D      = 0.40f;          // m (wheel-to-wheel)
constexpr float TAU_MAX      = 5.0f;           // N m per wheel, software limit (<= driver's peak)
constexpr float TILT_CUTOFF  = 0.60f;          // rad (~34 deg) -> motors off
constexpr float CMD_TIMEOUT  = 0.5f;           // s without a valid setpoint -> zero setpoint; 2 s -> motors off
constexpr float V_REF_RATE   = 1.0f;           // m/s^2 reference ramp (prevents 13-deg lean on a step)

// LQI gains (planned_10kg_6p5in_hub, Ts = 1 ms, Q = diag(0,1,25,0.25), R = 1/(0.5*tau_max_total)^2)
// u_total = -(K_v*(v - v_ref) + K_th*theta + K_thd*theta_dot + K_i*z),  z = int (v - v_ref) dt
// values from results/lqr_design_baseline.json  "K_lqi_1kHz": [0, -7.164, -35.588, -5.800, -3.940]
constexpr float K_V   = -7.164f;   // N m / (m/s)
constexpr float K_TH  = -35.588f;  // N m / rad
constexpr float K_THD = -5.800f;   // N m / (rad/s)
constexpr float K_I   = -3.940f;   // N m / m
constexpr float Z_MAX = 1.0f;      // anti-windup on the integral state (m)

// yaw: PD on yaw rate from the gyro (simple, robust). tau_diff = Kp_yaw*(psi_dot_ref - psi_dot)
constexpr float KP_YAW = 0.8f;     // N m per rad/s

// complementary filter
constexpr float CF_TAU = 0.5f;     // s ; alpha = CF_TAU/(CF_TAU+DT)

// ------------------------------------------------------------------------------------ state
volatile bool  tick = false;
IntervalTimer  loopTimer;

struct Setpoint { float v = 0, yaw_rate = 0; uint32_t t_ms = 0; bool valid = false; } sp;
struct State    { float theta = 0, theta_dot = 0, v = 0, psi_dot = 0, z = 0, v_ref_ramped = 0; } st;
struct Wheel    { float omega = 0; float tau_cmd = 0; uint32_t t_fb_ms = 0; } wheel[2];   // 0 = L, 1 = R
enum Mode { MODE_OFF, MODE_BALANCE, MODE_FAULT } mode = MODE_OFF;

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_64> can;   // use CAN3 (FD) + FlexCAN_T4FD for moteus

// ------------------------------------------------------------------------------------ IMU (stub)
// Replace with your driver (ICM-42688-P: gyro ODR 1 kHz, LPF ~100-200 Hz; accel ODR 1 kHz).
struct ImuSample { float gx, gy, gz, ax, ay, az; };
bool imuRead(ImuSample& s);     // returns true when a fresh sample is available
void imuInit();

// ------------------------------------------------------------------------------------ helpers
static inline float clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }

void loopISR() { tick = true; }

// Complementary filter for pitch. Pitch axis = IMU y (adjust to your mounting).
// Accel tilt is only trusted at low frequency; gyro integrates the rest.
void estimatePitch(const ImuSample& s) {
  static float theta_acc_f = 0;
  const float alpha = CF_TAU / (CF_TAU + DT);
  float theta_acc = atan2f(s.ax, s.az);            // forward lean -> +ax (check sign on hardware!)
  st.theta_dot = s.gy;                             // rad/s, no differentiation -> no noise amplification
  st.theta = alpha * (st.theta + s.gy * DT) + (1.0f - alpha) * theta_acc;
  st.psi_dot = s.gz;
  (void)theta_acc_f;
}

void balanceStep() {
  // 1) wheel speed (average of both), from driver feedback over CAN
  st.v = 0.5f * (wheel[0].omega + wheel[1].omega) * WHEEL_R;

  // 2) setpoint handling: watchdog + ramp
  uint32_t now = millis();
  float age = (now - sp.t_ms) * 1e-3f;
  float v_ref = (sp.valid && age < CMD_TIMEOUT) ? sp.v : 0.0f;
  float yaw_ref = (sp.valid && age < CMD_TIMEOUT) ? sp.yaw_rate : 0.0f;
  float dv = clampf(v_ref - st.v_ref_ramped, -V_REF_RATE * DT, V_REF_RATE * DT);
  st.v_ref_ramped += dv;

  // 3) LQI
  float ev = st.v - st.v_ref_ramped;
  st.z = clampf(st.z + ev * DT, -Z_MAX, Z_MAX);
  float u = -(K_V * ev + K_TH * st.theta + K_THD * st.theta_dot + K_I * st.z);   // total torque (both wheels)
  u = clampf(u, -2.0f * TAU_MAX, 2.0f * TAU_MAX);

  // 4) yaw
  float tau_diff = KP_YAW * (yaw_ref - st.psi_dot);

  // 5) split and limit
  wheel[0].tau_cmd = clampf(0.5f * u - 0.5f * tau_diff, -TAU_MAX, TAU_MAX);
  wheel[1].tau_cmd = clampf(0.5f * u + 0.5f * tau_diff, -TAU_MAX, TAU_MAX);
}

void safetyCheck() {
  uint32_t now = millis();
  if (fabsf(st.theta) > TILT_CUTOFF) mode = MODE_FAULT;                   // fell over
  if (mode == MODE_BALANCE && (now - sp.t_ms) > 2000 && sp.valid) mode = MODE_OFF;   // host gone -> off (balancing with v=0 first)
  for (int i = 0; i < 2; i++) if (now - wheel[i].t_fb_ms > 50) mode = MODE_FAULT;  // driver feedback lost
}

// ------------------------------------------------------------------------------------ host link (UART, 460800 8N1 like DIABLO)
// Frame: 0xAA 0x55 | type | len | payload | crc16  (keep it tiny; micro-ROS is the alternative)
void hostPoll();      // parse setpoint frames -> sp
void hostTelemetry(); // 100 Hz: theta, theta_dot, v, psi_dot, tau L/R, mode, battery

// ------------------------------------------------------------------------------------ setup / loop
void setup() {
  Serial1.begin(460800);
  imuInit();
  can.begin(); can.setBaudRate(1000000);
  driverInit();
  loopTimer.begin(loopISR, 1000000 / (int)LOOP_HZ);   // 1 kHz
}

void loop() {
  parseFeedbackCAN();   // non-blocking, updates wheel[].omega
  hostPoll();
  if (!tick) return;
  tick = false;

  ImuSample s;
  if (imuRead(s)) estimatePitch(s);
  safetyCheck();

  if (mode == MODE_BALANCE) {
    balanceStep();
  } else {
    wheel[0].tau_cmd = wheel[1].tau_cmd = 0; st.z = 0; st.v_ref_ramped = 0;
    // arm when upright and host (or RC) requests it
    if (mode == MODE_OFF && fabsf(st.theta) < 0.05f && sp.valid) mode = MODE_BALANCE;
  }
  sendTorqueCAN(0, wheel[0].tau_cmd);
  sendTorqueCAN(1, wheel[1].tau_cmd);

  static uint16_t n = 0;
  if (++n >= 10) { n = 0; hostTelemetry(); }   // 100 Hz
}

// ------------------------------------------------------------------------------------ driver adapters (choose ONE, fill in from the vendor protocol doc)
void driverInit();
void sendTorqueCAN(int idx, float tau);
void parseFeedbackCAN();
/*
 * (a) MyActuator RMD-L series (RMD-L-9015 / 9025 / 7015), integrated FOC, CAN 2.0B 1 Mbps, ID 0x140 + motor_id.
 *     Torque closed-loop command 0xA1: data[4..5] = int16 iqControl (scale per the RMD protocol manual, e.g.
 *     -2048..2048 <-> -33..33 A on older firmware, or 0.01 A/LSB on newer; CHECK the manual for your firmware).
 *     Reply carries temperature, iq, speed (dps) and encoder position -> wheel omega = speed_dps * pi/180.
 *     Iq = tau / Kt   (Kt: 0.45 Nm/A for 9015-35T, 0.76 for 9025-35T, 0.28 for 7015-23T; calibrate on the bench).
 *     Send 0xA1 to both motors every tick; the reply doubles as the feedback (no separate query needed).
 *
 * (b) mjbots moteus (r4.11 / n1 / c1) + U8/U10-class outrunner, CAN-FD 1M/5M (Teensy CAN3 + FlexCAN_T4FD).
 *     Use the Moteus Arduino library: controller.SetPosition(NaN, {.velocity=NaN, .feedforward_torque=tau, .kp_scale=0, .kd_scale=0})
 *     or the raw "position mode with zero gains + feedforward torque"; query returns velocity (rev/s).
 *
 * (c) ODrive S1/Pro, CAN 2.0B 1 Mbps: Set_Controller_Mode(TORQUE_CONTROL, PASSTHROUGH), Set_Input_Torque(0x00E) each tick,
 *     Get_Encoder_Estimates(0x009) cyclic at >= 200 Hz for wheel velocity. Set the watchdog (axis.config.watchdog_timeout ~0.1 s).
 */
