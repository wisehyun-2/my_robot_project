"""
controllers6.py -- balance laws of the six-motor (five-bar) wheel-leg projects, ported with their own gain tables,
state orders and rates, plus my own 6-state LQI baseline.  All act on the model frame of plant6.py:
    X = [theta, theta_dot, x, x_dot, phi, phi_dot],  u = [T (total wheel torque), Tp (total hip torque)]
Each controller: name, Ts, native_L0_range, init(plant), step(X, L0, v_ref, t) -> (T, Tp).
The harness handles sensor/motor sign conventions (sign search over theta, phi, T, Tp) and output scale.
"""
import numpy as np
import scipy.linalg as la

# ----------------------------------------------------------------------------- helpers
def cubic(c, L):            # c = [a3, a2, a1, a0]  (LZU / Zhouzzw / Multivac / Liu form)
    return c[0] * L**3 + c[1] * L**2 + c[2] * L + c[3]


def gen_cubic(c, L):        # MATLAB-Coder generated form: c = [a1, a2, a3, a0]  (Skythinker / LJW lqr_k.c)
    return c[0] * L + c[1] * L**2 + c[2] * L**3 + c[3]


def colmajor_2x6(k12):      # matlabFunction flattening: k[i*2 + j] = K(j, i)
    K = np.zeros((2, 6))
    for i in range(6):
        for j in range(2):
            K[j, i] = k12[i * 2 + j]
    return K


class Base:
    Ts = 1e-3
    native_L0 = (0.1, 0.4)
    scale = 1.0
    out_names = ("T", "Tp")

    def init(self, plant):
        self.plant = plant

    def set_scale(self, s):
        self.scale = s


# ----------------------------------------------------------------------------- mine
class MineLQI6(Base):
    """6-state LQR with velocity integral, designed online from the plant linearisation (Bryson weights),
    re-designed when L0 changes by > 1 cm (gain scheduling).  Both outputs (T, Tp) come from the same LQR."""
    name = "mine: LQI6 torque (online dLQR on [th, thd, v, phi, phid, int v], Q=1/(.3,3,1,.15,3,1)^2 R=1/(.25 Tw_max, .15 Tp_max)^2), 1 kHz"
    Ts = 1e-3

    def init(self, plant):
        self.plant = plant; self.z = 0.0; self.v_ramp = 0.0; self.L0_last = None; self.K = None

    def design(self, L0):
        A, B = self.plant.linearize(L0)
        keep = [0, 1, 3, 4, 5]                     # drop x (no position hold), add integral of x_dot
        A5 = A[np.ix_(keep, keep)]; B5 = B[keep, :]
        Aa = np.zeros((6, 6)); Aa[:5, :5] = A5; Aa[5, 2] = 1.0
        Ba = np.zeros((6, 2)); Ba[:5, :] = B5
        # Bryson weights.  First design (theta 0.2 rad, phi 0.1 rad, R from 50 % of the torque limits) balanced
        # everything but tolerated < 5 ms of latency (K_Tp(theta) = 104); the weights below give 20 ms with the
        # same 40 deg / 110 N limits and 4x less rest chatter (sweep in the report, sec. 8.4).
        # R scales with the robot's torque capability (25 % of the wheel limit, 15 % of the hip limit: 1.5 / 4.5 Nm on the
        # RM plant) -- with absolute R the 0.6 kg Skythinker plant (0.2 Nm motors) got a bang-bang controller.
        Q = np.diag([1 / 0.3**2, 1 / 3.0**2, 1 / 1.0**2, 1 / 0.15**2, 1 / 3.0**2, 1 / 1.0**2])
        R = np.diag([1 / (0.25 * self.plant.tau_wheel_max)**2, 1 / (0.15 * self.plant.tau_hip_max)**2])
        import control as ct
        sysd = ct.c2d(ct.ss(Aa, Ba, np.eye(6), np.zeros((6, 2))), self.Ts)
        Ad, Bd = np.asarray(sysd.A), np.asarray(sysd.B)
        P = la.solve_discrete_are(Ad, Bd, Q, R)
        self.K = np.linalg.solve(R + Bd.T @ P @ Bd, Bd.T @ P @ Ad)
        self.L0_last = L0

    def step(self, X, L0, v_ref, t):
        if self.K is None or abs(L0 - self.L0_last) > 0.01:
            self.design(L0)
        self.v_ramp += float(np.clip(v_ref - self.v_ramp, -1.0 * self.Ts, 1.0 * self.Ts))
        ev = X[3] - self.v_ramp
        self.z = float(np.clip(self.z + ev * self.Ts, -1, 1))
        xs = np.array([X[0], X[1], ev, X[4], X[5], self.z])
        u = -self.K @ xs
        return float(u[0]), float(u[1])


# ----------------------------------------------------------------------------- generic K(L0) * x laws
class PolyK(Base):
    """u = -K(L0) x  (optionally with a velocity reference on x_dot and position hold on x)."""
    hold_position = True
    state_order = ("theta", "theta_dot", "x", "x_dot", "phi", "phi_dot")

    def K_of(self, L0):
        raise NotImplementedError

    def init(self, plant):
        self.plant = plant; self.x_set = None

    def step(self, X, L0, v_ref, t):
        L = float(np.clip(L0, *self.native_L0))
        K = self.K_of(L)
        x = np.array(X, float)
        if self.x_set is None or abs(v_ref) > 1e-3:
            self.x_set = x[2]
        if not self.hold_position:
            self.x_set = x[2]
        err = x.copy(); err[2] = x[2] - self.x_set; err[3] = x[3] - v_ref
        idx = {"theta": 0, "theta_dot": 1, "x": 2, "x_dot": 3, "phi": 4, "phi_dot": 5}
        xo = np.array([err[idx[n]] for n in self.state_order])
        u = -K @ xo
        return float(self.scale * u[0]), float(self.scale * u[1])


class Skythinker(PolyK):
    name = "Skythinker616 foc-wheel-legged-robot: 6-state dLQR (5 ms), cubic K(L0) 0.04-0.14 m, 0.6 kg robot, Q=diag(1,10,100,20,1000,1)"
    Ts = 5e-3                                   # sys_calc.m: c2d(A, B, 0.005)
    native_L0 = (0.04, 0.14)
    C = [(-5.12699127, -39.913887, 78.6044769, 0.0248099864), (0.0164185613, 0.0612591766, -0.362157434, -0.00201288541),
         (-0.713818431, -7.75286484, 9.914464, 0.00633995887), (0.00259846705, -0.00533048, 0.00809646118, -1.80964162e-5),
         (-2.53893256, -13.9123669, 39.0028343, 0.0272015687), (-0.0322025791, 0.21739994, -0.565484107, 0.00397678185),
         (-2.33014488, -10.921422, 27.9311523, 0.0158132948), (0.00878450926, -0.0480066165, 0.108919211, 0.000384154031),
         (-20.1008434, 128.616592, -322.886169, 1.63913763), (0.212965459, -1.61672437, 4.40539169, 1.96408653),
         (-0.707069635, 4.5913167, -11.6047363, 0.060216561), (0.00349324, -0.0262558721, 0.0712726861, 0.0721909478)]

    def K_of(self, L):
        return colmajor_2x6([gen_cubic(c, L) for c in self.C])


class LJWFullK(PolyK):
    """The MATLAB-designed K applied as a full 2x6 state feedback (T and Tp).  NOT what the firmware does -- see LJW."""
    name = "LJW0401 Wheel_Leg_Balance_Vehicle: MATLAB K(L0) as full 2x6 state feedback (hypothetical, firmware uses only row T)"
    native_L0 = (0.10, 0.35)
    C = [(-206.18526870643271, 559.80501544035212, -675.859318129557, 5.9467306649301284),
         (55.558778538258437, -268.75940701624211, 398.477186543132, -0.25110271498387621),
         (-34.431076401868388, 80.536217377588713, -105.9817283020235, 0.9838792125417114),
         (9.5892373401100759, -45.177251881858467, 66.619481849787391, 0.0419468267521319),
         (-94.933786703806192, 321.76673551384749, -407.17643133698419, 2.3527511293681349),
         (18.163250173694049, -107.4899738376856, 171.95617007579429, 0.82394641504814559),
         (-81.365214366504517, 273.08216572697182, -352.0714199781566, 1.576386082492194),
         (15.107823913016, -90.493196317185578, 145.74157469805911, 0.77881611287744346),
         (-111.8450222223951, 302.41682689605631, -331.05486769210921, 19.764375296877951),
         (51.959869160832191, -184.15923211508391, 238.56478031136291, 23.529134542329071),
         (-3.4481460384458011, 4.7915423043224026, -1.281298165352478, 1.0560885192987131),
         (1.8188279846387789, -4.8650106466689564, 4.826971664619653, 1.1270554718470751)]

    def K_of(self, L):
        return colmajor_2x6([gen_cubic(c, L) for c in self.C])


class LJW(Base):
    """What Balance_Controler.c really runs in STAND/MOVING:
         res = k . x  (x = [theta, dtheta, x - x_set, dx - v_set, phi, dphi], kRatio zeroes the x gain)
         left_wheel  = -res[0] * 0.2,  right_wheel = +res[0] * 0.2   (LQR_T_ratio = 0.2, mirrored motors)
         res[1] (Tp) is NOT applied: the hips run in CyberGear position mode (MI_motor_LocationControl, kp 1..10, kd 0.5)
         holding leg_angle = pi/2 in the BODY frame (pitch_PID gains are 0) -> leg perpendicular to the body.
       Model: wheel torque T = 2 * 0.2 * res[0]; hip torque Tp = position loop on the leg-vs-body angle r = theta + phi
       with 2 joints per leg x 2 legs: Kp = 4 kp_joint, Kd = 4 kd_joint (Jacobian ~ 1 at the nominal pose)."""
    name = "LJW0401 Wheel_Leg_Balance_Vehicle (as coded): T = 0.4 x LQR row T (x gain 0), hips = CyberGear position loop (kp 5, kd 0.5 / joint)"
    Ts = 1e-3
    native_L0 = (0.10, 0.35)
    kp_joint = 5.0; kd_joint = 0.5

    def init(self, plant):
        self.plant = plant

    def step(self, X, L0, v_ref, t):
        L = float(np.clip(L0, *self.native_L0))
        K = colmajor_2x6([gen_cubic(c, L) for c in LJWFullK.C])
        kT = K[0].copy(); kT[2] = 0.0                                   # ratio.kRatio[j][2] = 0
        x = np.array([X[0], X[1], 0.0, X[3] - v_ref, X[4], X[5]])
        T = self.scale * 0.4 * float(kT @ x)                            # sign handled by the harness search
        r = X[0] + X[4]; rd = X[1] + X[5]                               # leg angle relative to the body axis
        Tp = -4 * self.kp_joint * r - 4 * self.kd_joint * rd
        return T, Tp


class Jerry(PolyK):
    name = "JerryGong0911 RM2024_Reborn (USTB): 6-state LQR, quadratic K(L) 1 kHz, Q=diag(160,160,200,120,2000,50) R=diag(1,.25)"
    native_L0 = (0.10, 0.40)
    k = np.array([[175.9120, -170.6678, -17.1254], [35.0040, -31.1551, -3.6713], [20.3988, -21.8621, -6.6372], [16.7956, -19.7366, -10.9749],
                  [101.1676, -109.3629, 43.8122], [18.3027, -19.2733, 8.2075], [-161.4978, 56.6788, 27.0371], [-1.5529, -14.1453, 11.7354],
                  [-36.9927, 11.8450, 6.5878], [-31.0895, 2.3105, 10.9002], [-188.9519, 156.6850, 49.1255], [-31.0208, 26.7038, 7.4055]])

    def K_of(self, L):
        return (self.k[:, 0] * L**2 + self.k[:, 1] * L + self.k[:, 2]).reshape(2, 6)


class Zhouzzw(PolyK):
    name = "Zhouzzw Balanced_WheelLeg_Robot: 6-state LQR per leg, cubic K(L), Q=diag(16000,1000,250,100,70000,1000)"
    native_L0 = (0.10, 0.40)
    C = [(-204.71377921, 289.63818168, -157.59015500, -4.96782823), (-7.82854075, 19.15680744, -17.41989049, -0.23003324),
         (-7.53928256, 12.07032519, -6.74694158, -0.05542880), (-13.69695924, 22.38839045, -12.69903703, -0.34124181),
         (202.15935394, -119.52253547, -28.73024512, 30.05658725), (8.53035961, 0.97505447, -8.73026755, 4.45533442),
         (1451.56173722, -1146.30857312, 122.25511452, 96.79758089), (170.43065142, -127.63122938, 1.84610137, 19.79612454),
         (55.36361918, -32.73259440, -7.86810167, 8.23133542), (36.92907744, 7.73400354, -41.51630122, 19.92978630),
         (825.88702496, -1322.23787701, 739.09041989, 6.07192070), (120.04567253, -175.60627295, 92.90386168, -0.44963915)]

    def K_of(self, L):
        return np.array([cubic(c, L) for c in self.C]).reshape(2, 6) * 2.0   # per-leg law applied to both legs -> total


class Multivac(PolyK):
    name = "Multivac11 Wheel-legged-robot: 6-state LQR per leg, cubic K(L0) (Q 1,1,100,600,5000,1; R 280,17), 1 kHz"
    native_L0 = (0.10, 0.40)
    C = [(-100.655147996577, 135.362790343647, -79.596213601427, 1.089002432284), (2.665942071491, -0.438340084664, -5.856323434443, 0.169408813369),
         (-11.171961500275, 11.516106667529, -4.215606831414, -0.032675655711), (-34.400714719128, 35.573401762736, -13.258532694772, -0.125352584988),
         (31.049143385638, -5.801383358831, -17.313582675353, 11.032367816697), (8.109425363740, -6.194435819240, 0.636785003640, 1.021329665182),
         (554.563978661317, -518.966099368473, 151.716425012557, 7.395173139575), (44.487979170543, -45.889184716769, 15.721937259169, 0.218884432357),
         (-8.294971844442, 15.561852250224, -10.432155703694, 3.036549354964), (-24.201116474481, 46.583144168461, -31.729972408617, 9.396187073589),
         (659.661121060661, -714.303308554839, 284.296082774399, -17.409591688557), (54.159430352799, -62.269387519146, 27.072877393905, -2.890560575532)]

    def K_of(self, L):
        # their law: wheel_T = K0 th + K1 thd + K2 (x_set - x) + K3 (v_set - v) + K4 phi + K5 phid  (same for Tp)
        K = np.array([cubic(c, L) for c in self.C]).reshape(2, 6) * 2.0
        K[:, 2] *= -1; K[:, 3] *= -1     # x, x_dot enter as (set - actual); theta/phi enter as (actual - 0)
        return -K                        # so that u = -K x reproduces  +K0 th ... -K2 x ...


class Liu(PolyK):
    name = "LiuDingchuan Graduate_Project (Webots): 6-state LQR, cubic K(L0), Q=diag(100,100,100,10,5000,1) R=diag(1,.25)"
    native_L0 = (0.15, 0.40)
    C = [(-230.48755905338, 296.612084269857, -162.934406813838, -27.7422261659726), (-316.413239331338, 495.106965204476, -291.174766699827, 70.3669238069017),
         (-25.7389799444334, 31.3965271835001, -27.6460735389962, -7.34997262239697), (-94.2384846090381, 141.133985623166, -80.0319836223607, 19.1580752261303),
         (-33.6318410559445, 40.9718297101160, -17.4781457291386, -7.32992836005870), (-108.359148869453, 155.384350192654, -83.1313241956693, 17.9192945391375),
         (-18.7580389710126, 21.6022273062709, -9.64342672001356, -10.0406069571186), (-147.665355439053, 204.804728569058, -105.576426772434, 21.8884640308065),
         (-383.107444865340, 549.366638576174, -293.913615344890, 63.3542734141590), (475.626057518179, -579.429172541268, 247.178307361983, 103.660840979197),
         (-13.5804334552840, 19.3130739456456, -10.5076763187910, 2.48909877341753), (13.8114871794700, -16.9279439837071, 7.29672666195982, 3.10075600724177)]

    def K_of(self, L):
        # MyRobot.cpp: num = col*2 + row; K(row, col) = cubic(K_coeff[num], L0);  u = K * (Xd - X)  ==  -K (X - Xd)
        return colmajor_2x6([cubic(c, L) for c in self.C])


class ATWheelLeg(PolyK):
    name = "lyz1202dt ATWheelLeg: 6-state LQR, 17-point gain table (0.18-0.34 m) interpolated, 500 Hz, state [x,dx,th,dth,phi,dphi]"
    Ts = 2e-3
    native_L0 = (0.18, 0.34)
    state_order = ("x", "x_dot", "theta", "theta_dot", "phi", "phi_dot")
    lengths = np.arange(0.18, 0.3401, 0.01)
    vals = np.array([
        [-0.2057, -0.9930, -6.6055, -2.1252, 8.7086, 2.2637, 0.0812, 0.4224, 6.5357, 2.8051, 22.2525, 5.9751],
        [-0.2066, -0.9927, -6.7770, -2.1761, 8.5228, 2.2189, 0.0822, 0.4239, 6.5207, 2.7550, 22.5611, 6.0582],
        [-0.2076, -0.9932, -6.9445, -2.2261, 8.3402, 2.1745, 0.0828, 0.4237, 6.4998, 2.7053, 22.8600, 6.1386],
        [-0.2087, -0.9946, -7.1079, -2.2753, 8.1607, 2.1307, 0.0830, 0.4220, 6.4734, 2.6558, 23.1489, 6.2162],
        [-0.2099, -0.9965, -7.2673, -2.3235, 7.9844, 2.0874, 0.0829, 0.4189, 6.4419, 2.6064, 23.4277, 6.2912],
        [-0.2111, -0.9988, -7.4226, -2.3708, 7.8114, 2.0448, 0.0826, 0.4148, 6.4055, 2.5572, 23.6962, 6.3633],
        [-0.2124, -1.0016, -7.5737, -2.4170, 7.6417, 2.0028, 0.0820, 0.4098, 6.3646, 2.5080, 23.9545, 6.4327],
        [-0.2136, -1.0046, -7.7208, -2.4622, 7.4754, 1.9616, 0.0812, 0.4039, 6.3197, 2.4590, 24.2027, 6.4993],
        [-0.2149, -1.0078, -7.8637, -2.5064, 7.3126, 1.9211, 0.0802, 0.3974, 6.2710, 2.4101, 24.4408, 6.5632],
        [-0.2162, -1.0112, -8.0026, -2.5495, 7.1532, 1.8813, 0.0791, 0.3903, 6.2190, 2.3615, 24.6691, 6.6245],
        [-0.2175, -1.0147, -8.1376, -2.5916, 6.9973, 1.8424, 0.0778, 0.3826, 6.1639, 2.3130, 24.8878, 6.6832],
        [-0.2188, -1.0183, -8.2686, -2.6326, 6.8449, 1.8042, 0.0764, 0.3746, 6.1062, 2.2647, 25.0972, 6.7394],
        [-0.2200, -1.0218, -8.3958, -2.6726, 6.6960, 1.7669, 0.0750, 0.3662, 6.0462, 2.2168, 25.2974, 6.7931],
        [-0.2212, -1.0254, -8.5192, -2.7116, 6.5506, 1.7304, 0.0734, 0.3576, 5.9841, 2.1692, 25.4889, 6.8445],
        [-0.2224, -1.0290, -8.6390, -2.7496, 6.4087, 1.6947, 0.0718, 0.3487, 5.9203, 2.1219, 25.6719, 6.8936],
        [-0.2236, -1.0325, -8.7552, -2.7867, 6.2703, 1.6599, 0.0701, 0.3397, 5.8550, 2.0751, 25.8468, 6.9404],
        [-0.2247, -1.0360, -8.8680, -2.8228, 6.1353, 1.6259, 0.0683, 0.3305, 5.7885, 2.0287, 26.0139, 6.9852]])

    def K_of(self, L):
        row = np.array([np.interp(L, self.lengths, self.vals[:, j]) for j in range(12)]).reshape(2, 6)
        return row                       # controller.cpp: lqr_control_ = ground_gain * (expected_state - state)


class QDU(PolyK):
    name = "QDU-Robomaster Wheelleg: 10-state/4-output LQR, bivariate quadratic K(L_l, L_r) (40x6 table), 1 kHz"
    native_L0 = (0.10, 0.40)
    # term order assumed [1, l_l, l_r, l_l^2, l_l*l_r, l_r^2] from the left/right symmetry of the table
    c = np.array([
        [-2.4926, -0.0085557, -1.4237, 3.7989, -1.0144, -0.45636], [-4.6042, -0.102, -2.2408, 6.1744, -2.2534, -2.0711], [-2.8566, -5.9061, 2.4219, 6.7974, -2.3655, -0.20335],
        [-0.64595, -1.7651, 0.94845, 2.0205, -0.64978, -0.34658], [-5.448, -43.896, 10.787, 41.689, 12.233, -10.496], [-0.64886, -3.8569, 0.6421, -0.093447, -0.9605, -0.7316],
        [-3.9692, 19.076, -41.709, -21.817, 13.501, 12.258], [-0.27776, 0.8594, -4.2875, -0.6321, 1.4201, -3.2795], [-3.0172, 8.7533, 2.546, -9.207, -7.7938, 1.7617],
        [-1.0966, 3.277, 0.85392, -3.5286, -2.8362, 0.82452],
        [-2.4926, -1.4237, -0.0085557, -0.45636, -1.0144, 3.7989], [-4.6042, -2.2408, -0.102, -2.0711, -2.2534, 6.1744], [2.8566, -2.4219, 5.9061, 0.20335, 2.3655, -6.7974],
        [0.64595, -0.94845, 1.7651, 0.34658, 0.64978, -2.0205], [-3.9692, -41.709, 19.076, 12.258, 13.501, -21.817], [-0.27776, -4.2875, 0.8594, -3.2795, 1.4201, -0.6321],
        [-5.448, 10.787, -43.896, -10.496, 12.233, 41.689], [-0.64886, 0.6421, -3.8569, -0.7316, -0.9605, -0.093447], [-3.0172, 2.546, 8.7533, 1.7617, -7.7938, -9.207],
        [-1.0966, 0.85392, 3.277, 0.82452, -2.8362, -3.5286],
        [3.8207, 13.933, -28.338, -33.981, 15.526, 37.126], [6.8208, 24.75, -50.247, -59.267, 27.54, 65.109], [-6.4027, 9.014, 4.0619, -9.9542, 30.275, -6.3869],
        [-1.5633, 2.5677, 0.87157, -2.7875, 7.9043, -1.0069], [10.631, 73.518, -15.946, -93.499, 109.25, 20.843], [1.8344, 2.6874, -3.3599, 3.6775, 0.81105, 6.3348],
        [1.1533, -15.818, -62.482, 11.797, -89.649, 54.039], [-0.39484, 0.79831, -1.0451, -4.6228, -0.42961, -5.669], [-14.198, -18.013, 6.6946, 18.333, 5.8638, -5.7598],
        [-5.6903, -7.0214, 2.9398, 7.4797, 1.9812, -2.8077],
        [3.8207, -28.338, 13.933, 37.126, 15.526, -33.981], [6.8208, -50.247, 24.75, 65.109, 27.54, -59.267], [6.4027, -4.0619, -9.014, 6.3869, -30.275, 9.9542],
        [1.5633, -0.87157, -2.5677, 1.0069, -7.9043, 2.7875], [1.1533, -62.482, -15.818, 54.039, -89.649, 11.797], [-0.39484, -1.0451, 0.79831, -5.669, -0.42961, -4.6228],
        [10.631, -15.946, 73.518, 20.843, 109.25, -93.499], [1.8344, -3.3599, 2.6874, 6.3348, 0.81105, 3.6775], [-14.198, 6.6946, -18.013, -5.7598, 5.8638, 18.333],
        [-5.6903, 2.9398, -7.0214, -2.8077, 1.9812, 7.4797]])

    def K_of(self, L):
        terms = np.array([1, L, L, L * L, L * L, L * L])
        k = self.c @ terms                                       # 40 gains
        # 10-state order [x, xd, yaw, yawd, th_l, thd_l, th_r, thd_r, pitch, gyro] -> model 6-state (planar: th_l = th_r)
        def row6(g):
            return np.array([g[4] + g[6], g[5] + g[7], g[0], g[1], g[8], g[9]])
        T = row6(k[0:10]) + row6(k[10:20]); Tp = row6(k[20:30]) + row6(k[30:40])
        return np.vstack([T, Tp])                                # their law: out = sum k*(target - state) = -K x


class HNUST(PolyK):
    name = "HNUST-Dust wbr_control: unified 10-state/4-output LQR, cubic in normalised leg length, 1 kHz"
    native_L0 = (0.15362, 0.31101)

    def __init__(self):
        import re, os
        path = os.environ.get("HNUST_INC", os.path.join(os.path.dirname(os.path.abspath(__file__)), "data", "hnust_coeffs.inc"))
        nums = [float(v) for v in re.findall(r"-?\d+\.\d+(?:e-?\d+)?", open(path).read())]
        self.coef = np.array(nums).reshape(4, 10, 4)

    def K_of(self, L):
        n = (np.clip(L, 0.15362, 0.31101) - 0.25) / 0.15
        g = ((self.coef[:, :, 0] * n + self.coef[:, :, 1]) * n + self.coef[:, :, 2]) * n + self.coef[:, :, 3]   # 4x10
        def row6(r):
            return np.array([r[4] + r[6], r[5] + r[7], r[0], r[1], r[8], r[9]])
        T = row6(g[0]) + row6(g[1]); Tp = row6(g[2]) + row6(g[3])
        return np.vstack([T, Tp])                                # torque = -sum gain*state_error


REGISTRY = {
    "mine_lqi6": MineLQI6, "sky": Skythinker, "jerry": Jerry, "at": ATWheelLeg, "qdu": QDU, "zhouzzw": Zhouzzw,
    "liu": Liu, "hnust": HNUST, "multivac": Multivac, "ljw": LJW, "ljw_fullK": LJWFullK,
}
