"""Dimensions of the AKE wheeled biped (all lengths in mm).

Robot frame: origin on the floor below the midpoint of the two hip axes,
x forward, y left, z up. The model is built in the nominal standing pose
(leg length L0, knee forward) of Plan 4.

Motors: hip = CubeMars AKE50-8 KV110, knee = AKE60-8 KV80, wheel = AKE50-8 KV110.
Both AKE motors carry the stator mount (Ø42 h7 pilot ring, 6 x M2.5 on PCD 36.5)
and the output disc (Ø27, 6 x M3 + 3 x Ø3 dowels on PCD 20) on the SAME front face;
the back is the spinning rotor cap with the encoder magnet. Every mount below
follows CubeMars' own test housing: a wall with a Ø42 pilot pocket, a Ø30 through
hole, and the driven link reaching the output through that hole with a Ø26 boss.
"""
import math

# --- legs (upkie_parts 2.0.0 femur / tibia) ---------------------------------
L1 = 175.0          # hip -> knee, fixed by the upkie upper_leg (hip boss (0,60), knee ring (0,-115))
L2 = 175.0          # knee -> wheel axle, chosen on the upkie lower_leg fork (axle at (0,-115))
R_WHEEL = 75.0      # 150 mm tyre
L0 = 310.0          # standing hip-wheel distance (Plan 4 nominal 0.31 m)
Z_HIP = L0 + R_WHEEL

# femur angle from the downward vertical (knee forward), law of cosines
PHI = math.acos((L1 ** 2 + L0 ** 2 - L2 ** 2) / (2 * L1 * L0))
KNEE_X = L1 * math.sin(PHI)
KNEE_Z = Z_HIP - L1 * math.cos(PHI)

# --- lateral stack of the LEFT leg (robot y of faces) -----------------------
YW = 178.0                  # outer face of the hip plate = torso side face
HIP_PLATE_T = 8.0           # hip plate: 5 mm flange + 3 mm boss flush with the shell
HIP_POCKET = 3.0            # Ø42 pilot pocket on the plate's inner face
GAP_FEMUR = 1.5             # hip plate outer face -> femur hip end
ADAPTER_T = 8.0             # knee adapter (holds the AKE60 stator)
ADAPTER_POCKET = 2.5
GAP_TIBIA = 1.0             # knee adapter outer face -> tibia
ANKLE_T = 8.0               # ankle plate (holds the wheel AKE50 stator)
ANKLE_POCKET = 2.5

Y_FEMUR_IN = YW + GAP_FEMUR                         # femur local z = 131 face
Y_ADAPTER_IN = Y_FEMUR_IN + (131.0 - 123.7)          # femur knee-loop outer face (local z 123.7)
Y_ADAPTER_OUT = Y_ADAPTER_IN + ADAPTER_T
Y_TIBIA_IN = Y_ADAPTER_OUT + GAP_TIBIA               # tibia local z = 131 face
Y_ANKLE_IN = Y_TIBIA_IN - ANKLE_T                    # ankle plate inner face

# upkie local z of a robot y (both links have local +z pointing inboard)
def femur_z(y):
    return 131.0 + Y_FEMUR_IN - y


def tibia_z(y):
    return 131.0 + Y_TIBIA_IN - y


# --- motor faces (from the CubeMars STEP files, motor local y) --------------
AKE50 = dict(ring=-7.0, out=-9.0, back=13.5, can_front=-0.5, can_r=30.6, mass=0.190)
AKE60 = dict(ring=-8.0, out=-8.5, back=16.5, can_front=-0.5, can_r=34.5, mass=0.260)

# --- wheel ------------------------------------------------------------------
TYRE_OD = 150.0
TYRE_ID = 134.0             # = rim seat diameter
TYRE_W = 38.0
RIM_BAND_ID = 126.0
RIM_FLANGE_OD = 138.0
HUB_FLANGE_D = 64.0
HUB_FLANGE_T = 5.0
HUB_BOSS_D = 26.0
RIM_DISC_T = 6.0

# --- torso ------------------------------------------------------------------
TORSO_R_BOTTOM = 66.0       # bottom of the shell is a half cylinder about the hip axis
TORSO_X = 66.0              # half depth
TORSO_TOP = 130.0           # top of the shell above the hip axis
TORSO_TOP_R = 30.0
SHELL_T = 3.0
LID_SPLIT_Z = 102.0         # lid / lower shell split above the hip axis
BOTTOM_SPLIT_Z = -22.0      # removable bottom cover (carries the electronics sled)
BATTERY = (75.0, 310.0, 75.0)   # x, y, z (24 V 15 Ah pack, 1.8 kg)
BATTERY_MASS = 1.8
BATTERY_X = 0.26            # fore-aft battery position, tuned with build_all.suggest_battery_offset()
                            # so the torso COM lies exactly over the hip-axis midpoint
BATTERY_Y = -0.19           # lateral battery position (same purpose)
BATTERY_Z0 = 49.0           # bottom of the battery above the hip axis
TUBE_X, TUBE_Z, TUBE_D, TUBE_WALL = 36.0, 38.0, 16.0, 1.5   # spine tubes between the hip plates
TRAY_T = 3.0
SLED_TOP = -45.0            # electronics sled (Raspberry Pi, pi3hat, power board) under the battery

# clearance the femur crank needs around the hip axis (the upkie femur moves inboard
# of the hip plate from 76 mm below the hip centre on)
FEMUR_CRANK_R = 76.0
