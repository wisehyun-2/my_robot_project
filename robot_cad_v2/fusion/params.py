"""Dimensions of the wheeled biped v2 (all lengths in mm, masses in kg).

Robot frame: origin on the floor below the midpoint of the two hip axes, x forward, y left, z up.
The model is built in the nominal standing pose (leg length L0 = 310 mm, knee forward, hip angle 0).
Side s = +1 is the left leg, s = -1 the right one; designed parts are mirror images, purchased parts
(motors, drivers) are placed with proper rotations, so their asymmetric features (output dowels,
moteus connectors) are taken from their real placement.

Fixed by the brief (Fusion 360 instructions, 2026-10-28):
  * motors: 6 x CubeMars AKE60-8 KV80 (8:1, 12.5 N m peak, 5 N m rated, 0.26 kg, Ø69 x 25)
  * drivers: 6 x moteus r4.11 (r4.5 mechanical STEP), pi3hat r4.5 on a Raspberry Pi 4B, mjpower-ss
  * legs: upkie 2.0 layout, thigh 175 mm, shank 175 mm, wheel inboard of the shank
  * wheel ~150 mm, bought 1/8 RC tyre on a 17 mm hex hub
  * battery 24 V 15 Ah, 75 x 310 x 75 mm, 1.8 kg, XT60, lying across the robot
  * printed parts PETG
"""
import math

# ----------------------------------------------------------------------------- legs and pose
L1 = 175.0                  # hip -> knee
L2 = 175.0                  # knee -> wheel axle
TYRE_OD = 202.0             # Xiaomi M365 8.5 x 2 solid tyre (10-31, the user's order; 150 RC wheel before)
R_WHEEL = TYRE_OD / 2
L0 = 310.0                  # standing hip-axle distance
L0_MIN, L0_MAX = 200.0, 335.0
HIP_MIN, HIP_MAX = -80.0, 60.0   # hip swing range checked [deg], + swings the leg forward
Z_HIP = L0 + R_WHEEL
PHI = math.acos((L1 ** 2 + L0 ** 2 - L2 ** 2) / (2 * L1 * L0))     # femur angle from the downward vertical
KNEE_X = L1 * math.sin(PHI)
KNEE_Z = Z_HIP - L1 * math.cos(PHI)

# ----------------------------------------------------------------------------- AKE60-8 (CubeMars STEP frame)
# motor local frame: +y = motor axis pointing to the back (rotor cap), the output face at y = -8.5;
# angles a = atan2(z, x).  The phase leads leave the stator front (y = -4) at a = 0, r 25.
AKE = dict(
    out=-8.5,               # output disc face (Ø27, 6 x M3 PCD 20 depth 5, 3 x Ø3 dowel depth 3, Ø12 x 1.5 recess)
    ring=-8.0,              # stator ring face (Ø42 pilot, 3 mm long, 6 x M2.5 PCD 36.5 depth 5)
    pilot_len=3.0,
    stator_front=-4.0,      # exposed stator / lead exit face (r 20.3 .. 30.75)
    can_front=-0.5,         # spinning bell, front rim
    can_r=34.5,
    can_back=14.2,          # bell rim at r 34.5
    cap_outer=14.5,         # cap face r 32 .. 34.2
    cap_face=15.5,          # cap face r 7.9 .. 31
    cap_r1=31.0,
    magnet=16.5,            # Ø9 boss end face = magnet face (Ø6 x 2.5 pocket)
    boss_r=4.5,
    lead_r=25.0,
    mass=0.260,
    out_mass=0.020,         # output carrier part of the mass (moves with the driven link)
)
AKE_RING_M25 = (-150, -90, -30, 30, 90, 150)     # stator ring screw angles (deg)
AKE_OUT_M3 = (-180, -120, -60, 0, 60, 120)
AKE_OUT_PIN = (-150, -30, 90)
AKE_PCD_RING = 36.5
AKE_PCD_OUT = 20.0

# ----------------------------------------------------------------------------- moteus r4.11 (r4.5 STEP)
# board frame: origin = encoder IC centre on the motor-side PCB face, +y away from the motor,
# +z towards the connector edge.  STEP frame = board frame + (0, 0, 8).
MOTEUS = dict(
    step_dz=8.0,
    pcb_t=1.65,
    ic_h=1.40,              # AS5047P above the motor-side face
    can_h=6.20,             # JST PH-3 side-entry headers on the motor side
    top_h=5.50,             # XT30PW on the far side
    edge_conn=35.0,         # connector edge, from the IC
    edge_far=-18.0,
    half_w=23.0,
    tab_half_w=16.0,        # the connector tab is 32 wide (z 26.3 .. 35)
    holes=((-20.0, -15.0), (20.0, -15.0), (-20.0, 23.0), (20.0, 23.0)),
    phase=((-14.5, 20.2), (0.0, 20.2), (14.5, 20.2)),
    mass=0.0142,
)
SPACER_H = 5.35             # non-magnetic (6061) spacer body under the magnet (±0.05): CAN headers / terminator clear
                            # the cap, and the magnet sits MAGNET_PCB from the board
SPACER_SPIGOT = (5.90, 2.3)  # Ø ±0.05 x length into the empty rotor magnet seat (AKE drawing: magnet Ø6 x 2.5, seat not
                             # dimensioned; Ø6 STEP)
SPACER_POCKET = (6.20, 2.5)  # magnet pocket Ø ±0.05 x depth ±0.05: magnet Ø6 ±0.1 (GB/T1804-m) always fits, offset <= 0.18
MAGNET_PCB = 2.55           # magnet top -> PCB motor-side face (10-29): AS5047P Bz 35..70 mT on the r 1.1 Hall circle at
                            # the die (~0.85 above the PCB; real TSSOP <= 1.20 tall, the STEP chip is ic_h) for N35..N42,
                            # 0..80 C, magnet ±0.1 -> window 2.34..2.76, so ±0.21 for the rest of the stack (~52 mT nominal)
LEAD_ANGLE = 20.0           # motor lead direction in the board frame (deg from +x towards +z)
CUP_WALL = 2.4
CUP_CLEAR_TOP = 1.0         # XT30 top -> cup floor


def board_offset():
    """Board PCB motor-side face along the motor axis (motor local y)."""
    return AKE['magnet'] + SPACER_H + MAGNET_PCB


def cup_floor_in():
    return board_offset() + MOTEUS['pcb_t'] + MOTEUS['top_h'] + CUP_CLEAR_TOP


# ----------------------------------------------------------------------------- lateral stack (left side, y outward)
POCKET = 2.5                # Ø42 pilot pocket depth (pilot is 3 mm long)
YW = 172.0                  # hip plate hub face = torso side face
HIP_PLATE_T = 8.0           # hub (5 mm web + 3 mm boss through the shell); with the 1.5 mm cup register boss
                            # on the inner face the machined plate is 9.5 mm thick (10 mm+ stock)
HIP_WEB_T = 5.0
GAP_FEMUR = 2.0
FEMUR_IN_RELIEF = (1.5, 40.0)  # thigh inner face set back 1.5 mm beyond r 40 from the hip axis, except the knee housing
                            # (10-31): the shell's ISO 7380 M3 x 6 heads (1.65 high on the y = YW face) stood 0.35 mm
                            # from the thigh, now 1.85; the hip head (boss joint) and the knee cup seat keep 174
FEMUR_T = 30.0
FEMUR_H = 50.0              # in-plane depth of the thigh beam
KNEE_PLATE_T = 9.0          # femur knee plate (outer face), the knee motor hangs inboard of it
GAP_TIBIA = 1.5
TIBIA_T = 30.0
TIBIA_H = 56.0
WHEEL_PLATE_T = 9.0         # shank wheel plate (inner face), the wheel motor sits outboard of it
# PETG lateral stiffening (calc/strength.py): both links grow outboard where no other part ever sweeps -
# the thigh from the hip down to WIDE_END before the knee (the shank's knee head sits outboard of the thigh
# there), the shank from the knee down to WIDE_END before the axle (wheel cup and driver plugs below);
# full width up to WIDE_END + WIDE_TAPER, then the outer face ramps back.
FEMUR_WIDE_OUT, FEMUR_WIDE_END = 250.0, 55.0
TIBIA_WIDE_OUT, TIBIA_WIDE_END = 260.5, 50.0
WIDE_TAPER = 15.0
HEAD_R = 30.0               # link head around a driven boss
HOUSING_R = 44.0            # link housing around a motor (can r 34.5 + 2.5 clearance + 7 wall with M2.5 inserts)
CAN_CLEAR_R = 37.0
BEARING_FACE = 11.2         # motor output face -> washer seat of the hip / knee M3 x 16 (washer 0.5, 4.3 mm engagement)
HUB_SCREW_SEAT = 9.5        # wheel hub flange: output face -> washer seat of its 6 x M3 x 14 (12.9) (10-30: with the
HUB_SCREW_L = 14            # hip / knee 11.2 seat the M3 x 16 heads stood 1.2 mm into the hex hub's Ø20 register spigot;
                            # now the heads stop 0.5 mm short of it, 4.0 mm engagement in the 5 mm output taps)
RING_T = 2.0                # laser-cut 2 mm stainless clamp rings (1.5 mm sheet not stocked; 1 mm too soft, strength.py):
                            # screw heads never bear on bare PETG at a joint; screw lengths do not depend on it
RING_OUT = (27.0, 12.5)     # driven bosses (OD inside the Ø28 access pocket, ID): 6 x M3 x 16 12.9 on PCD 20
RING_STATOR = (45.0, 30.0)  # knee / wheel plates, 6 x M2.5 on PCD 36.5; ID = the Ø30 plate hole (1.0 mm radial to the
                            # turning Ø28 boss) so the web inside the Ø2.8 holes is 1.85 mm (~0.9 x the 2 mm sheet)
STATOR_RECESS_D = 45.6      # recess on the far face of the PETG motor plates for the stator ring + low heads
CBORE_M25 = (4.8, 1.7)      # DIN 7984 low head M2.5 counterbore (dia, depth)
CBORE_M25_HIP = (4.8, 2.7)  # ISO 4762 M2.5 12.9 at the hip stator ring (strength.py: 8.8 is over proof at SF 2)
PILOT_D_HIP = 42.0125       # machined hip plate pilot pocket: Ø42 H7 (+0 / +0.025) modelled at mid tolerance
                            # (the printed knee / wheel pockets keep Ø42.05, legs.py)
HIP_STATOR_RELIEF = (63.0, 0.5)  # Ø x depth cut into the Ø80 boss face (10-30): the pocket is 4.0 deep and the AKE stator
                            # front (r <= 30.75) is 4.0 from the pilot end, so the stator met the boss face as the pilot
                            # met the pocket floor; with the relief the pilot end alone locates the motor (0.5 clear),
                            # like the printed knee / wheel plates (1.5 clear); the Ø63..Ø80 rim still registers the cup
CSK_M5 = (5.4, 11.2)        # hip plate web, ISO 10642 heads: through hole Ø, 90 deg countersink Ø = theoretical dk
CSK_M4 = (4.4, 9.0)         #   (M5 11.20, M4 8.96 rounded up), so no compliant head stands proud of the web
# lead groove of every motor plate (10-29; hip 6061, knee / wheel PETG): the AKE60 phase pins (Ø1.5 x 4 at r 25 on
# the lead direction, 3 mm apart) reach the hip boss face and stop 1.5 mm short of the PETG plate faces
LEAD_GROOVE_R = (23.0, 41.0)  # radial span: 2.0 mm wall to the Ø42 pocket (was 21.5 -> 0.5 mm fin)
LEAD_GROOVE_W = 12.0          # width: the pins span 7.5 mm, so 2.25 mm each side for the soldered wires (7 mm put the
                              # outer pins on the groove edges)
# hip plate thin walls (10-29, manufacturing/wall_check.py; nothing under 1.5 mm is left on the machined plate):
HIP_CBORE_OPEN = 14.0        # M2.5 counterbores run out into the Ø30 hole (to r 14): the 0.86 mm wall between them
                             # cannot grow (PCD 36.5 fixed by the AKE ring, Ø30 by the turning Ø28 boss), so it goes
HIP_CSK_EAR_R = 8.0          # web ear around the tube M5 countersinks (r 61.7): 2.4 mm wall to the countersink rim
                             # (0.83 mm at the r 68 disc), the ear stands 1.7 mm proud of the disc
TAP_DRILL = {'M3': 2.5, 'M4': 3.3, 'M5': 4.2}
END_TAP = {'M5': (14.0, 11.0), 'M4': (12.0, 9.0)}   # spine / rail end taps: drill depth, full-thread depth
                            # (the ISO 10642 x 12 screws from the hip plate engage ~7.5 mm)
GAP_TYRE = 5.0              # tyre sidewall -> shank plate

Y_FEMUR_IN = YW + GAP_FEMUR
Y_FEMUR_OUT = Y_FEMUR_IN + FEMUR_T
Y_TIBIA_IN = Y_FEMUR_OUT + GAP_TIBIA
Y_TIBIA_OUT = Y_TIBIA_IN + TIBIA_T

# motor ring faces (robot y, left side) and axis directions (motor +y_local)
Y_RING_HIP = YW - HIP_PLATE_T + POCKET          # ring on the plate's inner pocket, motor inboard
Y_RING_KNEE = Y_FEMUR_OUT - KNEE_PLATE_T + POCKET
Y_RING_WHEEL = Y_TIBIA_IN + WHEEL_PLATE_T - POCKET
AXIS_HIP, AXIS_KNEE, AXIS_WHEEL = -1, -1, +1    # sign of the motor back direction along robot y (left side)

# ----------------------------------------------------------------------------- wheel (10-31: the user's tyre)
# Xiaomi M365 8 1/2 x 2 solid honeycomb tyre (AliExpress listing photo: OD 202, tread 54 wide, bore Ø141 x 36.5 wide,
# 721 g; rectangular cavities open to the bore, the rubber between them sits on the rim).  The section between the
# bore band and the full-width tread is not given: modelled as a TYRE_BEAD_H band of bore width, then a straight
# flare to the full width at TYRE_FLARE_R (measure the real tyre).  Printed one-piece rim: legs.rim (10-31 rev 2:
# rim_out merged, lugs in the bore cavities, thick hub; strength.py section 3).
TYRE_W = 54.0
TYRE_BORE_D, TYRE_BORE_W = 141.0, 36.5
TYRE_BEAD_H, TYRE_FLARE_R = 5.0, 85.0
TYRE_CAVITY = (24, 9.0, 10.0, 27.5)  # bore cavities (modelled from the listing photo): count, width, depth, length
TYRE_MASS = 0.721
RIM_SEAT_T = 6.0            # seat ring wall (r 64.5 .. 70.5) the tyre is stretched onto
RIM_FLANGE_D = 150.0        # bead flanges (inside the bead band, 0.5 short of the flare)
RIM_FLANGE_T = 3.0          # open-side flange (the tyre goes on over it, like the M365 one-piece rim)
RIM_LUGS = (24, 8.0, 4.5, 26.5)  # lugs on the seat ring into the tyre's bore cavities: count, width, height (tips at the
                            # flange radius, so mounting stretches the tyre no further than the flange), length; both
                            # ends 45 deg (print overhang / mounting lead-in).  Re-measure the cavities on the real tyre.
RIM_HUB_T, RIM_HUB_R = 10.0, 32.0   # rim hub thickness from the flange face, out to this radius (the 4.75 mm disc of
                            # the first rim was 81 MPa at the design load at the flange edge r 23, strength.py)
RIM_WEB_T = 6.0             # web thickness at the seat ring (straight taper from RIM_HUB_T at RIM_HUB_R), no holes
RIM_SCREW_SEAT = 9.0        # flange face -> washer seat of the 6 x M3 x 14 (washer 0.5, 4.5 mm reach past the flange face:
                            # 4.4 mm in the 5.7 inserts, tip 1.5 mm above the Ø4 x 6 hole floor -- 10-31: 8 left 0.5 mm,
                            # less than screw length (js15 +-0.35) + print + melted-in PETG under a heat-set insert)
RIM_REG = (5.5, 3.0)        # centre register: Ø11 x 3 spigot on the hub flange (inside the output screw counterbores,
                            # 0.75 mm clear), so the rim's flange face prints flat on the bed
RIM_REG_GAP = (0.1, 0.2)    # rim recess clearance (radial, depth): Ø11.2 x 3.2 -- the radial gap is half the rim screw
                            # holes' (Ø3.4 on M3: 0.2), so the register takes the wheel load before the screws (printed
                            # holes come out ~0.1-0.2 small: near a snug fit)
HUB_BOSS_D = 28.0           # through the Ø30 plate hole


def tyre_centre_y():
    return Y_TIBIA_IN - GAP_TYRE - TYRE_W / 2


def wheel_offset():
    """Output face of the wheel motor -> tyre centre plane (bending arm on the motor output)."""
    return y_output('wheel') - tyre_centre_y()


def y_output(joint):
    """Robot y (left side) of a motor's output face."""
    ring, a = {'hip': (Y_RING_HIP, AXIS_HIP), 'knee': (Y_RING_KNEE, AXIS_KNEE), 'wheel': (Y_RING_WHEEL, AXIS_WHEEL)}[joint]
    return ring + a * (AKE['out'] - AKE['ring'])


# ----------------------------------------------------------------------------- torso (TITA-like chamfered box)
# TITA proportions (length ≈ 1.4 x width, hip axis at mid-length).  The front / rear sections are narrower than the
# hip section: the knee drivers (y >= 154) and the tyres (y >= 140.5) swing past them over the full hip range.
TORSO_X = 240.0             # front / rear end (length 480)
TORSO_TOP = 148.0           # top surface above the hip axis (the payload plate stands 2 mm proud)
TORSO_BOT = -70.0
TORSO_Y_END = 135.0         # half width of the front / rear sections
TORSO_X_MID, TORSO_X_TAPER = 75.0, 95.0     # full-width hip section |x| <= 75, tapering to TORSO_Y_END at |x| 95
FRONT_TOP_X = 210.0         # side view, front: top edge drops from x 210 to the front corner (TORSO_X, FRONT_Z) ...
FRONT_Z = 125.0
FRONT_BOT_X = 105.0         # ... and the face slopes back to the bottom at x 105 (TITA's raked face)
# side view, rear (user's sketch 2026-10-28; the end the knee crooks face): an upper slope from the top at -REAR_TOP_X
# down to a short vertical face (REAR_Z_TOP .. REAR_Z_BOT at x = -TORSO_X) and a lower slope back to the bottom at
# -REAR_BOT_X; the sketch's ratios: face ≈ 0.48 of the height, slopes ≈ 22 / 12 deg from horizontal.
REAR_TOP_X, REAR_Z_TOP = 73.0, 81.0
REAR_BOT_X, REAR_Z_BOT = 34.0, -25.0
PLAN_CORNER = 30.0          # vertical corner chamfers of the rear end (plan view; the front ones went 10-31)
SADDLE_Z = 95.0             # blue 'saddle' = hip-section shell above this height outside the end width
SWITCH_Y, SWITCH_Z = 88.0, 45.0            # rocker switch on the vertical rear face (inside the corner chamfers)
DECK_T_FLOOR = 2.4
SHELL_T = 1.4
BATTERY = (75.0, 310.0, 75.0)  # x, y, z
BATTERY_MASS = 1.8
BATTERY_X = 1.3             # tuned with check.suggest_battery_offset(): torso COM over the hip-axis midpoint
BATTERY_Y = -0.1
TUBE_X, TUBE_Z, TUBE_D = 47.0, 40.0, 16.0      # 2 x Ø16 6061-T6 solid spine bars (battery), ends tapped M5
RAIL_X, RAIL_Z, RAIL_S = 52.0, 135.5, 15.0     # 2 x 15 x 15 6061-T6 solid top rails (payload), ends tapped M4
# (10-29: solid bars tapped at the ends replace the Ø16 x 1.0 tube / 15 x 15 x 1.0 rail + 8 glued end plugs:
#  4 fewer part kinds and no glue joint in the load path, for +0.53 kg)
TRAY_Z = TUBE_Z + TUBE_D / 2           # tray underside on the tubes
TRAY_T = 2.4                           # floor; 7 ribs (2.4 x 8) span the 94 mm between the tubes (calc/strength.py)
TRAY_RIBS = (0.0, -38.0, 38.0, -76.0, 76.0, -114.0, 114.0)
BATTERY_Z0 = TRAY_Z + TRAY_T + 1.0     # foam pad
TOP_PLATE = (120.0, 210.0, 6.0)        # x, y, thickness (PETG, M4 inserts on a 25 mm grid)
DECK_Z = -55.0                          # electronics deck top
DECK_T = 3.0
HIP_CONN_DEG = 255.0        # hip driver connector edge (from down, towards +x): back, clear of the spine tube
KNEE_CONN_DEG = 270.0       # knee driver connector edge: straight back, into the knee wedge
WHEEL_CONN_DEG = 215.0      # wheel driver connector edge: up and back, clear of the widened shank (beam at ~152 deg)
GROMMET_DEG, GROMMET_R = 216.0, 75.0   # hip cable exit through the shell (outside the hip plate disc), never swept by the thigh

# ----------------------------------------------------------------------------- print settings (PETG, 0.4 nozzle)
# part name prefix: (perimeter / skin thickness mm, infill fraction).  Printed mass = rho * (A t + f (V - A t)).
PRINT = {
    'thigh': (4.8, 0.20), 'shank': (4.8, 0.20),                 # 12 walls, 8 top / bottom layers, 20 % gyroid
    'hub_flange': (99.0, 1.0),                                  # 100 %
    'rim_': (2.4, 0.40),                                        # 6 walls, 40 %
    'cup_': (1.6, 0.30), 'cable_clips': (1.6, 0.40),
    'top_plate': (1.6, 0.25), 'battery_tray': (1.6, 0.30), 'electronics_deck': (1.6, 0.30),
    'shell_': (1.6, 0.25), 'saddle_': (1.6, 0.25),
    'splice_tabs': (1.6, 1.0),
}
PETG_RHO = 1270.0

# ----------------------------------------------------------------------------- loads (brief, item 3/4)
G = 9.81
ROBOT_KG = 7.0
PAYLOAD_KG = 5.0
WHEEL_STATIC = (ROBOT_KG + PAYLOAD_KG) * G / 2      # ≈ 60 N
WHEEL_DESIGN = 5 * WHEEL_STATIC                      # ≈ 300 N
SAFETY = 2.0
LATERAL = 0.5
MOTOR_PEAK = 12.5


def theta_dir(deg):
    """Unit vector (x, z) in the leg plane for an angle measured from straight down towards +x."""
    t = math.radians(deg)
    return (math.sin(t), -math.cos(t))
