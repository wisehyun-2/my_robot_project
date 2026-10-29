"""Clearances that the pose sweep does not cover (brief items 2 and 10): the encoder stack of every joint
(chip - magnet, moteus headers / terminator / XT30 cap / PH plugs / cables - rotor, cup - rotor), the joint faces
and bosses, the clamp rings, the wheel hub and the tyre (wheel parts at several spin angles).

    rows = gaps.measure()      # [(item, mm, requirement)] measured with Fusion's minimum distance
"""
import adsk.core

import cadlib as C
import check as K
import params as P
import poses as PO

SIDES = ('L', 'R')
JOINT_LINK = {'hip': 'torso', 'knee': 'femur', 'wheel': 'tibia'}     # link carrying the stator / driver / cup


def occs(link, prefix):
    """Root-context occurrences below a link whose component name starts with prefix."""
    out = []
    for o in C.design().rootComponent.allOccurrences:
        if o.fullPathName.split('+')[0].split(':')[0] == link and o.component.name.startswith(prefix):
            out.append(o)
    return out


def body(occ, prefix):
    for b in occ.bRepBodies:
        if b.name.startswith(prefix):
            return b
    raise KeyError('%s has no body %s' % (occ.fullPathName, prefix))


def dist(a, b):
    return K.min_distance(a, b)


def _driver_set(joint, k):
    """(stator, spacer, board) occurrences of one joint; hip parts sit in the torso, both sides there."""
    link = JOINT_LINK[joint] if joint == 'hip' else JOINT_LINK[joint] + '_' + k
    st, sp, bd = occs(link, 'AKE60-8_KV80_stator'), occs(link, 'magnet spacer'), occs(link, 'moteus')
    if joint == 'hip':                     # pick this side's by y of the bounding box centre (Fusion X = robot y)
        sgn = 1 if k == 'L' else -1
        pick = lambda lst: [o for o in lst if sgn * (o.boundingBox.minPoint.x + o.boundingBox.maxPoint.x) > 0][0]
        return pick(st), pick(sp), pick(bd)
    return st[0], sp[0], bd[0]


def encoder_rows(k):
    rows = []
    for joint in ('hip', 'knee', 'wheel'):
        st, sp, bd = _driver_set(joint, k)
        rows.append(('%s %s: 엔코더 칩(STEP 1.4 높이) – 자석' % (k, joint), dist(body(sp, 'magnet'), bd),
                     '%.2f (자석–기판 %.2f, 실제 칩 1.2 이하면 ≥ %.2f)' % (P.MAGNET_PCB - P.MOTEUS['ic_h'], P.MAGNET_PCB,
                                                                  P.MAGNET_PCB - 1.2)))
        rows.append(('%s %s: moteus(CAN 헤더) – 모터 로터 캡' % (k, joint), dist(bd, st), '≥ 1.5'))
        link = 'torso' if joint == 'hip' else JOINT_LINK[joint] + '_' + k
        cup = [o for o in occs(link, 'cup_' + joint + '_' + k)][0]
        rows.append(('%s %s: 드라이버 컵 – 모터(로터 캔 포함)' % (k, joint), dist(cup, st), '> 0'))
    link = 'tibia_' + k
    st = occs(link, 'AKE60-8_KV80_stator')[0]
    rows.append(('%s wheel: CAN 종단 저항 – 로터 캡' % k, dist(occs(link, 'CAN terminator')[0], st), '≥ 1.5'))
    rows.append(('%s wheel: XT30 캡 – 로터 캡' % k, dist(occs(link, 'XT30 cap')[0], st), '≥ 1.5'))
    for joint in ('hip', 'knee', 'wheel'):
        st, sp, bd = _driver_set(joint, k)
        lk = 'torso' if joint == 'hip' else JOINT_LINK[joint] + '_' + k
        best = 1e9
        for o in occs(lk, 'JST PHR-3') + occs(lk, 'XT30U plug'):
            if (o.boundingBox.minPoint.x + o.boundingBox.maxPoint.x) * (1 if k == 'L' else -1) < 0:
                continue
            best = min(best, dist(o, st))
        rows.append(('%s %s: PH-3·XT30 플러그 – 모터' % (k, joint), best, '≥ 1.5'))
    return rows


def cable_rows():
    """Every harness wire against every motor stator (rotor can and cap) it comes near."""
    root = C.design().rootComponent
    wires = [o for o in root.allOccurrences if o.component.name.startswith('harness_')]
    stators = [o for o in root.allOccurrences if o.component.name.startswith('AKE60-8_KV80_stator')]
    best = (1e9, '', '')
    for st in stators:
        for w in wires:
            if PO._bb_gap(PO._bbox(st), PO._bbox(w)) > min(best[0], 30.0):
                continue
            d = dist(w, st)
            if d < best[0]:
                best = (d, w.fullPathName, st.fullPathName)
    return [('배선(16 AWG·PH-3 선) – 모터 로터 최소 (%s)' % best[1].split('+')[-1], best[0], '≥ 1.5')]


def joint_rows(k):
    th, sh = occs('femur_' + k, 'thigh_')[0], occs('tibia_' + k, 'shank_')[0]
    hp = occs('torso', 'hip_plate_' + k)[0]
    hf, hx = occs('wheel_' + k, 'hub_flange_')[0], occs('wheel_' + k, 'hex_hub_')[0]
    whl = occs('wheel_' + k, 'RC wheel')[0]
    rows = [('%s 허벅지 – hip 판 (면 2.0, 보스 반경 1.0)' % k, dist(th, hp), '> 0'),
            ('%s 허벅지 – 정강이 (면 1.5, 보스 반경 1.0)' % k, dist(th, sh), '> 0'),
            ('%s 무릎 고정 링(허벅지) – 정강이' % k, dist(occs('femur_' + k, 'clamp ring stator')[0], sh), '> 0'),
            ('%s 바퀴 고정 링(정강이) – 허브 플랜지' % k, dist(occs('tibia_' + k, 'clamp ring stator')[0], hf), '> 0'),
            ('%s 허브 플랜지 – 정강이' % k, dist(hf, sh), '> 0'),
            ('%s 육각 허브 – 정강이' % k, dist(hx, sh), '> 0'),
            ('%s 타이어·림 – 정강이' % k, dist(whl, sh), '> 0')]
    return rows


def spin_rows(spins=(0.0, 20.0, 40.0)):
    """Wheel parts against the shank at several wheel angles (the bolt pattern repeats every 60 deg)."""
    rows = []
    for sp in spins:
        PO.apply(PO.transforms(PO.P.L0, 0.0, sp))
        for k in SIDES:
            sh = occs('tibia_' + k, 'shank_')[0]
            best = min(dist(o, sh) for o in occs('wheel_' + k, '') if o.component.name != 'wheel_' + k)
            rows.append(('%s 바퀴 회전 %.0f°: 바퀴 링크 부품 – 정강이 최소' % (k, sp), best, '> 0'))
    PO.reset()
    return rows


def measure():
    rows = []
    for k in SIDES:
        rows += encoder_rows(k) + joint_rows(k)
    rows += cable_rows() + spin_rows()
    return rows
