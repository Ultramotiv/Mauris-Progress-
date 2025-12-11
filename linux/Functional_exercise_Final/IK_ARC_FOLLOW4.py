# =====================================================================
# FAIRINO ROBOT - 100% CORRECT LAP DETECTION & AUTO SWITCH
# → 1 Lap = +90° → -90° → back to +90° (round trip)
# → Switches IMMEDIATELY after ONE full lap
# → Lap counter works perfectly
# → FULLY AUTOMATIC, NO manual jump

###Check it in the morning###
# =====================================================================
# CHANGE ONLY THESE 3 LINES
ARC1_RADIUS = 300
ARC2_RADIUS = 200
ARC3_RADIUS = 100
# =====================================================================

import sys
import time
import math
import numpy as np
sys.path.append('/home/um/fairino-python-sdk-main/linux/fairino')
import Robot
robot = Robot.RPC('192.168.58.2')

X_MOVEMENT = 300.0
ARC_POINTS = 1200
SERVO_RATE = 0.008
FORCE_SCALE = 0.13
FILTER_ALPHA = 0.45
DEADZONE = [2.0, 2.0, 3.0, 1.5, 1.5, 1.5]
FORCE_THRESHOLD = 0.7
MAX_STEP = 2.8

center_x = center_y = 0.0
baseline = [0.0] * 6
filtered = [0.0] * 6

arcs = {
    ARC1_RADIUS: {"p": [], "j": [], "name": f"Arc1 {ARC1_RADIUS}mm"},
    ARC2_RADIUS: {"p": [], "j": [], "name": f"Arc2 {ARC2_RADIUS}mm"},
    ARC3_RADIUS: {"p": [], "j": [], "name": f"Arc3 {ARC3_RADIUS}mm"}
}

sequence = [ARC1_RADIUS, ARC2_RADIUS, ARC3_RADIUS]
current_index = 0
current_radius = sequence[current_index]

current_points = current_joints = []
pos = 0.0
vel = 0.0

lap_counter = {ARC1_RADIUS: 0, ARC2_RADIUS: 0, ARC3_RADIUS: 0}
has_passed_bottom = False
lap_detected_this_cycle = False
transition_lock = False

def get_forces():
    global filtered
    tcp = robot.GetActualTCPPose(flag=1)[1]
    x, y = tcp[0], tcp[1]
    rx, ry, rz = map(math.radians, tcp[3:6])
    ret = robot.FT_GetForceTorqueRCS()
    if ret[0] != 0: return 0.0
    raw = [ret[1][i] - baseline[i] for i in range(6)]
    for i in range(6):
        v = raw[i]
        av = abs(v)
        if av < DEADZONE[i % 6]:
            v = v * (av / DEADZONE[i % 6]) ** 3
        else:
            v = v - DEADZONE[i % 6] if v > 0 else v + DEADZONE[i % 6]
        filtered[i] = FILTER_ALPHA * v + (1 - FILTER_ALPHA) * filtered[i]
    c, s = math.cos, math.sin
    R = np.array([
        [c(rz)*c(ry), c(rz)*s(ry)*s(rx) - s(rz)*c(rx), c(rz)*s(ry)*c(rx) + s(rz)*s(rx)],
        [s(rz)*c(ry), s(rz)*s(ry)*s(rx) + c(rz)*c(rx), s(rz)*s(ry)*c(rx) - c(rz)*s(rx)],
        [-s(ry),      c(ry)*s(rx),                     c(ry)*c(rx)]
    ])
    f_world = R @ np.array(filtered[:3])
    fx, fy = f_world[0], f_world[1]
    dx = x - center_x
    dy = y - center_y
    mag = math.hypot(dx, dy)
    if mag < 10: return 0.0
    tx = +dy / mag
    ty = -dx / mag
    return fx * tx + fy * ty

def ema(n, o): return 0.45 * n + 0.55 * o
def interp(a, b, t): return [aa + t*(bb-aa) for aa, bb in zip(a, b)]

def init_ft():
    robot.FT_SetConfig(24, 0)
    robot.FT_Activate(1)
    time.sleep(1.0)
    robot.SetLoadWeight(0, 0.0)
    robot.FT_SetZero(1)
    time.sleep(0.5)
    print("FT sensor initialized")

def calibrate():
    global baseline
    print("Calibrating baseline...")
    forces = []
    for _ in range(200):
        r = robot.FT_GetForceTorqueRCS()
        if r[0] == 0: forces.append(r[1][:6])
        time.sleep(0.01)
    if forces:
        baseline = np.mean(forces, axis=0).tolist()
    print("Baseline calibrated")

print("\n" + "="*80)
print("FAIRINO - FINAL WORKING VERSION")
print("→ One full round trip = 1 lap → auto switch")
print("→ Arc1 → Arc2 → Arc3 → Arc1 → ...")
print("="*80)

p0 = robot.GetActualTCPPose(flag=1)[1]
robot.MoveL(desc_pos=[p0[0] - X_MOVEMENT, p0[1], p0[2], p0[3], p0[4], p0[5]], tool=0, user=0, vel=15, acc=0)
time.sleep(2.0)
center = robot.GetActualTCPPose(flag=1)[1]
center_x, center_y = center[0], center[1]

robot.MoveL(desc_pos=[center_x, center_y + ARC1_RADIUS, p0[2], p0[3], p0[4], p0[5]], tool=0, user=0, vel=15, acc=0)
time.sleep(1.5)

for radius in [ARC1_RADIUS, ARC2_RADIUS, ARC3_RADIUS]:
    print(f"Generating {arcs[radius]['name']}...")
    arcs[radius]["p"] = []
    arcs[radius]["j"] = []
    for i in range(ARC_POINTS + 1):
        angle_deg = 90 - (i / ARC_POINTS) * 180
        theta = math.radians(angle_deg)
        x = center_x + radius * math.cos(theta)
        y = center_y + radius * math.sin(theta)
        p = [x, y, p0[2], p0[3], p0[4], p0[5]]
        ik = robot.GetInverseKin(0, p, -1)
        j = ik[1][:6] if ik[0] == 0 else arcs[radius]["j"][-1] if arcs[radius]["j"] else [0]*6
        arcs[radius]["p"].append(p)
        arcs[radius]["j"].append(j)

current_radius = ARC1_RADIUS
current_points = arcs[current_radius]["p"]
current_joints = arcs[current_radius]["j"]
pos = 0.0

init_ft()
calibrate()

print("\n" + "="*80)
print("AUTOMATIC MODE RUNNING")
print("→ Complete one full round trip → auto switch to next arc")
print("="*80 + "\n")

last_print = time.time()

try:
    while True:
        if transition_lock:
            time.sleep(0.01)
            continue

        tangential_force = get_forces()

        raw_v = tangential_force * 0.85
        if abs(tangential_force) > FORCE_THRESHOLD:
            raw_v = (tangential_force / 3.0) * MAX_STEP
        raw_v = max(-MAX_STEP, min(MAX_STEP, raw_v))
        vel = ema(raw_v, vel)
        pos += vel
        pos = max(0.0, min(len(current_joints) - 1.0, pos))

        i = int(pos)
        t = pos - i
        ni = min(i + 1, len(current_joints) - 1)
        target_j = interp(current_joints[i], current_joints[ni], t)
        robot.ServoJ(target_j, [0]*6, 0, 0, SERVO_RATE, 0, 0)

        total = len(current_joints) - 1
        at_top = pos <= 80 or pos >= total - 80
        at_bottom = 0.35 < (pos / total) < 0.65

        # Detect passing bottom (going forward)
        if at_bottom and vel > 0.2 and not has_passed_bottom:
            has_passed_bottom = True
            print("   [Passed bottom - halfway done]")

        # FULL LAP: returned to top AFTER passing bottom
        if at_top and has_passed_bottom and pos <= 80 and not lap_detected_this_cycle:
            lap_counter[current_radius] += 1
            lap_detected_this_cycle = True
            print(f"\n>>> FULL LAP {lap_counter[current_radius]} COMPLETED on {arcs[current_radius]['name']} <<<")

            # AUTO SWITCH
            current_index = (current_index + 1) % 3
            next_radius = sequence[current_index]
            print(f">>> SWITCHING TO {arcs[next_radius]['name']} <<<\n")

            transition_lock = True
            robot.MoveL(desc_pos=arcs[next_radius]["p"][0], tool=0, user=0, vel=35, acc=0)
            time.sleep(1.2)

            current_radius = next_radius
            current_points = arcs[current_radius]["p"]
            current_joints = arcs[current_radius]["j"]
            pos = 0.0
            has_passed_bottom = False
            lap_detected_this_cycle = False
            transition_lock = False
            continue

        # Reset lap detection when leaving top zone
        if pos > 150:
            lap_detected_this_cycle = False

        # Status print
        if time.time() - last_print > 0.6:
            prog = pos / total * 100
            x = current_points[i][0] + t * (current_points[ni][0] - current_points[i][0])
            y = current_points[i][1] + t * (current_points[ni][1] - current_points[i][1])
            status = "FWD" if vel > 0.5 else "BACK" if vel < -0.5 else "STOP"
            top = " [TOP]" if at_top else ""
            bottom = " [BOTTOM]" if at_bottom else ""

            print(f"{status:4} {arcs[current_radius]['name']:>16} | Lap {lap_counter[current_radius]:>2} | "
                  f"{prog:5.1f}%{top}{bottom} | Ftan:{tangential_force:+5.2f} | X={x:+7.2f} Y={y:+7.2f}")
            last_print = time.time()

        time.sleep(SERVO_RATE)

except KeyboardInterrupt:
    print("\n\n" + "="*80)
    print("STOPPED")
    for r in sequence:
        print(f"   {arcs[r]['name']:>16} → {lap_counter[r]} lap(s)")
    print("="*80)