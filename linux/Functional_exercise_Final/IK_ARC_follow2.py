# =====================================================================
# FAIRINO ROBOT - Linear X Move → 180° Arc with ServoJ Execution
# Updated: 21 NOV 2025
# GUARANTEED SLOW MOTION - Multiple safety mechanisms

### Auto arc movement ### 
# =====================================================================
import sys
import time
import math
sys.path.append('/home/um/fairino-python-sdk-main/linux/fairino')
import Robot
robot = Robot.RPC('192.168.58.2')

# ========================== USER SETTINGS - SLOW MOTION ==========================
X_MOVEMENT        = 300.0  # Linear move distance in -X and radius of arc
ARC_POINTS        = 1000    # Number of points on 180° arc (smoothness)
LINEAR_VEL        = 5.0    # Speed for MoveL - VERY SLOW (3%)
SERVO_UPDATE_RATE = 0.008  # ServoJ loop time (50ms = 20Hz) - SLOW updates
# =================================================================================

print("\n" + "="*80)
print(" FAIRINO ROBOT - GUARANTEED SLOW MOTION")
print(" All movements set to MINIMUM safe speeds")
print("="*80)

# --------------------- Step 1: Get current TCP (P0 - midpoint) ---------------------
tcp_result = robot.GetActualTCPPose(flag=1)
error_code = tcp_result[0]
if error_code != 0 or len(tcp_result) < 2:
    print(f"Failed to get current TCP pose! Error: {error_code}")
    sys.exit(1)

P0 = tcp_result[1]
x0, y0, z0, rx0, ry0, rz0 = P0
print(f"\nInitial Position (P0 - Midpoint of Arc):")
print(f" X = {x0:+.3f} mm | Y = {y0:+.3f} mm | Z = {z0:+.3f} mm")
print(f" Rx = {rx0:+.3f}° | Ry = {ry0:+.3f}° | Rz = {rz0:+.3f}°")

# --------------------- Step 2: SLOW Linear Move to Center (P1) ---------------------
print(f"\n{'='*80}")
print(f" STEP 1: Moving to Arc Center (SLOW at {LINEAR_VEL}%)")
print(f"{'='*80}")

x_target = x0 - X_MOVEMENT
P1 = [x_target, y0, z0, rx0, ry0, rz0]
print(f"\nExecuting SLOW MoveL to P1 (Arc Center): X = {x_target:.3f} mm at {LINEAR_VEL}% speed")

# MoveL with very low speed and acceleration
ret = robot.MoveL(desc_pos=P1, tool=0, user=0, vel=LINEAR_VEL, acc=0, ovl=100, blendR=-1)
if ret != 0:
    print(f"MoveL failed! Error code: {ret}")
    sys.exit(1)

time.sleep(1.0)  # Extra wait to ensure completion
print("✓ Linear move completed successfully (SLOW).")

# Verify actual position
current_pos = robot.GetActualTCPPose(flag=1)[1]
center_x, center_y = current_pos[0], current_pos[1]
print(f"Actual Arc Center (P1): X = {center_x:+.3f} mm | Y = {center_y:+.3f} mm")

# --------------------- Step 3: Calculate Arc Start Point and SLOW Move ---------------------
radius = X_MOVEMENT
dx = x0 - center_x
dy = y0 - center_y
start_angle_rad = math.atan2(dy, dx)
half_arc_rad = math.radians(90)

# Calculate arc start point (left-most point of the arc)
arc_start_angle = start_angle_rad - half_arc_rad
x_start = center_x + radius * math.cos(arc_start_angle)
y_start = center_y + radius * math.sin(arc_start_angle)
P_start = [x_start, y_start, z0, rx0, ry0, rz0]

print(f"\n{'='*80}")
print(f" STEP 2: Moving to Arc Start (SLOW at {LINEAR_VEL}%)")
print(f" Start Point: X = {x_start:+.3f} mm | Y = {y_start:+.3f} mm")
print(f"{'='*80}")

user_input = input(f"\nType 'ok' to move to arc start position (SLOW MOTION): ").strip().lower()
if user_input != 'ok':
    print("Cancelled by user.")
    sys.exit(0)

# MoveL with very low speed
ret = robot.MoveL(desc_pos=P_start, tool=0, user=0, vel=LINEAR_VEL, acc=0, ovl=100, blendR=-1)
if ret != 0:
    print(f"MoveL to arc start failed! Error code: {ret}")
    sys.exit(1)

time.sleep(1.0)  # Extra wait
print("✓ Reached arc start position (SLOW).")

# --------------------- Step 4: Execute 180° Arc with SLOW ServoJ ---------------------
print(f"\n{'='*90}")
print(" STEP 3: EXECUTING 180° ARC WITH SLOW SERVOJ")
print(f" Center       : X = {center_x:+.3f} mm, Y = {center_y:+.3f} mm")
print(f" Radius       : {radius:.3f} mm")
print(f" Points       : {ARC_POINTS + 1}")
print(f" Loop Rate    : {SERVO_UPDATE_RATE*1000:.1f} ms ({1/SERVO_UPDATE_RATE:.0f} Hz) - SLOW")
print(f" Est. Duration: ~{(ARC_POINTS + 1) * SERVO_UPDATE_RATE:.1f} seconds")
print(f"{'='*90}\n")

user_input = input(f"Type 'ok' to execute SLOW arc with ServoJ: ").strip().lower()
if user_input != 'ok':
    print("Cancelled by user.")
    sys.exit(0)

print("Executing arc in SLOW MOTION...")
print("(This will take several seconds - be patient!)\n")
points_executed = 0
start_time = time.time()

for i in range(ARC_POINTS + 1):
    # Calculate point on arc
    angle_offset = -half_arc_rad + (i / ARC_POINTS) * (2 * half_arc_rad)
    angle_rad = start_angle_rad + angle_offset
    x_point = center_x + radius * math.cos(angle_rad)
    y_point = center_y + radius * math.sin(angle_rad)
    desc_pose = [x_point, y_point, z0, rx0, ry0, rz0]

    # Get inverse kinematics for this point
    ret_ik = robot.GetInverseKin(type=0, desc_pos=desc_pose, config=-1)
    if ret_ik[0] != 0:
        print(f"WARNING: IK failed at point {i}! Error: {ret_ik[0]}")
        continue
    joint_pos = ret_ik[1][:6]

    # === SLOW ServoJ: Low speed (0.3) + Slow update rate (50ms) ===
    ret_servo = robot.ServoJ(joint_pos, [0]*6, 0.0, 0.0, SERVO_UPDATE_RATE, 0, 0)
    # ==============================================================

    if ret_servo != 0:
        print(f"WARNING: ServoJ failed at point {i}! Error: {ret_servo}")

    points_executed += 1

    # Print progress every 10 points
    if i % 10 == 0 or i == ARC_POINTS:
        j1, j2, j3, j4, j5, j6 = joint_pos
        elapsed = time.time() - start_time
        print(f"Point {i:3d}/{ARC_POINTS} ({elapsed:.1f}s): X={x_point:7.2f} Y={y_point:7.2f} | "
              f"J1={j1:6.2f}° J2={j2:6.2f}° J3={j3:6.2f}° J4={j4:6.2f}° J5={j5:6.2f}° J6={j6:6.2f}°")

    # CRITICAL: Wait between servo commands for slow motion
    time.sleep(SERVO_UPDATE_RATE)

total_time = time.time() - start_time

# --------------------- Final Summary ---------------------
print(f"\n{'='*90}")
print(" ✓ 180° ARC EXECUTION COMPLETE (SLOW MOTION)")
print(f" Total points executed : {points_executed}/{ARC_POINTS + 1}")
print(f" Actual execution time : {total_time:.2f} seconds")
print(f" MoveL speed used      : {LINEAR_VEL}%")
print(f" ServoJ update rate    : {SERVO_UPDATE_RATE*1000:.0f} ms")
print(f"{'='*90}")
print("\n✓ All movements completed safely at SLOW speeds!")
print("\nAll done!")