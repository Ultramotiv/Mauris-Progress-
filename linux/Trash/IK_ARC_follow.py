# =====================================================================
# FAIRINO ROBOT - Linear X Move → 180° Arc with P1 as Center, P0 as Midpoint
# Updated: 21 NOV 2025

## Only prints values of the arc ###
# =====================================================================

import sys
import time
import math

sys.path.append('/home/um/fairino-python-sdk-main/linux/fairino')
import Robot

robot = Robot.RPC('192.168.58.2')

# ========================== USER SETTINGS ==========================
X_MOVEMENT = 300.0          # Linear move distance in -X and radius of arc
ARC_POINTS = 100            # Number of points to calculate on 180° arc (smoothness)
LINEAR_VEL = 8.0            # Speed for linear move (%)
# ==================================================================

print("\n" + "="*70)
print("  FAIRINO ROBOT - LINEAR MOVE + 180° ARC PATH GENERATION")
print("="*70)

# Step 1: Get current TCP → P0 (this will be the MIDPOINT of the arc)
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

# Step 2: Ask for confirmation
user_input = input(f"\nType 'ok' to move -{X_MOVEMENT} mm in X (to arc center): ").strip().lower()
if user_input != 'ok':
    print("Cancelled by user.")
    sys.exit(0)

# Step 3: Move linearly in -X direction → reach P1 (center of arc)
x_target = x0 - X_MOVEMENT
P1 = [x_target, y0, z0, rx0, ry0, rz0]

print(f"\nExecuting MoveL to P1 (Arc Center): X = {x_target:.3f} mm")
ret = robot.MoveL(
    desc_pos=P1,
    tool=0, user=0,
    joint_pos=[0.0]*7,
    vel=LINEAR_VEL,
    acc=0.0,
    ovl=100.0,
    blendR=-1.0,
    exaxis_pos=[0.0]*4,
    search=0,
    offset_flag=0,
    offset_pos=[0.0]*6
)

if ret != 0:
    print(f"MoveL failed! Error code: {ret}")
    sys.exit(1)

time.sleep(0.8)
print("Linear move completed successfully.")

# Verify current position (should be ≈ P1)
current_pos = robot.GetActualTCPPose(flag=1)[1]
center_x = current_pos[0]
center_y = current_pos[1]
print(f"Actual position after move → Arc Center (P1):")
print(f" X = {center_x:+.3f} mm | Y = {center_y:+.3f} mm")

# Step 4: Define 180° arc
# - Center: P1 (current position)
# - Radius: X_MOVEMENT
# - Midpoint: P0 → this defines the starting angle

dx = x0 - center_x
dy = y0 - center_y
radius = X_MOVEMENT

# Starting angle (from center to P0)
start_angle_rad = math.atan2(dy, dx)

# We want 180° arc with P0 as midpoint → so:
# Start angle = start_angle_rad - 90°
# End angle   = start_angle_rad + 90°
half_arc_rad = math.radians(90)

print(f"\n{'='*80}")
print("  180° ARC PATH GENERATION")
print(f"  Center (P1)      : X = {center_x:+.3f} mm, Y = {center_y:+.3f} mm")
print(f"  Radius           : {radius:.3f} mm")
print(f"  Midpoint (P0)    : X = {x0:+.3f} mm, Y = {y0:+.3f} mm")
print(f"  Arc Start Angle  : {math.degrees(start_angle_rad - half_arc_rad):.1f}°")
print(f"  Arc End Angle    : {math.degrees(start_angle_rad + half_arc_rad):.1f}°")
print(f"  Total Points     : {ARC_POINTS}")
print(f"{'='*80}\n")

# Header
print("No. |      X (mm)      |      Y (mm)      |   Z (mm)   |   Rx (°)   |   Ry (°)   |   Rz (°)")
print("-" * 95)

arc_points = []

for i in range(ARC_POINTS + 1):
    # Angle from -90° to +90° relative to midpoint direction
    angle_offset = -half_arc_rad + (i / ARC_POINTS) * (2 * half_arc_rad)
    angle_rad = start_angle_rad + angle_offset

    x_point = center_x + radius * math.cos(angle_rad)
    y_point = center_y + radius * math.sin(angle_rad)

    point = [x_point, y_point, z0, rx0, ry0, rz0]
    arc_points.append(point)

    # Print each point
    print(f"{i:3d} | {point[0]:+12.3f} | {point[1]:+12.3f} | {point[2]:+8.3f} | "
          f"{point[3]:+8.3f} | {point[4]:+8.3f} | {point[5]:+8.3f}")

# Final summary
print(f"\n{'='*80}")
print("  180° ARC GENERATION COMPLETE")
print(f"  Total TCP points generated: {len(arc_points)}")
print(f"  Start Point : [{arc_points[0][0]:.3f}, {arc_points[0][1]:.3f}, {z0:.3f}]")
print(f"  Mid Point   : [{x0:.3f}, {y0:.3f}, {z0:.3f}] ← P0")
print(f"  End Point   : [{arc_points[-1][0]:.3f}, {arc_points[-1][1]:.3f}, {z0:.3f}]")
print(f"  You can now copy these points and use robot.Circle() or MoveL sequence!")
print(f"{'='*80}")