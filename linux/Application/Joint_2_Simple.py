# Simple Joint 2 Movement to -150 degrees
# Get current joint angles -> Show target -> Confirm -> Move

import sys
sys.path.append('/home/um/fairino-python-sdk-main/linux/fairino')
import Robot
import time

# --- SAFETY LIMITS FOR EACH JOINT (degrees) ---
JOINT_SAFETY_LIMITS = {
    1: (-90.0, 85.0),      # Joint 1: min, max
    2: (-179.0, -35.0),   # Joint 2: min, max
    3: (60.0, 144.0),      # Joint 3: min, max
    4: (-258.0, 80.0),    # Joint 4: min, max
    5: (-170.0, 12.0),    # Joint 5: min, max
    6: (-170.0, 170.0),   # Joint 6: min, max
}

TARGET_JOINT_2 = -150.0

# --- MOVEMENT PARAMETERS ---
MOVEMENT_SPEED = 26.0  # degrees per second - CHANGE THIS TO CUSTOMIZE SPEED
MOVEMENT_ACCELERATION = 0.0  # 0 = use default
MOVEMENT_TIME_STEPS = 0  # 0 = automatic

# --- JOINT TORQUE MONITORING ---
def print_joint_torques():
    """Get and display current joint torques"""
    error, torques = robot.GetJointTorques(flag=1)
    if error == 0:
        print("\n📊 CURRENT JOINT TORQUES:")
        print("="*70)
        joint_names = ["Joint 1", "Joint 2", "Joint 3", "Joint 4", "Joint 5", "Joint 6"]
        for i, torque in enumerate(torques):
            print(f"  {joint_names[i]}: {torque:+8.2f} Nm")
        print("="*70 + "\n")
    else:
        print(f"❌ Error reading torques: {error}\n")

# --- CONNECT TO ROBOT ---
print("🤖 Connecting to Robot...")
robot = Robot.RPC('192.168.58.2')
print("✅ Robot connected.\n")

# --- GET CURRENT JOINT ANGLES ---
print("📍 Getting current joint angles...")
error, current_pos = robot.GetActualJointPosDegree()
if error != 0:
    print(f"❌ Error getting joint positions: {error}")
    sys.exit(1)

print("\n" + "="*70)
print("📊 CURRENT JOINT ANGLES:")
print("="*70)
joint_names = ["Joint 1", "Joint 2", "Joint 3", "Joint 4", "Joint 5", "Joint 6"]
for i, angle in enumerate(current_pos):
    print(f"  {joint_names[i]}: {angle:+8.2f}°")

# --- CREATE TARGET POSITION (Joint 2 at -150, others stay same) ---
target_pos = current_pos.copy()
target_pos[1] = TARGET_JOINT_2  # Index 1 is Joint 2

print("\n" + "="*70)
print("🎯 TARGET JOINT ANGLES:")
print("="*70)
for i, angle in enumerate(target_pos):
    if i == 1:  # Joint 2
        print(f"  {joint_names[i]}: {angle:+8.2f}° ⭐ TARGET")
    else:
        print(f"  {joint_names[i]}: {angle:+8.2f}°  (unchanged)")

# --- CHECK SAFETY LIMITS ---
print("\n" + "="*70)
print("🔒 SAFETY CHECK:")
print("="*70)
safety_ok = True
for j in range(6):
    joint_num = j + 1
    min_lim, max_lim = JOINT_SAFETY_LIMITS[joint_num]
    angle = target_pos[j]
    within_limits = min_lim <= angle <= max_lim
    
    status = "✅ OK" if within_limits else "❌ OUT OF RANGE"
    print(f"  {joint_names[j]}: {angle:+8.2f}° (range: {min_lim:+7.1f}° to {max_lim:+7.1f}°) {status}")
    
    if not within_limits:
        safety_ok = False

if not safety_ok:
    print("\n❌ One or more target angles are outside safety limits!")
    print("Cannot proceed. Exiting.")
    sys.exit(1)

# --- ASK USER FOR CONFIRMATION ---
print("\n" + "="*70)
print("⚠️  CONFIRMATION REQUIRED:")
print("="*70)
user_input = input("Are these target joint angles correct? (yes/no): ").strip().lower()

if user_input != "yes" and user_input != "y":
    print("\n❌ Operation cancelled by user. Exiting.")
    sys.exit(0)

# --- START SERVO MODE ---
print("\n🚀 Starting Servo Mode...")
if robot.ServoMoveStart() != 0:
    print("❌ Failed to start servo mode.")
    sys.exit(1)

print("✅ Servo mode started.\n")

# --- PRINT INITIAL TORQUES ---
print_joint_torques()

# --- MOVE TO TARGET ---
print("📌 Moving Joint 2 to -150° at 26°/sec...")
print(f"   Target position: {target_pos}")

# Calculate movement distance and time
j2_distance = abs(target_pos[1] - current_pos[1])
movement_speed = 26.0  # degrees per second
movement_time = j2_distance / movement_speed if movement_speed > 0 else 1.0

print(f"   Distance: {j2_distance:.2f}°")
print(f"   Speed: {movement_speed:.1f}°/sec")
print(f"   Estimated time: {movement_time:.2f} seconds\n")

# Move smoothly with velocity control
dt = 0.01  # 10ms update rate
elapsed_time = 0.0
start_pos = current_pos.copy()

while elapsed_time <= movement_time:
    # Calculate progress (0.0 to 1.0)
    progress = elapsed_time / movement_time if movement_time > 0 else 1.0
    progress = min(progress, 1.0)  # Cap at 1.0
    
    # Interpolate position for Joint 2
    interp_pos = start_pos.copy()
    interp_pos[1] = start_pos[1] + (target_pos[1] - start_pos[1]) * progress
    
    # Send servo command
    err = robot.ServoJ(joint_pos=interp_pos, axisPos=[0]*6)
    if err != 0:
        print(f"❌ ServoJ error: {err}")
        robot.ServoMoveEnd()
        sys.exit(1)
    
    # Print torques every 200ms (20 iterations at 10ms each)
    if int(elapsed_time * 1000) % 200 == 0:
        error, torques = robot.GetJointTorques(flag=1)
        if error == 0:
            progress_pct = (progress * 100)
            print(f"📍 Progress: {progress_pct:5.1f}% | Joint 2 Torque: {torques[1]:+7.2f} Nm", end='\r')
    
    elapsed_time += dt
    time.sleep(dt)

print("✅ Movement complete!")
print(f"   Total time: {elapsed_time:.2f} seconds\n")

# --- PRINT FINAL TORQUES ---
print_joint_torques()

# --- STOP SERVO MODE ---
print("\n🛑 Stopping Servo Mode...")
robot.ServoMoveEnd()
print("✅ Servo mode stopped.\n")

# --- PRINT FINAL TORQUES AFTER STOPPING ---
print_joint_torques()

# --- VERIFY FINAL POSITION ---
print("📍 Verifying final joint angles...")
error, final_pos = robot.GetActualJointPosDegree()
if error == 0:
    print("\n" + "="*70)
    print("📊 FINAL JOINT ANGLES:")
    print("="*70)
    for i, angle in enumerate(final_pos):
        diff = angle - target_pos[i]
        print(f"  {joint_names[i]}: {angle:+8.2f}° (target: {target_pos[i]:+8.2f}°, diff: {diff:+7.2f}°)")
else:
    print(f"❌ Error reading final position: {error}")

print("\n✅ Movement complete!")
