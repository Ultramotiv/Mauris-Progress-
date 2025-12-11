# this code created on 31st oct 2025
# Move Z_MOVEMENT mm in Z-axis from CURRENT robot position using MoveL
# WITH FT SENSOR + CONTINUOUS FORCE LOGGING (until Ctrl+C)

import sys
sys.path.append('/home/um/fairino-python-sdk-main/linux/fairino')
import Robot
import time
import signal

# --- CONNECT TO ROBOT ---
robot = Robot.RPC('192.168.58.2')
print("Robot connected.")

# --- FT SENSOR INITIALIZATION ---
def init_ft_sensor():
    company = 24
    device = 0
    robot.FT_SetConfig(company, device)
    robot.FT_Activate(0)
    time.sleep(0.5)
    robot.FT_Activate(1)
    time.sleep(0.5)
    robot.SetLoadWeight(0, 0.0)
    robot.SetLoadCoord(0.0, 0.0, 0.0)
    robot.FT_SetZero(0)
    time.sleep(0.5)
    robot.FT_SetZero(1)
    time.sleep(0.5)
    print("FT Sensor initialized and zeroed.")

# --- GRAVITY COMPENSATION ---
baseline_forces = None
gravity_compensation_samples = 100

def calibrate_baseline_forces():
    global baseline_forces
    print("Calibrating baseline forces (gravity compensation)...")
    force_samples = []
    for _ in range(gravity_compensation_samples):
        ft_data = robot.FT_GetForceTorqueRCS()
        if ft_data[0] == 0:
            forces = [
                ft_data[1][0],   # Fx
                -ft_data[1][1],  # Fy (inverted)
                ft_data[1][2],   # Fz
                ft_data[1][3],   # Mx
                ft_data[1][4],   # My
                ft_data[1][5]    # Mz
            ]
            force_samples.append(forces)
        time.sleep(0.01)
    if force_samples:
        baseline_forces = [sum(col) / len(force_samples) for col in zip(*force_samples)]
        print(f"Baseline forces: {[f'{f:.2f}' for f in baseline_forces]}")
    else:
        print("Warning: No baseline captured.")
        baseline_forces = [0.0] * 6

# --- SIGNAL HANDLER (Ctrl+C) ---
def shutdown(sig, frame):
    print("\n\nCtrl+C detected. Stopping robot and exiting...")
    robot.ServoMoveEnd()
    sys.exit(0)

signal.signal(signal.SIGINT, shutdown)

# =============================================================================
# --- INITIAL SETUP ---
# =============================================================================
init_ft_sensor()
calibrate_baseline_forces()

# Display current joint position
current_pose = robot.GetActualJointPosDegree(flag=1)
print("\nCurrent Joint Position [J1, J2, J3, J4, J5, J6]:")
print(current_pose)

# ============================================================================
# LINEAR MOTION: Move Z_MOVEMENT mm in Z-axis + LOG FORCES
# ============================================================================

Z_MOVEMENT = 40.0  # CHANGE THIS (mm)
print("\n" + "="*70)
print(f"Starting Linear Motion: +{Z_MOVEMENT} mm in Z-axis")
print("="*70)

# Get current TCP
tcp_result = robot.GetActualTCPPose(flag=1)
error_code = tcp_result[0]
current_tcp = tcp_result[1] if len(tcp_result) > 1 else None

if error_code != 0 or not current_tcp:
    print(f"Failed to get TCP pose. Error: {error_code}")
    sys.exit(1)

x, y, z, rx, ry, rz = current_tcp
print(f"\nCurrent TCP: X={x:.2f}, Y={y:.2f}, Z={z:.2f}, Rx={rx:.2f}°, Ry={ry:.2f}°, Rz={rz:.2f}°")

# Target: only Z changes
target_desc_pos = [x, y, z + Z_MOVEMENT, rx, ry, rz]
print(f"Target TCP:  X={x:.2f}, Y={y:.2f}, Z={z + Z_MOVEMENT:.2f}, Rx={rx:.2f}°, Ry={ry:.2f}°, Rz={rz:.2f}°")

# Start MoveL
linear_vel = 5.0
print(f"\nExecuting MoveL (vel={linear_vel}%)...")

ret_linear = robot.MoveL(
    desc_pos=target_desc_pos,
    tool=0, user=0,
    joint_pos=[0.0]*7,
    vel=linear_vel,
    acc=0.0,
    ovl=100.0,
    blendR=-1.0,
    exaxis_pos=[0.0]*4,
    search=0,
    offset_flag=0,
    offset_pos=[0.0]*6
)

if ret_linear != 0:
    print(f"MoveL failed immediately: {ret_linear}")
    sys.exit(1)

# =============================================================================
# CONTINUOUS FORCE LOGGING DURING + AFTER MOTION (until Ctrl+C)
# =============================================================================
print("\n" + "FORCE LOGGING (Press Ctrl+C to stop)".center(70))
print("-" * 70)
print(f"{'Fx':>8} {'Fy':>8} {'Fz':>8} {'Mx':>8} {'My':>8} {'Mz':>8}")
print("-" * 70)

try:
    while True:
        # Read force/torque
        ft_data = robot.FT_GetForceTorqueRCS()
        if ft_data[0] == 0:
            raw = [ft_data[1][0], -ft_data[1][1], ft_data[1][2], ft_data[1][3], ft_data[1][4], ft_data[1][5]]
            forces = [raw[i] - baseline_forces[i] for i in range(6)]
            # Apply deadband
            for i in range(6):
                if abs(forces[i]) < 0.5:
                    forces[i] = 0.0
            print(f"{forces[0]:8.2f} {forces[1]:8.2f} {forces[2]:8.2f} {forces[3]:8.2f} {forces[4]:8.2f} {forces[5]:8.2f}", end='\r')
        else:
            print(f"FT error: {ft_data[0]:>54}", end='\r')
        
        time.sleep(0.01)  # ~100 Hz

except KeyboardInterrupt:
    # This will be caught by signal handler too, but we handle it gracefully
    pass

# =============================================================================
# FINAL VERIFICATION (after Ctrl+C)
# =============================================================================
print("\n\n" + "-"*70)
print("Motion + Logging Stopped by User".center(70))
print("-"*70)

# Final TCP position
final_tcp_result = robot.GetActualTCPPose(flag=1)
if final_tcp_result[0] == 0:
    final_tcp = final_tcp_result[1]
    actual_z_move = final_tcp[2] - z
    error_z = abs(actual_z_move - Z_MOVEMENT)
    print(f"Z Movement: {actual_z_move:+.2f} mm (target: +{Z_MOVEMENT} mm)")
    print(f"Error: {error_z:.2f} mm → {'OK' if error_z < 1.0 else 'Warning'}")
    print(f"Final TCP: X={final_tcp[0]:.2f}, Y={final_tcp[1]:.2f}, Z={final_tcp[2]:.2f}")
else:
    print("Failed to read final TCP.")

print("\n" + "="*70)
print("Program ended. Robot is safe.")
print("="*70)