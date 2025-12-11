#!/usr/bin/env python3
# FAIRINO COBOT — PURE Y-AXIS ADMITTANCE WITH FOXGLOVE INTEGRATION
# Velocity increases smoothly with applied force → feels PERFECT
# absolutely no vibrations | Push harder = move faster
# Only in Y movement | MAX VELOCITY: 60deg/s
# Foxglove streaming: ws://localhost:8765

import sys
sys.path.append('/home/um/fairino-python-sdk-main/linux/fairino')
import Robot
import time
import signal
import numpy as np
import math

# FOXGLOVE SDK IMPORTS
try:
    import foxglove
    from foxglove.schemas import (
        PoseInFrame, KeyValue, Log, Time
    )
    FOXGLOVE_AVAILABLE = True
    print("✓ Foxglove SDK imported successfully")
except ImportError:
    print("⚠ Foxglove SDK not found. Install with: pip install foxglove-sdk")
    FOXGLOVE_AVAILABLE = False

from datetime import datetime

# ============================================================================
# GLOBAL VARIABLES & TUNING (Y-AXIS VERSION)
# ============================================================================
robot = None
running = True
fixed_tcp_ref = None

# TUNING — Optimized for pure Y-axis admittance
FORCE_TO_MOTION_SCALE = 6.0  # How much Y movement per Newton
M = [1.6, 1.6, 1.4, 1.8, 1.8, 1.8]  # Virtual mass (light = responsive)
B = [2.5, 2.5, 2.5, 3.0, 3.0, 3.0]  # Damping per joint
IK_TO_SERVO_RATIO = 2
IK_UPDATE_RATE = 0.0025
SERVO_UPDATE_RATE = 0.008
FORCE_THRESHOLD = 0.8
FORCE_FILTER_ALPHA = 0.28
force_thresholds = [2.0, 2.0, 2.5, 1.0, 1.0, 1.0]  # Deadzones per axis

joint_velocity = [0.0] * 6
desired_joint_pos = [0.0] * 6
filtered_desired_joints = None
filtered_fy_world = 0.0
baseline_forces = [0.0] * 6

# NEW: Safe joint velocity limit
MAX_JOINT_VELOCITY = 60.0  # deg/s (safe operating speed)

# FOXGLOVE GLOBAL VARIABLES
foxglove_client = None
foxglove_context = None
last_foxglove_log_time = 0
FOXGLOVE_LOG_INTERVAL = 0.005  # Log at 200Hz max

# ============================================================================
# FOXGLOVE INITIALIZATION
# ============================================================================
def init_foxglove():
    global foxglove_client, foxglove_context
    if not FOXGLOVE_AVAILABLE:
        print("Foxglove not available, skipping initialization")
        return False
    
    try:
        # Create Foxglove context and client
        foxglove_context = foxglove.Context()
        foxglove_client = foxglove.connect("ws://localhost:8765", context=foxglove_context)
        
        # Define channels (Foxglove will auto-create them)
        channels = [
            ("tcp_pose", "foxglove.PoseInFrame"),
            ("ft_force", "foxglove.KeyValue"),
            ("joint_state", "foxglove.KeyValue"),
            ("admittance", "foxglove.KeyValue"),
            ("control_debug", "foxglove.Log")
        ]
        
        for channel_name, schema in channels:
            foxglove_client.get_or_create_channel(channel_name, schema)
        
        print("✅ Foxglove WebSocket server started: ws://localhost:8765")
        print("📊 Available channels:")
        for name, _ in channels:
            print(f"   • /{name}")
        print("🌐 Connect Foxglove Studio to ws://localhost:8765")
        
        return True
        
    except Exception as e:
        print(f"❌ Foxglove initialization failed: {e}")
        return False

def log_to_foxglove(channel, message, timestamp=None):
    global foxglove_client, last_foxglove_log_time
    if not foxglove_client:
        return
    
    try:
        current_time = time.time()
        if current_time - last_foxglove_log_time < FOXGLOVE_LOG_INTERVAL:
            return
        last_foxglove_log_time = current_time
        
        if timestamp is None:
            timestamp = int(time.time_ns())
        
        foxglove_client.log(channel, message, timestamp=timestamp)
        
    except Exception as e:
        pass  # Silent fail - don't break robot control

# ============================================================================
# HELPERS
# ============================================================================
def euler_to_rotation_matrix(rx, ry, rz):
    rx = math.radians(rx)
    ry = math.radians(ry)
    rz = math.radians(rz)
    c, s = math.cos, math.sin
    Rx = np.array([[1, 0, 0], [0, c(rx), -s(rx)], [0, s(rx), c(rx)]])
    Ry = np.array([[c(ry), 0, s(ry)], [0, 1, 0], [-s(ry), 0, c(ry)]])
    Rz = np.array([[c(rz), -s(rz), 0], [s(rz), c(rz), 0], [0, 0, 1]])
    return Rz @ Ry @ Rx

def euler_to_quaternion(rx, ry, rz):
    """Convert Euler angles (degrees) to quaternion for Foxglove"""
    rx = math.radians(rx)
    ry = math.radians(ry)
    rz = math.radians(rz)
    
    cx = math.cos(rx * 0.5)
    sx = math.sin(rx * 0.5)
    cy = math.cos(ry * 0.5)
    sy = math.sin(ry * 0.5)
    cz = math.cos(rz * 0.5)
    sz = math.sin(rz * 0.5)
    
    w = cx * cy * cz + sx * sy * sz
    x = sx * cy * cz - cx * sy * sz
    y = cx * sy * cz + sx * cy * sz
    z = cx * cy * sz - sx * sy * cz
    
    return {"x": x, "y": y, "z": z, "w": w}

def transform_force_to_world(ft_forces, orientation):
    R = euler_to_rotation_matrix(*orientation)
    tcp_force = np.array([ft_forces[0], ft_forces[1], -ft_forces[2]])
    world_force = R @ tcp_force
    return world_force[1]

def ema(new, old, alpha):
    return alpha * new + (1 - alpha) * old

# ============================================================================
# FT SENSOR
# ============================================================================
def init_ft_sensor():
    robot.FT_SetConfig(24, 0)
    robot.FT_Activate(1)
    time.sleep(1.0)
    robot.SetLoadWeight(0, 0.0)
    robot.FT_SetZero(1)
    time.sleep(0.5)
    print("✅ FT sensor ready")

def calibrate_baseline(samples=150):
    print("🔧 Calibrating baseline... (keep tool still)")
    forces = []
    for _ in range(samples):
        ret = robot.FT_GetForceTorqueRCS()
        if ret[0] == 0:
            forces.append(ret[1][:6])
        time.sleep(0.01)
    
    if forces:
        global baseline_forces
        baseline_forces = np.mean(forces, axis=0).tolist()
        print(f"📊 Baseline: {[f'{x:+.3f}' for x in baseline_forces]}")
    else:
        print("⚠ No force data collected during calibration")

# ============================================================================
# MAIN CONTROL LOOP — PURE Y-AXIS ADMITTANCE WITH FOXGLOVE
# ============================================================================
def control_loop():
    global running, filtered_fy_world, desired_joint_pos, joint_velocity
    global filtered_desired_joints, fixed_tcp_ref, foxglove_client
    
    print("\n" + "="*80)
    print(" 🚀 PERFECT Y-AXIS ADMITTANCE CONTROL WITH LIVE FOXGLOVE STREAMING")
    print(" 📡 Connect Foxglove: ws://localhost:8765")
    print(" 🎮 Push forward/backward → Smooth Y motion (MAX 60°/s per joint)")
    print(" 📊 Real-time: Forces, TCP pose, joint velocities, admittance state")
    print("="*80)
    
    # Initial setup
    err, tcp = robot.GetActualTCPPose()
    if err != 0:
        print("❌ Failed to get initial TCP pose")
        return
    
    fixed_tcp_ref = tcp.copy()
    print(f"🔒 Fixed reference: X={tcp[0]:.1f}mm, Y={tcp[1]:.1f}mm, Z={tcp[2]:.1f}mm")
    
    if robot.ServoMoveStart() != 0:
        print("❌ Failed to start servo mode")
        return
    
    # Initialize joint positions
    j = robot.GetActualJointPosDegree(flag=0)
    if j[0] == 0:
        desired_joint_pos[:] = j[1][:6]
        filtered_desired_joints = desired_joint_pos[:]
    
    acc_joints = None
    ik_count = servo_count = 0
    loop_count = 0
    
    try:
        while running:
            t0 = time.time()
            
            # Get current TCP pose
            err, current_tcp = robot.GetActualTCPPose()
            if err != 0:
                time.sleep(IK_UPDATE_RATE)
                continue
            
            # Get force/torque data
            ft = robot.FT_GetForceTorqueRCS()
            if ft[0] != 0:
                time.sleep(IK_UPDATE_RATE)
                continue
            
            # Process forces
            raw = ft[1][:6]
            compensated = [raw[i] - baseline_forces[i] for i in range(6)]
            
            # Apply deadzone
            for i in range(6):
                if abs(compensated[i]) < force_thresholds[i]:
                    compensated[i] = 0.0
            
            # Transform force to world Y
            fy_world = transform_force_to_world(compensated, current_tcp[3:6])
            filtered_fy_world = ema(fy_world, filtered_fy_world, FORCE_FILTER_ALPHA)
            active_force = filtered_fy_world if abs(filtered_fy_world) > FORCE_THRESHOLD else 0.0
            
            # Calculate target Y position
            delta_y = active_force * FORCE_TO_MOTION_SCALE
            target_y = current_tcp[1] + delta_y
            
            # Create target TCP (only Y changes)
            target_tcp = [
                fixed_tcp_ref[0],    # X fixed
                target_y,           # Y target
                fixed_tcp_ref[2],    # Z fixed
                fixed_tcp_ref[3],    # Rx fixed
                fixed_tcp_ref[4],    # Ry fixed
                fixed_tcp_ref[5]     # Rz fixed
            ]
            
            # Inverse kinematics
            ik = robot.GetInverseKin(0, target_tcp, -1)
            if ik[0] != 0:
                time.sleep(IK_UPDATE_RATE)
                continue
            
            tj = np.array(ik[1][:6])
            acc_joints = tj if acc_joints is None else acc_joints + tj
            ik_count += 1
            
            if ik_count >= IK_TO_SERVO_RATIO:
                avg_joints = (acc_joints / IK_TO_SERVO_RATIO).tolist()
                
                # Admittance control with velocity limiting
                for j in range(6):
                    err = avg_joints[j] - desired_joint_pos[j]
                    f = err * 3.9
                    acc = (f - B[j] * joint_velocity[j]) / M[j]
                    joint_velocity[j] += acc * SERVO_UPDATE_RATE
                    # CLAMP to safe 60°/s maximum
                    joint_velocity[j] = np.clip(joint_velocity[j], -MAX_JOINT_VELOCITY, MAX_JOINT_VELOCITY)
                    desired_joint_pos[j] += joint_velocity[j] * SERVO_UPDATE_RATE
                
                # Smooth desired joints
                alpha = 0.32
                if filtered_desired_joints is None:
                    filtered_desired_joints = desired_joint_pos[:]
                else:
                    for j in range(6):
                        filtered_desired_joints[j] = alpha * desired_joint_pos[j] + (1 - alpha) * filtered_desired_joints[j]
                
                # Send joint commands
                robot.ServoJ(filtered_desired_joints, [0]*6, 0, 0, SERVO_UPDATE_RATE, 0, 0)
                
                # FOXGLOVE LOGGING (every loop for real-time data)
                if foxglove_client:
                    timestamp_ns = int(time.time_ns())
                    
                    # 1. TCP Pose
                    quat = euler_to_quaternion(current_tcp[3], current_tcp[4], current_tcp[5])
                    pose_msg = PoseInFrame(
                        timestamp=timestamp_ns,
                        frame_id="world",
                        pose={
                            "position": {
                                "x": current_tcp[0] / 1000.0,  # mm to m
                                "y": current_tcp[1] / 1000.0,
                                "z": current_tcp[2] / 1000.0
                            },
                            "orientation": quat
                        }
                    )
                    log_to_foxglove("tcp_pose", pose_msg)
                    
                    # 2. Force/Torque Data
                    force_kv = KeyValue(
                        timestamp=timestamp_ns,
                        values=[
                            KeyValue.Entry(key="fy_world", value=str(filtered_fy_world)),
                            KeyValue.Entry(key="fx", value=str(compensated[0])),
                            KeyValue.Entry(key="fz", value=str(compensated[2])),
                            KeyValue.Entry(key="tx", value=str(compensated[3])),
                            KeyValue.Entry(key="ty", value=str(compensated[4])),
                            KeyValue.Entry(key="tz", value=str(compensated[5])),
                            KeyValue.Entry(key="active_force", value=str(active_force))
                        ]
                    )
                    log_to_foxglove("ft_force", force_kv)
                    
                    # 3. Joint State
                    joint_kv = KeyValue(
                        timestamp=timestamp_ns,
                        values=[
                            KeyValue.Entry(key=f"joint{i+1}_pos", value=str(filtered_desired_joints[i])) for i in range(6)
                        ] + [
                            KeyValue.Entry(key=f"joint{i+1}_vel", value=str(joint_velocity[i])) for i in range(6)
                        ]
                    )
                    log_to_foxglove("joint_state", joint_kv)
                    
                    # 4. Admittance State
                    max_jv = max(abs(v) for v in joint_velocity)
                    admit_kv = KeyValue(
                        timestamp=timestamp_ns,
                        values=[
                            KeyValue.Entry(key="target_y", value=str(target_y)),
                            KeyValue.Entry(key="delta_y", value=str(delta_y)),
                            KeyValue.Entry(key="max_joint_vel", value=str(max_jv)),
                            KeyValue.Entry(key="velocity_limit", value=str(MAX_JOINT_VELOCITY))
                        ]
                    )
                    log_to_foxglove("admittance", admit_kv)
                
                # Debug print (every 20th servo cycle ~25Hz)
                if servo_count % 20 == 0:
                    max_jv = max(abs(v) for v in joint_velocity)
                    current_y = current_tcp[1]
                    dy_total = current_y - fixed_tcp_ref[1]
                    print(f" Fy={filtered_fy_world:+6.2f}N → Y={current_y:8.1f}mm (Δ{dy_total:+6.1f}) | "
                          f"MaxVel={max_jv:5.1f}°/s | Loops={loop_count}")
                
                acc_joints = None
                ik_count = 0
                servo_count += 1
            
            # Maintain timing
            sleep_t = IK_UPDATE_RATE - (time.time() - t0)
            if sleep_t > 0:
                time.sleep(sleep_t)
            loop_count += 1
            
    except Exception as e:
        print(f"❌ Control loop error: {e}")
    finally:
        robot.ServoMoveEnd()
        print("🛑 Servo mode ended")

# ============================================================================
# SHUTDOWN
# ============================================================================
def shutdown(sig, frame):
    global running, foxglove_client, foxglove_context
    print("\n🛑 Shutting down...")
    running = False
    
    if foxglove_client:
        try:
            foxglove_client.close()
            print("✅ Foxglove connection closed")
        except:
            pass
    
    time.sleep(0.5)
    
    err, tcp = robot.GetActualTCPPose()
    if err == 0 and fixed_tcp_ref is not None:
        dy = tcp[1] - fixed_tcp_ref[1]
        print(f"📏 Final position: ΔY = {dy:+.1f} mm")
    
    sys.exit(0)

# Signal handlers
signal.signal(signal.SIGINT, shutdown)
signal.signal(signal.SIGTERM, shutdown)

# ============================================================================
# MAIN EXECUTION
# ============================================================================
if __name__ == "__main__":
    print("🤖 FAIRINO Y-AXIS ADMITTANCE CONTROL WITH FOXGLOVE")
    print("="*60)
    
    # Connect to robot
    try:
        robot = Robot.RPC('192.168.58.2')
        print("✅ Connected to Fairino Cobot at 192.168.58.2")
    except Exception as e:
        print(f"❌ Failed to connect to robot: {e}")
        sys.exit(1)
    
    # Initialize systems
    init_ft_sensor()
    calibrate_baseline()
    
    # Initialize Foxglove
    foxglove_success = init_foxglove()
    
    print("\n🚀 STARTING CONTROL LOOP")
    print("💡 Instructions:")
    print("   1. Open Foxglove Studio → Connect to ws://localhost:8765")
    print("   2. Add panels: 3D (tcp_pose), Plot (ft_force/fy_world), Table (admittance)")
    print("   3. Push/pull tool along Y-axis for smooth admittance motion")
    print("   4. Press Ctrl+C to stop safely")
    print("="*60)
    
    try:
        control_loop()
    except KeyboardInterrupt:
        shutdown(None, None)
    except Exception as e:
        print(f"❌ Unexpected error: {e}")
        shutdown(None, None)