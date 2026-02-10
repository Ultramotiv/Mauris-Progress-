# FT Sensor Calibration & Monitoring
# Simple terminal-based program to connect to robot, calibrate FT sensor, and display values

import sys
import os
import time
import threading
import numpy as np

# === Robot Setup ===
sys.path.append('/home/um/fairino-python-sdk-main/linux/fairino')
import Robot

ROBOT_IP = '192.168.58.2'
robot = None
running = True

# === Sensor Data ===
baseline_forces = np.array([0.0] * 6)
current_ft_raw = np.array([0.0] * 6)
sensor_history = []  # Store last 10 readings for noise filtering
HISTORY_SIZE = 10

# Threading Locks
sensor_lock = threading.Lock()


# ==================================================================================
# FT SENSOR FUNCTIONS
# ==================================================================================
def calibrate_ft_sensor(num_samples=100):
    """Calibrate FT sensor with stability verification"""
    global baseline_forces
    
    print("\n🔧 PRE-CALIBRATION SETTLING PHASE (30 seconds)")
    print(f"   📍 MAKE SURE ROBOT IS STATIONARY, RELAXED, AND NOT MOVING!")
    print(f"   ⏳ Waiting for sensor thermal stability...\n")
    
    # Phase 1: Pre-calibration settling (let sensor stabilize)
    for i in range(30):
        ret = robot.FT_GetForceTorqueRCS()
        if ret[0] == 0:
            if (i + 1) % 5 == 0:
                print(f"   ✓ Settling: {i+1}/30 - Fz: {ret[1][2]:+8.2f} N")
        time.sleep(1)  # 1 second per reading
    
    print(f"\n🔧 Calibrating FT Sensor...")
    print(f"   Taking {num_samples} readings (this may take 20 seconds)...\n")
    
    readings = []
    stability_threshold = 0.5  # N tolerance for stability
    
    for i in range(num_samples):
        ret = robot.FT_GetForceTorqueRCS()
        if ret[0] == 0:
            readings.append(np.array(ret[1]))
            if i < 10:
                print(f"   ✓ Reading {i+1:2d}/{num_samples} - Raw Fz: {ret[1][2]:+8.2f} N", end='\r')
            elif i % 10 == 0:
                print(f"   ✓ Reading {i+1:2d}/{num_samples} - Raw Fz: {ret[1][2]:+8.2f} N", end='\r')
        time.sleep(0.2)  # Longer delay for stability
    
    print()
    
    if len(readings) < num_samples * 0.9:  # At least 90% success rate
        print(f"❌ Insufficient readings. Got {len(readings)}/{num_samples}")
        return False
    
    readings_array = np.array(readings)
    
    # Calculate baseline and statistics
    baseline_forces = np.mean(readings_array, axis=0)
    std_dev = np.std(readings_array, axis=0)
    min_vals = np.min(readings_array, axis=0)
    max_vals = np.max(readings_array, axis=0)
    
    print("\n✅ Calibration Complete!")
    print(f"   Readings taken: {len(readings)}")
    print(f"\n   📊 BASELINE (Pre-Calibrated) Values:")
    print(f"   ┌─ Force ────────────────────────────────────────────┬─ Torque ────────────┐")
    print(f"   │ Fx: {baseline_forces[0]:+8.2f} N (σ={std_dev[0]:.3f}, range={max_vals[0]-min_vals[0]:.1f}N) │ Tx: {baseline_forces[3]:+8.2f} Nm │")
    print(f"   │ Fy: {baseline_forces[1]:+8.2f} N (σ={std_dev[1]:.3f}, range={max_vals[1]-min_vals[1]:.1f}N) │ Ty: {baseline_forces[4]:+8.2f} Nm │")
    print(f"   │ Fz: {baseline_forces[2]:+8.2f} N (σ={std_dev[2]:.3f}, range={max_vals[2]-min_vals[2]:.1f}N) │ Tz: {baseline_forces[5]:+8.2f} Nm │")
    print(f"   └────────────────────────────────────────────────────┴────────────────────┘")
    
    # Check stability - Fz should be relatively constant if nothing is moving
    fz_range = max_vals[2] - min_vals[2]
    fz_stability_threshold = 5.0  # N
    
    print(f"\n   🔍 STABILITY ANALYSIS:")
    print(f"   Fz range during calibration: {fz_range:.2f} N (min={min_vals[2]:.2f}, max={max_vals[2]:.2f})")
    
    if fz_range > fz_stability_threshold:
        print(f"\n   ❌ WARNING: LARGE Fz VARIATION DETECTED ({fz_range:.2f}N > {fz_stability_threshold}N)!")
        print(f"   This explains why Fz keeps changing even with constant weight!")
        print(f"\n   📌 LIKELY CAUSES (in order of probability):")
        print(f"   1. Robot servo still settling - requires 30+ seconds idle time")
        print(f"   2. Sensor thermal drift - needs 5+ minutes warm-up")
        print(f"   3. Load shifting/flexing - secure the load better")
        print(f"   4. External vibrations - check surroundings")
        print(f"   5. FT sensor mounting loose - verify connection")
        print(f"\n   📋 FIX STEPS:")
        print(f"   1. Move robot to HOME position (fully extended, no load)")
        print(f"   2. Wait 10 minutes for sensor thermal stabilization")
        print(f"   3. Re-run calibration")
        return False
    
    # Warn if readings are too noisy
    if np.any(std_dev > 2.0):
        print(f"\n   ⚠️  WARNING: High sensor noise (σ > 2.0) - check vibrations")
    
    print(f"\n   ✅ Sensor STABLE and ready for monitoring!\n")
    return True

def ft_sensor_monitor():
    """Continuously read and display FT sensor values with filtering"""
    global current_ft_raw, sensor_history
    
    print("\n📊 FT Sensor Monitoring Started (Press Ctrl+C to stop)")
    print("="*140)
    print(f"{'Time':<8} | {'Fx_raw':<9} | {'Fy_raw':<9} | {'Fz_raw':<9} | {'Fx_net':<9} | {'Fy_net':<9} | {'Fz_net':<9} | {'Tx_net':<9} | {'Ty_net':<9} | {'Tz_net':<9}")
    print("="*140)
    
    start_time = time.time()
    
    try:
        while running:
            ret = robot.FT_GetForceTorqueRCS()
            
            if ret[0] == 0:
                current_reading = np.array(ret[1])
                
                # Add to history for filtering
                sensor_history.append(current_reading)
                if len(sensor_history) > HISTORY_SIZE:
                    sensor_history.pop(0)
                
                # Apply moving average filter
                filtered_reading = np.mean(sensor_history, axis=0)
                
                with sensor_lock:
                    current_ft_raw = filtered_reading.copy()
                
                # Calculate net force (current - baseline)
                net_ft = filtered_reading - baseline_forces
                
                elapsed = time.time() - start_time
                
                # Print: RAW values | NET (calibrated) values
                print(f"{elapsed:>6.1f}s | "
                      f"{current_reading[0]:>+8.2f} | {current_reading[1]:>+8.2f} | {current_reading[2]:>+8.2f} | "
                      f"{net_ft[0]:>+8.2f} | {net_ft[1]:>+8.2f} | {net_ft[2]:>+8.2f} | "
                      f"{net_ft[3]:>+8.2f} | {net_ft[4]:>+8.2f} | {net_ft[5]:>+8.2f}")
            else:
                print(f"❌ Sensor read error: {ret[0]}")
            
            time.sleep(0.1)
    
    except KeyboardInterrupt:
        print("\n\n⏹️  Monitoring stopped by user")

# ==================================================================================
# MAIN
# ==================================================================================
if __name__ == '__main__':
    print("="*80)
    print("🏥 FT SENSOR CALIBRATION & MONITORING SYSTEM")
    print("="*80)
    
    # Connect to Robot
    print("\n🤖 Connecting to Robot...")
    try:
        robot = Robot.RPC(ROBOT_IP)
        print(f"✅ Connected to Robot: {ROBOT_IP}")
    except Exception as e:
        print(f"❌ ERROR: Could not connect to robot. Check IP.")
        print(f"   Details: {e}")
        sys.exit(1)
    
    # Calibrate FT Sensor
    if not calibrate_ft_sensor(num_samples=50):
        print("❌ Calibration failed. Exiting...")
        sys.exit(1)
    
    # Start FT Sensor Monitoring
    print("\n🌟 Starting continuous monitoring...")
    print("    📌 Left columns (Fx_raw, Fy_raw, Fz_raw): Raw sensor readings")
    print("    📌 Right columns (Fx_net, Fy_net, Fz_net): Calibrated readings (raw - baseline)\n")
    
    try:
        # Run monitoring in main thread (blocking)
        ft_sensor_monitor()
    except KeyboardInterrupt:
        print("\n\n⏹️  Program interrupted")
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        running = False
        print("✅ Shutdown complete")