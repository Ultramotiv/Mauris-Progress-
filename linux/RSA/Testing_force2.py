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
    
    # --- ZERO FT SENSOR AFTER GRAVITY COMPENSATION ---
    print("\nZeroing FT sensor after gravity compensation...")
    robot.FT_SetZero(0)
    time.sleep(0.5)
    robot.FT_SetZero(1)
    time.sleep(0.5)
    print("FT sensor zeroed successfully.")

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

# --- DISPLAY POST-CALIBRATION FT SENSOR READINGS ---
print("\n" + "="*70)
print("FT Sensor Readings Post-Calibration (After Zeroing)")
print("="*70)
print(f"{'Axis':>6} {'Reading (N/Nm)':>18} {'Baseline (N/Nm)':>20}")
print("-" * 70)

axis_names = ['Fx', 'Fy', 'Mx', 'My', 'Fz', 'Mz']
sample_readings = []

for _ in range(50):  # Take 50 samples to average
    ft_data = robot.FT_GetForceTorqueRCS()
    if ft_data[0] == 0:
        reading = [
            ft_data[1][0],   # Fx
            -ft_data[1][1],  # Fy (inverted)
            ft_data[1][2],   # Fz
            ft_data[1][3],   # Mx
            ft_data[1][4],   # My
            ft_data[1][5]    # Mz
        ]
        sample_readings.append(reading)
    time.sleep(0.02)

if sample_readings:
    avg_readings = [sum(col) / len(sample_readings) for col in zip(*sample_readings)]
    for i, (axis, avg_reading) in enumerate(zip(axis_names, avg_readings)):
        baseline_val = baseline_forces[i] if i < len(baseline_forces) else 0.0
        print(f"{axis:>6} {avg_reading:>18.3f} {baseline_val:>20.3f}")
else:
    print("Warning: Could not read FT sensor post-calibration.")

print("="*70)
print("Ready for FT Sensor Monitoring")
print("="*70)

# ============================================================================
# DISPLAY INITIAL RAW FT SENSOR VALUES (Before Calibration)
# ============================================================================
print("\n" + "="*70)
print("INITIAL RAW FT SENSOR VALUES (Before Zeroing)")
print("="*70)
print(f"\n{'Fx(N)':>10} {'Fy(N)':>10} {'Fz(N)':>10} {'Mx(Nm)':>10} {'My(Nm)':>10} {'Mz(Nm)':>10}")
print("-" * 75)

# Capture raw values before zeroing
for i in range(5):
    ft_data = robot.FT_GetForceTorqueRCS()
    if ft_data[0] == 0:
        fx_raw = ft_data[1][0]
        fy_raw = -ft_data[1][1]  # Inverted
        fz_raw = ft_data[1][2]
        mx_raw = ft_data[1][3]
        my_raw = ft_data[1][4]
        mz_raw = ft_data[1][5]
        
        print(f"{fx_raw:10.2f} {fy_raw:10.2f} {fz_raw:10.2f} {mx_raw:10.3f} {my_raw:10.3f} {mz_raw:10.3f}")
    
    time.sleep(0.1)

print("\n" + "-"*70)
print("Zeroing FT Sensor...")
robot.FT_SetZero(0)
time.sleep(0.5)
robot.FT_SetZero(1)
time.sleep(0.5)
print("FT Sensor Zeroed Successfully!")

# ============================================================================
# CONTINUOUS POST-CALIBRATED FT SENSOR READINGS
# ============================================================================
print("\n" + "="*70)
print("POST-CALIBRATED FT SENSOR READINGS (Continuously Updating)")
print("="*70)
print(f"\n{'Fx(N)':>10} {'Fy(N)':>10} {'Fz(N)':>10} {'Mx(Nm)':>10} {'My(Nm)':>10} {'Mz(Nm)':>10}")
print("-" * 75)

try:
    while True:
        ft_data = robot.FT_GetForceTorqueRCS()
        
        if ft_data[0] == 0:
            # Get raw values
            fx_raw = ft_data[1][0]
            fy_raw = -ft_data[1][1]  # Inverted
            fz_raw = ft_data[1][2]
            mx_raw = ft_data[1][3]
            my_raw = ft_data[1][4]
            mz_raw = ft_data[1][5]
            
            # Subtract baseline to get zeroed values
            fx = fx_raw - baseline_forces[0]
            fy = fy_raw - baseline_forces[1]
            fz = fz_raw - baseline_forces[4]
            mx = mx_raw - baseline_forces[2]
            my = my_raw - baseline_forces[3]
            mz = mz_raw - baseline_forces[5]
            
            # Apply deadband
            deadband = 0.1
            if abs(fx) < deadband: fx = 0.0
            if abs(fy) < deadband: fy = 0.0
            if abs(fz) < deadband: fz = 0.0
            if abs(mx) < deadband: mx = 0.0
            if abs(my) < deadband: my = 0.0
            if abs(mz) < deadband: mz = 0.0
            
            # Display FT sensor readings continuously
            print(f"{fx:10.2f} {fy:10.2f} {fz:10.2f} {mx:10.3f} {my:10.3f} {mz:10.3f}", end='\r')
        
        time.sleep(0.01)

except KeyboardInterrupt:
    pass

# =============================================================================
# STOPPED
# =============================================================================
print("\n\n" + "="*70)
print("FT Sensor Monitoring Stopped by User (Ctrl+C)")
print("="*70)
print("Program ended safely.")
print("="*70)