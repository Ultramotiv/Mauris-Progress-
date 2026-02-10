"""
Fairino Robot — Drag Mode + Continuous FT Sensor Print
- Uses YOUR exact force processing logic
- Prints a NEW LINE continuously (no overwriting)
- Baseline captured at startup
- Deadband: forces < 0.5 → zeroed
"""

import sys
import time
import signal

sys.path.append('/home/um/fairino-python-sdk-main/linux/fairino')
import Robot

ROBOT_IP = '192.168.58.2'

# Connect to robot
robot = Robot.RPC(ROBOT_IP)
print("✅ Connected to Fairino robot")

# Wait for stability before baseline
print("⏳ Waiting 3 seconds to capture baseline (keep robot completely still!)...")
time.sleep(3)

# === CAPTURE BASELINE (50-sample average) ===
print("📌 Capturing FT sensor baseline...")
baseline_forces = None
for _ in range(50):
    d = robot.FT_GetForceTorqueRCS()
    if d[0] == 0:
        raw = [d[1][0], -d[1][1], d[1][2], d[1][3], d[1][4], d[1][5]]
        if baseline_forces is None:
            baseline_forces = [x / 50.0 for x in raw]
        else:
            for i in range(6):
                baseline_forces[i] += raw[i] / 50.0
    time.sleep(0.002)

if baseline_forces is None:
    print("⚠️  Warning: Baseline read failed. Using zeros.")
    baseline_forces = [0.0] * 6
else:
    print(f"✅ Baseline captured: [Fx, Fy, Fz, Tx, Ty, Tz] = {[round(x, 2) for x in baseline_forces]}")

# Enable drag mode
print("🔧 Enabling drag teaching mode...")
robot.DragTeachSwitch(1)
print("✅ Drag mode ON")
print("\n🟢 Continuous FT data stream (press Ctrl+C to stop):\n")

def shutdown():
    print("\n\n🛑 Disabling drag mode...")
    try:
        robot.DragTeachSwitch(0)
    except:
        pass
    print("👋 Goodbye!")
    sys.exit(0)

def signal_handler(sig, frame):
    shutdown()

signal.signal(signal.SIGINT, signal_handler)

# ==============================
# MAIN LOOP — CONTINUOUS PRINT
# ==============================
try:
    while True:
        d = robot.FT_GetForceTorqueRCS()
        if d[0] == 0:
            # YOUR EXACT LOGIC:
            raw = [d[1][0], -d[1][1], d[1][2], d[1][3], d[1][4], d[1][5]]
            forces = raw if not baseline_forces else [raw[i] - baseline_forces[i] for i in range(6)]
            for i in range(6):
                if abs(forces[i]) < 0.5:
                    forces[i] = 0.0

            # ✅ CONTINUOUS PRINT — NEW LINE EVERY TIME (as requested)
            print(f"Forces: Fx={forces[0]:.2f}, Fy={forces[1]:.2f}, Fz={forces[2]:.2f}, "
                  f"Tx={forces[3]:.2f}, Ty={forces[4]:.2f}, Tz={forces[5]:.2f}")
        else:
            print(f"[ERROR] FT read failed (code: {d[0]})")

        time.sleep(0.02)  # ~50 Hz → 20 ms

except KeyboardInterrupt:
    shutdown()