# FAIRINO REHAB ROBOT - WITH MODE LED INDICATORS
# Web GUI auto-opens + real-time updates with LED status indicators



# Active Assistive Force > 0 AND Force < 5.0 N
# Assistive Active Force ≤ 0 OR Force ≥ 5.0 N
# Active Passive Speed < 0.5 °/s AND Position ≤ 130°
# Passive Active Movement complete + 0.5s cooldown

import sys
sys.path.append('/home/um/fairino-python-sdk-main/linux/fairino')
import Robot
import time
import signal
import numpy as np
import threading
import webbrowser
from flask import Flask, render_template_string
from flask_socketio import SocketIO

# ====================== WEB GUI SETUP ======================
app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading', ping_timeout=30, ping_interval=5)

gui_data = {
    "mode": "Starting...",
    "joint2_pos": 100.0,
    "rep_count": 0,
    "speed": 0.0,
    "avg_speed": 0.0,
    "force": 0.0,
    "assistance": 0,
    "state": "unknown",
    "direction": "Still"
}

@app.route('/')
def index():
    return render_template_string('''
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Fairino Robot - Mode LED Dashboard</title>
    <script src="https://cdn.socket.io/4.5.4/socket.io.min.js"></script>
    <style>
        body { margin:0; background:#0a0a0a; color:white; font-family: 'Segoe UI', sans-serif; }
        .container { max-width:900px; margin:20px auto; padding:20px; background:#111; border-radius:20px; box-shadow:0 0 40px rgba(0,0,0,0.8); transition: background 0.3s ease; }
        h1 { text-align:center; color:#00ff9d; margin-bottom:10px; }
        
        /* LED Indicator Section */
        .led-panel {
            display: flex;
            justify-content: center;
            align-items: center;
            gap: 40px;
            padding: 30px;
            background: #1a1a1a;
            border-radius: 15px;
            margin: 20px 0;
        }
        
        .led-container {
            display: flex;
            flex-direction: column;
            align-items: center;
            gap: 10px;
        }
        
        .led {
            width: 60px;
            height: 60px;
            border-radius: 50%;
            border: 3px solid #333;
            transition: all 0.3s ease;
            box-shadow: 0 0 10px rgba(0,0,0,0.5);
        }
        
        .led.active {
            box-shadow: 0 0 30px currentColor, 0 0 60px currentColor;
            border-color: currentColor;
        }
        
        .led.blue { background: #1e3a5f; color: #3b82f6; }
        .led.blue.active { background: #3b82f6; }
        
        .led.orange { background: #4a3520; color: #fb923c; }
        .led.orange.active { background: #fb923c; }
        
        .led.red { background: #4a1f1f; color: #ef4444; }
        .led.red.active { background: #ef4444; }
        
        .led-label {
            font-size: 16px;
            font-weight: bold;
            text-transform: uppercase;
            letter-spacing: 1px;
        }
        
        .led-label.blue { color: #3b82f6; }
        .led-label.orange { color: #fb923c; }
        .led-label.red { color: #ef4444; }
        
        .stats { display:grid; grid-template-columns:repeat(auto-fit, minmax(180px,1fr)); gap:15px; margin-top:20px; }
        .card { background:#222; padding:20px; border-radius:15px; text-align:center; }
        .value { font-size:40px; font-weight:bold; margin:10px 0; color:#00ff9d; }
        .label { color:#aaa; font-size:14px; }
        .mode-text { font-size:24px; font-weight:bold; text-align:center; margin:15px 0; color:#00ff9d; }
    </style>
</head>
<body>
    <div class="container">
        <h1>Robot ADAPTIVE MODE VISUALIZATION</h1>
        
        <!-- LED Indicator Panel -->
        <div class="led-panel">
            <div class="led-container">
                <div class="led blue" id="led-active"></div>
                <div class="led-label blue">Active</div>
            </div>
            <div class="led-container">
                <div class="led orange" id="led-assistive"></div>
                <div class="led-label orange">Assistive</div>
            </div>
            <div class="led-container">
                <div class="led red" id="led-passive"></div>
                <div class="led-label red">Passive</div>
            </div>
        </div>
        
        <div class="mode-text" id="mode-text">Starting...</div>
        
        <div class="stats">
            <div class="card"><div class="label">Reps</div><div class="value" id="reps">0</div></div>
            <div class="card"><div class="label">Speed</div><div class="value" id="speed">0.0 °/s</div></div>
            <div class="card"><div class="label">Avg Speed</div><div class="value" id="avg">0.0 °/s</div></div>
            <div class="card"><div class="label">Force</div><div class="value" id="force">0.0 N</div></div>
        </div>
    </div>

    <script>
        const modeEl = document.getElementById('mode-text');
        const ledActive = document.getElementById('led-active');
        const ledAssistive = document.getElementById('led-assistive');
        const ledPassive = document.getElementById('led-passive');

        const socket = io();
        
        socket.on('connect', () => console.log("Connected to server"));
        socket.on('update', (data) => {
            document.getElementById('reps').textContent = data.rep_count;
            document.getElementById('speed').textContent = data.speed.toFixed(1) + " °/s";
            document.getElementById('avg').textContent = data.avg_speed.toFixed(1) + " °/s";
            document.getElementById('force').textContent = data.force.toFixed(1) + " N";
            modeEl.textContent = data.mode.toUpperCase() + " MODE";

            // Update LED indicators
            ledActive.classList.remove('active');
            ledAssistive.classList.remove('active');
            ledPassive.classList.remove('active');
            
            if (data.mode === "Active") {
                ledActive.classList.add('active');
            } else if (data.mode === "Assistive") {
                ledAssistive.classList.add('active');
            } else if (data.mode === "Passive") {
                ledPassive.classList.add('active');
            }

            let bg = "#113355";

            if (data.mode === "Assistive") {
                bg = "#442211";
            } else if (data.mode === "Passive") {
                bg = "#331111";
            }

            document.querySelector('.container').style.background = bg;
        });
    </script>
</body>
</html>
    ''')

def run_web_server():
    print("Starting web server with LED mode indicators...")
    socketio.run(app, host='0.0.0.0', port=5000, debug=False, use_reloader=False)

threading.Thread(target=run_web_server, daemon=True).start()
time.sleep(2)
webbrowser.open('http://127.0.0.1:5000')
print("Web GUI with MODE LED INDICATORS opened! Also available on phone at your PC IP:5000")
# =====================================================================

# --- ROBOT PARAMETERS ---
M = [3.0, 3.0, 2.0, 3.0, 3.0, 3.0]
B = [2.5, 2.5, 2.5, 3.0, 3.0, 3.0]
K = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
force_thresholds = [2.5, 2.5, 3.0, 1.2, 1.2, 1.2]

ASSISTANCE_HIGH = 0.5

UPPER_LIMIT = 60.0
LOWER_LIMIT = 143.0
LIMIT_BUFFER = 5.0
DECEL_ZONE = 8.0

JOINT_SAFETY_LIMITS = {
    1: (-90.0, 85.0), 2: (-179.0, -35.0), 3: (2.0, 144.0),
    4: (-258.0, 80.0), 5: (-170.0, 12.0), 6: (-170.0, 170.0)
}

force_to_deg = 7.70
dt = 0.008
gravity_compensation_samples = 100
baseline_forces = None
startup_delay = 0.5
PASSIVE_START_LIMIT = 130.0
PASSIVE_COOLDOWN = 0.5

rep_count = 0
movement_threshold = 2.0
rep_state = None

robot = Robot.RPC('192.168.58.2')
print("Robot connected.")

def check_joint2_limits(p): return UPPER_LIMIT <= p <= LOWER_LIMIT
def is_within_safety_limits(joint_idx, angle):
    if joint_idx in JOINT_SAFETY_LIMITS:
        min_lim, max_lim = JOINT_SAFETY_LIMITS[joint_idx]
        return min_lim <= angle <= max_lim
    return True

def calculate_deceleration_force(position, velocity, joint_idx=2):
    decel_force = 0.0
    dist_to_upper = position - UPPER_LIMIT
    dist_to_lower = LOWER_LIMIT - position
    if dist_to_upper < DECEL_ZONE and velocity < 0:
        decel_strength = max(0.0, (DECEL_ZONE - dist_to_upper) / DECEL_ZONE)
        decel_force = decel_strength ** 2 * 15.0
    elif dist_to_lower < DECEL_ZONE and velocity > 0:
        decel_strength = max(0.0, (DECEL_ZONE - dist_to_lower) / DECEL_ZONE)
        decel_force = -decel_strength ** 2 * 15.0
    return decel_force

def apply_enhanced_limits(position, velocity, joint_idx=2):
    new_pos, new_vel = position, velocity
    decel_force = calculate_deceleration_force(position, velocity, joint_idx)
    if abs(decel_force) > 0.1:
        new_vel = velocity + (decel_force / M[joint_idx]) * dt
    dist_to_upper = position - UPPER_LIMIT
    dist_to_lower = LOWER_LIMIT - position
    if dist_to_upper < DECEL_ZONE and velocity < 0:
        max_speed = max(0.5, (dist_to_upper / DECEL_ZONE) * 3.0)
        if abs(new_vel) > max_speed: new_vel = -max_speed
    elif dist_to_lower < DECEL_ZONE and velocity > 0:
        max_speed = max(0.5, (dist_to_lower / DECEL_ZONE) * 3.0)
        if abs(new_vel) > max_speed: new_vel = max_speed
    if position <= UPPER_LIMIT: new_pos, new_vel = UPPER_LIMIT, 0.0 if velocity < 0 else new_vel
    if position >= LOWER_LIMIT: new_pos, new_vel = LOWER_LIMIT, 0.0 if velocity > 0 else new_vel
    return new_pos, new_vel

def count_repetitions(current_pos):
    global rep_count, rep_state
    near_min = abs(current_pos - UPPER_LIMIT) < movement_threshold
    near_max = abs(current_pos - LOWER_LIMIT) < movement_threshold
    if rep_state == "at_max" and not near_max: rep_state = "moving_down"; print("Moving DOWN")
    elif rep_state == "moving_down" and near_min: rep_state = "at_min"; print("Reached MIN")
    elif rep_state == "at_min" and not near_min: rep_state = "moving_up"; print("Moving UP")
    elif rep_state == "moving_up" and near_max:
        rep_state = "at_max"; rep_count += 1; print(f"REP {rep_count} COMPLETED!")

def init_ft_sensor():
    robot.FT_SetConfig(24, 0)
    robot.FT_Activate(1); time.sleep(1)
    robot.SetLoadWeight(0, 0.0)
    robot.SetLoadCoord(0.0, 0.0, 0.0)
    robot.FT_SetZero(1); time.sleep(0.5)
    print("FT Sensor ready")

def calibrate_baseline_forces():
    global baseline_forces
    samples = []
    print("Calibrating gravity...")
    for _ in range(100):
        d = robot.FT_GetForceTorqueRCS()
        if d[0] == 0:
            f = [d[1][0], -d[1][1], d[1][2], d[1][3], d[1][4], d[1][5]]
            samples.append(f)
        time.sleep(0.01)
    if samples:
        baseline_forces = np.mean(samples, axis=0).tolist()
        print("Baseline calibrated")

def shutdown(sig, frame):
    robot.ServoMoveEnd()
    print("\nShutting down...")
    sys.exit(0)
signal.signal(signal.SIGINT, shutdown)

# ====================== ROBOT STARTUP ======================
init_ft_sensor()
err, pos = robot.GetActualJointPosDegree()
if err != 0 or not check_joint2_limits(pos[2]): sys.exit(1)
calibrate_baseline_forces()

home_pos = pos.copy()
desired_pos = pos.copy()
velocity = [0.0]*6

rep_state = "at_max" if abs(pos[2]-LOWER_LIMIT) < abs(pos[2]-UPPER_LIMIT) else "at_min"

# Only Joint 2 is controlled (removed sitting/standing mode selection)
free_joints = [2]

robot.ServoMoveStart()
time.sleep(startup_delay)

# ====================== MAIN CONTROL LOOP ======================
def control_loop():
    global desired_pos, velocity, home_pos, rep_count, rep_state
    j = 2
    avg_window = []
    mode_state = "Active"
    passive_waiting = False

    while True:
        data = robot.FT_GetForceTorqueRCS()
        if data[0] != 0: time.sleep(dt); continue

        raw = [data[1][0], -data[1][1], data[1][2], data[1][3], data[1][4], data[1][5]]
        forces = [raw[i] - baseline_forces[i] for i in range(6)] if baseline_forces else raw
        for i in range(6):
            if abs(forces[i]) < 0.5:
                forces[i] = 0.0
        fz_force = forces[2]

        current_speed = abs(velocity[j] * force_to_deg)
        avg_window.append(current_speed)
        if len(avg_window) > 30: avg_window.pop(0)
        avg_speed = sum(avg_window)/len(avg_window) if avg_window else 0

        # Passive mode trigger
        if current_speed < 0.5 and desired_pos[j] <= PASSIVE_START_LIMIT and not passive_waiting:
            passive_waiting = True
            mode_state = "Passive"
            target = UPPER_LIMIT if desired_pos[j] > 100 else LOWER_LIMIT
            sign = -10.0 if target == UPPER_LIMIT else 10.0
            while abs(desired_pos[j] - target) > 0.5:
                acc = (sign - B[j]*velocity[j]) / M[j]
                velocity[j] += acc * dt
                delta = velocity[j] * dt * force_to_deg
                pos_new = desired_pos[j] + delta if sign > 0 else desired_pos[j] - abs(delta)
                pos_new, velocity[j] = apply_enhanced_limits(pos_new, velocity[j], j)
                desired_pos[j] = pos_new
                
                # Update GUI during passive movement
                gui_data.update({
                    "mode": "Passive",
                    "joint2_pos": round(desired_pos[j], 1),
                    "rep_count": rep_count,
                    "speed": round(abs(velocity[j] * force_to_deg), 1),
                    "avg_speed": round(avg_speed, 1),
                    "force": round(fz_force, 2),
                    "assistance": 0,
                    "state": rep_state or "unknown",
                    "direction": "Up" if velocity[j] < 0 else "Down" if velocity[j] > 0 else "Still"
                })
                socketio.emit('update', gui_data)
                
                robot.ServoJ(desired_pos, [0]*6)
                time.sleep(dt)
            time.sleep(PASSIVE_COOLDOWN)
            velocity[j] *= 0.1
            passive_waiting = False
            mode_state = "Active"
            continue

        # Assistance logic
        assistance_level = ASSISTANCE_HIGH if 0 < abs(fz_force) < 5.0 else 0.0
        mode_state = "Assistive" if assistance_level > 0 else "Active"
        assisted_force = fz_force * (1 + assistance_level)

        if rep_state == "moving_down" and assisted_force > 0: assisted_force = 0
        if rep_state == "moving_up" and assisted_force < 0: assisted_force = 0

        if abs(assisted_force) < 1.0 and current_speed < 0.5:
            acc = (-K[j]*(desired_pos[j]-home_pos[j])/force_to_deg - B[j]*velocity[j]) / M[j]
        else:
            acc = (assisted_force - B[j]*velocity[j]) / M[j]

        velocity[j] += acc * dt
        delta = velocity[j] * dt * force_to_deg
        new_pos = desired_pos[j] + delta
        new_pos, velocity[j] = apply_enhanced_limits(new_pos, velocity[j], j)
        desired_pos[j] = new_pos

        count_repetitions(desired_pos[j])

        for i in range(6):
            if i != 2:
                desired_pos[i] = home_pos[i]
                velocity[i] = 0.0

        if not all(is_within_safety_limits(i+1, desired_pos[i]) for i in range(6)):
            time.sleep(dt); continue

        # SEND DATA TO WEB GUI
        gui_data.update({
            "mode": mode_state,
            "joint2_pos": round(desired_pos[j], 1),
            "rep_count": rep_count,
            "speed": round(current_speed, 1),
            "avg_speed": round(avg_speed, 1),
            "force": round(fz_force, 2),
            "assistance": int(assistance_level * 100),
            "state": rep_state or "unknown",
            "direction": "Up" if velocity[j] < 0 else "Down" if velocity[j] > 0 else "Still"
        })
        socketio.emit('update', gui_data)

        print(f"{mode_state.upper()} | Pos: {desired_pos[j]:.1f}° | Reps: {rep_count} | Speed: {current_speed:.1f}°/s")

        robot.ServoJ(joint_pos=desired_pos, axisPos=[0]*6)
        time.sleep(dt)

control_loop()