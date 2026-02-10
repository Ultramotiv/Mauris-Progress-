"""
Rehabilitation Robot Controller — Force-Controlled Trajectory Playback
- Robot follows the loaded trajectory based on raw FZ force from FT sensor.
- Positive FZ advances forward, negative FZ moves backward along trajectory.
- Speed is proportional to force magnitude.
"""

import sys
import os
import time
import json
import threading
import signal
import numpy as np
from flask import Flask, request, jsonify, render_template_string
from flask_cors import CORS

# === Robot Setup ===
sys.path.append('/home/um/fairino-python-sdk-main/linux/fairino')
import Robot

ROBOT_IP = '192.168.58.2'
RECORDINGS_DIR = 'recordings'
os.makedirs(RECORDINGS_DIR, exist_ok=True)

robot = None
running = True

# === Global State ===
is_recording = False
current_recording = []
active_assist_enabled = False
drag_mode_enabled = False
loaded_trajectory = None
trajectory_index = 0.0
state_lock = threading.Lock()
baseline_forces = None  # Baseline for FT sensor calibration

# === Parameters ===
MAX_JOINT_VELOCITY = 30.0     # deg/s (Increased from 20.0 - still safe, robot max is 180°/s)
SERVO_UPDATE_RATE = 0.008     # 8ms control cycle
FORCE_SCALE = 0.1             # How much FZ affects trajectory speed (INCREASED from 0.01!)
MIN_FORCE_THRESHOLD = 1.0     # Minimum FZ to start moving (LOWERED from 1.0 N)
DEADBAND_THRESHOLD = 0.5      # Values below this are zeroed out

# === Flask App ===
app = Flask(__name__)
CORS(app)

# ==================================================================================
# FRONTEND HTML (Embedded)
# ==================================================================================
FRONTEND_HTML = '''
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Rehab Robot Controller</title>
    <style>
        body { font-family: Arial, sans-serif; max-width: 900px; margin: 20px auto; padding: 20px; }
        h1 { text-align: center; color: #2c3e50; }
        .grid { display: grid; grid-template-columns: repeat(2, 1fr); gap: 15px; margin: 20px 0; }
        .section { background: #f9f9f9; padding: 15px; border-radius: 8px; margin-bottom: 20px; }
        button { width: 100%; padding: 12px; font-size: 16px; cursor: pointer; border: none; border-radius: 6px; }
        .record { background: #e74c3c; color: white; }
        .record:hover { background: #c0392b; }
        .stop { background: #e67e22; color: white; }
        .stop:hover { background: #d35400; }
        .assist { background: #2ecc71; color: white; }
        .assist:hover { background: #27ae60; }
        .drag { background: #3498db; color: white; }
        .drag:hover { background: #2980b9; }
        .load { background: #9b59b6; color: white; }
        .load:hover { background: #8e44ad; }
        .shutdown { background: #95a5a6; color: white; }
        .shutdown:hover { background: #7f8c8d; }
        .telemetry { text-align: center; font-size: 18px; margin: 20px 0; }
        .files { max-height: 200px; overflow-y: auto; border: 1px solid #ddd; padding: 10px; border-radius: 4px; }
        .status { padding: 10px; margin: 10px 0; border-radius: 4px; color: white; text-align: center; }
        .active { background: #27ae60; }
        .inactive { background: #e74c3c; }
        select, input { width: 100%; padding: 8px; margin: 5px 0; }
        .force-display { background: #34495e; color: white; padding: 15px; border-radius: 8px; margin: 10px 0; }
        .force-bar { height: 30px; background: #3498db; border-radius: 4px; transition: width 0.1s; }
        .ft-raw { font-family: monospace; background: #2c3e50; padding: 10px; border-radius: 4px; margin-top: 10px; font-size: 14px; }
        .ft-raw-value { display: inline-block; margin-right: 15px; }
    </style>
</head>
<body>
    <h1>🩺 Force-Controlled Rehabilitation Robot</h1>

    <div class="section">
        <h3>🔴 Recording (Robot enters DRAG MODE automatically)</h3>
        <div class="grid">
            <button class="record" onclick="startRecording()">Start Recording</button>
            <button class="stop" onclick="stopRecording()">Stop Recording</button>
        </div>
    </div>

    <div class="section">
        <h3>📁 Saved Trajectories</h3>
        <div class="files" id="fileList">Loading...</div>
        <button class="load" onclick="refreshFiles()">Refresh List</button>
    </div>

    <div class="section">
        <h3>💪 Force-Controlled Playback</h3>
        <p><strong>Push/Pull in Z direction to move along trajectory!</strong></p>
        <p>• Positive FZ (push down) → moves forward<br>• Negative FZ (pull up) → moves backward</p>
        <select id="trajectorySelect" style="margin: 10px 0;"></select>
        <div class="grid">
            <button class="assist" onclick="startActiveAssist()">Start Force Control</button>
            <button class="stop" onclick="stopActiveAssist()">Stop Force Control</button>
        </div>
        <button class="load" onclick="zeroFTSensor()" style="margin-top: 10px;">🔄 Zero FT Sensor (Calibrate)</button>
        <div class="force-display">
            <div>FZ Force: <span id="forceValue">0.0</span> N</div>
            <div style="margin-top: 10px;">
                <div class="force-bar" id="forceBar" style="width: 0%"></div>
            </div>
            <div class="ft-raw">
                <strong>Raw FT Sensor Values:</strong><br>
                <div style="margin-top: 5px;">
                    <span class="ft-raw-value">Fx: <span id="fx">0.000</span>N</span>
                    <span class="ft-raw-value">Fy: <span id="fy">0.000</span>N</span>
                    <span class="ft-raw-value">Fz: <span id="fz">0.000</span>N</span>
                </div>
                <div style="margin-top: 5px;">
                    <span class="ft-raw-value">Tx: <span id="tx">0.000</span>Nm</span>
                    <span class="ft-raw-value">Ty: <span id="ty">0.000</span>Nm</span>
                    <span class="ft-raw-value">Tz: <span id="tz">0.000</span>Nm</span>
                </div>
            </div>
        </div>
    </div>

    <div class="section">
        <h3>🎛️ Drag Teaching (Manual Free-Drive)</h3>
        <button class="drag" onclick="toggleDrag()">Toggle Drag Teaching</button>
        <div class="status" id="dragStatus">Drag: OFF</div>
    </div>

    <div class="telemetry">
        <div>Progress: <span id="progress">0.0</span>%</div>
        <div>Playback: <span id="assistStatus">OFF</span></div>
    </div>

    <div class="section">
        <button class="shutdown" onclick="shutdown()">🛑 Shutdown Server</button>
    </div>

    <script>
        let telemetryInterval;

        async function apiCall(endpoint, method='POST', data={}) {
            try {
                const res = await fetch(endpoint, {
                    method, 
                    headers: {'Content-Type': 'application/json'},
                    body: method !== 'GET' ? JSON.stringify(data) : null
                });
                return await res.json();
            } catch (e) {
                alert('Error: ' + e.message);
                return null;
            }
        }

        async function startRecording() {
            const res = await apiCall('/start_recording');
            if (res?.status === 'success') alert('Recording started! Robot is now in DRAG MODE.');
        }

        async function stopRecording() {
            const res = await apiCall('/stop_recording');
            if (res?.status === 'success') alert('Saved as: ' + res.filename);
        }

        async function refreshFiles() {
            const res = await apiCall('/list_recordings', 'GET');
            if (res?.files) {
                const sel = document.getElementById('trajectorySelect');
                sel.innerHTML = '';
                res.files.forEach(f => {
                    const opt = document.createElement('option');
                    opt.value = f;
                    opt.textContent = f;
                    sel.appendChild(opt);
                });
                document.getElementById('fileList').innerHTML = 
                    res.files.length ? res.files.map(f => `<div>• ${f}</div>`).join('') : 'No files';
            }
        }

        async function startActiveAssist() {
            const file = document.getElementById('trajectorySelect').value;
            if (!file) { alert('Select a trajectory first!'); return; }
            const loadRes = await apiCall('/load_trajectory', 'POST', {filename: file});
            if (loadRes?.status !== 'success') return;
            const res = await apiCall('/start_active_assist');
            if (res?.status === 'success') {
                document.getElementById('assistStatus').textContent = 'ON';
                document.getElementById('assistStatus').className = 'status active';
            } else {
                alert('Failed to start playback: ' + (res?.message || 'Unknown error'));
            }
        }

        async function stopActiveAssist() {
            await apiCall('/stop_active_assist');
            document.getElementById('assistStatus').textContent = 'OFF';
            document.getElementById('assistStatus').className = 'status inactive';
        }

        async function zeroFTSensor() {
            const res = await apiCall('/zero_ft_sensor');
            if (res?.status === 'success') {
                alert('FT Sensor calibrated! Baseline set.');
            } else {
                alert('Failed to calibrate: ' + (res?.message || 'Unknown error'));
            }
        }

        async function toggleDrag() {
            const res = await apiCall('/drag_mode', 'POST', {enable: null});
            if (res?.status === 'success') {
                document.getElementById('dragStatus').textContent = 
                    `Drag: ${res.enabled ? 'ON' : 'OFF'}`;
                document.getElementById('dragStatus').className = 
                    `status ${res.enabled ? 'active' : 'inactive'}`;
            }
        }

        async function shutdown() {
            if (confirm('Shutdown server?')) {
                await apiCall('/shutdown');
                alert('Server shutting down...');
            }
        }

        async function fetchTelemetry() {
            const res = await apiCall('/get_telemetry', 'GET');
            if (res) {
                document.getElementById('progress').textContent = res.progress_percent.toFixed(1);
                document.getElementById('forceValue').textContent = res.fz_force.toFixed(2);
                
                // Update raw FT sensor values
                if (res.ft_raw && res.ft_raw.length === 6) {
                    document.getElementById('fx').textContent = res.ft_raw[0].toFixed(3);
                    document.getElementById('fy').textContent = res.ft_raw[1].toFixed(3);
                    document.getElementById('fz').textContent = res.ft_raw[2].toFixed(3);
                    document.getElementById('tx').textContent = res.ft_raw[3].toFixed(3);
                    document.getElementById('ty').textContent = res.ft_raw[4].toFixed(3);
                    document.getElementById('tz').textContent = res.ft_raw[5].toFixed(3);
                }
                
                // Update force bar visualization
                const maxForce = 20; // Max force for visualization
                const barWidth = Math.min(Math.abs(res.fz_force) / maxForce * 100, 100);
                const forceBar = document.getElementById('forceBar');
                forceBar.style.width = barWidth + '%';
                forceBar.style.background = res.fz_force > 0 ? '#2ecc71' : '#e74c3c';
                
                if (!res.active) {
                    document.getElementById('assistStatus').textContent = 'OFF';
                    document.getElementById('assistStatus').className = 'status inactive';
                }
            }
        }

        // Init
        refreshFiles();
        telemetryInterval = setInterval(fetchTelemetry, 200);
    </script>
</body>
</html>
'''

# ==================================================================================
# BACKEND ENDPOINTS
# ==================================================================================

@app.route('/')
def index():
    return render_template_string(FRONTEND_HTML)

@app.route('/start_recording', methods=['POST'])
def start_recording():
    global is_recording, current_recording, drag_mode_enabled
    if is_recording:
        return jsonify({'status': 'error', 'message': 'Already recording'}), 400
    
    try:
        robot.DragTeachSwitch(1)
        drag_mode_enabled = True
    except Exception as e:
        return jsonify({'status': 'error', 'message': f'Failed to enable drag: {str(e)}'}), 500

    is_recording = True
    current_recording = []
    return jsonify({'status': 'success'})

@app.route('/stop_recording', methods=['POST'])
def stop_recording():
    global is_recording, current_recording, drag_mode_enabled
    if not is_recording:
        return jsonify({'status': 'error', 'message': 'Not recording'}), 400
    
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    filename = f"exercise_{timestamp}.json"
    filepath = os.path.join(RECORDINGS_DIR, filename)
    with open(filepath, 'w') as f:
        json.dump(current_recording, f)
    
    try:
        robot.DragTeachSwitch(0)
        drag_mode_enabled = False
    except Exception as e:
        print(f"Warning: Could not disable drag mode: {e}")
    
    is_recording = False
    current_recording = []
    return jsonify({'status': 'success', 'filename': filename})

@app.route('/list_recordings', methods=['GET'])
def list_recordings():
    files = [f for f in os.listdir(RECORDINGS_DIR) if f.endswith('.json')]
    return jsonify({'files': sorted(files)})

@app.route('/load_trajectory', methods=['POST'])
def load_trajectory():
    global loaded_trajectory
    filename = request.json.get('filename')
    if not filename:
        return jsonify({'status': 'error', 'message': 'Filename required'}), 400
    filepath = os.path.join(RECORDINGS_DIR, filename)
    if not os.path.exists(filepath):
        return jsonify({'status': 'error', 'message': 'File not found'}), 400
    try:
        with open(filepath, 'r') as f:
            loaded_trajectory = json.load(f)
        return jsonify({'status': 'success', 'points': len(loaded_trajectory)})
    except Exception as e:
        return jsonify({'status': 'error', 'message': f'Failed to load: {str(e)}'}), 500

@app.route('/start_active_assist', methods=['POST'])
def start_active_assist():
    global active_assist_enabled, loaded_trajectory
    
    with state_lock:
        if active_assist_enabled:
            return jsonify({'status': 'error', 'message': 'Already running'}), 400
        if not loaded_trajectory:
            return jsonify({'status': 'error', 'message': 'No trajectory loaded'}), 400
        if len(loaded_trajectory) < 2:
            return jsonify({'status': 'error', 'message': 'Trajectory too short'}), 400
        
        active_assist_enabled = True
    
    thread = threading.Thread(target=force_controlled_playback, daemon=True)
    thread.start()
    return jsonify({'status': 'success'})

@app.route('/stop_active_assist', methods=['POST'])
def stop_active_assist():
    global active_assist_enabled
    with state_lock:
        active_assist_enabled = False
    return jsonify({'status': 'success'})

@app.route('/drag_mode', methods=['POST'])
def drag_mode():
    global drag_mode_enabled
    data = request.json
    enable = data.get('enable')
    if enable is None:
        enable = not drag_mode_enabled
    try:
        robot.DragTeachSwitch(1 if enable else 0)
        drag_mode_enabled = enable
        return jsonify({'status': 'success', 'enabled': enable})
    except Exception as e:
        return jsonify({'status': 'error', 'message': str(e)}), 500

@app.route('/zero_ft_sensor', methods=['POST'])
def zero_ft_sensor():
    global baseline_forces
    try:
        # Read current FT values multiple times and average
        samples = []
        for _ in range(10):
            d = robot.FT_GetForceTorqueRCS()
            if d[0] == 0:
                raw = [d[1][0], -d[1][1], d[1][2], d[1][3], d[1][4], d[1][5]]
                samples.append(raw)
            time.sleep(0.02)
        
        if samples:
            # Calculate average baseline
            baseline_forces = [sum(s[i] for s in samples) / len(samples) for i in range(6)]
            return jsonify({'status': 'success', 'baseline': baseline_forces})
        else:
            return jsonify({'status': 'error', 'message': 'Failed to read FT sensor'}), 500
    except Exception as e:
        return jsonify({'status': 'error', 'message': str(e)}), 500

current_fz = 0.0  # Global for telemetry
current_ft_raw = [0.0] * 6  # All 6 FT sensor values (Fx, Fy, Fz, Tx, Ty, Tz)

@app.route('/get_telemetry', methods=['GET'])
def get_telemetry():
    with state_lock:
        progress_pct = 0.0
        if loaded_trajectory and len(loaded_trajectory) > 1:
            max_idx = len(loaded_trajectory) - 1
            progress_pct = (trajectory_index / max_idx) * 100.0
        return jsonify({
            'progress_percent': round(progress_pct, 1),
            'active': active_assist_enabled,
            'fz_force': round(current_fz, 2),
            'ft_raw': [round(v, 3) for v in current_ft_raw]  # All 6 raw values
        })

@app.route('/shutdown', methods=['POST'])
def shutdown():
    global running, active_assist_enabled, is_recording, drag_mode_enabled
    running = False
    with state_lock:
        active_assist_enabled = False
        is_recording = False
    try:
        robot.DragTeachSwitch(0)
        drag_mode_enabled = False
    except:
        pass
    time.sleep(0.5)
    os._exit(0)

# ==================================================================================
# BACKGROUND TASKS
# ==================================================================================

def background_recorder():
    global is_recording, current_recording
    while running:
        if is_recording:
            err, pose = robot.GetActualTCPPose()
            if err == 0:
                current_recording.append({'tcp': pose, 'timestamp': time.time()})
            time.sleep(0.02)
        else:
            time.sleep(0.1)

def get_tcp_pose_at_index(traj, idx):
    """Interpolate TCP pose at fractional trajectory index"""
    idx = max(0, min(idx, len(traj) - 1))  # Clamp to valid range
    
    if idx <= 0:
        return traj[0]['tcp'][:]
    if idx >= len(traj) - 1:
        return traj[-1]['tcp'][:]
    
    i = int(idx)
    frac = idx - i
    p0 = np.array(traj[i]['tcp'])
    p1 = np.array(traj[i+1]['tcp'])
    interpolated = p0 + frac * (p1 - p0)
    return interpolated.tolist()

# ==================================================================================
# FORCE-CONTROLLED TRAJECTORY PLAYBACK
# ==================================================================================
def force_controlled_playback():
    global active_assist_enabled, loaded_trajectory, trajectory_index, current_fz, current_ft_raw, baseline_forces

    with state_lock:
        if not active_assist_enabled:
            return
        traj_points = [point.copy() for point in loaded_trajectory] if loaded_trajectory else None

    if not traj_points or len(traj_points) < 2:
        with state_lock:
            active_assist_enabled = False
        return

    # Initialize FT sensor
    try:
        robot.FT_SetRCS(0)  # 0 = tool frame
        robot.FT_SetZero(0)  # Zero the sensor
        time.sleep(0.5)
        print("✅ FT sensor zeroed for force-controlled playback")
        
        # If no baseline set, create one now
        if baseline_forces is None:
            print("📊 Creating baseline calibration...")
            samples = []
            for _ in range(10):
                d = robot.FT_GetForceTorqueRCS()
                if d[0] == 0:
                    raw = [d[1][0], -d[1][1], d[1][2], d[1][3], d[1][4], d[1][5]]
                    samples.append(raw)
                time.sleep(0.02)
            if samples:
                baseline_forces = [sum(s[i] for s in samples) / len(samples) for i in range(6)]
                print(f"✅ Baseline set: {[f'{v:.2f}' for v in baseline_forces]}")
        
    except Exception as e:
        print(f"FT sensor initialization failed: {e}")
        with state_lock:
            active_assist_enabled = False
        return

    if robot.ServoMoveStart() != 0:
        with state_lock:
            active_assist_enabled = False
        return

    try:
        # Start at beginning of trajectory
        traj_idx = 0.0
        print("🎮 Force-controlled playback started - apply FZ force to move!")

        while True:
            with state_lock:
                if not active_assist_enabled:
                    break

            t0 = time.time()

            # Initialize default values
            trajectory_speed = 0.0
            fz = 0.0

            # Read FT sensor with CORRECT processing
            d = robot.FT_GetForceTorqueRCS()
            if d[0] == 0:
                # YOUR EXACT LOGIC: Apply sign correction and baseline subtraction
                raw = [d[1][0], -d[1][1], d[1][2], d[1][3], d[1][4], d[1][5]]
                forces = raw if not baseline_forces else [raw[i] - baseline_forces[i] for i in range(6)]
                
                # Apply deadband threshold
                for i in range(6):
                    if abs(forces[i]) < DEADBAND_THRESHOLD:
                        forces[i] = 0.0
                
                current_ft_raw = forces  # Update global for telemetry
                fz = forces[2]  # Index 2 = FZ
                current_fz = fz
                
                # Calculate trajectory speed based on force
                # IMPORTANT: Robot only moves when force is applied!
                if abs(fz) < MIN_FORCE_THRESHOLD:
                    # Below threshold: NO MOVEMENT - stay exactly at current position
                    trajectory_speed = 0.0
                else:
                    # Above threshold: speed proportional to force
                    trajectory_speed = fz * FORCE_SCALE
                
                # Print raw values to console continuously (NEW LINE EVERY TIME)
                print(f"Forces: Fx={forces[0]:.2f}, Fy={forces[1]:.2f}, Fz={forces[2]:.2f}, "
                      f"Tx={forces[3]:.2f}, Ty={forces[4]:.2f}, Tz={forces[5]:.2f} | "
                      f"Traj Speed: {trajectory_speed:.4f} | Idx: {traj_idx:.2f}/{len(traj_points)-1}")
            else:
                print(f"[ERROR] FT read failed (code: {d[0]})")
                current_fz = 0.0
                current_ft_raw = [0.0] * 6
                trajectory_speed = 0.0

            # Only update trajectory index if there's actual force
            if trajectory_speed != 0.0:
                traj_idx += trajectory_speed
                # Clamp to trajectory bounds (no looping)
                traj_idx = max(0.0, min(traj_idx, len(traj_points) - 1))

            with state_lock:
                trajectory_index = traj_idx  # Update for telemetry

            # Get interpolated target pose
            target_tcp = get_tcp_pose_at_index(traj_points, traj_idx)

            # Inverse kinematics
            ik = robot.GetInverseKin(0, target_tcp, -1)
            if ik[0] != 0:
                time.sleep(SERVO_UPDATE_RATE)
                continue
            desired_joints = ik[1][:6]

            # Read current joint angles
            j_read = robot.GetActualJointPosDegree(flag=0)
            if j_read[0] != 0:
                time.sleep(SERVO_UPDATE_RATE)
                continue
            current_joints = j_read[1][:6]

            # Generate smooth joint command with velocity limiting
            joint_pos = []
            joint_vel = []
            for i in range(6):
                pos_error = desired_joints[i] - current_joints[i]
                vel_cmd = pos_error / SERVO_UPDATE_RATE
                vel_cmd = np.clip(vel_cmd, -MAX_JOINT_VELOCITY, MAX_JOINT_VELOCITY)
                joint_pos.append(current_joints[i] + vel_cmd * SERVO_UPDATE_RATE)
                joint_vel.append(abs(vel_cmd))

            # Send command - use actual velocities instead of zeros
            robot.ServoJ(joint_pos, joint_vel, 0, 0, SERVO_UPDATE_RATE, 0, 0)

            # Maintain loop timing
            elapsed = time.time() - t0
            sleep_t = SERVO_UPDATE_RATE - elapsed
            if sleep_t > 0:
                time.sleep(sleep_t)

    except Exception as e:
        print(f"\nPlayback error: {e}")
    finally:
        print("\n🛑 Force-controlled playback stopped")
        robot.ServoMoveEnd()
        with state_lock:
            active_assist_enabled = False
            trajectory_index = 0.0
            current_fz = 0.0
            current_ft_raw = [0.0] * 6

# ==================================================================================
# MAIN
# ==================================================================================
def signal_handler(sig, frame):
    global running
    running = False
    try:
        robot.DragTeachSwitch(0)
    except:
        pass
    time.sleep(1)
    os._exit(0)

signal.signal(signal.SIGINT, signal_handler)

if __name__ == '__main__':
    robot = Robot.RPC(ROBOT_IP)
    print("✅ Connected to robot")
    
    # Initialize FT sensor
    try:
        robot.FT_SetRCS(0)
        robot.FT_SetZero(0)
        print("✅ FT sensor initialized")
    except Exception as e:
        print(f"⚠️ FT sensor initialization warning: {e}")
    
    threading.Thread(target=background_recorder, daemon=True).start()
    print("🚀 Web UI running on http://localhost:5000")
    app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False)