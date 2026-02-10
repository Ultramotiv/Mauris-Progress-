# -*- coding: utf-8 -*-
"""
Rehabilitation Robot Controller — Continuous Trajectory Playback (NO FORCE ASSIST)
- Robot repeats the loaded trajectory forever once started.
- Ignores all user force input.
- Smooth looping with constant speed.
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

# === Parameters ===
MAX_JOINT_VELOCITY = 20.0     # deg/s
SERVO_UPDATE_RATE = 0.008     # 8ms control cycle
TRAJECTORY_STEP = 0.02        # Smaller = slower playback; adjust as needed

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
    </style>
</head>
<body>
    <h1>🩺 Rehabilitation Robot Controller</h1>

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
        <h3>🔁 Continuous Playback Mode</h3>
        <p>Robot will repeat the selected trajectory forever until stopped.</p>
        <select id="trajectorySelect" style="margin: 10px 0;"></select>
        <div class="grid">
            <button class="assist" onclick="startActiveAssist()">Start Playback</button>
            <button class="stop" onclick="stopActiveAssist()">Stop Playback</button>
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
    
    thread = threading.Thread(target=active_assist_loop, daemon=True)
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

@app.route('/get_telemetry', methods=['GET'])
def get_telemetry():
    with state_lock:
        progress_pct = 0.0
        if loaded_trajectory and len(loaded_trajectory) > 1:
            max_idx = len(loaded_trajectory) - 1
            # Wrap index for looping
            wrapped_idx = trajectory_index % max_idx
            progress_pct = (wrapped_idx / max_idx) * 100.0
        return jsonify({
            'progress_percent': round(progress_pct, 1),
            'active': active_assist_enabled
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
# CONTINUOUS TRAJECTORY PLAYBACK LOOP (NO FORCE GATING)
# ==================================================================================
def active_assist_loop():
    global active_assist_enabled, loaded_trajectory, trajectory_index

    with state_lock:
        if not active_assist_enabled:
            return
        traj_points = [point.copy() for point in loaded_trajectory] if loaded_trajectory else None

    if not traj_points or len(traj_points) < 2:
        with state_lock:
            active_assist_enabled = False
        return

    if robot.ServoMoveStart() != 0:
        with state_lock:
            active_assist_enabled = False
        return

    try:
        # Start from beginning of trajectory
        traj_idx = 0.0

        while True:
            with state_lock:
                if not active_assist_enabled:
                    break
                trajectory_index = traj_idx  # Update for telemetry

            t0 = time.time()

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

            # Generate smooth joint command
            joint_pos = current_joints[:]
            for i in range(6):
                pos_error = desired_joints[i] - joint_pos[i]
                vel_cmd = pos_error / SERVO_UPDATE_RATE
                vel_cmd = np.clip(vel_cmd, -MAX_JOINT_VELOCITY, MAX_JOINT_VELOCITY)
                joint_pos[i] += vel_cmd * SERVO_UPDATE_RATE

            # Send command
            robot.ServoJ(joint_pos, [0]*6, 0, 0, SERVO_UPDATE_RATE, 0, 0)

            # Advance trajectory index
            traj_idx += TRAJECTORY_STEP

            # Loop back to start when reaching end
            if traj_idx >= len(traj_points) - 1:
                traj_idx = 0.0

            # Maintain loop timing
            elapsed = time.time() - t0
            sleep_t = SERVO_UPDATE_RATE - elapsed
            if sleep_t > 0:
                time.sleep(sleep_t)

    except Exception as e:
        print(f"Playback error: {e}")
    finally:
        robot.ServoMoveEnd()
        with state_lock:
            active_assist_enabled = False
            trajectory_index = 0.0

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
    # FT sensor initialization REMOVED — not used in playback mode
    threading.Thread(target=background_recorder, daemon=True).start()
    print("🚀 Web UI running on http://localhost:5000")
    app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False)