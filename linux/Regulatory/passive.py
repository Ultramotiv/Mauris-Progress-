# -*- coding: utf-8 -*-
"""
Minimal Flask Backend for Passive Therapy Mode
- Record trajectory (TCP poses)
- Playback recorded trajectory
- Drag Teach toggle
- Telemetry (Fz, TCP Z)
- Shutdown
"""

import sys
import os
import time
import json
import threading
import signal
from flask import Flask, request, jsonify
from flask_cors import CORS

# === Robot Setup ===
sys.path.append('/home/um/fairino-python-sdk-main/linux/fairino')
import Robot

ROBOT_IP = '192.168.58.2'
RECORDINGS_DIR = 'recordings'
os.makedirs(RECORDINGS_DIR, exist_ok=True)

robot = None
running = True

# === State ===
is_recording = False
current_recording = []
playback_active = False
drag_mode_enabled = False

# === Telemetry ===
current_fz = 0.0
current_tcp_z = 0.0
baseline_forces = [0.0] * 6

# === Flask App ===
app = Flask(__name__)
CORS(app)

# === Helper: Initialize FT Sensor & Calibrate ===
def init_ft_sensor():
    robot.FT_SetConfig(24, 0)
    robot.FT_Activate(1)
    time.sleep(1.0)
    robot.SetLoadWeight(0, 0.0)
    robot.FT_SetZero(1)
    time.sleep(0.5)

def calibrate_baseline(samples=100):
    global baseline_forces
    forces = []
    for _ in range(samples):
        ret = robot.FT_GetForceTorqueRCS()
        if ret[0] == 0:
            forces.append(ret[1][:6])
        time.sleep(0.01)
    if forces:
        baseline_forces = (sum(f) / len(forces) for f in zip(*forces))

# === Background Recorder ===
def background_recorder():
    global is_recording, current_recording
    while running:
        if is_recording:
            err, pose = robot.GetActualTCPPose()
            if err == 0:
                current_recording.append({
                    'tcp': pose,
                    'timestamp': time.time()
                })
            time.sleep(0.02)  # 50 Hz
        else:
            time.sleep(0.1)

# === Playback Executor ===
def playback_trajectory(filepath, repetitions):
    global playback_active
    try:
        with open(filepath, 'r') as f:
            trajectory = json.load(f)
        
        playback_active = True
        err, current = robot.GetActualTCPPose()
        if err != 0:
            return

        for rep in range(repetitions):
            if not playback_active:
                break
            # Move to start
            start_pose = trajectory[0]['tcp']
            robot.MoveL(start_pose, 0, 0)
            time.sleep(0.5)
            # Play recorded poses
            for point in trajectory:
                if not playback_active:
                    break
                robot.MoveL(point['tcp'], 50, 0.5)  # Adjust v, blend as needed
                time.sleep(0.02)
    except Exception as e:
        print(f"Playback error: {e}")
    finally:
        playback_active = False

# === API Endpoints ===
@app.route('/drag_mode', methods=['POST'])
def drag_mode():
    global drag_mode_enabled
    data = request.json
    enable = data.get('enable', not drag_mode_enabled)
    try:
        robot.DragTeachSwitch(1 if enable else 0)
        drag_mode_enabled = enable
        return jsonify({'status': 'success', 'enabled': enable})
    except Exception as e:
        return jsonify({'status': 'error', 'message': str(e)}), 500

@app.route('/start_recording', methods=['POST'])
def start_recording():
    global is_recording, current_recording
    if is_recording:
        return jsonify({'status': 'error', 'message': 'Already recording'}), 400
    is_recording = True
    current_recording = []
    return jsonify({'status': 'success'})

@app.route('/stop_recording', methods=['POST'])
def stop_recording():
    global is_recording, current_recording
    if not is_recording:
        return jsonify({'status': 'error', 'message': 'Not recording'}), 400
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    filename = f"exercise_{timestamp}.json"
    filepath = os.path.join(RECORDINGS_DIR, filename)
    with open(filepath, 'w') as f:
        json.dump(current_recording, f)
    is_recording = False
    current_recording = []
    return jsonify({'status': 'success', 'filename': filename})

@app.route('/list_recordings', methods=['GET'])
def list_recordings():
    files = [f for f in os.listdir(RECORDINGS_DIR) if f.endswith('.json')]
    return jsonify({'files': sorted(files)})

@app.route('/start_playback', methods=['POST'])
def start_playback():
    global playback_active
    if playback_active:
        return jsonify({'status': 'error', 'message': 'Playback already active'}), 400
    data = request.json
    filename = data.get('filename')
    repetitions = data.get('repetitions', 1)
    if not filename:
        return jsonify({'status': 'error', 'message': 'Filename required'}), 400
    filepath = os.path.join(RECORDINGS_DIR, filename)
    if not os.path.exists(filepath):
        return jsonify({'status': 'error', 'message': 'File not found'}), 400
    thread = threading.Thread(target=playback_trajectory, args=(filepath, repetitions), daemon=True)
    thread.start()
    return jsonify({'status': 'success'})

@app.route('/stop_playback', methods=['POST'])
def stop_playback():
    global playback_active
    playback_active = False
    return jsonify({'status': 'success'})

@app.route('/get_telemetry', methods=['GET'])
def get_telemetry():
    global current_fz, current_tcp_z
    return jsonify({
        'fz': round(current_fz, 2),
        'tcp_z': round(current_tcp_z, 2)
    })

@app.route('/shutdown', methods=['POST'])
def shutdown():
    global running
    running = False
    time.sleep(0.5)
    os._exit(0)

# === Background Telemetry Updater ===
def update_telemetry():
    global current_fz, current_tcp_z
    while running:
        try:
            err, tcp = robot.GetActualTCPPose()
            if err == 0:
                current_tcp_z = tcp[2]
            ft = robot.FT_GetForceTorqueRCS()
            if ft[0] == 0:
                raw = ft[1][:6]
                compensated = [raw[i] - baseline_forces[i] for i in range(6)]
                # Simplified: assume Z force in tool frame
                current_fz = compensated[2]
        except:
            pass
        time.sleep(0.1)

# === Shutdown Handler ===
def signal_handler(sig, frame):
    global running
    running = False
    time.sleep(1)
    os._exit(0)

signal.signal(signal.SIGINT, signal_handler)

# === Main ===
if __name__ == '__main__':
    robot = Robot.RPC(ROBOT_IP)
    print("✅ Connected to robot")
    init_ft_sensor()
    calibrate_baseline()
    threading.Thread(target=background_recorder, daemon=True).start()
    threading.Thread(target=update_telemetry, daemon=True).start()
    print("🚀 Passive Therapy Server running on http://localhost:5000")
    app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False)