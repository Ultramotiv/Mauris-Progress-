# Passive Therapy Robot - Smooth Playback Only jerks all over the trajectory ! 
# this code follows the trajectory in forward and in reverse But based on odd/Even so change this as well ! 
# Removed: Force control, force sensor, tangential force calculations

import sys
import os
import time
import json
import threading
import signal
import math
import io
import bisect
import numpy as np

# === Matplotlib Setup (Server Side) ===
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from flask import Flask, request, jsonify, render_template_string, Response
from flask_cors import CORS

# === Robot Setup ===
sys.path.append('/home/um/fairino-python-sdk-main/linux/fairino')
import Robot

ROBOT_IP = '192.168.58.2'
RECORDINGS_DIR = 'therapy_recordings'
os.makedirs(RECORDINGS_DIR, exist_ok=True)

robot = None
running = True

# === Global State ===
is_recording = False
current_recording = []
drag_mode_enabled = False
playback_active = False

# Trajectory Data
loaded_trajectory = None
trajectory_points_np = None
cum_arc_lengths = None
total_arc_length = 0.0
current_actual_tcp = [0.0, 0.0, 0.0]
playback_repetition_count = 0

# Threading Locks
state_lock = threading.Lock()
plot_lock = threading.Lock()

# === Playback Parameters ===
PLAYBACK_SPEED = 26.0       # deg/sec - constant joint velocity for smooth motion
SERVO_UPDATE_RATE = 0.008   # 8ms servo loop (125Hz) for smooth continuous motion
SKIP_POINTS = 1             # Use all trajectory points for accuracy
BLEND_RADIUS = 0.0          # No blending needed with continuous ServoJ
RECORD_RATE = 0.0025        # 400Hz recording rate (2.5ms interval) - matches app.py TPD recorder
MAX_JOINT_VELOCITY = 26.0   # Maximum 26 deg/sec for safe smooth motion

# === Flask App ===
app = Flask(__name__)
CORS(app)

# === Global Parameters ===
IMPEDANCE_PARAMS = {
    'lamde_dain': [2.5, 2.0, 2.0, 2.0, 2.0, 2.0],
    'b_gain': [10.0, 5.0, 5.0, 5.0, 5.0, 1.0],
    'k_gain': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    'max_tcp_vel': 500,
    'max_tcp_ori_vel': 90
}

# ==================================================================================
# FRONTEND HTML
# ==================================================================================
FRONTEND_HTML = '''
<!DOCTYPE html>
<html>
<head>
    <title>Rehab Robot - Passive Therapy</title>
    <style>
        body { font-family: 'Segoe UI', sans-serif; max-width: 950px; margin: 0 auto; padding: 20px; background: #eef2f5; }
        h1 { text-align: center; color: #2c3e50; }
        .grid { display: grid; grid-template-columns: 1fr 1fr; gap: 20px; }
        .section { background: white; padding: 20px; border-radius: 12px; box-shadow: 0 4px 6px rgba(0,0,0,0.1); }
        .full-width { grid-column: span 2; }
        
        button { 
            width: 100%; padding: 12px; margin: 5px 0; border: none; border-radius: 6px; 
            font-weight: bold; cursor: pointer; transition: 0.2s; font-size: 14px;
        }
        .btn-blue { background: #3498db; color: white; }
        .btn-red { background: #e74c3c; color: white; }
        .btn-green { background: #2ecc71; color: white; }
        .btn-orange { background: #f39c12; color: white; }
        button:hover { opacity: 0.9; transform: translateY(-1px); }
        button:disabled { background: #bdc3c7; cursor: not-allowed; }

        .status-box { 
            background: #34495e; color: white; padding: 15px; border-radius: 6px; 
            margin-top: 10px; font-family: monospace; font-size: 13px; line-height: 1.6;
        }
        .highlight { color: #f1c40f; font-weight: bold; font-size: 1.1em; }
        
        .vis-container { 
            width: 100%; height: 500px; 
            background: #fff; border: 1px solid #ddd;
            display: flex; align-items: center; justify-content: center;
            border-radius: 8px; overflow: hidden;
        }
        .vis-container img { width: 100%; height: 100%; object-fit: contain; }
    </style>
</head>
<body>
    <h1>🏥 Passive Trajectory Playback</h1>
    
    <div class="grid">
        <div class="section">
            <h2>1. Record Path</h2>
            <button class="btn-blue" onclick="toggleDrag()">Toggle Drag Mode</button>
            <button class="btn-red" onclick="startRecording()">Start Recording</button>
            <button class="btn-red" onclick="stopRecording()">Stop & Save</button>
        </div>

        <div class="section">
            <h2>2. Load & Play</h2>
            <select id="fileList" style="width:100%; padding:10px; margin-bottom:10px; border-radius:4px; border:1px solid #ccc;"></select>
            <button class="btn-orange" onclick="loadTrajectory()">Load Selected Path</button>
            <hr style="margin: 15px 0;">
            <label for="reps">Repetitions:</label> <input id="reps" type="number" min="1" value="1" style="width:60px; margin-bottom:10px;">
            <button class="btn-green" onclick="startPlayback()">▶ Start Playback</button>
            <button class="btn-red" onclick="stopPlayback()">⏹ Stop Playback</button>
            
            <div class="status-box">
                <div>Status: <span id="sysStatus">Idle</span></div>
                <div>Repetition: <span id="repCount" class="highlight">0</span></div>
                <div>Path Length: <span id="pathLen">-</span> m</div>
            </div>
        </div>

        <div class="section full-width">
            <h2>📊 Trajectory Visualization</h2>
            <div class="vis-container">
                <img src="/video_feed" alt="Load a trajectory to see the graph">
            </div>
            <p style="text-align:center; color:#7f8c8d; font-size:0.9em; margin-top:5px;">
                <span style="color:blue; font-weight:bold;">Blue Line</span> = Path &nbsp;|&nbsp; 
                <span style="color:red; font-weight:bold;">Red Dot</span> = Robot Position
            </p>
        </div>
    </div>

    <script>
        async function api(url, method='POST', body=null) {
            try {
                const opts = { method, headers: {'Content-Type': 'application/json'} };
                if(body) opts.body = JSON.stringify(body);
                const res = await fetch(url, opts);
                return await res.json();
            } catch(e) { console.error(e); alert("Connection Error"); }
        }

        async function toggleDrag() { 
            const res = await api('/drag_mode'); 
            updateStatus(res.enabled ? "Drag Mode ON" : "Drag Mode OFF");
        }
        
        async function startRecording() { 
            await api('/start_recording'); 
            updateStatus("Recording..."); 
        }
        
        async function stopRecording() { 
            await api('/stop_recording'); 
            updateStatus("Path Saved"); 
            refreshFiles(); 
        }
        
        async function refreshFiles() {
            const res = await api('/list_files', 'GET');
            const sel = document.getElementById('fileList');
            if (res.files.length === 0) {
                sel.innerHTML = '<option>No files found</option>';
                return;
            }
            sel.innerHTML = res.files.map(f => `<option value="${f}">${f}</option>`).join('');
        }
        
        async function loadTrajectory() {
            const fname = document.getElementById('fileList').value;
            if(!fname || fname.includes('No files')) return alert("Record a file first!");
            const res = await api('/load_traj', 'POST', {filename: fname});
            if(res.status === 'ok') {
                updateStatus(`Loaded: ${res.length.toFixed(2)}m long`);
                document.getElementById('pathLen').innerText = res.length.toFixed(2);
            }
            else alert("Error loading file");
        }

        async function startPlayback() { 
            const reps = document.getElementById('reps').value;
            const res = await api('/start_playback', 'POST', {repetitions: parseInt(reps)});
            if(res.status === 'ok') updateStatus("Playing...");
            else alert(res.error || "Failed to start");
        }
        
        async function stopPlayback() { 
            await api('/stop_playback'); 
            updateStatus("Stopped");
        }

        function updateStatus(msg) { 
            document.getElementById('sysStatus').innerText = msg; 
        }

        // Telemetry Loop
        setInterval(async () => {
            try {
                const res = await fetch('/telemetry');
                const data = await res.json();
                
                if(data.active) {
                    document.getElementById('sysStatus').innerText = "Playing...";
                    document.getElementById('repCount').innerText = data.repetition;
                }
            } catch(e) {}
        }, 200);

        refreshFiles();
    </script>
</body>
</html>
'''

# ==================================================================================
# VISUALIZATION
# ==================================================================================
def generate_plot_frame():
    """Generates a 3D plot of trajectory and robot position"""
    with plot_lock:
        fig = plt.figure(figsize=(6, 5), dpi=80)
        ax = fig.add_subplot(111, projection='3d')
        
        if trajectory_points_np is not None and len(trajectory_points_np) > 1:
            step = 1 if len(trajectory_points_np) < 500 else 5
            xs = trajectory_points_np[::step, 0]
            ys = trajectory_points_np[::step, 1]
            zs = trajectory_points_np[::step, 2]

            ax.plot(xs, ys, zs, color='blue', linewidth=1.5, label='Path')
            
            if loaded_trajectory:
                ax.scatter([current_actual_tcp[0]], [current_actual_tcp[1]], [current_actual_tcp[2]], 
                          color='red', s=100, label='TCP', edgecolors='white')

            x_min, x_max = xs.min(), xs.max()
            y_min, y_max = ys.min(), ys.max()
            z_min, z_max = zs.min(), zs.max()

            max_range = np.array([x_max-x_min, y_max-y_min, z_max-z_min]).max() / 2.0
            if max_range == 0: max_range = 1.0 
            
            mid_x = (x_max+x_min) * 0.5
            mid_y = (y_max+y_min) * 0.5
            mid_z = (z_max+z_min) * 0.5

            ax.set_xlim(mid_x - max_range*1.1, mid_x + max_range*1.1)
            ax.set_ylim(mid_y - max_range*1.1, mid_y + max_range*1.1)
            ax.set_zlim(mid_z - max_range*1.1, mid_z + max_range*1.1)

            ax.set_xlabel('X')
            ax.set_ylabel('Y')
            ax.set_zlabel('Z')
            ax.set_title('Trajectory View')
            ax.legend()
        else:
            ax.text2D(0.5, 0.5, "No Trajectory Loaded", transform=ax.transAxes, 
                     ha='center', va='center', fontsize=12, color='red')
            ax.set_axis_off()

        buf = io.BytesIO()
        plt.savefig(buf, format='jpg', bbox_inches='tight')
        plt.close(fig)
        buf.seek(0)
        return buf.read()

def video_stream_gen():
    while True:
        frame = generate_plot_frame()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        time.sleep(0.1)

# ==================================================================================
# DRAG MODE HELPER
# ==================================================================================
def custom_drag_teach_mode(enable=True):
    global drag_mode_enabled, robot
    if enable:
        robot.DragTeachSwitch(1)
        time.sleep(0.5)
        rtn = robot.ForceAndJointImpedanceStartStop(
            status=1, impedanceFlag=1,
            lamdeDain=IMPEDANCE_PARAMS['lamde_dain'],
            KGain=IMPEDANCE_PARAMS['k_gain'],
            BGain=IMPEDANCE_PARAMS['b_gain'],
            dragMaxTcpVel=IMPEDANCE_PARAMS['max_tcp_vel'],
            dragMaxTcpOriVel=IMPEDANCE_PARAMS['max_tcp_ori_vel']
        )
        drag_mode_enabled = (rtn == 0)
        return rtn == 0
    else:
        robot.ForceAndJointImpedanceStartStop(
            status=0, impedanceFlag=0,
            lamdeDain=[0]*6, KGain=[0]*6, BGain=[0]*6,
            dragMaxTcpVel=1000, dragMaxTcpOriVel=180
        )
        robot.DragTeachSwitch(0)
        time.sleep(0.5)
        drag_mode_enabled = False
        return True

# ==================================================================================
# SMOOTH TRAJECTORY PLAYBACK
# ==================================================================================
def trajectory_playback_loop(repetitions=1):
    """Smooth ServoJ-based playback at constant 26 deg/sec"""
    global loaded_trajectory, playback_repetition_count, playback_active, current_actual_tcp
    
    if not loaded_trajectory or len(loaded_trajectory) < 2:
        print("❌ No trajectory loaded!")
        playback_active = False
        return
    
    print(f"▶️ ServoJ playback at {PLAYBACK_SPEED}°/sec for {repetitions} repetitions...")
    robot.DragTeachSwitch(0)
    time.sleep(0.2)
    
    try:
        # Pre-compute all joint solutions via IK
        print("🔄 Pre-computing joint solutions via IK...")
        joint_solutions = []
        
        for tcp_data in loaded_trajectory:
            target_tcp = tcp_data['tcp']
            err, joints = robot.GetInverseKin(0, target_tcp, -1)
            if err == 0:
                joint_solutions.append(joints[:6])
            else:
                # Use last valid solution if IK fails
                if joint_solutions:
                    joint_solutions.append(joint_solutions[-1])
        
        if not joint_solutions:
            print("❌ Failed to compute joint solutions!")
            return
        
        print(f"✅ Pre-computed {len(joint_solutions)} joint configurations")
        
        for rep in range(repetitions):
            playback_repetition_count = rep + 1
            is_forward = (playback_repetition_count % 2 == 0)
            direction_str = "FORWARD" if is_forward else "REVERSE"
            
            print(f"\n📍 Rep {rep+1}/{repetitions} | {direction_str}")
            
            # Select waypoint order
            if is_forward:
                waypoints = joint_solutions[::SKIP_POINTS]
            else:
                waypoints = joint_solutions[::-1][::SKIP_POINTS]
            
            if not waypoints:
                print(f"❌ No waypoints for {direction_str}")
                continue
            
            # Start ServoJ mode
            print(f"▶️ Starting ServoJ motion {direction_str}...")
            robot.ServoMoveStart()
            time.sleep(0.1)
            
            # Execute continuous ServoJ trajectory
            try:
                for i, joint_pos in enumerate(waypoints):
                    if not playback_active:
                        print("🛑 Stopped by user")
                        robot.ServoMoveEnd()
                        return
                    
                    # Send joint command with constant velocity
                    robot.ServoJ(
                        joint_pos,
                        [0] * 6,           # 0 velocity (ServoJ handles motion)
                        0,                 # timeout
                        0,                 # lookahead
                        SERVO_UPDATE_RATE, # 8ms update
                        0,                 # reserve1
                        0                  # reserve2
                    )
                    
                    # Update TCP visualization
                    if i % 50 == 0:
                        err, actual_pose = robot.GetActualTCPPose(flag=1)
                        if err == 0:
                            current_actual_tcp = actual_pose[:3]
                        
                        progress = (i / len(waypoints)) * 100
                        print(f"  ✓ {direction_str}: {progress:5.1f}%")
                    
                    # Constant time step for smooth motion
                    time.sleep(SERVO_UPDATE_RATE)
                
                # End ServoJ mode for this rep
                robot.ServoMoveEnd()
                time.sleep(0.2)
                
            except Exception as e:
                print(f"❌ ServoJ error: {e}")
                try:
                    robot.ServoMoveEnd()
                except:
                    pass
                continue
            
            print(f"✅ Repetition {rep+1} complete")
            time.sleep(0.3)
        
        playback_repetition_count = repetitions
        print(f"\n✅ All {repetitions} repetitions completed!")
        
    except Exception as e:
        print(f"❌ Playback error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("⏹️ Playback finished.")
        playback_active = False
        try:
            robot.ServoMoveEnd()
        except:
            pass
        playback_active = False

# ==================================================================================
# BACKGROUND THREAD
# ==================================================================================
def background_recorder():
    """Records trajectory points at 400Hz when enabled - matches app.py TPD recorder"""
    global current_recording, current_actual_tcp
    print(f"🔄 Background recorder started @ {1/RECORD_RATE:.0f}Hz")
    
    while running:
        err, pose = robot.GetActualTCPPose(flag=1)
        if err == 0:
            current_actual_tcp = pose[:3]
            
            if is_recording:
                current_recording.append({
                    'timestamp': time.time(),
                    'tcp': pose  # [x, y, z, rx, ry, rz] - 6D pose
                })
        
        time.sleep(RECORD_RATE)  # 2.5ms interval = 400Hz sampling

# ==================================================================================
# API ROUTES
# ==================================================================================
@app.route('/')
def index(): 
    return render_template_string(FRONTEND_HTML)

@app.route('/video_feed')
def video_feed():
    return Response(video_stream_gen(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/drag_mode', methods=['POST'])
def toggle_drag():
    global drag_mode_enabled
    try:
        drag_mode_enabled = not drag_mode_enabled
        robot.DragTeachSwitch(1 if drag_mode_enabled else 0)
        return jsonify({'status': 'ok', 'enabled': drag_mode_enabled})
    except Exception as e: 
        return jsonify({'status': 'error', 'msg': str(e)})

@app.route('/start_recording', methods=['POST'])
def start_rec():
    global is_recording, current_recording, drag_mode_enabled
    if not drag_mode_enabled:
        robot.DragTeachSwitch(1)
        drag_mode_enabled = True
    current_recording = []
    is_recording = True
    return jsonify({'status': 'ok'})

@app.route('/stop_recording', methods=['POST'])
def stop_rec():
    global is_recording, current_recording, drag_mode_enabled
    is_recording = False
    try:
        robot.DragTeachSwitch(0)
        drag_mode_enabled = False
    except Exception:
        pass
    
    # Save trajectory
    filename = f"traj_{int(time.time())}.json"
    path = os.path.join(RECORDINGS_DIR, filename)
    with open(path, 'w') as f:
        json.dump(current_recording, f, indent=2)
    
    current_recording = []
    return jsonify({'status': 'ok', 'filename': filename})

@app.route('/list_files', methods=['GET'])
def list_files():
    files = sorted([f for f in os.listdir(RECORDINGS_DIR) if f.endswith('.json')], reverse=True)
    return jsonify({'files': files})

@app.route('/load_traj', methods=['POST'])
def load_traj():
    global loaded_trajectory, cum_arc_lengths, total_arc_length, trajectory_points_np
    
    try:
        name = request.json['filename']
        path = os.path.join(RECORDINGS_DIR, name)
        
        with open(path, 'r') as f: 
            loaded_trajectory = json.load(f)
        
        if len(loaded_trajectory) < 2: 
            return jsonify({'status': 'error', 'msg': 'Path too short'})
        
        points = [np.array(p['tcp'][:3]) for p in loaded_trajectory]
        
        with plot_lock:
            trajectory_points_np = np.array(points)
        
        # Calculate arc lengths
        dists = [0.0]
        for i in range(1, len(points)):
            d = np.linalg.norm(points[i] - points[i-1])
            dists.append(dists[-1] + d)
        
        cum_arc_lengths = dists
        total_arc_length = dists[-1]
        
        print(f"✅ Loaded {len(points)} points. Length: {total_arc_length:.3f}m")
        return jsonify({'status': 'ok', 'length': total_arc_length})
        
    except Exception as e:
        print(f"❌ Load Error: {e}")
        return jsonify({'status': 'error', 'msg': str(e)}), 500

@app.route('/start_playback', methods=['POST'])
def start_playback_route():
    global playback_active
    if not loaded_trajectory: 
        return jsonify({'error': 'No trajectory loaded'}), 400
    if playback_active:
        return jsonify({'error': 'Playback already active'}), 400
    
    repetitions = request.json.get('repetitions', 1)
    playback_active = True
    threading.Thread(target=trajectory_playback_loop, args=(repetitions,), daemon=True).start()
    return jsonify({'status': 'ok'})

@app.route('/stop_playback', methods=['POST'])
def stop_playback_route():
    global playback_active
    playback_active = False
    try:
        robot.StopMotion()
    except:
        pass
    return jsonify({'status': 'ok'})

@app.route('/telemetry')
def telemetry():
    return jsonify({
        'active': playback_active,
        'repetition': playback_repetition_count
    })

# ==================================================================================
# MAIN
# ==================================================================================
if __name__ == '__main__':
    try:
        robot = Robot.RPC(ROBOT_IP)
        print(f"✅ Connected to Robot: {ROBOT_IP}")
    except:
        print("❌ ERROR: Could not connect to robot. Check IP.")
        sys.exit(1)

    rec_thread = threading.Thread(target=background_recorder, daemon=True)
    rec_thread.start()

    print("🌐 Starting Web Server on http://localhost:5000")
    app.run(host='0.0.0.0', port=5000, threaded=True)