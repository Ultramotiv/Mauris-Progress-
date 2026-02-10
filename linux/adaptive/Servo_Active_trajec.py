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
IK_SOLUTIONS_DIR = 'ik_solutions'  # NEW: Store pre-computed IK solutions
os.makedirs(RECORDINGS_DIR, exist_ok=True)
os.makedirs(IK_SOLUTIONS_DIR, exist_ok=True)

robot = None
running = True

# === Global State ===
is_recording = False
current_recording = []
force_control_active = False
drag_mode_enabled = False

# Trajectory Data
loaded_trajectory = None
loaded_ik_solutions = None  # NEW: Pre-computed joint solutions
trajectory_points_np = None
cum_arc_lengths = None
total_arc_length = 0.0
current_arc_position = 0.0

# Sensor & Control Data
baseline_forces = [0.0] * 6
current_ft_raw = [0.0] * 6
current_tangential_force = 0.0
current_actual_tcp = [0.0, 0.0, 0.0]

# Threading Locks
state_lock = threading.Lock()
plot_lock = threading.Lock()

# === Control Parameters ===
SERVO_UPDATE_RATE = 0.008      # 8ms servo loop
IK_COMPUTE_RATE = 0.0025       # 2.5ms IK computation (400Hz) - NEW
FORCE_TO_VELOCITY_SCALE = 0.006
MIN_FORCE_THRESHOLD = 0.3
FILTER_ALPHA = 0.25
VELOCITY_SMOOTHING = 0.8
PASSIVE_PLAYBACK_SPEED = 26.0  # mm/s
MAX_JOINT_VELOCITY = 26.0      # deg/sec (safety limit)
JOINT_FILTER_ALPHA = 0.35      # 0..1 low-pass for joint commands (higher = snappier)

# === Flask App ===
app = Flask(__name__)
CORS(app)

# === Global Parameters ===
IMPEDANCE_PARAMS = {
    'lamde_dain': [2.5, 2.0, 2.0, 2.0, 2.0, 2.0],
    'b_gain': [20.0, 10.0, 10.0, 5.0, 5.0, 1.0],
    'k_gain': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    'max_tcp_vel': 500,
    'max_tcp_ori_vel': 90
}

# ==================================================================================
# FRONTEND HTML (UNCHANGED)
# ==================================================================================
FRONTEND_HTML = '''
<!DOCTYPE html>
<html>
<head>
    <title>Rehab Robot - Servo Playback</title>
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
    <h1>🏥 ServoJ Trajectory Playback (IK Pre-Computed)</h1>
    
    <div class="grid">
        <div class="section">
            <h2>1. Setup</h2>
            <button class="btn-blue" onclick="toggleDrag()">Toggle Drag Mode</button>
            <button class="btn-red" onclick="startRecording()">Start Recording Path</button>
            <button class="btn-red" onclick="stopRecording()">Stop & Save Path</button>
            <hr>
            <h2>2. Calibration</h2>
            <select id="fileList" style="width:100%; padding:10px; margin-bottom:10px; border-radius:4px; border:1px solid #ccc;"></select>
            <button class="btn-orange" onclick="loadTrajectory()">Load & Compute IK</button>
            <button class="btn-orange" onclick="calibrateSensor()">Zero Force Sensor</button>
        </div>

        <div class="section">
            <h2>3. ServoJ Playback</h2>
            <label for="reps">Repetitions:</label> <input id="reps" type="number" min="1" value="1" style="width:60px; margin-bottom:10px;">
            <button class="btn-green" onclick="startTherapy()">Start Playback</button>
            <button class="btn-red" onclick="stopTherapy()">Stop Playback</button>
            
            <div class="status-box">
                <div>Status: <span id="sysStatus">Idle</span></div>
                <div>Progress: <span id="progress" class="highlight">0.0</span> %</div>
                <div>Repetition: <span id="rep" class="highlight">0</span></div>
                <div>IK Points Computed: <span id="ikCount">0</span></div>
                <hr style="border-color: #555;">
                <div style="font-size: 0.9em; color: #aaa;">Joint Angles (deg):</div>
                <div>J1-J3: <span id="j123">0.0 0.0 0.0</span></div>
                <div>J4-J6: <span id="j456">0.0 0.0 0.0</span></div>
            </div>
        </div>

        <div class="section full-width">
            <h2>Trajectory Visualization</h2>
            <div class="vis-container">
                <img src="/video_feed" alt="Load a trajectory to see the graph">
            </div>
            <p style="text-align:center; color:#7f8c8d; font-size:0.9em; margin-top:5px;">
                <span style="color:blue; font-weight:bold;">Blue Line</span> = Path &nbsp;|&nbsp; 
                <span style="color:red; font-weight:bold;">Red Dot</span> = Current TCP Position
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
            document.getElementById('sysStatus').innerText = res.enabled ? "Drag Mode ON" : "Drag Mode OFF";
        }
        async function startRecording() { await api('/start_recording'); document.getElementById('sysStatus').innerText = "Recording..."; }
        async function stopRecording() { 
            await api('/stop_recording'); 
            document.getElementById('sysStatus').innerText = "Path Saved"; 
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
            document.getElementById('sysStatus').innerText = "Computing IK...";
            const res = await api('/load_traj', 'POST', {filename: fname});
            if(res.status === 'ok') {
                document.getElementById('sysStatus').innerText = `Loaded: ${res.ik_count} IK points`;
                document.getElementById('ikCount').innerText = res.ik_count;
            }
            else alert("Error loading file");
        }

        async function calibrateSensor() {
            document.getElementById('sysStatus').innerText = "Calibrating...";
            await api('/zero_sensor');
            document.getElementById('sysStatus').innerText = "Sensor Zeroed";
        }

        async function startTherapy() { 
            const reps = document.getElementById('reps').value;
            const res = await api('/start_therapy', 'POST', {repetitions: parseInt(reps)});
            if(res.status === 'ok') document.getElementById('sysStatus').innerText = "Playback Active";
            else alert(res.error || "Failed to start");
        }
        async function stopTherapy() { 
            await api('/stop_therapy'); 
            document.getElementById('sysStatus').innerText = "Playback Stopped";
        }

        setInterval(async () => {
            try {
                const res = await fetch('/telemetry');
                const data = await res.json();
                document.getElementById('progress').innerText = data.progress.toFixed(1);
                document.getElementById('rep').innerText = data.rep;
                document.getElementById('j123').innerText = data.joints.slice(0,3).map(j => j.toFixed(1)).join(' ');
                document.getElementById('j456').innerText = data.joints.slice(3,6).map(j => j.toFixed(1)).join(' ');
            } catch(e) {}
        }, 200);

        refreshFiles();
    </script>
</body>
</html>
'''

# ==================================================================================
# MATPLOTLIB VISUALIZATION
# ==================================================================================
def generate_plot_frame():
    with plot_lock:
        fig = plt.figure(figsize=(6, 5), dpi=80)
        ax = fig.add_subplot(111, projection='3d')
        
        if trajectory_points_np is not None and len(trajectory_points_np) > 1:
            step = 1 if len(trajectory_points_np) < 500 else 5
            xs = trajectory_points_np[::step, 0]
            ys = trajectory_points_np[::step, 1]
            zs = trajectory_points_np[::step, 2]

            ax.plot(xs, ys, zs, color='blue', linewidth=1.5, label='Path')
            ax.scatter([current_actual_tcp[0]], [current_actual_tcp[1]], [current_actual_tcp[2]], 
                      color='red', s=100, label='TCP')

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
            ax.set_title('ServoJ Trajectory Playback')
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
# NEW: IK PRE-COMPUTATION FUNCTION
# ==================================================================================
def precompute_ik_solutions(trajectory_data, output_filename):
    """
    NEW: Pre-compute all IK solutions at 0.0025s (400Hz) rate
    Stores joint solutions in JSON for fast playback
    """
    print(f"\n🔄 Pre-computing IK solutions at {1/IK_COMPUTE_RATE:.0f}Hz...")
    
    ik_solutions = []
    failures = 0
    last_valid_joints = None
    
    # Interpolate trajectory to 0.0025s intervals
    total_time = trajectory_data[-1]['timestamp'] - trajectory_data[0]['timestamp']
    num_points = int(total_time / IK_COMPUTE_RATE) + 1
    
    print(f"   Total trajectory time: {total_time:.2f}s → {num_points} IK points")
    
    for i in range(num_points):
        if not force_control_active and i % 100 == 0:
            print(f"   Progress: {(i/num_points)*100:.1f}%")
        
        # Interpolate TCP position at this time
        target_time = trajectory_data[0]['timestamp'] + i * IK_COMPUTE_RATE
        
        # Find surrounding points
        idx = bisect.bisect_left([p['timestamp'] for p in trajectory_data], target_time)
        idx = max(0, min(idx, len(trajectory_data)-1))
        
        if idx > 0 and idx < len(trajectory_data):
            t0 = trajectory_data[idx-1]['timestamp']
            t1 = trajectory_data[idx]['timestamp']
            ratio = (target_time - t0) / (t1 - t0 + 1e-9)
            
            p0 = np.array(trajectory_data[idx-1]['tcp'][:3])
            p1 = np.array(trajectory_data[idx]['tcp'][:3])
            target_tcp = p0 + ratio * (p1 - p0)
            
            # Use orientation from nearest point
            target_rpy = trajectory_data[idx]['tcp'][3:]
        else:
            target_tcp = trajectory_data[idx]['tcp'][:3]
            target_rpy = trajectory_data[idx]['tcp'][3:]
        
        full_target = list(target_tcp) + list(target_rpy)
        
        # Solve IK
        err, joint_pos = robot.GetInverseKin(0, full_target, -1)
        
        if err == 0:
            joints = joint_pos[:6]
            ik_solutions.append({
                'time_offset': i * IK_COMPUTE_RATE,
                'tcp': full_target,
                'joints': joints
            })
            last_valid_joints = joints
        else:
            failures += 1
            if last_valid_joints:
                ik_solutions.append({
                    'time_offset': i * IK_COMPUTE_RATE,
                    'tcp': full_target,
                    'joints': last_valid_joints  # Use last valid solution
                })
    
    print(f"✅ IK Computation Complete: {len(ik_solutions)} points (failures: {failures})")
    
    # Save to file
    ik_file = os.path.join(IK_SOLUTIONS_DIR, output_filename.replace('.json', '_ik.json'))
    with open(ik_file, 'w') as f:
        json.dump(ik_solutions, f, indent=2)
    
    print(f"💾 Saved IK solutions: {ik_file}")
    return ik_solutions

# ==================================================================================
# PASSIVE THERAPY CLASS
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

class PassiveTherapy:
    def __init__(self, robot_instance):
        self.robot = robot_instance
        self.is_recording = False
        self.current_recording_name = None
        self.playback_active = False

    def start_recording(self, name):
        if self.is_recording:
            return False, "Already recording"
        if not name:
            return False, "Name is required"
        try:
            success = custom_drag_teach_mode(enable=True)
            if not success:
                return False, "Failed to enable drag mode"
            ret1 = self.robot.SetTPDParam(name, 4, 0)
            if ret1 != 0:
                custom_drag_teach_mode(enable=False)
                return False, f"SetTPDParam failed (code: {ret1})"
            ret2 = self.robot.SetTPDStart(name, 4, 0)
            if ret2 != 0:
                custom_drag_teach_mode(enable=False)
                return False, f"SetTPDStart failed (code: {ret2})"
            self.is_recording = True
            self.current_recording_name = name
            return True, f"Recording started: {name}"
        except Exception as e:
            custom_drag_teach_mode(enable=False)
            return False, str(e)

    def stop_recording(self):
        if not self.is_recording:
            return False, "Not recording"
        try:
            ret = self.robot.SetWebTPDStop()
            custom_drag_teach_mode(enable=False)
            name = self.current_recording_name
            self.is_recording = False
            self.current_recording_name = None
            if ret != 0:
                return False, f"SetWebTPDStop failed (code: {ret})"
            return True, name
        except Exception as e:
            return False, str(e)

    def save_trajectory(self, trajectory_data):
        recordings_dir = RECORDINGS_DIR
        os.makedirs(recordings_dir, exist_ok=True)
        filename = f"traj_{int(time.time())}.json"
        path = os.path.join(recordings_dir, filename)
        with open(path, 'w') as f:
            json.dump(trajectory_data, f, indent=2)
        return filename

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
    therapy = PassiveTherapy(robot)
    filename = therapy.save_trajectory(current_recording)
    current_recording = []
    return jsonify({'status': 'ok', 'filename': filename})

@app.route('/list_files', methods=['GET'])
def list_files():
    files = sorted([f for f in os.listdir(RECORDINGS_DIR) if f.endswith('.json')], reverse=True)
    return jsonify({'files': files})

@app.route('/load_traj', methods=['POST'])
def load_traj():
    global loaded_trajectory, loaded_ik_solutions, trajectory_points_np, cum_arc_lengths, total_arc_length
    
    try:
        name = request.json['filename']
        path = os.path.join(RECORDINGS_DIR, name)
        
        with open(path, 'r') as f: 
            loaded_trajectory = json.load(f)
        
        if len(loaded_trajectory) < 2: 
            return jsonify({'status': 'error', 'msg': 'Path too short'})
        
        # Extract points for visualization
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
        
        # NEW: Pre-compute IK solutions
        print(f"\n▶️ Loading trajectory: {name}")
        loaded_ik_solutions = precompute_ik_solutions(loaded_trajectory, name)
        
        print(f"✅ Trajectory loaded: {len(loaded_trajectory)} TCP points, {len(loaded_ik_solutions)} IK points")
        return jsonify({
            'status': 'ok', 
            'length': total_arc_length,
            'tcp_count': len(loaded_trajectory),
            'ik_count': len(loaded_ik_solutions)
        })
        
    except Exception as e:
        print(f"❌ Load Error: {e}")
        import traceback
        traceback.print_exc()
        return jsonify({'status': 'error', 'msg': str(e)}), 500

@app.route('/zero_sensor', methods=['POST'])
def zero_sensor():
    global baseline_forces
    readings = []
    for _ in range(10):
        ret = robot.FT_GetForceTorqueRCS()
        if ret[0] == 0: 
            readings.append(np.array(ret[1]))
        time.sleep(0.05)
    if readings:
        baseline_forces = np.mean(readings, axis=0).tolist()
        print("✅ Sensor Zeroed")
    return jsonify({'status': 'ok'})

playback_repetition_count = 0
current_joint_angles = [0.0] * 6

@app.route('/start_therapy', methods=['POST'])
def start_therapy_route():
    global force_control_active
    if not loaded_ik_solutions: 
        return jsonify({'error': 'No IK solutions loaded'}), 400
    if force_control_active:
        return jsonify({'error': 'Playback already active'}), 400
    
    repetitions = request.json.get('repetitions', 1)
    force_control_active = True
    threading.Thread(target=servo_playback_loop, args=(repetitions,), daemon=True).start()
    return jsonify({'status': 'ok'})

@app.route('/stop_therapy', methods=['POST'])
def stop_therapy_route():
    global force_control_active
    force_control_active = False
    return jsonify({'status': 'ok'})

@app.route('/telemetry')
def telemetry():
    return jsonify({
        'progress': (len(loaded_ik_solutions) / len(loaded_ik_solutions) * 100) if loaded_ik_solutions else 0,
        'rep': playback_repetition_count,
        'joints': current_joint_angles
    })

# ==================================================================================
# JOINT INTERPOLATION + SMOOTHING
# ==================================================================================
def interpolate_joints(waypoints, t):
    if not waypoints:
        return None
    if t <= 0:
        return list(waypoints[0]['joints'])
    last_time = waypoints[-1]['time_offset']
    if t >= last_time:
        return list(waypoints[-1]['joints'])

    idx = int(t / IK_COMPUTE_RATE)
    if idx >= len(waypoints) - 1:
        return list(waypoints[-1]['joints'])

    t0 = waypoints[idx]['time_offset']
    t1 = waypoints[idx + 1]['time_offset']
    if t1 <= t0:
        return list(waypoints[idx]['joints'])

    frac = (t - t0) / (t1 - t0)
    j0 = np.array(waypoints[idx]['joints'], dtype=float)
    j1 = np.array(waypoints[idx + 1]['joints'], dtype=float)
    return (j0 + frac * (j1 - j0)).tolist()

def smooth_joint_command(prev_joints, target_joints):
    if prev_joints is None or target_joints is None:
        return list(target_joints) if target_joints is not None else None

    max_step = MAX_JOINT_VELOCITY * SERVO_UPDATE_RATE
    smoothed = []
    for pj, tj in zip(prev_joints, target_joints):
        step = tj - pj
        if step > max_step:
            step = max_step
        elif step < -max_step:
            step = -max_step
        limited = pj + step
        smoothed_val = (1.0 - JOINT_FILTER_ALPHA) * pj + JOINT_FILTER_ALPHA * limited
        smoothed.append(smoothed_val)
    return smoothed

def get_actual_joint_degrees():
    try:
        err, joints = robot.GetActualJointPosDegree(flag=0)
        if err == 0:
            return list(joints[:6])
    except Exception:
        pass
    return None

# ==================================================================================
# NEW: SERVO-BASED PLAYBACK USING PRE-COMPUTED IK
# ==================================================================================
def servo_playback_loop(repetitions=1):
    """
    NEW: Use pre-computed IK solutions with ServoJ for smooth playback
    Follows forward/reverse based on repetition count
    """
    global loaded_ik_solutions, playback_repetition_count, force_control_active
    global current_actual_tcp, current_joint_angles
    
    if not loaded_ik_solutions or len(loaded_ik_solutions) < 2:
        print("❌ No IK solutions available!")
        force_control_active = False
        return
    
    print(f"\n▶️ Starting ServoJ Playback ({repetitions} reps)...")
    robot.DragTeachSwitch(0)
    time.sleep(0.2)
    
    try:
        for rep in range(repetitions):
            playback_repetition_count = rep + 1
            
            # Odd reps (1,3,5) = REVERSE, Even reps (2,4,6) = FORWARD
            is_forward = (playback_repetition_count % 2 == 0)
            direction_str = "FORWARD" if is_forward else "REVERSE"
            
            print(f"\n📍 Repetition {rep+1}/{repetitions} | Direction: {direction_str}")
            
            # Build waypoint list
            if is_forward:
                waypoints = loaded_ik_solutions[::1]  # Forward
            else:
                waypoints = loaded_ik_solutions[::-1]  # Reverse
            
            # Start servo mode
            robot.ServoMoveStart()
            time.sleep(0.2)
            
            # Execute trajectory via ServoJ (time-based interpolation + smoothing)
            prev_cmd_joints = get_actual_joint_degrees()
            total_time = waypoints[-1]['time_offset'] if waypoints else 0.0
            start_time = time.perf_counter()
            next_tick = start_time

            while True:
                if not force_control_active:
                    print("🛑 Stopped by user")
                    robot.ServoMoveEnd()
                    return

                now = time.perf_counter()
                elapsed = now - start_time
                if elapsed > total_time:
                    break

                t = elapsed if is_forward else (total_time - elapsed)
                target_joints = interpolate_joints(loaded_ik_solutions, t)
                cmd_joints = smooth_joint_command(prev_cmd_joints, target_joints)
                prev_cmd_joints = cmd_joints
                current_joint_angles = cmd_joints  # For telemetry

                # Send servo command
                robot.ServoJ(cmd_joints, [0]*6, 0, 0, SERVO_UPDATE_RATE, 0, 0)

                # Update actual TCP
                err, actual_pose = robot.GetActualTCPPose(flag=1)
                if err == 0:
                    current_actual_tcp = actual_pose[:3]

                # Progress update
                if int(elapsed / IK_COMPUTE_RATE) % 50 == 0:
                    progress = (elapsed / total_time) * 100 if total_time > 0 else 100.0
                    print(f"  ✓ {direction_str}: {progress:5.1f}%")

                next_tick += SERVO_UPDATE_RATE
                sleep_time = next_tick - time.perf_counter()
                if sleep_time > 0:
                    time.sleep(sleep_time)
            
            # End servo mode for this rep
            robot.ServoMoveEnd()
            time.sleep(0.3)
            
            print(f"✅ Repetition {rep+1} complete")
            time.sleep(0.5)
        
        playback_repetition_count = repetitions
        print(f"\n✅ All {repetitions} repetitions completed successfully!")
        
    except Exception as e:
        print(f"❌ Servo playback error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        try:
            robot.ServoMoveEnd()
        except:
            pass
        force_control_active = False

# ==================================================================================
# BACKGROUND RECORDER
# ==================================================================================
def background_recorder():
    """Records trajectory points when enabled"""
    global current_recording, current_actual_tcp
    while running:
        err, pose = robot.GetActualTCPPose(flag=1)
        if err == 0:
            current_actual_tcp = pose[:3]
            if is_recording:
                current_recording.append({
                    'timestamp': time.time(),
                    'tcp': pose
                })
        time.sleep(0.02)

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
    print("📁 IK Solutions saved in: ik_solutions/")
    app.run(host='0.0.0.0', port=5000, threaded=True)