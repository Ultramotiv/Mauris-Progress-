# Friday 16 th jan code 
# To DO: Make it smoother 
# this code follows the trajectory in forward and in reverse But based on odd/Even so change this as well ! 

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
matplotlib.use('Agg') # Non-interactive backend prevents GUI errors
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from flask import Flask, request, jsonify, render_template_string, Response
from flask_cors import CORS

# === Robot Setup ===
# UPDATE THIS PATH if your SDK is in a different folder
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
force_control_active = False
drag_mode_enabled = False

# Trajectory Data
loaded_trajectory = None       # List of {tcp: [], timestamp: t}
trajectory_points_np = None    # Numpy array of (N, 3) for plotting
cum_arc_lengths = None         # Array of distances along path
total_arc_length = 0.0
current_arc_position = 0.0     # Current distance along path

# Sensor & Control Data
baseline_forces = [0.0] * 6
current_ft_raw = [0.0] * 6     # For display
current_tangential_force = 0.0 # The force driving motion
current_actual_tcp = [0.0, 0.0, 0.0]  # Actual TCP position from robot

# Threading Locks
state_lock = threading.Lock()
plot_lock = threading.Lock()   # Prevents reading data while writing

# === Control Parameters ===
SERVO_UPDATE_RATE = 0.008      # 8ms loop
# 1N = 6mm/s -> 0.006 m/s per Newton
FORCE_TO_VELOCITY_SCALE = 0.006 
MIN_FORCE_THRESHOLD = 0.3      # Minimum force to start moving (N) - reduced for responsiveness
FILTER_ALPHA = 0.25            # Force smoothing (0.0 to 1.0) - increased responsiveness
VELOCITY_SMOOTHING = 0.8       # Velocity smoothing

# Set a safe MoveL speed to avoid joint speed limit errors
PASSIVE_PLAYBACK_SPEED = 26.0  # deg/sec

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
# FRONTEND HTML
# ==================================================================================
FRONTEND_HTML = '''
<!DOCTYPE html>
<html>
<head>
    <title>Rehab Robot - Active Therapy</title>
    <style>
        body { font-family: 'Segoe UI', sans-serif; max-width: 950px; margin: 0 auto; padding: 20px; background: #eef2f5; }
        h1 { text-align: center; color: #2c3e50; }
        .grid { display: grid; grid-template-columns: 1fr 1fr; gap: 20px; }
        .section { background: white; padding: 20px; border-radius: 12px; box-shadow: 0 4px 6px rgba(0,0,0,0.1); }
        .full-width { grid-column: span 2; }
        
        button { 
            width: 100%; padding: 12px; margin: 5px 0; border: none; border-radius: 6px; 
            font-weight: bold, cursor: pointer; transition: 0.2s; font-size: 14px;
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
        
        /* Visualization Container */
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
    <h1>🏥 Trajectory Playback Control</h1>
    
    <div class="grid">
        <div class="section">
            <h2>1. Setup</h2>
            <button class="btn-blue" onclick="toggleDrag()">Toggle Drag Mode</button>
            <button class="btn-red" onclick="startRecording()">Start Recording Path</button>
            <button class="btn-red" onclick="stopRecording()">Stop & Save Path</button>
            <hr>
            <h2>2. Calibration</h2>
            <select id="fileList" style="width:100%; padding:10px; margin-bottom:10px; border-radius:4px; border:1px solid #ccc;"></select>
            <button class="btn-orange" onclick="loadTrajectory()">Load Selected Path</button>
            <button class="btn-orange" onclick="calibrateSensor()">Zero Force Sensor</button>
        </div>

        <div class="section">
            <h2>3. Playback Control</h2>
            <label for="reps">Repetitions:</label> <input id="reps" type="number" min="1" value="1" style="width:60px; margin-bottom:10px;">
            <button class="btn-green" onclick="startTherapy()">Start Playback</button>
            <button class="btn-red" onclick="stopTherapy()">Stop Playback</button>
            
            <div class="status-box">
                <div>Status: <span id="sysStatus">Idle</span></div>
                <div>Progress: <span id="progress" class="highlight">0.0</span> %</div>
                <div>Tangential Force: <span id="tForce" class="highlight">0.00</span> N</div>
                <div>Target Vel: <span id="tVel">0.00</span> mm/s</div>
                <hr style="border-color: #555;">
                <div style="font-size: 0.9em; color: #aaa;">Raw Sensor Data:</div>
                <div>Fx: <span id="fx">0</span> | Fy: <span id="fy">0</span> | Fz: <span id="fz">0</span></div>
            </div>
        </div>

        <div class="section full-width">
            <h2>Trajectory Visualization</h2>
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
        // API Helper
        async function api(url, method='POST', body=null) {
            try {
                const opts = { method, headers: {'Content-Type': 'application/json'} };
                if(body) opts.body = JSON.stringify(body);
                const res = await fetch(url, opts);
                return await res.json();
            } catch(e) { console.error(e); alert("Connection Error"); }
        }

        // Functions
        async function toggleDrag() { 
            const res = await api('/drag_mode'); 
            updateStatus(res.enabled ? "Drag Mode ON" : "Drag Mode OFF");
        }
        async function startRecording() { await api('/start_recording'); updateStatus("Recording..."); }
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
            if(res.status === 'ok') updateStatus(`Loaded: ${res.length.toFixed(2)}m long`);
            else alert("Error loading file");
        }

        async function calibrateSensor() {
            updateStatus("Calibrating...");
            await api('/zero_sensor');
            updateStatus("Sensor Zeroed");
        }

        async function startTherapy() { 
            const reps = document.getElementById('reps').value;
            const res = await api('/start_therapy', 'POST', {repetitions: parseInt(reps)});
            if(res.status === 'ok') updateStatus("Playback Active");
            else alert(res.error || "Failed to start");
        }
        async function stopTherapy() { 
            await api('/stop_therapy'); 
            updateStatus("Playback Stopped");
        }

        async function playTrajectory() {
            const res = await api('/play_trajectory', 'POST', {repetitions: reps});
            if(res.status === "ok") updateStatus("Playing trajectory...");
            else alert(res.msg || "Failed to play trajectory");
        }

        function updateStatus(msg) { document.getElementById('sysStatus').innerText = msg; }

        // Telemetry Loop (5Hz)
        setInterval(async () => {
            try {
                const res = await fetch('/telemetry');
                const data = await res.json();
                
                if(data.active) document.getElementById('sysStatus').innerText = "Therapy Active";
                
                document.getElementById('progress').innerText = data.progress.toFixed(1);
                document.getElementById('tForce').innerText = data.tangential_force.toFixed(2);
                document.getElementById('tVel').innerText = (data.velocity * 1000).toFixed(1);
                
                document.getElementById('fx').innerText = data.ft[0].toFixed(1);
                document.getElementById('fy').innerText = data.ft[1].toFixed(1);
                document.getElementById('fz').innerText = data.ft[2].toFixed(1);
            } catch(e) {}
        }, 200);

        refreshFiles();
    </script>
</body>
</html>
'''

# ==================================================================================
# MATPLOTLIB VISUALIZATION GENERATOR (With Auto-Scale Fix)
# ==================================================================================
def generate_plot_frame():
    """Generates a static image of the 3D plot with auto-scaling."""
    with plot_lock:
        fig = plt.figure(figsize=(6, 5), dpi=80)
        ax = fig.add_subplot(111, projection='3d')
        
        # Check if we have data to plot
        if trajectory_points_np is not None and len(trajectory_points_np) > 1:
            # 1. Downsample for speed (plot every 5th point if array is large)
            step = 1 if len(trajectory_points_np) < 500 else 5
            xs = trajectory_points_np[::step, 0]
            ys = trajectory_points_np[::step, 1]
            zs = trajectory_points_np[::step, 2]

            # 2. Plot Trajectory Path
            ax.plot(xs, ys, zs, color='blue', linewidth=1.5, label='Path')
            
            # 3. Plot Robot Position (Red Dot) - Using ACTUAL TCP from robot
            if loaded_trajectory:
                # Use the actual TCP position from the robot
                ax.scatter([current_actual_tcp[0]], [current_actual_tcp[1]], [current_actual_tcp[2]], 
                          color='red', s=100, label='TCP', edgecolors='white')

            # 4. AUTO-SCALING LOGIC
            # This ensures the plot works for both Meters (0.5) and Millimeters (500.0)
            x_min, x_max = xs.min(), xs.max()
            y_min, y_max = ys.min(), ys.max()
            z_min, z_max = zs.min(), zs.max()

            # Find the largest dimension to create a cubic view
            max_range = np.array([x_max-x_min, y_max-y_min, z_max-z_min]).max() / 2.0
            
            # Default buffer if path is a single dot
            if max_range == 0: max_range = 1.0 
            
            mid_x = (x_max+x_min) * 0.5
            mid_y = (y_max+y_min) * 0.5
            mid_z = (z_max+z_min) * 0.5

            # Apply limits with 10% padding
            ax.set_xlim(mid_x - max_range*1.1, mid_x + max_range*1.1)
            ax.set_ylim(mid_y - max_range*1.1, mid_y + max_range*1.1)
            ax.set_zlim(mid_z - max_range*1.1, mid_z + max_range*1.1)

            ax.set_xlabel('X')
            ax.set_ylabel('Y')
            ax.set_zlabel('Z')
            ax.set_title(f'Force: {current_tangential_force:.2f} N')
            ax.legend()
        else:
            # Placeholder text if no data
            ax.text2D(0.5, 0.5, "No Trajectory Loaded", transform=ax.transAxes, 
                     ha='center', va='center', fontsize=12, color='red')
            ax.set_axis_off()

        # Save to memory buffer
        buf = io.BytesIO()
        plt.savefig(buf, format='jpg', bbox_inches='tight')
        plt.close(fig) # Important: Close figure to free memory
        buf.seek(0)
        return buf.read()

def video_stream_gen():
    """Generator for the MJPEG stream"""
    while True:
        frame = generate_plot_frame();
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n');
        time.sleep(0.1) # Limit FPS to 10 to save CPU

# ==================================================================================
# MATH HELPERS
# ==================================================================================
def get_tcp_from_arc_length(s):
    """Returns [x,y,z] at distance s along path"""
    if cum_arc_lengths is None or loaded_trajectory is None:
        return [0,0,0]
    
    idx = bisect.bisect_left(cum_arc_lengths, s)
    if idx == 0: return loaded_trajectory[0]['tcp'][:3]
    if idx >= len(cum_arc_lengths): return loaded_trajectory[-1]['tcp'][:3]
    
    # Linear Interpolation
    s_start = cum_arc_lengths[idx-1]
    s_end = cum_arc_lengths[idx]
    
    p0 = np.array(loaded_trajectory[idx-1]['tcp'][:3])
    p1 = np.array(loaded_trajectory[idx]['tcp'][:3])
    
    ratio = (s - s_start) / (s_end - s_start + 1e-9)
    p_interp = p0 + ratio * (p1 - p0)
    return p_interp.tolist()

def get_path_direction(s):
    """Returns normalized direction vector at distance s"""
    if cum_arc_lengths is None: return np.array([0,0,0])
    
    idx = bisect.bisect_left(cum_arc_lengths, s)
    idx = max(1, min(idx, len(loaded_trajectory)-1))
    
    p_prev = np.array(loaded_trajectory[idx-1]['tcp'][:3])
    p_next = np.array(loaded_trajectory[idx]['tcp'][:3])
    
    vec = p_next - p_prev
    norm = np.linalg.norm(vec)
    if norm < 1e-6: return np.array([0,0,0])
    return vec / norm

# ============================================================================
# PASSIVE THERAPY CLASS (EXACT COPY)
# ============================================================================
# === SHARED GLOBAL FUNCTION (used by BOTH classes) ===
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

    def list_recordings(self):
        recordings_dir = 'therapy_recordings'
        if not os.path.exists(recordings_dir):
            return []
        files = [f for f in os.listdir(recordings_dir) if f.endswith('.json')]
        return sorted(files)

    def save_trajectory(self, trajectory_data):
        """
        Save trajectory data as a JSON file in 'therapy_recordings' folder.
        Each entry should have: timestamp, tcp (coordinates)
        """
        recordings_dir = 'therapy_recordings'
        os.makedirs(recordings_dir, exist_ok=True)
        filename = f"traj_{int(time.time())}.json"
        path = os.path.join(recordings_dir, filename)
        with open(path, 'w') as f:
            json.dump(trajectory_data, f, indent=2)
        return filename

    def start_playback(self, filename, repetitions=1):
        global playback_repetition_count
        if self.playback_active:
            return False, "Playback already active"
        def playback_task():
            global playback_repetition_count
            try:
                self.playback_active = True
                playback_repetition_count = 0
                ret = self.robot.LoadTPD(filename)
                if ret != 0:
                    print(f"LoadTPD failed: {ret}")
                    return
                for rep in range(repetitions):
                    if not self.playback_active:
                        break
                    playback_repetition_count = rep + 1
                    err, start_pose = self.robot.GetTPDStartPose(filename)
                    if err != 0:
                        print(f"GetTPDStartPose failed: {err}")
                        break
                    self.robot.MoveL(start_pose, 0, 0)
                    time.sleep(0.5)
                    self.robot.MoveTPD(filename, 1, 100)
                    while self.playback_active:
                        err, done = self.robot.GetRobotMotionDone()
                        if err == 0 and done == 1:
                            break
                        time.sleep(0.1)
                    if not self.playback_active:
                        break
                    time.sleep(0.5)
                playback_repetition_count = repetitions
            except Exception as e:
                print(f"Playback error: {e}")
            finally:
                self.playback_active = False
        threading.Thread(target=playback_task, daemon=True).start()
        return True, "Playback started"

    def stop_playback(self):
        self.playback_active = False
        try:
            self.robot.StopMotion()
        except:
            pass
        return True, "Playback stopped"

    def is_playback_active(self):
        return self.playback_active
# ==================================================================================
# API ROUTES
# ==================================================================================
@app.route('/')
def index(): return render_template_string(FRONTEND_HTML)

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
    except Exception as e: return jsonify({'status': 'error', 'msg': str(e)})

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
    # Turn off drag mode after recording
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
    global loaded_trajectory, cum_arc_lengths, total_arc_length
    global current_arc_position, trajectory_points_np
    
    try:
        name = request.json['filename']
        path = os.path.join(RECORDINGS_DIR, name)
        
        with open(path, 'r') as f: 
            loaded_trajectory = json.load(f)
        
        if len(loaded_trajectory) < 2: 
            return jsonify({'status': 'error', 'msg': 'Path too short'})
        
        # Parse points for math and plotting
        points = [np.array(p['tcp'][:3]) for p in loaded_trajectory]
        
        with plot_lock:
            trajectory_points_np = np.array(points)
        
        # Calculate Arc Lengths
        dists = [0.0]
        for i in range(1, len(points)):
            d = np.linalg.norm(points[i] - points[i-1])
            dists.append(dists[-1] + d)
        
        cum_arc_lengths = dists
        total_arc_length = dists[-1]
        current_arc_position = 0.0
        
        print(f"✅ Loaded {len(points)} points. Length: {total_arc_length:.3f}")
        return jsonify({'status': 'ok', 'length': total_arc_length})
        
    except Exception as e:
        print(f"❌ Load Error: {e}")
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

@app.route('/start_therapy', methods=['POST'])
def start_therapy_route():
    global force_control_active
    if not loaded_trajectory: return jsonify({'error': 'No trajectory loaded'}), 400
    if force_control_active:
        return jsonify({'error': 'Playback already active'}), 400
    
    repetitions = request.json.get('repetitions', 1)
    force_control_active = True
    threading.Thread(target=trajectory_playback_loop, args=(repetitions,), daemon=True).start()
    return jsonify({'status': 'ok'})

@app.route('/stop_therapy', methods=['POST'])
def stop_therapy_route():
    global force_control_active
    force_control_active = False
    return jsonify({'status': 'ok'})

@app.route('/telemetry')
def telemetry():
    return jsonify({
        'active': force_control_active,
        'progress': (current_arc_position/total_arc_length*100) if total_arc_length else 0,
        'tangential_force': current_tangential_force,
        'velocity': current_tangential_force * FORCE_TO_VELOCITY_SCALE,
        'ft': current_ft_raw[:3]
    })

# ==================================================================================
# PASSIVE-LIKE TRAJECTORY PLAYBACK (using MoveL for smooth execution)
# ==================================================================================
playback_repetition_count = 0  # For GUI feedback if needed

def trajectory_playback_loop(repetitions=1):
    """Play trajectory forward/reverse based on repetition count (like app.py)"""
    global loaded_trajectory, playback_repetition_count, force_control_active, current_actual_tcp
    
    if not loaded_trajectory or len(loaded_trajectory) < 2:
        print("❌ No trajectory loaded!")
        force_control_active = False
        return
    
    print(f"▶️ Playing back trajectory at {PASSIVE_PLAYBACK_SPEED} mm/s for {repetitions} repetitions...")
    robot.DragTeachSwitch(0)
    time.sleep(0.2)
    
    try:
        for rep in range(repetitions):
            playback_repetition_count = rep + 1
            
            # Odd repetition count (1,3,5...) = Reverse, Even repetition count (2,4,6...) = Forward
            is_forward = (playback_repetition_count % 2 == 0)
            direction_str = "FORWARD" if is_forward else "REVERSE"
            
            print(f"\n📍 Repetition {rep+1}/{repetitions} | Direction: {direction_str}")
            last_print = time.time()
            
            # Get start position based on direction
            if is_forward:
                start_pose = loaded_trajectory[0]['tcp']
            else:
                start_pose = loaded_trajectory[-1]['tcp']
            
            # Move to starting position
            print(f"Moving to start position: {[f'{x:.1f}' for x in start_pose[:3]]}")
            robot.MoveL(start_pose, 0, 0, vel=PASSIVE_PLAYBACK_SPEED)
            time.sleep(0.5)
            
            # Traverse trajectory
            if is_forward:
                for i in range(1, len(loaded_trajectory)):
                    if not force_control_active:
                        print("\n🛑 Playback stopped by user")
                        return
                    
                    target_tcp = loaded_trajectory[i]['tcp']
                    robot.MoveL(target_tcp, 0, 0, vel=PASSIVE_PLAYBACK_SPEED)
                    
                    err, actual_pose = robot.GetActualTCPPose(flag=1)
                    if err == 0:
                        current_actual_tcp = actual_pose[:3]
                    
                    if time.time() - last_print > 1.0:
                        progress = ((i + 1) / len(loaded_trajectory)) * 100
                        print(f"  ✓ FORWARD: {progress:5.1f}% | Point {i+1}/{len(loaded_trajectory)}")
                        last_print = time.time()
            else:
                for i in range(len(loaded_trajectory)-2, -1, -1):
                    if not force_control_active:
                        print("\n🛑 Playback stopped by user")
                        return
                    
                    target_tcp = loaded_trajectory[i]['tcp']
                    robot.MoveL(target_tcp, 0, 0, vel=PASSIVE_PLAYBACK_SPEED)
                    
                    err, actual_pose = robot.GetActualTCPPose(flag=1)
                    if err == 0:
                        current_actual_tcp = actual_pose[:3]
                    
                    if time.time() - last_print > 1.0:
                        idx = len(loaded_trajectory) - 1 - i
                        progress = (idx / len(loaded_trajectory)) * 100
                        print(f"  ✓ REVERSE: {progress:5.1f}% | Point {idx}/{len(loaded_trajectory)}")
                        last_print = time.time()
            
            print(f"✅ Repetition {rep+1} complete")
            time.sleep(0.5)
        
        playback_repetition_count = repetitions
        print(f"\n✅ All {repetitions} repetitions completed successfully!")
        
    except Exception as e:
        print(f"❌ Playback error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("⏹️ Trajectory playback finished.")
        force_control_active = False

# ==================================================================================
# CONTROL LOOPS
# ==================================================================================
def background_recorder():
    """Records points when enabled and continuously updates actual TCP position"""
    global current_recording, current_actual_tcp
    while running:
        # Continuously update actual TCP position for visualization
        err, pose = robot.GetActualTCPPose(flag=1)
        if err == 0:
            current_actual_tcp = pose[:3]  # Store [x, y, z]
            
            if is_recording:
                current_recording.append({
                    'timestamp': time.time(),
                    'tcp': pose  # [x, y, z, rx, ry, rz]
                })
        
        time.sleep(0.02)

def force_control_loop():
    """Real-time Force Control Loop (8ms)"""
    global current_arc_position, current_tangential_force, current_ft_raw, force_control_active
    global loaded_trajectory, cum_arc_lengths, total_arc_length
    
    # Safety check
    if not loaded_trajectory or len(loaded_trajectory) < 2:
        print("❌ No trajectory loaded! Cannot start therapy.")
        force_control_active = False
        return
    
    # Init Servo Mode
    robot.DragTeachSwitch(0) 
    time.sleep(0.2)
    robot.ServoMoveStart()
    time.sleep(0.5)
    
    s = current_arc_position 
    filtered_force = 0.0
    debug_counter = 0
    
    print("🚀 Therapy Loop Started")
    print(f"📊 Path length: {total_arc_length:.3f}m, Points: {len(loaded_trajectory)}")
    
    try:
        while force_control_active and running:
            t_start = time.time()
            
            # 1. Read Force
            ret = robot.FT_GetForceTorqueRCS()
            if ret[0] != 0: 
                print(f"⚠️  FT sensor read error: {ret[0]}")
                continue
            
            raw_ft = np.array(ret[1])
            net_ft = raw_ft - np.array(baseline_forces)
            
            # Apply Deadband
            net_ft = np.where(np.abs(net_ft) < MIN_FORCE_THRESHOLD, 0.0, net_ft)
            current_ft_raw = net_ft.tolist()
            
            # 2. Get Path Direction & Calculate Tangential Force
            u_dir = get_path_direction(s)
            
            # Debug: Check if direction is valid
            if np.linalg.norm(u_dir) < 1e-6:
                if debug_counter % 100 == 0:
                    print(f"⚠️  Warning: Path direction is zero at s={s:.3f}")
                u_dir = np.array([1.0, 0.0, 0.0])  # Default direction
            
            f_tan = np.dot(net_ft[:3], u_dir)
            
            # Smoothing
            filtered_force = FILTER_ALPHA * f_tan + (1 - FILTER_ALPHA) * filtered_force
            current_tangential_force = filtered_force
            
            # 3. Calculate Velocity (1N = 6mm/s)
            velocity = filtered_force * FORCE_TO_VELOCITY_SCALE
            
            # 4. Update Position
            ds = velocity * SERVO_UPDATE_RATE
            s += ds
            s = max(0.0, min(s, total_arc_length))
            current_arc_position = s
            
            # 5. Inverse Kinematics for Target
            target_tcp = get_tcp_from_arc_length(s)
            
            # Find orientation (simplify by using orientation of closest recorded point)
            idx = bisect.bisect_left(cum_arc_lengths, s)
            idx = min(idx, len(loaded_trajectory)-1)
            target_rpy = loaded_trajectory[idx]['tcp'][3:]
            
            full_target = target_tcp + target_rpy
            
            # Solve IK (seed = current joints)
            err, joint_pos = robot.GetInverseKin(0, full_target, -1)
            
            if err != 0:
                if debug_counter % 100 == 0:
                    print(f"⚠️  IK failed at s={s:.3f}, target={[f'{x:.2f}' for x in full_target[:3]]}")
                debug_counter += 1
                continue
            
            # Send Servo Command
            target_j = joint_pos[:6]
            robot.ServoJ(target_j, [0]*6, 0, 0, SERVO_UPDATE_RATE, 0, 0)
            
            # Debug output every 100 cycles (~0.8s)
            if debug_counter % 100 == 0:
                print(f"✓ Force: {filtered_force:+6.2f}N | Vel: {velocity*1000:+6.2f}mm/s | "
                      f"Progress: {(s/total_arc_length)*100:5.1f}% | Pos: {[f'{x:.1f}' for x in target_tcp[:3]]}")
            
            debug_counter += 1
            
            # Timing correction
            elapsed = time.time() - t_start
            if elapsed < SERVO_UPDATE_RATE:
                time.sleep(SERVO_UPDATE_RATE - elapsed)
                
    except Exception as e:
        print(f"❌ Control Error: {e}")
    finally:
        print("🛑 Therapy Loop Stopped")
        robot.ServoMoveEnd()
        force_control_active = False

# ==================================================================================
# MAIN
# ==================================================================================
if __name__ == '__main__':
    # Init Robot
    try:
        robot = Robot.RPC(ROBOT_IP)
        print(f"✅ Connected to Robot: {ROBOT_IP}")
    except:
        print("❌ ERROR: Could not connect to robot. Check IP.")
        sys.exit(1)

    # Start Recorder Thread
    rec_thread = threading.Thread(target=background_recorder, daemon=True)
    rec_thread.start()

    print("🌐 Starting Web Server on http://localhost:5000")
    app.run(host='0.0.0.0', port=5000, threaded=True)