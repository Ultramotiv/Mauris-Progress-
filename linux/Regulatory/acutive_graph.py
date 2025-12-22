# Shoulder,Hip :=  flexion and shoulder extention
# Shoulder,Hip := Abduction and Adduction
# COBOT — PURE Z-AXIS ADMITTANCE
# Velocity increases smoothly with applied force → feels PERFECT
# absolutely no vibrations | Push harder = move faster
# Only in Z mmovement | ****MAX VELOCITY: 60deg/s****

import sys
sys.path.append('/home/um/fairino-python-sdk-main/linux/fairino')
import Robot
import time
import signal
import numpy as np
import math
import threading
import webbrowser
from flask import Flask, request, jsonify

# ============================================================================
# GLOBAL VARIABLES & PERFECT TUNING
# ============================================================================
robot = None
running = True
fixed_tcp_ref = None
app = Flask(__name__)

# THESE ARE THE ONLY CHANGES YOU NEED — COPY THIS BLOCK
FORCE_TO_MOTION_SCALE = 6.0
M = [1.6, 1.6, 1.4, 1.8, 1.8, 1.8]           # Light mass → instant response
B = [2.5, 2.5, 2.5, 3.0, 3.0, 3.0]     # Damping per joint
IK_TO_SERVO_RATIO = 2                         # Smoother than 4
IK_UPDATE_RATE    = 0.0025
SERVO_UPDATE_RATE = 0.008
FORCE_THRESHOLD   = 0.8
FORCE_FILTER_ALPHA = 0.28
force_thresholds = [2.0, 2.0, 2.5, 1.0, 1.0, 1.0]
joint_velocity = [0.0] * 6
desired_joint_pos = [0.0] * 6
filtered_desired_joints = None
filtered_fz_world = 0.0
baseline_forces = [0.0] * 6

# NEW: Safe joint velocity limit
MAX_JOINT_VELOCITY = 60.0  # deg/s

# >>> USER-DEFINED Z-LIMITS (set by buttons) <<<
USER_MIN_Z = None      # Set by "Min" button
USER_MAX_Z = None      # Set by "Max" button
# <<< END USER LIMITS <<<

# >>> NEW: IMPEDANCE PARAMETERS FOR BUILT-IN DRAG TEACH <<<
IMPEDANCE_PARAMS = {
    'lamde_dain': [2.5, 2.0, 2.0, 2.0, 2.0, 2.0],
    'b_gain': [20.0, 10.0, 10.0, 5.0, 5.0, 1.0],
    'k_gain': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    'max_tcp_vel': 500,      # Maximum linear velocity limit (mm/s)
    'max_tcp_ori_vel': 90    # Maximum angular velocity limit (deg/s)
}
# <<< END IMPEDANCE PARAMS <<<

# Application state
current_mode = None  # 'passive' or 'active'
drag_mode_enabled = False
active_movements = {
    'Shoulder_flexion': False,
    'Shoulder_extention': False,
    'Hip_Flexion': False,
    'Hip_Extention': False
}

# For real-time telemetry
current_fz = 0.0
current_tcp_z = 0.0  # <-- ADD THIS LINE
current_joints = [0.0] * 6

# For telemetry history
telemetry_buffer = []  # List of {time, tcp_z, fz}
last_telemetry_time = time.time()
telemetry_lock = threading.Lock()
MAX_BUFFER_SIZE = 5000  # Keep last N samples

# ============================================================================
# HELPERS (unchanged)
# ============================================================================
def euler_to_rotation_matrix(rx, ry, rz):
    rx = math.radians(rx); ry = math.radians(ry); rz = math.radians(rz)
    c, s = math.cos, math.sin
    Rx = np.array([[1,0,0], [0,c(rx),-s(rx)], [0,s(rx),c(rx)]])
    Ry = np.array([[c(ry),0,s(ry)], [0,1,0], [-s(ry),0,c(ry)]])
    Rz = np.array([[c(rz),-s(rz),0], [s(rz),c(rz),0], [0,0,1]])
    return Rz @ Ry @ Rx

def transform_force_to_world(ft_forces, orientation):
    R = euler_to_rotation_matrix(*orientation)
    tcp_force = np.array([ft_forces[0], ft_forces[1], -ft_forces[2]])
    world_force = R @ tcp_force
    return world_force[2]

def ema(new, old, alpha):
    return alpha * new + (1 - alpha) * old

# ============================================================================
# NEW FUNCTION: CUSTOM IMPEDANCE DRAG TEACH MODE
# ============================================================================
def custom_drag_teach_mode(enable=True):
    """
    Custom impedance control using the single IMPEDANCE_PARAMS set above
    
    Args:
        enable (bool): True to enable, False to disable
    """
    global robot, drag_mode_enabled
    
    if enable:
        print("Enabling custom impedance control...")
        print(f"Using parameters: {IMPEDANCE_PARAMS}")
        
        # First enable drag teach mode (this is required)
        robot.DragTeachSwitch(1)
        time.sleep(0.5)
        
        # Then apply your custom impedance parameters
        rtn = robot.ForceAndJointImpedanceStartStop(
            status=1,  # Turn on
            impedanceFlag=1,  # Enable impedance control
            lamdeDain=IMPEDANCE_PARAMS['lamde_dain'],
            KGain=IMPEDANCE_PARAMS['k_gain'], 
            BGain=IMPEDANCE_PARAMS['b_gain'],
            dragMaxTcpVel=IMPEDANCE_PARAMS['max_tcp_vel'],
            dragMaxTcpOriVel=IMPEDANCE_PARAMS['max_tcp_ori_vel']
        )
        
        if rtn == 0:
            drag_mode_enabled = True
            print("Custom impedance control enabled successfully")
            return True
        else:
            print(f"Error enabling custom impedance control: {rtn}")
            return False
            
    else:
        print("Disabling custom impedance control...")
        
        # Turn off custom impedance parameters
        rtn = robot.ForceAndJointImpedanceStartStop(
            status=0,  # Turn off
            impedanceFlag=0,
            lamdeDain=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            KGain=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            BGain=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            dragMaxTcpVel=1000,
            dragMaxTcpOriVel=180
        )
        
        # Then disable drag teach mode
        robot.DragTeachSwitch(0)
        time.sleep(0.5)
        
        if rtn == 0:
            drag_mode_enabled = False
            print("Custom impedance control disabled successfully")
            return True
        else:
            print(f"Error disabling custom impedance control: {rtn}")
            return False

# ============================================================================
# FT SENSOR (unchanged)
# ============================================================================
def init_ft_sensor():
    robot.FT_SetConfig(24, 0)
    robot.FT_Activate(1)
    time.sleep(1.0)
    robot.SetLoadWeight(0, 0.0)
    robot.FT_SetZero(1)
    time.sleep(0.5)
    print("FT sensor ready")

def calibrate_baseline(samples=150):
    print("Calibrating baseline... (keep tool still)")
    forces = []
    for _ in range(samples):
        ret = robot.FT_GetForceTorqueRCS()
        if ret[0] == 0:
            forces.append(ret[1][:6])
        time.sleep(0.01)
    if forces:
        global baseline_forces
        baseline_forces = np.mean(forces, axis=0).tolist()
        print(f"Baseline: {[f'{x:+.3f}' for x in baseline_forces]}")

# ============================================================================
# MAIN LOOP — VELOCITY INCREASES WITH FORCE (Z-LIMITED)
# ============================================================================
def control_loop(movement_type):
    global running, filtered_fz_world, desired_joint_pos, joint_velocity
    global filtered_desired_joints, fixed_tcp_ref, USER_MIN_Z, USER_MAX_Z
    global current_fz, current_joints

    print("\n" + "="*70)
    print(f"   STARTING {movement_type.upper()} MODE")
    print("="*70)

    # Get current pose for reference
    err, tcp = robot.GetActualTCPPose()
    if err != 0: 
        print("Failed to get TCP pose")
        return
    fixed_tcp_ref = tcp.copy()

    # Initialize limits if not set
    if USER_MIN_Z is None or USER_MAX_Z is None:
        current_z = tcp[2]
        USER_MIN_Z = current_z - 300.0
        USER_MAX_Z = current_z + 300.0
        print(f"Initialized Z limits: {USER_MIN_Z:.1f} to {USER_MAX_Z:.1f} mm")

    START_Z = USER_MIN_Z
    GOAL_Z = USER_MAX_Z

    print(f"Using Z limits: {START_Z:.1f} ≤ Z ≤ {GOAL_Z:.1f} mm")

    if robot.ServoMoveStart() != 0: 
        print("Failed to start servo move")
        return

    j = robot.GetActualJointPosDegree(flag=0)
    if j[0] == 0:
        desired_joint_pos[:] = j[1][:6]
    filtered_desired_joints = desired_joint_pos[:]

    acc_joints = None
    ik_count = servo_count = 0

    try:
        while running and active_movements.get(movement_type, False):
            t0 = time.time()

            err, current_tcp = robot.GetActualTCPPose()
            current_tcp_z = current_tcp[2]  # <-- ADD THIS LINE
            # >>> RECORD TELEMETRY <<<
            now = time.time()
            with telemetry_lock:
                telemetry_buffer.append({
                    'time': now,
                    'tcp_z': current_tcp_z,
                    'fz': current_fz
                })
                # Keep buffer size manageable
                if len(telemetry_buffer) > MAX_BUFFER_SIZE:
                    telemetry_buffer.pop(0)
            last_telemetry_time = now
            # <<< END RECORDING <<<
            if err != 0: 
                time.sleep(IK_UPDATE_RATE); continue

            # Update global joint positions
            j_pos = robot.GetActualJointPosDegree(flag=0)
            if j_pos[0] == 0:
                current_joints = j_pos[1][:6]

            ft = robot.FT_GetForceTorqueRCS()
            if ft[0] != 0: 
                time.sleep(IK_UPDATE_RATE); continue

            raw = ft[1][:6]
            compensated = [raw[i] - baseline_forces[i] for i in range(6)]
            
            # Deadzone
            for i in range(6):
                if abs(compensated[i]) < force_thresholds[i]:
                    compensated[i] = 0.0

            # Transform force to world Z
            fz_world = transform_force_to_world(compensated, current_tcp[3:6])
            filtered_fz_world = ema(fz_world, filtered_fz_world, FORCE_FILTER_ALPHA)
            current_fz = filtered_fz_world  # Update global Fz

            active_force = filtered_fz_world if abs(filtered_fz_world) > FORCE_THRESHOLD else 0.0

            # Compute desired Z movement
            delta_z = -active_force * FORCE_TO_MOTION_SCALE
            target_z = current_tcp[2] + delta_z

            # HARD LIMIT: CLAMP Z BETWEEN USER-DEFINED BOUNDARIES
            target_z = np.clip(target_z, START_Z, GOAL_Z)

            # Keep X, Y, and orientation fixed — only Z changes
            target_tcp = [
                fixed_tcp_ref[0],
                fixed_tcp_ref[1],
                target_z,
                fixed_tcp_ref[3],
                fixed_tcp_ref[4],
                fixed_tcp_ref[5]
            ]

            ik = robot.GetInverseKin(0, target_tcp, -1)
            if ik[0] != 0: 
                time.sleep(IK_UPDATE_RATE); continue

            tj = np.array(ik[1][:6])
            acc_joints = tj if acc_joints is None else acc_joints + tj
            ik_count += 1

            if ik_count >= IK_TO_SERVO_RATIO:
                avg_joints = (acc_joints / IK_TO_SERVO_RATIO).tolist()

                # Apply admittance control with velocity limit
                for j in range(6):
                    err = avg_joints[j] - desired_joint_pos[j]
                    f = err * 3.9
                    acc = (f - B[j] * joint_velocity[j]) / M[j]
                    joint_velocity[j] += acc * SERVO_UPDATE_RATE
                    joint_velocity[j] = np.clip(joint_velocity[j], -MAX_JOINT_VELOCITY, MAX_JOINT_VELOCITY)
                    desired_joint_pos[j] += joint_velocity[j] * SERVO_UPDATE_RATE

                # Smooth joint command
                alpha = 0.32
                if filtered_desired_joints is None:
                    filtered_desired_joints = desired_joint_pos[:]
                else:
                    for j in range(6):
                        filtered_desired_joints[j] = alpha * desired_joint_pos[j] + (1 - alpha) * filtered_desired_joints[j]

                robot.ServoJ(filtered_desired_joints, [0]*6, 0, 0, SERVO_UPDATE_RATE, 0, 0)

                if servo_count % 20 == 0:
                    max_jv = max(abs(v) for v in joint_velocity)
                    print(f"Fz={filtered_fz_world:+6.2f}N → Z={current_tcp[2]:8.2f}mm | "
                          f"MaxJointVel={max_jv:5.1f}°/s")

                acc_joints = None
                ik_count = 0
                servo_count += 1

            sleep_t = IK_UPDATE_RATE - (time.time() - t0)
            if sleep_t > 0: 
                time.sleep(sleep_t)

    except Exception as e:
        print("Error in control loop:", e)
    finally:
        robot.ServoMoveEnd()

# ============================================================================
# SHUTDOWN & MAIN
# ============================================================================
def shutdown(sig, frame):
    global running
    print("\nStopping...")
    running = False
    time.sleep(0.5)
    err, tcp = robot.GetActualTCPPose()
    if err == 0 and fixed_tcp_ref is not None:
        dz = tcp[2] - fixed_tcp_ref[2]
        print(f"Final ΔZ = {dz:+.2f} mm")
    sys.exit(0)

signal.signal(signal.SIGINT, shutdown)

# ============================================================================
# FLASK ROUTES FOR TELEMETRY
# ============================================================================
@app.route('/get_telemetry')
def get_telemetry():
    """Return current Fz and TCP Z position for live display"""
    return jsonify({
        'fz': round(current_fz, 2),
        'tcp_z': round(current_tcp_z, 2),  # <-- CHANGED
        'max_velocity': MAX_JOINT_VELOCITY
    })

# ============================================================================
# FLASK ROUTES (UNCHANGED EXCEPT STYLING)
# ============================================================================
@app.route('/')
def index():
    html = '''
<!DOCTYPE html>
<html>
<head>
    <title>COBOT Control</title>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <style>
        body { 
            font-family: Arial, sans-serif; 
            background-color: #44b8a6;  /* SOLID COLOR AS REQUESTED */
            margin: 0; 
            padding: 0;
            min-height: 100vh;
            display: flex;
            justify-content: center;
            align-items: center;
        }
        .container {
            background: white;
            padding: 2rem;
            border-radius: 15px;
            box-shadow: 0 10px 30px rgba(0,0,0,0.3);
            text-align: center;
            max-width: 500px;
            width: 90%;
        }
        h1 { 
            color: #2c3e50; 
            margin-bottom: 2rem; 
            font-size: 2.2rem;
        }
        .mode-btn {
            background: #3498db;
            color: white;
            border: none;
            padding: 15px 30px;
            font-size: 1.2rem;
            border-radius: 50px;
            cursor: pointer;
            margin: 10px;
            transition: all 0.3s ease;
            width: 100%;
            max-width: 300px;
        }
        .mode-btn:hover {
            background: #2980b9;
            transform: translateY(-3px);
            box-shadow: 0 5px 15px rgba(0,0,0,0.2);
        }
        .mode-btn:active {
            transform: translateY(0);
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>COBOT Therapy System</h1>
        <button class="mode-btn" onclick="selectMode('passive')">Passive Mode</button>
        <button class="mode-btn" onclick="selectMode('active')">Active Mode</button>
    </div>

    <script>
        function selectMode(mode) {
            fetch('/select_mode', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({mode: mode})
            })
            .then(response => response.json())
            .then(data => {
                if(data.status === 'success') {
                    if(mode === 'active') {
                        window.location.href = '/active_page';
                    } else {
                        alert('Passive mode selected!\\nThis mode is not yet implemented.');
                    }
                }
            })
            .catch(error => console.error('Error:', error));
        }
    </script>
</body>
</html>
    '''
    return html


@app.route('/select_mode', methods=['POST'])
def select_mode():
    global current_mode
    mode = request.json['mode']
    current_mode = mode
    return jsonify({'status': 'success', 'mode': mode})


@app.route('/active_page')
def active_page():
    html = '''
<!DOCTYPE html>
<html>
<head>
    <title>Active Mode</title>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
    <style>
        body { 
            font-family: Arial, sans-serif; 
            background-color: #44b8a6;
            margin: 0; 
            padding: 0;
            min-height: 100vh;
        }
        .container {
            background: white;
            padding: 1.5rem;
            border-radius: 15px;
            box-shadow: 0 10px 30px rgba(0,0,0,0.3);
            margin: 20px;
            max-width: 800px;
            margin: 20px auto;
        }
        h1 { 
            color: #2c3e50; 
            margin: 10px 0 20px; 
            font-size: 1.8rem;
            text-align: center;
        }
        .limits-section {
            background: #f8f9fa;
            padding: 15px;
            border-radius: 10px;
            margin-bottom: 20px;
            display: flex;
            justify-content: space-between;
            flex-wrap: wrap;
            gap: 10px;
        }
        .limit-display {
            background: white;
            padding: 8px 12px;
            border-radius: 8px;
            border: 1px solid #ddd;
            min-width: 120px;
            text-align: center;
        }
        .limit-btn {
            background: #3498db;
            color: white;
            border: none;
            padding: 8px 15px;
            border-radius: 5px;
            cursor: pointer;
            margin-top: 5px;
        }
        .limit-btn.min { background: #e74c3c; }
        .limit-btn.max { background: #2ecc71; }
        .btn-grid {
            display: grid;
            grid-template-columns: repeat(2, 1fr);
            gap: 12px;
            margin-top: 10px;
        }
        @media (max-width: 600px) {
            .btn-grid {
                grid-template-columns: 1fr;
            }
            .limits-section {
                flex-direction: column;
                align-items: center;
            }
        }
        .control-btn {
            background: #34495e;
            color: white;
            border: none;
            padding: 14px;
            font-size: 1rem;
            border-radius: 10px;
            cursor: pointer;
            transition: all 0.2s ease;
            text-transform: capitalize;
        }
        .control-btn:hover {
            background: #2c3e50;
        }
        .control-btn.drag-mode {
            grid-column: span 2;
            background: #9b59b6;
        }
        .control-btn.drag-mode.active {
            background: #2ecc71;
        }
        .control-btn.movement.active {
            background: #3498db;
        }
        .back-btn, .plot-btn {
            background: #95a5a6;
            color: white;
            border: none;
            padding: 10px 20px;
            border-radius: 8px;
            cursor: pointer;
            margin-top: 15px;
            font-size: 1rem;
            width: 100%;
        }
        .plot-btn {
            background: #27ae60;
        }
        .status {
            margin-top: 15px;
            padding: 10px;
            background: #e8f4fc;
            border-radius: 8px;
            font-family: monospace;
            font-size: 0.9rem;
        }
        .telemetry {
            margin-top: 15px;
            padding: 12px;
            background: #f1f9ff;
            border-radius: 8px;
            font-family: monospace;
            font-size: 0.85rem;
            text-align: left;
        }
        .telemetry h3 {
            margin-top: 0;
            color: #2980b9;
        }
        .graphs {
            margin-top: 20px;
            display: none;
        }
        .graph-container {
            margin-top: 15px;
            height: 250px;
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>Active Therapy Mode</h1>
        
        <div class="limits-section">
            <div>
                <div class="limit-display" id="min-display">Min Z: -- mm</div>
                <button class="limit-btn min" onclick="setLimit('min')">Set Min</button>
            </div>
            <div>
                <div class="limit-display" id="max-display">Max Z: -- mm</div>
                <button class="limit-btn max" onclick="setLimit('max')">Set Max</button>
            </div>
        </div>
        
        <div class="btn-grid">
            <button id="drag-mode" class="control-btn drag-mode" onclick="toggleDragMode()">
                Drag Mode
            </button>
            
            <button class="control-btn movement" onclick="startMovement('Shoulder_flexion')">
                Shoulder Flexion
            </button>
            <button class="control-btn movement" onclick="startMovement('Shoulder_extention')">
                Shoulder Extension
            </button>
            <button class="control-btn movement" onclick="startMovement('Hip_Flexion')">
                Hip Flexion
            </button>
            <button class="control-btn movement" onclick="startMovement('Hip_Extention')">
                Hip Extension
            </button>
        </div>
        
        <button class="plot-btn" onclick="plotTelemetry()">Plot Real-Time Data</button>
        
        <div class="status" id="status">Ready</div>
        
        <div class="telemetry">
            <h3>Real-time Telemetry</h3>
            <div><strong>Fz Force:</strong> <span id="fz-value">0.00</span> N</div>
            <div><strong>TCP Z Position:</strong> <span id="z-value">0.00</span> mm</div>
            <div><strong>Max Velocity Limit:</strong> 60°/s</div>
        </div>

        <div class="graphs" id="graphs-section">
            <h3>Z-Axis Speed (mm/s)</h3>
            <div class="graph-container">
                <canvas id="zVelChart"></canvas>
            </div>
            <h3>Force Rate (dFz/dt in N/s)</h3>
            <div class="graph-container">
                <canvas id="fzRateChart"></canvas>
            </div>
        </div>
        
        <button class="back-btn" onclick="window.location.href='/'">Back to Main Menu</button>
    </div>

    <script>
        let dragModeActive = false;
        let activeMovement = null;
        let zVelChart = null;
        let fzRateChart = null;

        // Load current limits on page load
        function loadLimits() {
            fetch('/get_limits')
            .then(response => response.json())
            .then(data => {
                document.getElementById('min-display').textContent = 
                    data.min_z !== null ? `Min Z: ${data.min_z.toFixed(1)} mm` : 'Min Z: -- mm';
                document.getElementById('max-display').textContent = 
                    data.max_z !== null ? `Max Z: ${data.max_z.toFixed(1)} mm` : 'Max Z: -- mm';
            })
            .catch(err => console.error('Failed to load limits'));
        }
        
        function setLimit(type) {
            fetch('/set_limit', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({limit_type: type})
            })
            .then(response => response.json())
            .then(data => {
                if(data.status === 'success') {
                    document.getElementById(type + '-display').textContent = 
                        `${type === 'min' ? 'Min' : 'Max'} Z: ${data.z_value.toFixed(1)} mm`;
                } else {
                    alert('Error: ' + data.message);
                }
            })
            .catch(err => alert('Failed to set limit'));
        }
        
        function toggleDragMode() {
            const btn = document.getElementById('drag-mode');
            fetch('/drag_mode', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({})
            })
            .then(response => response.json())
            .then(data => {
                if(data.status === 'success') {
                    btn.classList.toggle('active', data.enabled);
                    dragModeActive = data.enabled;
                    updateStatus();
                    document.querySelectorAll('.movement').forEach(btn => {
                        btn.disabled = data.enabled;
                        if(data.enabled) btn.classList.remove('active');
                    });
                }
            });
        }
        
        function startMovement(movement) {
            if(dragModeActive) return;
            document.querySelectorAll('.movement').forEach(btn => {
                btn.classList.remove('active');
            });
            event.target.classList.add('active');
            activeMovement = movement;
            fetch('/stop_movement', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({movement: activeMovement})
            });
            fetch('/start_movement', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({movement: movement})
            })
            .then(response => response.json())
            .then(data => {
                if(data.status === 'success') {
                    updateStatus();
                }
            });
        }
        
        function updateStatus() {
            let status = 'Ready';
            if(dragModeActive) {
                status = 'Drag mode: ACTIVE';
            } else if(activeMovement) {
                status = 'Active movement: ' + activeMovement.replace('_', ' ');
            }
            document.getElementById('status').textContent = status;
        }
        
        function updateTelemetry() {
            fetch('/get_telemetry')
            .then(response => response.json())
            .then(data => {
                document.getElementById('fz-value').textContent = data.fz.toFixed(2);
                document.getElementById('z-value').textContent = data.tcp_z.toFixed(2);
            })
            .catch(err => console.error('Telemetry error:', err));
        }

        function plotTelemetry() {
            fetch('/get_telemetry_history')
            .then(response => response.json())
            .then(data => {
                const graphsSection = document.getElementById('graphs-section');
                graphsSection.style.display = 'block';

                // Convert timestamps to relative seconds
                if (data.timestamps.length === 0) return;

                const start = data.timestamps[0];
                const labels = data.timestamps.map(t => (t - start).toFixed(2));

                // Z Velocity Chart
                const zCtx = document.getElementById('zVelChart').getContext('2d');
                if (zVelChart) zVelChart.destroy();
                zVelChart = new Chart(zCtx, {
                    type: 'line',
                    data: {
                        labels: labels,
                        datasets: [{
                            label: 'Z Speed (mm/s)',
                            data: data.z_velocity,
                            borderColor: '#3498db',
                            backgroundColor: 'rgba(52, 152, 219, 0.1)',
                            borderWidth: 2,
                            tension: 0.3
                        }]
                    },
                    options: {
                        responsive: true,
                        maintainAspectRatio: false,
                        plugins: {
                            legend: { display: true }
                        },
                        scales: {
                            x: { title: { display: true, text: 'Time (s)' } },
                            y: { title: { display: true, text: 'Speed (mm/s)' } }
                        }
                    }
                });

                // Fz Rate Chart
                const fzCtx = document.getElementById('fzRateChart').getContext('2d');
                if (fzRateChart) fzRateChart.destroy();
                fzRateChart = new Chart(fzCtx, {
                    type: 'line',
                    data: {
                        labels: labels,
                        datasets: [{
                            label: 'dFz/dt (N/s)',
                            data: data.fz_rate,
                            borderColor: '#e74c3c',
                            backgroundColor: 'rgba(231, 76, 60, 0.1)',
                            borderWidth: 2,
                            tension: 0.3
                        }]
                    },
                    options: {
                        responsive: true,
                        maintainAspectRatio: false,
                        plugins: {
                            legend: { display: true }
                        },
                        scales: {
                            x: { title: { display: true, text: 'Time (s)' } },
                            y: { title: { display: true, text: 'Force Rate (N/s)' } }
                        }
                    }
                });
            })
            .catch(err => {
                console.error('Plot error:', err);
                alert('Failed to load data for plotting.');
            });
        }

        window.onload = function() {
            loadLimits();
            setInterval(updateTelemetry, 200);
        };
    </script>
</body>
</html>
    '''
    return html

@app.route('/drag_mode', methods=['POST'])
def toggle_drag_mode():
    global drag_mode_enabled
    if drag_mode_enabled:
        success = custom_drag_teach_mode(enable=False)
        if success:
            return jsonify({'status': 'success', 'enabled': False})
        else:
            return jsonify({'status': 'error', 'message': 'Failed to disable drag mode'}), 500
    else:
        success = custom_drag_teach_mode(enable=True)
        if success:
            return jsonify({'status': 'success', 'enabled': True})
        else:
            return jsonify({'status': 'error', 'message': 'Failed to enable drag mode'}), 500


@app.route('/start_movement', methods=['POST'])
def start_movement():
    movement = request.json['movement']
    
    if movement not in active_movements:
        return jsonify({'status': 'error', 'message': 'Invalid movement'}), 400
    
    # Stop all movements
    for key in active_movements:
        active_movements[key] = False
    
    active_movements[movement] = True
    
    # Start in thread
    thread = threading.Thread(target=control_loop, args=(movement,))
    thread.daemon = True
    thread.start()
    
    return jsonify({'status': 'success', 'movement': movement})


@app.route('/stop_movement', methods=['POST'])
def stop_movement():
    movement = request.json['movement']
    if movement in active_movements:
        active_movements[movement] = False
    return jsonify({'status': 'success', 'movement': movement})


@app.route('/set_limit', methods=['POST'])
def set_limit():
    """Set Min or Max Z limit to current robot Z position"""
    global USER_MIN_Z, USER_MAX_Z
    
    limit_type = request.json.get('limit_type')
    if limit_type not in ['min', 'max']:
        return jsonify({'status': 'error', 'message': 'Invalid limit type'}), 400
    
    # Get current TCP pose
    err, tcp = robot.GetActualTCPPose()
    if err != 0:
        return jsonify({'status': 'error', 'message': 'Failed to read robot position'}), 500
    
    current_z = tcp[2]
    
    if limit_type == 'min':
        USER_MIN_Z = current_z
    else:
        USER_MAX_Z = current_z
    
    return jsonify({
        'status': 'success',
        'limit_type': limit_type,
        'z_value': current_z
    })


@app.route('/get_limits')
def get_limits():
    return jsonify({
        'min_z': USER_MIN_Z,
        'max_z': USER_MAX_Z
    })

@app.route('/get_telemetry_history')
def get_telemetry_history():
    with telemetry_lock:
        data = telemetry_buffer.copy()
    
    if len(data) < 2:
        return jsonify({'z_velocity': [], 'fz_rate': [], 'timestamps': []})

    # Compute derivatives
    times = np.array([d['time'] for d in data])
    z_vals = np.array([d['tcp_z'] for d in data])
    fz_vals = np.array([d['fz'] for d in data])

    # Compute dt (avoid division by zero)
    dt = np.diff(times)
    dt = np.where(dt == 0, 1e-6, dt)

    # Z velocity in mm/sec
    z_velocity = np.diff(z_vals) / dt
    # Fz rate in N/sec
    fz_rate = np.diff(fz_vals) / dt

    # Use midpoints for time alignment
    mid_times = (times[:-1] + times[1:]) / 2

    # Convert to lists for JSON
    return jsonify({
        'timestamps': mid_times.tolist(),
        'z_velocity': z_velocity.tolist(),
        'fz_rate': fz_rate.tolist()
    })
# ============================================================================
# MAIN EXECUTION
# ============================================================================
if __name__ == "__main__":
    # Initialize robot
    robot = Robot.RPC('192.168.58.2')
    print("Connected to robot")
    init_ft_sensor()
    calibrate_baseline()
    
    print("\n" + "="*70)
    print("Starting Web Interface...")
    print("Open your browser and go to: http://localhost:5000")
    print("="*70)
    
    # Automatically open browser
    webbrowser.open('http://localhost:5000')
    
    # Start Flask app
    app.run(host='0.0.0.0', port=5000, debug=False)