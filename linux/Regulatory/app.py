# -*- coding: utf-8 -*-
"""
Unified Flask Backend: Active by Default, Passive Optional
- Active mode is ENABLED BY DEFAULT (no /set_mode needed for active features)
- Passive mode available via /set_mode
- ActiveTherapy class unchanged
- Both modes use same custom_drag_teach_mode()
"""

import sys
import os
import time
import json
import threading
import signal
import math
import numpy as np
from flask import Flask, request, jsonify
from flask_cors import CORS

# === Robot SDK ===
sys.path.append('/home/um/fairino-python-sdk-main/linux/fairino')
import Robot

ROBOT_IP = '192.168.58.2'
RECORDINGS_DIR = 'recordings'
os.makedirs(RECORDINGS_DIR, exist_ok=True)

# === Global Helper Functions ===
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

# === Global Parameters ===
IMPEDANCE_PARAMS = {
    'lamde_dain': [2.5, 2.0, 2.0, 2.0, 2.0, 2.0],
    'b_gain': [20.0, 10.0, 10.0, 5.0, 5.0, 1.0],
    'k_gain': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    'max_tcp_vel': 500,
    'max_tcp_ori_vel': 90
}

FORCE_TO_MOTION_SCALE = 6.0
M = [1.3, 1.3, 1.2, 1.4, 1.4, 1.4]
B = [2.5, 2.5, 2.5, 3.0, 3.0, 3.0]
IK_TO_SERVO_RATIO = 2
IK_UPDATE_RATE = 0.0025
SERVO_UPDATE_RATE = 0.008
FORCE_THRESHOLD = 0.8
FORCE_FILTER_ALPHA = 0.28
force_thresholds = [2.0, 2.0, 2.5, 1.0, 1.0, 1.0]
MAX_JOINT_VELOCITY = 60.0

# === Global State ===
app = Flask(__name__)
CORS(app, supports_credentials=True, allow_headers=['Content-Type'], methods=['GET', 'POST', 'OPTIONS'])

robot = None
running = True

# Shared telemetry
current_fz = 0.0
current_tcp_z = 0.0
baseline_forces = [0.0] * 6

# Mode state — DEFAULT TO ACTIVE (key fix!)
current_mode = 'active'  # ← Was None, now 'active' by default
drag_mode_enabled = False
playback_repetition_count = 0

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


# ============================================================================
# ACTIVE THERAPY CLASS (UNCHANGED from your active-only version)
# ============================================================================
class ActiveTherapy:
    def __init__(self, robot_instance, baseline_forces):
        self.robot = robot_instance
        self.baseline_forces = baseline_forces
        self.movement_active = False
        self.z_limits = {'min': None, 'max': None}
        self.filtered_fz_world = 0.0
        self.joint_velocity = [0.0] * 6
        self.desired_joint_pos = [0.0] * 6
        self.filtered_desired_joints = None

    def set_z_limit(self, limit_type, z_val):
        if limit_type == 'min':
            if self.z_limits['max'] is not None and z_val >= self.z_limits['max']:
                return False, "Min Z >= Max Z"
            self.z_limits['min'] = z_val
        elif limit_type == 'max':
            if self.z_limits['min'] is not None and z_val <= self.z_limits['min']:
                return False, "Max Z <= Min Z"
            self.z_limits['max'] = z_val
        else:
            return False, 'Use "min" or "max"'
        return True, z_val

    def start_movement(self):
        if self.movement_active:
            return False, "Movement already active"
        if self.z_limits['min'] is None or self.z_limits['max'] is None:
            return False, "Set Z limits first"
        if self.z_limits['min'] >= self.z_limits['max']:
            return False, "Min Z must be < Max Z"
        self.movement_active = True
        threading.Thread(target=self._control_loop, daemon=True).start()
        return True, "Movement started"

    def stop_movement(self):
        self.movement_active = False
        return True, "Movement stopped"

    def _control_loop(self):
        global running, current_tcp_z, current_fz
        err, tcp = self.robot.GetActualTCPPose()
        if err != 0:
            self.movement_active = False
            return
        fixed_tcp_ref = tcp.copy()
        START_Z = self.z_limits['min']
        GOAL_Z = self.z_limits['max']
        if self.robot.ServoMoveStart() != 0:
            self.movement_active = False
            return
        j = self.robot.GetActualJointPosDegree(flag=0)
        if j[0] == 0:
            self.desired_joint_pos[:] = j[1][:6]
        self.filtered_desired_joints = self.desired_joint_pos[:]
        acc_joints = None
        ik_count = 0
        try:
            while running and self.movement_active:
                t0 = time.time()
                err, current_tcp = self.robot.GetActualTCPPose()
                current_tcp_z = current_tcp[2]
                if err != 0:
                    time.sleep(IK_UPDATE_RATE)
                    continue
                ft = self.robot.FT_GetForceTorqueRCS()
                if ft[0] != 0:
                    time.sleep(IK_UPDATE_RATE)
                    continue
                raw = ft[1][:6]
                compensated = [raw[i] - self.baseline_forces[i] for i in range(6)]
                for i in range(6):
                    if abs(compensated[i]) < force_thresholds[i]:
                        compensated[i] = 0.0
                fz_world = transform_force_to_world(compensated, current_tcp[3:6])
                self.filtered_fz_world = ema(fz_world, self.filtered_fz_world, FORCE_FILTER_ALPHA)
                current_fz = self.filtered_fz_world
                active_force = self.filtered_fz_world if abs(self.filtered_fz_world) > FORCE_THRESHOLD else 0.0
                delta_z = -active_force * FORCE_TO_MOTION_SCALE
                target_z = np.clip(current_tcp[2] + delta_z, START_Z, GOAL_Z)
                target_tcp = [
                    fixed_tcp_ref[0], fixed_tcp_ref[1], target_z,
                    fixed_tcp_ref[3], fixed_tcp_ref[4], fixed_tcp_ref[5]
                ]
                ik = self.robot.GetInverseKin(0, target_tcp, -1)
                if ik[0] != 0:
                    time.sleep(IK_UPDATE_RATE)
                    continue
                tj = np.array(ik[1][:6])
                acc_joints = tj if acc_joints is None else acc_joints + tj
                ik_count += 1
                if ik_count >= IK_TO_SERVO_RATIO:
                    avg_joints = (acc_joints / IK_TO_SERVO_RATIO).tolist()
                    for j_idx in range(6):
                        err_val = avg_joints[j_idx] - self.desired_joint_pos[j_idx]
                        f = err_val * 3.9
                        acc = (f - B[j_idx] * self.joint_velocity[j_idx]) / M[j_idx]
                        self.joint_velocity[j_idx] += acc * SERVO_UPDATE_RATE
                        self.joint_velocity[j_idx] = np.clip(self.joint_velocity[j_idx], -MAX_JOINT_VELOCITY, MAX_JOINT_VELOCITY)
                        self.desired_joint_pos[j_idx] += self.joint_velocity[j_idx] * SERVO_UPDATE_RATE
                    alpha = 0.32
                    if self.filtered_desired_joints is None:
                        self.filtered_desired_joints = self.desired_joint_pos[:]
                    else:
                        for j_idx in range(6):
                            self.filtered_desired_joints[j_idx] = alpha * self.desired_joint_pos[j_idx] + (1 - alpha) * self.filtered_desired_joints[j_idx]
                    self.robot.ServoJ(self.filtered_desired_joints, [0]*6, 0, 0, SERVO_UPDATE_RATE, 0, 0)
                    acc_joints = None
                    ik_count = 0
                sleep_t = IK_UPDATE_RATE - (time.time() - t0)
                if sleep_t > 0:
                    time.sleep(sleep_t)
        except Exception as e:
            print("Active control error:", e)
        finally:
            self.robot.ServoMoveEnd()
            self.movement_active = False

    def is_active(self):
        return self.movement_active

    def get_z_limits(self):
        return self.z_limits


# ============================================================================
# PASSIVE THERAPY CLASS
# ============================================================================
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
        log_file = 'trajectory_log.csv'
        names = []
        if os.path.exists(log_file):
            with open(log_file, 'r') as f:
                for line in f:
                    if ',' in line:
                        parts = line.strip().split(',')
                        if len(parts) >= 2:
                            names.append(parts[1])
        return sorted(set(names))

    def _log_trajectory(self, name):
        log_file = 'trajectory_log.csv'
        file_exists = os.path.exists(log_file)
        with open(log_file, 'a') as f:
            if not file_exists:
                f.write('Timestamp,TrajectoryName,Notes\n')
            timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
            f.write(f'{timestamp},{name},Recorded via Flask\n')

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

# ============================================================================
# Guided THERAPY CLASS
# ============================================================================

# ============================================================================
# Passive THERAPY CLASS
# ============================================================================


# ============================================================================
# GLOBAL INSTANCES
# ============================================================================
active_therapy = None
passive_therapy = None


# ============================================================================
# TELEMETRY THREAD
# ============================================================================
def start_telemetry():
    global current_fz, current_tcp_z, running, baseline_forces, robot
    while running:
        try:
            err, tcp = robot.GetActualTCPPose()
            if err == 0:
                current_tcp_z = tcp[2]
            ft = robot.FT_GetForceTorqueRCS()
            if ft[0] == 0:
                raw = ft[1][:6]
                compensated = [raw[i] - baseline_forces[i] for i in range(6)]
                current_fz = compensated[2]
        except:
            pass
        time.sleep(0.1)


# ============================================================================
# SHUTDOWN HANDLER
# ============================================================================
def safe_shutdown():
    global running, robot
    print("Shutting down robot...")
    running = False
    time.sleep(0.5)
    try:
        if robot:
            if active_therapy:
                active_therapy.stop_movement()
            if passive_therapy:
                passive_therapy.stop_playback()
            custom_drag_teach_mode(enable=False)
            robot.ServoMoveEnd()
    except:
        pass
    print("Robot stopped.")

def signal_handler(sig, frame):
    safe_shutdown()
    os._exit(0)

signal.signal(signal.SIGINT, signal_handler)


# ============================================================================
# API ENDPOINTS
# ============================================================================

@app.route('/set_mode', methods=['POST'])
def set_mode():
    global current_mode, active_therapy, passive_therapy
    data = request.get_json()
    if not isinstance(data, dict) or 'mode' not in data:
        return jsonify({'status': 'error', 'message': 'Missing "mode"'}), 400
    mode = data['mode']
    if mode not in ['active', 'passive']:
        return jsonify({'status': 'error', 'message': 'Invalid mode'}), 400

    if mode == 'active' and current_mode != 'active':
        # Switch to active
        if passive_therapy:
            passive_therapy.stop_playback()
            passive_therapy = None
        if active_therapy is None:
            active_therapy = ActiveTherapy(robot, baseline_forces)
        current_mode = 'active'
        print("✅ Switched to ACTIVE mode")

    elif mode == 'passive' and current_mode != 'passive':
        # Switch to passive
        if active_therapy:
            active_therapy.stop_movement()
            active_therapy = None
        passive_therapy = PassiveTherapy(robot)
        current_mode = 'passive'
        print("✅ Switched to PASSIVE mode")

    return jsonify({'status': 'success', 'mode': current_mode})


@app.route('/set_limit', methods=['POST'])
def set_limit():
    # Since current_mode defaults to 'active', this works on startup
    if current_mode != 'active':
        return jsonify({'status': 'error', 'message': f'Not in active mode (current: {current_mode})'}), 400
    if not active_therapy:
        return jsonify({'status': 'error', 'message': 'Active therapy not ready'}), 500

    data = request.get_json()
    if not isinstance(data, dict) or 'limit_type' not in data:
        return jsonify({'status': 'error', 'message': 'Missing "limit_type"'}), 400

    limit_type = data['limit_type']
    if limit_type not in ('min', 'max'):
        return jsonify({'status': 'error', 'message': 'limit_type must be "min" or "max"'}), 400

    err, tcp = robot.GetActualTCPPose()
    if err != 0:
        return jsonify({'status': 'error', 'message': 'TCP read failed'}), 500
    z_val = tcp[2]

    success, msg = active_therapy.set_z_limit(limit_type, z_val)
    if success:
        return jsonify({'status': 'success', 'z_value': round(z_val, 2)})
    else:
        return jsonify({'status': 'error', 'message': msg}), 400


@app.route('/start_movement', methods=['POST'])
def start_movement():
    if current_mode != 'active':
        return jsonify({'status': 'error', 'message': 'Not in active mode'}), 400
    if not active_therapy:
        return jsonify({'status': 'error', 'message': 'Active therapy not ready'}), 500
    success, msg = active_therapy.start_movement()
    if success:
        return jsonify({'status': 'success'})
    else:
        return jsonify({'status': 'error', 'message': msg}), 400

@app.route('/stop_movement', methods=['POST'])
def stop_movement():
    if active_therapy:
        active_therapy.stop_movement()
    return jsonify({'status': 'success'})

@app.route('/drag_mode', methods=['POST'])
def drag_mode():
    data = request.get_json() or {}
    enable = data.get('enable')
    if enable is None:
        enable = not drag_mode_enabled
    success = custom_drag_teach_mode(enable)
    if success:
        return jsonify({'status': 'success', 'enabled': enable, 'mode': current_mode})
    else:
        return jsonify({'status': 'error', 'message': 'Drag mode failed'}), 500

@app.route('/start_recording', methods=['POST'])
def start_recording():
    if current_mode != 'passive':
        return jsonify({'status': 'error', 'message': 'Only in passive mode'}), 400
    if not passive_therapy:
        return jsonify({'status': 'error', 'message': 'Passive therapy not ready'}), 500
    data = request.get_json() or {}
    name = data.get('name')
    if not name:
        return jsonify({'status': 'error', 'message': 'Name is required'}), 400
    success, msg = passive_therapy.start_recording(name)
    if success:
        passive_therapy._log_trajectory(name)
        return jsonify({'status': 'success', 'name': name})
    else:
        return jsonify({'status': 'error', 'message': msg}), 400

@app.route('/stop_recording', methods=['POST'])
def stop_recording():
    if current_mode != 'passive':
        return jsonify({'status': 'error', 'message': 'Only in passive mode'}), 400
    if not passive_therapy:
        return jsonify({'status': 'error', 'message': 'Passive therapy not ready'}), 500
    success, name = passive_therapy.stop_recording()
    if success:
        return jsonify({'status': 'success', 'name': name})
    else:
        return jsonify({'status': 'error', 'message': name}), 400

@app.route('/list_recordings', methods=['GET'])
def list_recordings():
    if passive_therapy:
        return jsonify({'files': passive_therapy.list_recordings()})
    else:
        return jsonify({'files': []})

@app.route('/start_playback', methods=['POST'])
def start_playback():
    if current_mode != 'passive':
        return jsonify({'status': 'error', 'message': 'Only in passive mode'}), 400
    if not passive_therapy:
        return jsonify({'status': 'error', 'message': 'Passive therapy not ready'}), 500
    data = request.get_json()
    filename = data.get('filename')
    repetitions = data.get('repetitions', 1)
    success, msg = passive_therapy.start_playback(filename, repetitions)
    if success:
        return jsonify({'status': 'success'})
    else:
        return jsonify({'status': 'error', 'message': msg}), 400

@app.route('/stop_playback', methods=['POST'])
def stop_playback():
    if passive_therapy:
        passive_therapy.stop_playback()
    return jsonify({'status': 'success'})

@app.route('/get_telemetry', methods=['GET'])
def get_telemetry():
    return jsonify({
        'fz': round(current_fz, 2),
        'tcp_z': round(current_tcp_z, 2),
        'mode': current_mode,
        'active_movement': active_therapy.is_active() if current_mode == 'active' else False,
        'playback_active': passive_therapy.is_playback_active() if current_mode == 'passive' else False,
        'playback_repetition_count': playback_repetition_count,
        'drag_mode_enabled': drag_mode_enabled,
        'z_limits': active_therapy.get_z_limits() if current_mode == 'active' else None
    })

@app.route('/shutdown', methods=['POST'])
def shutdown_endpoint():
    threading.Thread(target=safe_shutdown).start()
    return jsonify({'status': 'shutting down'})


# ============================================================================
# MAIN
# ============================================================================
if __name__ == "__main__":
    robot = Robot.RPC(ROBOT_IP)
    print("✅ Connected to robot")
    
    robot.FT_SetConfig(24, 0)
    robot.FT_Activate(1)
    time.sleep(1.0)
    robot.SetLoadWeight(0, 0.0)
    robot.FT_SetZero(1)
    time.sleep(0.5)
    
    print("Calibrating force baseline...")
    forces = []
    for _ in range(100):
        ret = robot.FT_GetForceTorqueRCS()
        if ret[0] == 0:
            forces.append(ret[1][:6])
        time.sleep(0.01)
    if forces:
        baseline_forces = np.mean(forces, axis=0).tolist()
        print("✅ Baseline force calibrated")
    else:
        print("⚠️ Warning: Could not calibrate baseline forces")

    # 🔑 KEY CHANGE: Initialize ActiveTherapy at startup (active mode is default)
    active_therapy = ActiveTherapy(robot, baseline_forces)

    # Start telemetry
    threading.Thread(target=start_telemetry, daemon=True).start()

    print("🚀 Unified Therapy Server running on http://localhost:5000 (Active mode by default)")
    app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False)