"""
Passive Therapy Desktop App (PyQt6)
- Standalone (no web server)
- Implements passive-mode features from app.py + passive_therapy.html
"""

import sys
import os
import time
import threading
import math
import numpy as np
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QLabel, QPushButton, QVBoxLayout,
    QHBoxLayout, QGridLayout, QComboBox, QSpinBox, QMessageBox, QInputDialog, QFrame
)

# === Robot SDK ===
sys.path.append('/home/um/fairino-python-sdk-main/linux/fairino')
import Robot

ROBOT_IP = '192.168.58.2'
RECORDINGS_DIR = 'recordings'
os.makedirs(RECORDINGS_DIR, exist_ok=True)

# === Parameters (from app.py passive mode) ===
IMPEDANCE_PARAMS = {
    'lamde_dain': [2.5, 2.0, 2.0, 2.0, 2.0, 2.0],
    'b_gain': [20.0, 10.0, 10.0, 5.0, 5.0, 1.0],
    'k_gain': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    'max_tcp_vel': 500,
    'max_tcp_ori_vel': 26
}

PASSIVE_MODE_SPEED = 5.0  # Velocity percentage [0-100]
TELEMETRY_UPDATE_RATE = 0.02  # 20 ms

HOME_TARGET_JOINTS = [
    2.608,    # J1
    -88.384,  # J2
    127.473,  # J3
    -137.27,  # J4
    -92.275,  # J5
    -90.118   # J6
]
HOME_MAX_JOINT_SPEED = 180.0  # deg/s
HOME_DESIRED_SPEED = 7.70     # deg/s


class PassiveTherapy:
    def __init__(self, robot_instance):
        self.robot = robot_instance
        self.is_recording = False
        self.current_recording_name = None
        self.playback_active = False
        self.current_direction = None
        self.trajectory_cache = {}
        self.playback_repetition_count = 0

    def custom_drag_teach_mode(self, enable=True):
        if enable:
            self.robot.DragTeachSwitch(1)
            time.sleep(0.5)
            rtn = self.robot.ForceAndJointImpedanceStartStop(
                status=1, impedanceFlag=1,
                lamdeDain=IMPEDANCE_PARAMS['lamde_dain'],
                KGain=IMPEDANCE_PARAMS['k_gain'],
                BGain=IMPEDANCE_PARAMS['b_gain'],
                dragMaxTcpVel=IMPEDANCE_PARAMS['max_tcp_vel'],
                dragMaxTcpOriVel=IMPEDANCE_PARAMS['max_tcp_ori_vel']
            )
            return rtn == 0
        self.robot.ForceAndJointImpedanceStartStop(
            status=0, impedanceFlag=0,
            lamdeDain=[0] * 6, KGain=[0] * 6, BGain=[0] * 6,
            dragMaxTcpVel=1000, dragMaxTcpOriVel=180
        )
        self.robot.DragTeachSwitch(0)
        time.sleep(0.5)
        return True

    def start_recording(self, name):
        if self.is_recording:
            return False, "Already recording"
        if not name:
            return False, "Name is required"
        try:
            success = self.custom_drag_teach_mode(enable=True)
            if not success:
                return False, "Failed to enable drag mode"
            ret1 = self.robot.SetTPDParam(name, 4, 0)
            if ret1 != 0:
                self.custom_drag_teach_mode(enable=False)
                return False, f"SetTPDParam failed (code: {ret1})"
            ret2 = self.robot.SetTPDStart(name, 4, 0)
            if ret2 != 0:
                self.custom_drag_teach_mode(enable=False)
                return False, f"SetTPDStart failed (code: {ret2})"
            self.is_recording = True
            self.current_recording_name = name
            return True, f"Recording started: {name}"
        except Exception as e:
            self.custom_drag_teach_mode(enable=False)
            return False, str(e)

    def stop_recording(self):
        if not self.is_recording:
            return False, "Not recording"
        try:
            ret = self.robot.SetWebTPDStop()
            self.custom_drag_teach_mode(enable=False)
            name = self.current_recording_name
            self.is_recording = False
            self.current_recording_name = None
            if ret != 0:
                return False, f"SetWebTPDStop failed (code: {ret})"
            if name in self.trajectory_cache:
                del self.trajectory_cache[name]
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
            f.write(f'{timestamp},{name},Recorded via Desktop App\n')

    def _get_trajectory_endpoints(self, filename, allow_probe=True):
        if filename in self.trajectory_cache:
            return self.trajectory_cache[filename]
        try:
            ret = self.robot.LoadTPD(filename)
            if ret != 0:
                return None, None
            err, start_pose = self.robot.GetTPDStartPose(filename)
            if err != 0:
                return None, None
            if not allow_probe:
                return start_pose, None

            self.robot.MoveL(start_pose, 0, 0, vel=PASSIVE_MODE_SPEED, overSpeedStrategy=1, speedPercent=2)
            time.sleep(0.3)
            self.robot.MoveTPD(filename, 1, 100)

            timeout = 30
            start_time = time.time()
            while True:
                if time.time() - start_time > timeout:
                    return None, None
                err, done = self.robot.GetRobotMotionDone()
                if err == 0 and done == 1:
                    break
                time.sleep(0.05)

            err, end_pose = self.robot.GetActualTCPPose()
            if err != 0:
                return None, None

            self.trajectory_cache[filename] = (start_pose, end_pose)
            return start_pose, end_pose
        except Exception:
            return None, None

    def _calculate_distance(self, pose1, pose2):
        return math.sqrt(sum((pose1[i] - pose2[i]) ** 2 for i in range(3)))

    def _determine_direction(self, filename, current_pose):
        start_pose, end_pose = self._get_trajectory_endpoints(filename, allow_probe=False)
        if start_pose is None or end_pose is None:
            return 1
        dist_to_start = self._calculate_distance(current_pose, start_pose)
        dist_to_end = self._calculate_distance(current_pose, end_pose)
        return 1 if dist_to_start <= dist_to_end else -1

    def start_playback(self, filename, repetitions=1):
        if self.playback_active:
            return False, "Playback already active"
        try:
            repetitions = int(repetitions)
        except Exception:
            repetitions = 1
        repetitions = max(1, min(100, repetitions))

        def playback_task():
            try:
                self.playback_active = True
                self.playback_repetition_count = 0

                ret = self.robot.LoadTPD(filename)
                if ret != 0:
                    return

                err, current_pose = self.robot.GetActualTCPPose()
                if err != 0:
                    return

                direction = self._determine_direction(filename, current_pose)
                self.current_direction = 'forward' if direction == 1 else 'reverse'

                start_pose, end_pose = self._get_trajectory_endpoints(filename, allow_probe=False)
                if start_pose is None:
                    return
                if end_pose is None and direction == -1:
                    direction = 1
                    self.current_direction = 'forward'

                for _ in range(repetitions):
                    if not self.playback_active:
                        break

                    if direction == 1:
                        self.robot.MoveL(start_pose, 0, 0, vel=PASSIVE_MODE_SPEED, overSpeedStrategy=1, speedPercent=2)
                    else:
                        self.robot.MoveL(end_pose, 0, 0, vel=PASSIVE_MODE_SPEED, overSpeedStrategy=1, speedPercent=2)

                    while self.playback_active:
                        err, done = self.robot.GetRobotMotionDone()
                        if err == 0 and done == 1:
                            break
                        time.sleep(0.02)

                    if not self.playback_active:
                        break

                    time.sleep(0.05)
                    self.robot.MoveTPD(filename, direction, 100)

                    while self.playback_active:
                        err, done = self.robot.GetRobotMotionDone()
                        if err == 0 and done == 1:
                            break
                        time.sleep(0.02)

                    if not self.playback_active:
                        break

                    self.playback_repetition_count += 1
                    time.sleep(0.05)
            finally:
                self.playback_active = False
                self.current_direction = None

        threading.Thread(target=playback_task, daemon=True).start()
        return True, "Playback started"

    def stop_playback(self):
        self.playback_active = False
        try:
            self.robot.StopMotion()
        except Exception:
            pass
        return True, "Playback stopped"

    def pause_playback(self):
        if not self.playback_active:
            return False, "Playback not active"
        try:
            ret = self.robot.PauseMotion()
            if ret != 0:
                return False, f"PauseMotion failed (code: {ret})"
            return True, "Playback paused"
        except Exception as e:
            return False, str(e)

    def resume_playback(self):
        if not self.playback_active:
            return False, "Playback not active"
        try:
            ret = self.robot.ResumeMotion()
            if ret != 0:
                return False, f"ResumeMotion failed (code: {ret})"
            return True, "Playback resumed"
        except Exception as e:
            return False, str(e)


class PassiveTherapyWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Passive Therapy")
        self.setMinimumWidth(640)

        self.setStyleSheet("""
            QMainWindow {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:1,
                                            stop:0 #000000, stop:0.5 #1a3a34,
                                            stop:0.75 #2d5a4f, stop:1 #44b8a6);
            }
            QLabel { color: #e2e8f0; font-size: 18px; }
            QLabel#sectionTitle { font-size: 20px; font-weight: 600; color: #93c5fd; }
            QLabel#sectionSub { color: #94a3b8; font-size: 20px; }
            QPushButton {
                color: white; border: 1px solid rgba(68, 184, 166, 0.3);
                padding: 8px 14px; border-radius: 8px; font-weight: 600;
                font-size: 16px;
                min-height: 40px; max-height: 40px;
            }
            QPushButton:disabled { background: #2a3a3f; color: #94a3b8; }

            QPushButton[class="mode"] {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                                            stop:0 #9f7aea, stop:1 #805ad5);
            }
            QPushButton[class="mode"]:hover {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                                            stop:0 #b779da, stop:1 #9f7aea);
            }

            QPushButton[class="record"] {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                                            stop:0 #d97706, stop:1 #b45309);
            }
            QPushButton[class="record"]:hover {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                                            stop:0 #f59e0b, stop:1 #d97706);
            }

            QPushButton[class="stop"] {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                                            stop:0 #e53e3e, stop:1 #c53030);
            }
            QPushButton[class="stop"]:hover {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                                            stop:0 #f56565, stop:1 #e53e3e);
            }
            QComboBox, QSpinBox {
                background: #0f1f22; color: #e2e8f0; border: 1px solid #2a3a3f;
                border-radius: 6px; padding: 4px 8px;
            }
            QFrame#container {
                background: rgba(50, 54, 58, 0.95);
                border-radius: 16px;
                padding: 16px;
                color: #f0f4f8;
            }
            QFrame#card {
                background: rgba(30, 41, 59, 0.7);
                border: 1px solid rgba(68, 184, 166, 0.4);
                border-radius: 8px;
                padding: 12px;
            }
        """)

        self.robot = Robot.RPC(ROBOT_IP)
        self.therapy = PassiveTherapy(self.robot)

        self.drag_active = False
        self.recording = False
        self.playback_running = False
        self.current_fz = 0.0
        self.current_tcp_z = 0.0

        self._build_ui()
        self._start_telemetry_thread()
        self._start_ui_timer()

    def _build_ui(self):
        root = QWidget()
        outer_layout = QHBoxLayout()
        outer_layout.setContentsMargins(16, 16, 16, 16)

        left_panel = QFrame()
        left_panel.setObjectName("container")
        left_panel.setMinimumWidth(520)
        layout = QVBoxLayout()
        layout.setSpacing(12)

        title = QLabel("Mauris: \n Modular Adaptive universal Robotic Intelligence System")
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        title.setStyleSheet("font-size: 18px; font-weight: 700; color: #e2e8f0;")
        layout.addWidget(title)

        subtitle = QLabel("Robotic Rehabilitation • Passive Mode")
        subtitle.setAlignment(Qt.AlignmentFlag.AlignCenter)
        subtitle.setObjectName("sectionSub")
        layout.addWidget(subtitle)

        # Recording card
        record_card = QFrame()
        record_card.setObjectName("card")
        record_card_layout = QVBoxLayout()
        record_title = QLabel("Recording")
        record_title.setObjectName("sectionTitle")
        record_card_layout.addWidget(record_title)

        record_row = QHBoxLayout()
        self.start_record_btn = QPushButton("Start Recording")
        self.stop_record_btn = QPushButton("Stop Recording")
        self.stop_record_btn.setEnabled(False)
        self.start_record_btn.setProperty("class", "record")
        self.stop_record_btn.setProperty("class", "stop")
        self.start_record_btn.clicked.connect(self.start_recording)
        self.stop_record_btn.clicked.connect(self.stop_recording)
        record_row.addWidget(self.start_record_btn)
        record_row.addWidget(self.stop_record_btn)
        record_card_layout.addLayout(record_row)

        self.record_status = QLabel("Idle")
        self.record_status.setStyleSheet("color: #93c5fd;")
        record_card_layout.addWidget(self.record_status)
        record_card.setLayout(record_card_layout)
        layout.addWidget(record_card)

        # Playback card
        playback_card = QFrame()
        playback_card.setObjectName("card")
        playback_card_layout = QVBoxLayout()
        playback_title = QLabel("Playback Recorded Exercise")
        playback_title.setObjectName("sectionTitle")
        playback_card_layout.addWidget(playback_title)

        playback_grid = QGridLayout()
        playback_grid.addWidget(QLabel("Recorded Exercise"), 0, 0)

        self.exercise_select = QComboBox()
        self.exercise_select.currentIndexChanged.connect(self._toggle_playback_button)
        playback_grid.addWidget(self.exercise_select, 0, 1)

        playback_grid.addWidget(QLabel("Repetitions"), 1, 0)
        self.reps_input = QSpinBox()
        self.reps_input.setRange(1, 100)
        self.reps_input.setValue(5)
        playback_grid.addWidget(self.reps_input, 1, 1)

        self.current_rep_label = QLabel("Current Rep: 0")
        self.current_rep_label.setStyleSheet("color: #93c5fd; font-weight: 600;")
        playback_grid.addWidget(self.current_rep_label, 2, 0, 1, 2)

        self.playback_btn = QPushButton("Start Playback")
        self.pause_btn = QPushButton("Pause")
        self.resume_btn = QPushButton("Play")
        self.pause_btn.setEnabled(False)
        self.resume_btn.setEnabled(False)

        self.playback_btn.setProperty("class", "mode")
        self.pause_btn.setProperty("class", "mode")
        self.resume_btn.setProperty("class", "mode")

        self.playback_btn.clicked.connect(self.start_playback)
        self.pause_btn.clicked.connect(self.pause_playback)
        self.resume_btn.clicked.connect(self.resume_playback)

        playback_grid.addWidget(self.playback_btn, 3, 0)
        playback_grid.addWidget(self.pause_btn, 3, 1)
        playback_grid.addWidget(self.resume_btn, 4, 0)
        playback_card_layout.addLayout(playback_grid)
        playback_card.setLayout(playback_card_layout)
        layout.addWidget(playback_card)

        # Controls card
        controls_card = QFrame()
        controls_card.setObjectName("card")
        controls_layout = QVBoxLayout()
        controls_title = QLabel("Controls")
        controls_title.setObjectName("sectionTitle")
        controls_layout.addWidget(controls_title)

        drag_home_row = QHBoxLayout()
        self.drag_btn = QPushButton("Move Robot")
        self.home_btn = QPushButton("Home")
        self.drag_btn.setProperty("class", "mode")
        self.home_btn.setProperty("class", "mode")
        self.drag_btn.clicked.connect(self.toggle_drag_mode)
        self.home_btn.clicked.connect(self.move_home)
        drag_home_row.addWidget(self.drag_btn)
        drag_home_row.addWidget(self.home_btn)
        controls_layout.addLayout(drag_home_row)
        controls_card.setLayout(controls_layout)
        layout.addWidget(controls_card)

        # Status + telemetry card
        telemetry_card = QFrame()
        telemetry_card.setObjectName("card")
        telemetry_layout = QVBoxLayout()
        telemetry_title = QLabel("Real-time Telemetry")
        telemetry_title.setObjectName("sectionTitle")
        telemetry_layout.addWidget(telemetry_title)

        self.status_label = QLabel("Ready")
        self.status_label.setStyleSheet("color: #fbbf24; font-weight: 600;")
        telemetry_layout.addWidget(self.status_label)

        self.fz_label = QLabel("Fz Force: 0.00 N")
        self.z_label = QLabel("TCP Z Position: 0.00 mm")
        telemetry_layout.addWidget(self.fz_label)
        telemetry_layout.addWidget(self.z_label)
        telemetry_card.setLayout(telemetry_layout)
        layout.addWidget(telemetry_card)

        # Uniform button sizing
        for btn in [
            self.start_record_btn, self.stop_record_btn,
            self.playback_btn, self.pause_btn, self.resume_btn,
            self.drag_btn, self.home_btn
        ]:
            btn.setMinimumWidth(160)
            btn.setMaximumHeight(40)

        left_panel.setLayout(layout)
        outer_layout.addStretch(1)
        outer_layout.addWidget(left_panel, 1)
        outer_layout.addStretch(1)
        root.setLayout(outer_layout)
        self.setCentralWidget(root)

        self._load_recorded_exercises()
        self._toggle_playback_button()

    def _load_recorded_exercises(self):
        self.exercise_select.clear()
        self.exercise_select.addItem("Select a recorded exercise...")
        for name in self.therapy.list_recordings():
            self.exercise_select.addItem(name)

    def _toggle_playback_button(self):
        enabled = self.exercise_select.currentIndex() > 0
        self.playback_btn.setEnabled(enabled)

    def _start_telemetry_thread(self):
        def telemetry_loop():
            while True:
                try:
                    err, tcp = self.robot.GetActualTCPPose()
                    if err == 0:
                        self.current_tcp_z = tcp[2]
                    ft = self.robot.FT_GetForceTorqueRCS()
                    if ft[0] == 0:
                        raw = ft[1][:6]
                        self.current_fz = raw[2]
                except Exception:
                    pass
                time.sleep(TELEMETRY_UPDATE_RATE)

        threading.Thread(target=telemetry_loop, daemon=True).start()

    def _start_ui_timer(self):
        self.ui_timer = QTimer(self)
        self.ui_timer.timeout.connect(self._refresh_ui)
        self.ui_timer.start(50)

    def _refresh_ui(self):
        self.fz_label.setText(f"Fz Force: {self.current_fz:.2f} N")
        self.z_label.setText(f"TCP Z Position: {self.current_tcp_z:.2f} mm")
        self.current_rep_label.setText(f"Current Rep: {self.therapy.playback_repetition_count}")

    def start_recording(self):
        if self.drag_active:
            QMessageBox.warning(self, "Drag Mode Active", "Disable Drag Mode before recording")
            return
        name, ok = QInputDialog.getText(self, "Start Recording", "Enter a name for this trajectory:")
        if not ok or not name.strip():
            return
        success, msg = self.therapy.start_recording(name.strip())
        if success:
            self.therapy._log_trajectory(name.strip())
            self.recording = True
            self.start_record_btn.setEnabled(False)
            self.stop_record_btn.setEnabled(True)
            self.record_status.setText(f"Recording: {name.strip()}")
            self.status_label.setText("Recording Trajectory...")
        else:
            QMessageBox.warning(self, "Recording Failed", msg)

    def stop_recording(self):
        success, msg = self.therapy.stop_recording()
        if success:
            self.recording = False
            self.start_record_btn.setEnabled(True)
            self.stop_record_btn.setEnabled(False)
            self.record_status.setText(f"Saved as: {msg}")
            self.status_label.setText("Ready")
            self._load_recorded_exercises()
        else:
            QMessageBox.warning(self, "Stop Recording Failed", msg)

    def start_playback(self):
        if self.drag_active:
            QMessageBox.warning(self, "Drag Mode Active", "Disable Drag Mode first")
            return
        filename = self.exercise_select.currentText()
        if filename == "Select a recorded exercise...":
            return
        reps = self.reps_input.value()
        success, msg = self.therapy.start_playback(filename, reps)
        if success:
            self.playback_running = True
            self.playback_btn.setText("Stop Playback")
            self.playback_btn.clicked.disconnect()
            self.playback_btn.clicked.connect(self.stop_playback)
            self.pause_btn.setEnabled(True)
            self.resume_btn.setEnabled(False)
            self.status_label.setText("Playing Back Recorded Exercise")
        else:
            QMessageBox.warning(self, "Playback Failed", msg)

    def stop_playback(self):
        self.therapy.stop_playback()
        self.playback_running = False
        self.playback_btn.setText("Start Playback")
        self.playback_btn.clicked.disconnect()
        self.playback_btn.clicked.connect(self.start_playback)
        self.pause_btn.setEnabled(False)
        self.resume_btn.setEnabled(False)
        self.status_label.setText("Ready")

    def pause_playback(self):
        success, msg = self.therapy.pause_playback()
        if success:
            self.pause_btn.setEnabled(False)
            self.resume_btn.setEnabled(True)
        else:
            QMessageBox.warning(self, "Pause Failed", msg)

    def resume_playback(self):
        success, msg = self.therapy.resume_playback()
        if success:
            self.pause_btn.setEnabled(True)
            self.resume_btn.setEnabled(False)
        else:
            QMessageBox.warning(self, "Resume Failed", msg)

    def toggle_drag_mode(self):
        success = self.therapy.custom_drag_teach_mode(enable=not self.drag_active)
        if success:
            self.drag_active = not self.drag_active
            self.drag_btn.setText("Drag Mode (Active)" if self.drag_active else "Drag Mode")
            self.status_label.setText("Drag Mode: Active" if self.drag_active else "Ready")
        else:
            QMessageBox.warning(self, "Drag Mode Failed", "Failed to toggle drag mode")

    def move_home(self):
        if self.playback_running or self.recording:
            QMessageBox.warning(self, "Busy", "Stop playback/recording before moving home")
            return
        try:
            vel_percent = round((HOME_DESIRED_SPEED / HOME_MAX_JOINT_SPEED) * 100.0, 3)
            ret = self.robot.MoveJ(
                joint_pos=HOME_TARGET_JOINTS,
                tool=0,
                user=0,
                desc_pos=[0.0] * 7,
                vel=vel_percent,
                acc=0.0,
                ovl=100.0,
                exaxis_pos=[0.0] * 4,
                blendT=-1.0,
                offset_flag=0,
                offset_pos=[0.0] * 6
            )
            if ret != 0:
                QMessageBox.warning(self, "Home Failed", f"MoveJ failed (code: {ret})")
            else:
                self.status_label.setText("Moving to Home...")
        except Exception as e:
            QMessageBox.warning(self, "Home Failed", str(e))


def main():
    app = QApplication(sys.argv)
    window = PassiveTherapyWindow()
    window.show()
    sys.exit(app.exec())


if __name__ == '__main__':
    main()
