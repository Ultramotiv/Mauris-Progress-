# DMP+Force_Code3_FINAL.py
# Created on 29th Oct 2025
# FULLY ADAPTIVE DMP + FORCE-GUIDED + IN-MEMORY TRAJECTORY
# • Learns SHAPE from CSV demo
# • new_start = current_pose, new_goal = current + (demo_goal-demo_start)
# • Force control → pause 0.5 s → DMP finishes the path
# • ALL 6 JOINTS ARE FREE (NO LOCKING)
# • PRINTS FINAL POSE & EXITS CLEANLY
# =============================================================================

import sys
sys.path.append('/home/um/fairino-python-sdk-main/linux/fairino')
import Robot
import time
import signal
import numpy as np
import pandas as pd
import os  # ← ADDED FOR CLEAN EXIT

robot = Robot.RPC('192.168.58.2')


# -------------------------------------------------------------------------
# 1. DMP CLASSES – WITH ADAPTIVE TRAJECTORY GENERATION
# -------------------------------------------------------------------------
class DMP1D:
    def __init__(self, n_basis=50, alpha_y=25.0, beta_y=6.25, alpha_x=1.0):
        self.n_basis = n_basis
        self.alpha_y = alpha_y
        self.beta_y = beta_y
        self.alpha_x = alpha_x
        self.c = np.exp(-self.alpha_x * np.linspace(0, 1, n_basis))
        self.h = np.ones(n_basis) * n_basis / (self.c ** 2)
        self.w = np.zeros(n_basis)
        self.y0 = 0.0
        self.g = 0.0
        self.tau = 1.0

    def basis_functions(self, x):
        psi = np.exp(-self.h * (x - self.c) ** 2)
        return psi / (np.sum(psi) + 1e-10)

    def forcing_term(self, x):
        psi = self.basis_functions(x)
        return np.dot(psi, self.w) * x * (self.g - self.y0)

    def learn_from_trajectory(self, positions, velocities, accelerations, dt):
        positions = np.array(positions, dtype=float)
        velocities = np.array(velocities, dtype=float)
        accelerations = np.array(accelerations, dtype=float)
        self.y0 = positions[0]
        self.g = positions[-1]
        self.tau = len(positions) * dt
        n_steps = len(positions)
        x = np.exp(-self.alpha_x * np.linspace(0, 1, n_steps))
        f_target = (self.tau**2 * accelerations -
                    self.alpha_y * (self.beta_y * (self.g - positions) - self.tau * velocities))
        X = np.zeros((n_steps, self.n_basis))
        for i in range(n_steps):
            psi = self.basis_functions(x[i])
            X[i, :] = psi * x[i] * (self.g - self.y0)
        lambda_reg = 1e-8
        A = X.T @ X + lambda_reg * np.eye(X.shape[1])
        b = X.T @ f_target
        self.w = np.linalg.solve(A, b)

    def step(self, y, dy, x, g_current, dt, max_vel=15.0, max_accel=1000.0):
        self.g = g_current
        f = self.forcing_term(x)
        ddy = (self.alpha_y * (self.beta_y * (self.g - y) - self.tau * dy) + f) / (self.tau ** 2)
        ddy = np.clip(ddy, -max_accel, max_accel)
        dy_new = dy + ddy * dt
        dy_new = np.clip(dy_new, -max_vel, max_vel)
        y_new = y + dy_new * dt
        return y_new, dy_new, ddy


class MultiJointDMP:
    def __init__(self, n_joints=6, n_basis=50, max_vel=15.0, max_accel=1000.0):
        self.n_joints = n_joints
        self.max_vel = max_vel
        self.max_accel = max_accel
        self.dmps = [DMP1D(n_basis=n_basis) for _ in range(n_joints)]
        self.trained = False
        self.goal_offset = np.zeros(n_joints)
        self.planned_trajectory = None

    def filter_trajectory(self, positions, dt, max_vel, max_accel):
        pos = np.array(positions, dtype=float)
        vel = np.gradient(pos, dt)
        acc = np.gradient(vel, dt)
        acc = np.clip(acc, -max_accel, max_accel)
        vel = np.zeros_like(pos)
        vel[1:] = np.cumsum(acc[:-1]) * dt
        vel = np.clip(vel, -max_vel, max_vel)
        pos = np.zeros_like(positions)
        pos[0] = positions[0]
        pos[1:] = positions[0] + np.cumsum(vel[:-1]) * dt
        return pos.tolist(), vel.tolist(), acc.tolist()

    def learn_from_csv(self, csv_filename):
        print(f"\n{'=' * 60}")
        print("STEP 1: LOADING & FILTERING DEMO")
        print(f"{'=' * 60}")
        print(f"File: {csv_filename}")
        print(f"Max Velocity: {self.max_vel}°/s | Max Accel: {self.max_accel}°/s²")

        if not os.path.exists(csv_filename):
            raise FileNotFoundError(f"CSV not found: {csv_filename}")

        df = pd.read_csv(csv_filename)
        timestamps = df['timestamp'].values
        joint_cols = [f'joint{i+1}' for i in range(self.n_joints)]
        joints = df[joint_cols].values
        dt = np.mean(np.diff(timestamps))

        print(f"  - Samples: {len(df)}")
        print(f"  - dt: {dt*1000:.2f} ms")
        print(f"  - Duration: {timestamps[-1] - timestamps[0]:.2f}s")

        self.training_data = []
        max_v_demo = 0.0

        for j in range(self.n_joints):
            pos_demo = joints[:, j]
            original_start = pos_demo[0]
            original_goal = pos_demo[-1]
            pos_filt, vel_filt, acc_filt = self.filter_trajectory(pos_demo, dt,
                                                                 self.max_vel, self.max_accel)
            v_max = max(abs(v) for v in vel_filt)
            max_v_demo = max(max_v_demo, v_max)
            print(f"  Joint {j+1}: {original_start:.2f}° → {original_goal:.2f}° | Filtered MaxV: {v_max:.2f}°/s")
            self.training_data.append({
                'positions': pos_filt, 'velocities': vel_filt, 'accelerations': acc_filt,
                'original_start': original_start, 'original_goal': original_goal
            })
            self.goal_offset[j] = original_goal - original_start

        print(f"\nAll joints filtered. Max velocity in filtered demo: {max_v_demo:.2f}°/s")

        print(f"\n{'=' * 60}")
        print("STEP 2: TRAINING DMPs (SHAPE ONLY)")
        print(f"{'=' * 60}")

        for j in range(self.n_joints):
            data = self.training_data[j]
            self.dmps[j].learn_from_trajectory(data['positions'],
                                               data['velocities'],
                                               data['accelerations'], dt)
            print(f"  Joint {j+1}: Offset={self.goal_offset[j]:.2f}°, Duration={self.dmps[j].tau:.2f}s")

        # ADJUSTMENT: Note (print) the weights after training (norm for summary, as full weights are long)
        print("\nNoting DMP weights (summary: norm of weights vector per joint):")
        for j in range(self.n_joints):
            w_norm = np.linalg.norm(self.dmps[j].w)
            print(f"  Joint {j+1}: Weight norm = {w_norm:.2f} | Full weights: {self.dmps[j].w}")

        self.trained = True
        print(f"\n{'=' * 60}")
        print("DMP SHAPE TRAINED! Ready for new start/goal.")
        print(f"{'=' * 60}\n")
        return dt

    def generate_trajectory(self, start_pos, dt):
        if not self.trained:
            raise RuntimeError("Train DMPs first!")

        y = np.array(start_pos, dtype=float)
        dy = np.zeros(self.n_joints)
        goals = y + self.goal_offset

        for i, dmp in enumerate(self.dmps):
            dmp.y0 = y[i]
            dmp.g = goals[i]

        tau = self.dmps[0].tau
        x = 1.0
        pos_list = [y.copy()]

        while x > 0.01:
            y_new = np.zeros(self.n_joints)
            dy_new = np.zeros(self.n_joints)
            ddy = np.zeros(self.n_joints)
            for i in range(self.n_joints):
                y_new[i], dy_new[i], ddy[i] = self.dmps[i].step(
                    y[i], dy[i], x, goals[i], dt, self.max_vel, self.max_accel
                )
            pos_list.append(y_new.copy())
            x = x - self.dmps[0].alpha_x * x * dt / tau
            y, dy = y_new, dy_new
            if np.all(np.abs(y_new - goals) < 0.1):
                break

        trajectory = np.array(pos_list)
        print(f"Generated {len(trajectory)} points | Duration: {len(trajectory)*dt:.3f}s")
        print(f"  Start: {trajectory[0]}")
        print(f"  Goal:  {trajectory[-1]}")
        return trajectory

    # ← MODIFIED: Print final pose + clean exit
    def execute_remaining(self, dt, start_pose, trajectory):
        dists = np.sum((trajectory - np.array(start_pose))**2, axis=1)
        current_idx = np.argmin(dists)
        print(f"\nDMP RESUMING from index {current_idx} / {len(trajectory)-1}")

        for k in range(current_idx, len(trajectory)):
            target = trajectory[k]
            loop_start = time.time()
            robot.ServoJ(list(target), [0]*6, int(dt*1000))
            elapsed = time.time() - loop_start
            if elapsed < dt:
                time.sleep(dt - elapsed)

        final_pos = trajectory[-1].tolist()
        print("\n" + "="*50)
        print("DMP FINISHED → Goal Reached!")
        print(f"Final Joint Positions (deg): {final_pos}")
        print("="*50 + "\n")

        # ← CLEAN SHUTDOWN & EXIT
        robot.ServoMoveEnd()
        robot.Mode(0)
        print("Robot stopped. Servo mode ended. Exiting script...")
        os._exit(0)  # Hard exit
        return final_pos  # (never reached, but for completeness)


# -------------------------------------------------------------------------
# 2. PARAMETERS
# -------------------------------------------------------------------------
M = [3.0, 3.0, 2.0, 3.0, 3.0, 3.0]
B = [2.5, 2.5, 2.5, 3.0, 3.0, 3.0]
K = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
force_to_deg = 7.70
dt = 0.008
gravity_compensation_samples = 100
baseline_forces = None
startup_delay = 0.5

JOINT_SAFETY_LIMITS = {
    1: (-90.0, 85.0), 2: (-179.0, -35.0), 3: (2.0, 144.0),
    4: (-258.0, 80.0), 5: (-170.0, 12.0), 6: (-170.0, 170.0),
}
GOAL_TOLERANCE = 1.0

LOCKED_JOINTS = []


# -------------------------------------------------------------------------
# 3. ROBOT INIT
# -------------------------------------------------------------------------
print("Robot connected.")

def init_ft_sensor():
    company = 24; device = 0
    robot.FT_SetConfig(company, device)
    robot.FT_Activate(0); time.sleep(0.5)
    robot.FT_Activate(1); time.sleep(0.5)
    robot.SetLoadWeight(0, 0.0)
    robot.SetLoadCoord(0.0, 0.0, 0.0)
    robot.FT_SetZero(0); time.sleep(0.5)
    robot.FT_SetZero(1); time.sleep(0.5)
    print("FT Sensor initialized.")

def calibrate_baseline_forces():
    global baseline_forces
    print("Calibrating baseline forces...")
    samples = []
    for _ in range(gravity_compensation_samples):
        ft = robot.FT_GetForceTorqueRCS()
        if ft[0] == 0:
            forces = [ft[1][0], -ft[1][1], ft[1][2], ft[1][3], ft[1][4], ft[1][5]]
            samples.append(forces)
        time.sleep(0.01)
    if samples:
        baseline_forces = [sum(col)/len(col) for col in zip(*samples)]
        print("Baseline:", baseline_forces)
    else:
        baseline_forces = [0.0]*6

def is_within_safety_limits(joint_idx, angle):
    if joint_idx in JOINT_SAFETY_LIMITS:
        mn, mx = JOINT_SAFETY_LIMITS[joint_idx]
        return mn <= angle <= mx
    return True

def shutdown(sig, frame):
    robot.ServoMoveEnd()
    robot.Mode(0)
    print("\nStopped. Exiting.")
    sys.exit(0)
signal.signal(signal.SIGINT, shutdown)


# -------------------------------------------------------------------------
# 4. SETUP & ADAPTIVE TRAJECTORY GENERATION
# -------------------------------------------------------------------------
init_ft_sensor()
err, current_pose = robot.GetActualJointPosDegree(flag=1)
if err != 0:
    print("Failed to get current pose. Exiting.")
    sys.exit(1)
calibrate_baseline_forces()

csv_file = input("\nEnter CSV filename for DMP demo: ").strip()
if not csv_file:
    print("No CSV → exiting.")
    sys.exit(0)

dmp = MultiJointDMP(n_joints=6, n_basis=50, max_vel=15.0, max_accel=1000.0)
dmp_dt = dmp.learn_from_csv(csv_file)

print(f"\n{'=' * 60}")
print("GENERATING ADAPTIVE TRAJECTORY FROM CURRENT POSE")
print(f"{'=' * 60}")
planned_trajectory = dmp.generate_trajectory(current_pose, dmp_dt)
print(f"Trajectory stored in 'planned_trajectory' ({len(planned_trajectory)} points)")
print(f"{'=' * 60}\n")

if robot.ServoMoveStart() != 0:
    print("Failed to start servo mode.")
    sys.exit(1)
time.sleep(startup_delay)


# -------------------------------------------------------------------------
# 5. HYBRID CONTROL LOOP – ALL 6 JOINTS FREE
# -------------------------------------------------------------------------
def hybrid_control_loop():
    global desired_pos, velocity
    desired_pos = current_pose.copy()
    velocity = [0.0]*6
    home_pos = current_pose.copy()

    rep_active = False
    stop_timer = None
    STOP_THRESHOLD = 0.5
    START_THRESHOLD = 0.5
    START_DIST_THRESHOLD = 24.0
    FORCE_PAUSE_THRESHOLD = 0.5

    trajectory = planned_trajectory
    n_steps = len(trajectory)
    goal = trajectory[-1]

    free_indices = list(range(6))

    print("HYBRID CONTROL STARTED. Push with force to move along trajectory.")
    print("Remove force for 0.5 s → DMP completes path.\n")

    while True:
        # ----------------------------------------------------------
        # 1. READ FT SENSOR
        # ----------------------------------------------------------
        ft_data = robot.FT_GetForceTorqueRCS()
        if ft_data[0] != 0:
            time.sleep(dt)
            continue

        raw_forces = [ft_data[1][0], -ft_data[1][1], ft_data[1][2],
                      ft_data[1][3], ft_data[1][4], ft_data[1][5]]
        forces = [raw_forces[i] - baseline_forces[i] for i in range(6)] \
                 if baseline_forces else raw_forces

        for i in range(6):
            if abs(forces[i]) < 0.5:
                forces[i] = 0.0

        # ----------------------------------------------------------
        # 2. CURRENT INDEX & TANGENT
        # ----------------------------------------------------------
        dists = np.sum((trajectory - np.array(desired_pos))**2, axis=1)
        current_idx = np.argmin(dists)

        if current_idx == 0:
            tangent = trajectory[1] - trajectory[0]
        elif current_idx == n_steps - 1:
            tangent = trajectory[-1] - trajectory[-2]
        else:
            tangent = trajectory[current_idx + 1] - trajectory[current_idx - 1]
        tangent_norm = np.linalg.norm(tangent)
        unit_tangent = tangent / tangent_norm if tangent_norm > 1e-6 else np.zeros(6)

        intended_delta = np.zeros(6)

        # ----------------------------------------------------------
        # 3. FORCE → ACCELERATION FOR ALL 6 JOINTS
        # ----------------------------------------------------------
        force_map = [
            (0, forces[0]), (1, forces[2]), (2, -forces[0]),
            (3, -forces[0]), (4, forces[1]), (5, forces[2])
        ]

        for j, f in force_map:
            if abs(f) < 1.0:
                home_pos[j] = desired_pos[j]
                spring = -K[j] * (desired_pos[j] - home_pos[j]) / force_to_deg
                acc = (spring - B[j] * velocity[j]) / M[j]
            else:
                acc = (f - B[j] * velocity[j]) / M[j]
            velocity[j] += acc * dt
            intended_delta[j] = velocity[j] * dt * force_to_deg

        # ----------------------------------------------------------
        # 4. PROJECT ONTO TRAJECTORY TANGENT
        # ----------------------------------------------------------
        proj_length = np.dot(intended_delta, unit_tangent)
        projected_delta = proj_length * unit_tangent
        desired_pos += projected_delta

        # ----------------------------------------------------------
        # 5. CLAMP TO START
        # ----------------------------------------------------------
        dists_new = np.sum((trajectory - desired_pos)**2, axis=1)
        new_idx = np.argmin(dists_new)
        if new_idx < current_idx and new_idx == 0:
            if np.dot((desired_pos - trajectory[0]), unit_tangent) < 0:
                desired_pos = trajectory[0].copy()
                for jj in free_indices:
                    velocity[jj] = 0.0

        for jj in free_indices:
            velocity[jj] = (projected_delta[jj] / force_to_deg) / dt

        # ----------------------------------------------------------
        # 6. SAFETY CHECK
        # ----------------------------------------------------------
        safe = all(is_within_safety_limits(i+1, desired_pos[i]) for i in range(6))
        if not safe:
            desired_pos = home_pos[:]
            velocity = [0.0]*6
            time.sleep(dt)
            continue

        robot.ServoJ(list(desired_pos), [0]*6, int(dt*1000))

        cur_speed = abs(proj_length / dt)

        # ----------------------------------------------------------
        # 8. START REPRODUCTION
        # ----------------------------------------------------------
        if not rep_active and cur_speed > START_THRESHOLD:
            err, cur = robot.GetActualJointPosDegree(flag=1)
            if err == 0:
                start_dists = np.sum((trajectory[0] - cur) ** 2)
                if start_dists < START_DIST_THRESHOLD:
                    rep_active = True
                    print(f"\nREP STARTED (dist²={start_dists:.1f})")
                else:
                    print(f"Movement but far from start (dist²={start_dists:.1f})")

        # ----------------------------------------------------------
        # 9. PAUSE → DMP TAKE-OVER
        # ----------------------------------------------------------
        pause_condition = all(abs(forces[i]) < FORCE_PAUSE_THRESHOLD for i in [0, 2])

        if rep_active:
            if pause_condition:
                if stop_timer is None:
                    stop_timer = time.time()
                elif time.time() - stop_timer >= STOP_THRESHOLD:
                    print("\nUser paused ≥ 0.5 s → DMP TAKES OVER")
                    err, now = robot.GetActualJointPosDegree(flag=1)
                    start_pose_for_dmp = now if err == 0 else desired_pos
                    final_pos = dmp.execute_remaining(dmp_dt, start_pose_for_dmp, trajectory)
                    desired_pos = final_pos[:]
                    velocity = [0.0] * 6
                    home_pos = desired_pos[:]
                    rep_active = False
                    stop_timer = None
            else:
                stop_timer = None

        # ----------------------------------------------------------
        # 10. GOAL REACHED → PRINT & EXIT (even under force)
        # ----------------------------------------------------------
        goal_dist = np.sum((np.array(desired_pos) - goal) ** 2)
        if goal_dist < GOAL_TOLERANCE:
            print(f"\nGOAL REACHED (dist²={goal_dist:.2f})")
            print(f"Final Joint Positions (deg): {desired_pos}")
            print("="*50 + "\n")
            robot.ServoMoveEnd()
            robot.Mode(0)
            print("Robot stopped. Exiting script...")
            os._exit(0)

        # ----------------------------------------------------------
        # 11. LOG
        # ----------------------------------------------------------
        print(f"Rep:{rep_active} | Spd:{cur_speed:5.2f}°/s | Idx:{current_idx:3d}/{n_steps-1} | "
              f"J1:{desired_pos[0]:6.2f}° | J2:{desired_pos[1]:6.2f}° | J3:{desired_pos[2]:6.2f}° | "
              f"J4:{desired_pos[3]:6.2f}° | J5:{desired_pos[4]:6.2f}° | J6:{desired_pos[5]:6.2f}° | "
              f"Fx:{forces[0]:5.2f}N | Fz:{forces[2]:5.2f}N", end='\r')

        time.sleep(dt)


# -------------------------------------------------------------------------
# 6. RUN
# -------------------------------------------------------------------------
try:
    hybrid_control_loop()
except KeyboardInterrupt:
    shutdown(None, None)