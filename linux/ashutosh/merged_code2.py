# this code lets you move the robot with joint 2,3,4 in **DMP trained trajectory** 
# and then if you pasue it waits for 0.5 sec and then dmp takes over and completes the rest of the path 
# this code is created on 28th oct 2025
# =============================================================================
#  hybrid_adaptive_dmp.py  (CORRECTED: J1 LOCKED, J2=Fz, J3=J4=Fx)
#  --------------------------------------------------------------
#  • Joint 1: LOCKED
#  • Joint 2: Controlled by Fz
#  • Joint 3 & 4: Controlled by Fx (with sign inversion)
#  • Joint 5 & 6: LOCKED
#  • DMP takes over after 0.5s pause
#  • Force control constrained to DMP trajectory
#  • Stop at goal in both modes
# =============================================================================

import sys
sys.path.append('/home/um/fairino-python-sdk-main/linux/fairino')
import Robot
import time
import signal
import numpy as np
import pandas as pd
import os

# -------------------------------------------------------------------------
# 1. DMP CLASSES – UPDATED FOR TRAJECTORY GENERATION & REMAINING EXECUTION
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
        self.demo_traj = None

    def generate_trajectory(self, dt):
        if not self.trained:
            raise RuntimeError("Train DMPs first!")

        n_steps = int(self.dmps[0].tau / dt) + 1
        y = np.array([d.y0 for d in self.dmps])
        dy = np.zeros(self.n_joints)
        x = 1.0
        trajectory = [y.copy()]
        tau = self.dmps[0].tau

        for _ in range(n_steps - 1):
            ddy = np.zeros(self.n_joints)
            for i in range(self.n_joints):
                y[i], dy[i], ddy[i] = self.dmps[i].step(
                    y[i], dy[i], x, self.dmps[i].g, dt,
                    self.max_vel, self.max_accel
                )
            trajectory.append(y.copy())
            x = x - self.dmps[0].alpha_x * x * dt / tau
            if x < 0.01:
                break

        return np.array(trajectory)

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
        print(f"\n{'='*60}")
        print("STEP 1: LOADING & FILTERING DEMO")
        print(f"{'='*60}")
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

            pos_filt, vel_filt, acc_filt = self.filter_trajectory(
                pos_demo, dt, self.max_vel, self.max_accel
            )
            v_max = max(abs(v) for v in vel_filt)
            max_v_demo = max(max_v_demo, v_max)

            print(f"  Joint {j+1}: {original_start:.2f}° → {original_goal:.2f}° | Filtered MaxV: {v_max:.2f}°/s")

            self.training_data.append({
                'positions': pos_filt,
                'velocities': vel_filt,
                'accelerations': acc_filt,
                'original_start': original_start,
                'original_goal': original_goal
            })
            self.goal_offset[j] = original_goal - original_start

        print(f"\nAll joints filtered. Max velocity in filtered demo: {max_v_demo:.2f}°/s")

        print(f"\n{'='*60}")
        print("STEP 2: TRAINING DMPs")
        print(f"{'='*60}")

        for j in range(self.n_joints):
            data = self.training_data[j]
            self.dmps[j].learn_from_trajectory(
                data['positions'],
                data['velocities'],
                data['accelerations'],
                dt
            )
            self.dmps[j].y0 = data['original_start']
            self.dmps[j].g = data['original_goal']

            print(f"  Joint {j+1}: Start={self.dmps[j].y0:.2f}°, Goal={self.dmps[j].g:.2f}°, "
                  f"Offset={self.goal_offset[j]:.2f}°, Duration={self.dmps[j].tau:.2f}s")

        self.trained = True
        print(f"\n{'='*60}")
        print("ALL 6 DMPs TRAINED SUCCESSFULLY!")
        print(f"{'='*60}")

        # Generate demo trajectory
        print("\nGenerating reference trajectory from DMP...")
        self.demo_traj = self.generate_trajectory(dt)
        print(f"Trajectory generated: {len(self.demo_traj)} steps")

        print(f"\n{'='*60}\n")
        return dt

    def execute_remaining(self, dt, start_pose):
        if self.demo_traj is None:
            raise ValueError("No demo trajectory available. Train first.")

        dists = np.sum((self.demo_traj - np.array(start_pose))**2, axis=1)
        current_idx = np.argmin(dists)
        print(f"\nDMP RESUMING from index {current_idx} / {len(self.demo_traj)-1}")

        remaining_steps = len(self.demo_traj) - current_idx
        print(f"Remaining steps: {remaining_steps}, estimated time: {remaining_steps * dt:.2f}s")

        y = self.demo_traj[current_idx]
        for k in range(current_idx + 1, len(self.demo_traj)):
            target_y = self.demo_traj[k]
            loop_start = time.time()
            robot.ServoJ(list(target_y), [0]*6, int(dt*1000))
            elapsed = time.time() - loop_start
            if elapsed < dt:
                time.sleep(dt - elapsed)
            y = target_y

        print("DMP FINISHED → Goal reached. Returning to user-guided mode.\n")
        print(f"Final joint angles: "
              f"J1: {y[0]:6.2f}° | "
              f"J2: {y[1]:6.2f}° | "
              f"J3: {y[2]:6.2f}° | "
              f"J4: {y[3]:6.2f}° | "
              f"J5: {y[4]:6.2f}° | "
              f"J6: {y[5]:6.2f}°")

        return y.tolist()


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

GOAL_TOLERANCE = 1.0  # squared degrees total

# -------------------------------------------------------------------------
# 3. ROBOT INIT
# -------------------------------------------------------------------------
robot = Robot.RPC('192.168.58.2')
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
# 4. SETUP
# -------------------------------------------------------------------------
init_ft_sensor()
err, joint_pos = robot.GetActualJointPosDegree()
if err != 0: sys.exit(1)
calibrate_baseline_forces()

home_pos = joint_pos.copy()
desired_pos = joint_pos.copy()
velocity = [0.0]*6

# FIXED: J1 locked, J2-J4 active
free_joints = [2, 3, 4]  # Joint 2, 3, 4 (1-indexed)

# Load DMP
csv_file = input("\nEnter CSV filename for DMP demo: ").strip()
if not csv_file:
    print("No CSV → exiting.")
    sys.exit(0)

dmp = MultiJointDMP(n_joints=6, n_basis=50, max_vel=15.0, max_accel=1000.0)
dmp_dt = dmp.learn_from_csv(csv_file)

if robot.ServoMoveStart() != 0: sys.exit(1)
time.sleep(startup_delay)

# -------------------------------------------------------------------------
# 5. HYBRID CONTROL LOOP
# -------------------------------------------------------------------------
def hybrid_control_loop():
    global desired_pos, velocity, home_pos, baseline_forces

    rep_active = False
    stop_timer = None
    STOP_THRESHOLD = 0.5
    START_THRESHOLD = 0.5  # deg/s
    START_DIST_THRESHOLD = 24.0  # sum sq deg ~ 2 deg per joint across 6
    FORCE_PAUSE_THRESHOLD = 0.5  # N

    demo_traj = dmp.demo_traj
    n_steps = len(demo_traj)
    goal = demo_traj[-1]
    if n_steps < 2:
        print("Invalid trajectory length. Exiting.")
        return

    free_indices = [1, 2, 3]  # 0-indexed for J2, J3, J4

    while True:
        ft_data = robot.FT_GetForceTorqueRCS()
        if ft_data[0] != 0:
            time.sleep(dt)
            continue

        raw_forces = [
            ft_data[1][0],   # Fx
            -ft_data[1][1],  # Fy
            ft_data[1][2],   # Fz
            ft_data[1][3],   # Mx
            ft_data[1][4],   # My
            ft_data[1][5]    # Mz
        ]

        forces = [raw_forces[i] - baseline_forces[i] for i in range(6)] if baseline_forces else raw_forces

        for i in range(6):
            if abs(forces[i]) < 0.5:
                forces[i] = 0.0

        # Find current index on trajectory (using desired_pos as proxy for actual)
        dists = np.sum((demo_traj - np.array(desired_pos))**2, axis=1)
        current_idx = np.argmin(dists)

        # Compute local tangent
        if current_idx == 0:
            tangent = demo_traj[1] - demo_traj[0]
        elif current_idx == n_steps - 1:
            tangent = demo_traj[-1] - demo_traj[-2]
        else:
            tangent = demo_traj[current_idx + 1] - demo_traj[current_idx - 1]
        tangent_norm = np.linalg.norm(tangent)
        unit_tangent = np.zeros(6)
        if tangent_norm > 1e-6:
            unit_tangent = tangent / tangent_norm

        # Compute intended_delta for free joints (original impedance logic)
        intended_delta = np.zeros(6)

        # Joint 2 (index 1) -> Fz
        j = 1
        f = forces[2]
        if abs(f) < 1.0:
            home_pos[j] = desired_pos[j]
            spring = -K[j] * (desired_pos[j] - home_pos[j]) / force_to_deg
            acc = (spring - B[j] * velocity[j]) / M[j]
        else:
            acc = (f - B[j] * velocity[j]) / M[j]
        velocity[j] += acc * dt
        intended_delta[j] = velocity[j] * dt * force_to_deg

        # Joint 3 (index 2) -> -Fx
        j = 2
        f = forces[0]
        if abs(f) < 1.0:
            home_pos[j] = desired_pos[j]
            spring = -K[j] * (desired_pos[j] - home_pos[j]) / force_to_deg
            acc = (spring - B[j] * velocity[j]) / M[j]
        else:
            acc = (-f - B[j] * velocity[j]) / M[j]
        velocity[j] += acc * dt
        intended_delta[j] = velocity[j] * dt * force_to_deg

        # Joint 4 (index 3) -> -Fx
        j = 3
        f = forces[0]
        if abs(f) < 1.0:
            home_pos[j] = desired_pos[j]
            spring = -K[j] * (desired_pos[j] - home_pos[j]) / force_to_deg
            acc = (spring - B[j] * velocity[j]) / M[j]
        else:
            acc = (-f - B[j] * velocity[j]) / M[j]
        velocity[j] += acc * dt
        intended_delta[j] = velocity[j] * dt * force_to_deg

        # Project intended_delta onto trajectory tangent
        proj_length = np.dot(intended_delta, unit_tangent)
        projected_delta = proj_length * unit_tangent

        # Update desired_pos with projected movement
        desired_pos += projected_delta

        # Clamp to trajectory bounds (prevent going before start)
        dists_new = np.sum((demo_traj - desired_pos)**2, axis=1)
        new_idx = np.argmin(dists_new)
        if new_idx < current_idx and new_idx == 0:
            # Check if trying to go before start
            if np.dot((desired_pos - demo_traj[0]), unit_tangent) < 0:
                desired_pos = demo_traj[0].copy()
                new_idx = 0
                # Reset velocities
                for jj in free_indices:
                    velocity[jj] = 0.0

        # Update velocities to match projected movement (for damping consistency)
        for jj in free_indices:
            velocity[jj] = (projected_delta[jj] / force_to_deg) / dt

        # Lock J1, J5, J6
        for j in [0, 4, 5]:
            desired_pos[j] = home_pos[j]
            velocity[j] = 0.0

        # Safety check
        safe = all(is_within_safety_limits(i+1, desired_pos[i]) for i in range(6))
        if not safe:
            desired_pos = home_pos[:]
            velocity = [0.0]*6
            time.sleep(dt)
            continue

        # Send command
        err = robot.ServoJ(joint_pos=desired_pos, axisPos=[0]*6)
        if err != 0:
            print("ServoJ error:", err)

        # Effective speed along path
        cur_speed = abs(proj_length / dt)

        # === CHECK FOR GOAL REACHED IN FORCE MODE ===
        goal_dist = np.sum((np.array(desired_pos) - goal)**2)
        if rep_active and goal_dist < GOAL_TOLERANCE:
            print(f"\nGoal reached in force mode! (dist²={goal_dist:.2f}) Stopping.")
            velocity = [0.0]*6
            rep_active = False
            continue

        # === REP START ===
        if not rep_active and cur_speed > START_THRESHOLD:
            err, cur = robot.GetActualJointPosDegree(flag=1)
            if err == 0:
                start_dists = np.sum((demo_traj[0] - cur)**2)
                if start_dists < START_DIST_THRESHOLD:
                    rep_active = True
                    print(f"\nNEW REP STARTED – Near trajectory start (dist²={start_dists:.1f}). Current idx: {current_idx}")
                else:
                    print(f"Movement detected but far from start (dist²={start_dists:.1f}), ignoring.")

        # === PAUSE → DMP ===
        pause_condition = (abs(forces[0]) < FORCE_PAUSE_THRESHOLD) and (abs(forces[2]) < FORCE_PAUSE_THRESHOLD)
        if rep_active and pause_condition:
            if stop_timer is None:
                stop_timer = time.time()
            elif time.time() - stop_timer >= STOP_THRESHOLD:
                print("\nUser paused ≥ 0.5s → DMP TAKES OVER")
                err, now = robot.GetActualJointPosDegree(flag=1)
                final_pos = dmp.execute_remaining(dmp_dt, now) if err == 0 else desired_pos[:]
                desired_pos = final_pos[:]
                velocity = [0.0]*6
                home_pos = desired_pos[:]
                rep_active = False
                stop_timer = None
                continue
        else:
            stop_timer = None

        # --- LOG ---
        print(f"Rep:{rep_active} | Spd:{cur_speed:5.2f}°/s | "
              f"Idx:{current_idx:3d}/{n_steps-1} | "
              f"J2:{desired_pos[1]:6.2f}° | J3:{desired_pos[2]:6.2f}° | J4:{desired_pos[3]:6.2f}° | "
              f"Fz:{forces[2]:5.2f}N | Fx:{forces[0]:5.2f}N")

        time.sleep(dt)

hybrid_control_loop()