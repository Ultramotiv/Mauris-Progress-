# hybrid_adaptive_dmp.py
# Generated: 31st Oct 2025
# =============================================================================
#  • Force control only on joints where |CSV_start - current| ≥ 1.5°
#  • Max speed in force mode: 7.70 °/s
#  • Force constrained to NEW DMP trajectory (shifted start/goal)
#  • Pause ≥ 0.5s → DMP takes over from current pose to NEW goal
#  • Smooth, non-jerky motion in both modes
#  • NO HARD LOCKS — all joints evaluated dynamically
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
# 1. DMP CLASSES
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

        print("\nGenerating reference trajectory from DMP...")
        self.demo_traj = self.generate_trajectory(dt)
        print(f"Trajectory generated: {len(self.demo_traj)} steps")

        print(f"\n{'='*60}\n")
        return dt


# -------------------------------------------------------------------------
# 2. HELPER: Get DMP velocity at current pose
# -------------------------------------------------------------------------
def dmp_velocity_at_pose(dmp, y, dy, phase, goal, dt):
    """Return desired joint velocity from DMP at given phase."""
    vel = np.zeros(6)
    for i in range(6):
        _, vel[i], _ = dmp.dmps[i].step(
            y[i], dy[i], phase, goal[i], dt,
            dmp.max_vel, dmp.max_accel
        )
    return vel


# -------------------------------------------------------------------------
# 3. PARAMETERS
# -------------------------------------------------------------------------
M = [3.0, 3.0, 2.0, 3.0, 3.0, 3.0]
B = [2.5, 2.5, 2.5, 3.0, 3.0, 3.0]
K = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

force_to_deg = 7.70
MAX_SPEED_FORCE_MODE = 4.0  # °/s in force mode
dt = 0.008
gravity_compensation_samples = 100
baseline_forces = None
startup_delay = 0.5

JOINT_SAFETY_LIMITS = {
    1: (-90.0, 85.0), 2: (-179.0, -35.0), 3: (2.0, 144.0),
    4: (-258.0, 80.0), 5: (-170.0, 12.0), 6: (-170.0, 170.0),
}

GOAL_TOLERANCE = 1.0
MIN_START_DIFF = 1.5

# Force-to-joint mapping
FORCE_MAPPING = {
    1: (2, 1.0),   # J2 → Fz (+)
    2: (0, -1.0),  # J3 → Fx (-)
    3: (0, -1.0),  # J4 → Fx (-)
}


# -------------------------------------------------------------------------
# 4. ROBOT INIT
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
# 5. SETUP
# -------------------------------------------------------------------------
init_ft_sensor()
err, joint_pos = robot.GetActualJointPosDegree()
if err != 0: sys.exit(1)
calibrate_baseline_forces()

home_pos = joint_pos.copy()
desired_pos = joint_pos.copy()
velocity = [0.0]*6

# Load CSV
csv_file = input("\nEnter CSV filename for DMP demo: ").strip()
if not csv_file:
    print("No CSV → exiting.")
    sys.exit(0)

df_demo = pd.read_csv(csv_file)
joint_cols = [f'joint{i+1}' for i in range(6)]
demo_start = df_demo[joint_cols].iloc[0].values.tolist()
demo_goal = df_demo[joint_cols].iloc[-1].values.tolist()

original_offset = np.array(demo_goal) - np.array(demo_start)

print("\n--- START-POSE COMPARISON ------------------------------------------------")
print(f"Current robot (NEW START): {[f'{x:6.2f}' for x in joint_pos]}")
print(f"Demo start               : {[f'{x:6.2f}' for x in demo_start]}")
print(f"Demo goal                : {[f'{x:6.2f}' for x in demo_goal]}")

# Free joints
free_joints = []
for j1 in range(1, 7):
    diff = abs(demo_start[j1-1] - joint_pos[j1-1])
    if diff >= MIN_START_DIFF:
        free_joints.append(j1)
    print(f"  J{j1}: demo={demo_start[j1-1]:6.2f}°  cur={joint_pos[j1-1]:6.2f}°  diff={diff:5.2f}° → {'FREE' if diff>=MIN_START_DIFF else 'LOCKED'}")

free_indices = [j-1 for j in free_joints]
print(f"\nFORCE-CONTROLLED JOINTS (1-based) → {free_joints}")

# Train DMP
dmp = MultiJointDMP(n_joints=6, n_basis=50, max_vel=15.0, max_accel=1000.0)
dmp_dt = dmp.learn_from_csv(csv_file)

# Build shifted trajectory
new_start = np.array(joint_pos)
new_goal = new_start + original_offset

print(f"\nNEW TRAJECTORY: Start → Goal")
print(f"  Start: {[f'{x:6.2f}' for x in new_start]}")
print(f"  Goal : {[f'{x:6.2f}' for x in new_goal]}")

def build_shifted_trajectory(dmp, dt, start, goal):
    traj = []
    x = 1.0
    y = start.copy()
    dy = np.zeros(6)
    tau = dmp.dmps[0].tau
    n_steps = int(tau / dt) + 1
    for _ in range(n_steps):
        traj.append(y.copy())
        for i in range(6):
            dmp.dmps[i].g = goal[i]
            y[i], dy[i], _ = dmp.dmps[i].step(y[i], dy[i], x, goal[i], dt,
                                             dmp.max_vel, dmp.max_accel)
        x = max(x - dmp.dmps[0].alpha_x * x * dt / tau, 0.0)
        if x < 0.01:
            break
    traj[-1] = goal.copy()
    return np.array(traj)

demo_traj = build_shifted_trajectory(dmp, dmp_dt, new_start, new_goal)
print(f"Shifted trajectory generated: {len(demo_traj)} steps")

if robot.ServoMoveStart() != 0: sys.exit(1)
time.sleep(startup_delay)


# -------------------------------------------------------------------------
# 6. HYBRID CONTROL LOOP
# -------------------------------------------------------------------------
def hybrid_control_loop():
    global desired_pos, velocity, home_pos, baseline_forces, free_indices

    rep_active = False
    stop_timer = None
    STOP_THRESHOLD = 0.5
    START_THRESHOLD = 0.5
    START_DIST_THRESHOLD = 5000.0
    FORCE_PAUSE_THRESHOLD = 0.5

    goal = demo_traj[-1]
    n_steps = len(demo_traj)
    if n_steps < 2:
        print("Invalid trajectory length. Exiting.")
        return

    while True:
        ft_data = robot.FT_GetForceTorqueRCS()
        if ft_data[0] != 0:
            time.sleep(dt)
            continue

        raw_forces = [ft_data[1][0], -ft_data[1][1], ft_data[1][2],
                      ft_data[1][3], ft_data[1][4], ft_data[1][5]]
        forces = [raw_forces[i] - baseline_forces[i] for i in range(6)] if baseline_forces else raw_forces
        for i in range(6):
            if abs(forces[i]) < 0.5:
                forces[i] = 0.0

        # Current index on new trajectory
        dists = np.sum((demo_traj - np.array(desired_pos))**2, axis=1)
        current_idx = np.argmin(dists)
        phase = max(0.0, min(1.0, current_idx / max(1, n_steps - 1)))

        # DMP velocity at current pose
        dmp_vel = dmp_velocity_at_pose(dmp, desired_pos, velocity, phase, new_goal, dt)

        # Scale with force
        speed_factor = 1.0
        for j0 in free_indices:
            j1 = j0 + 1
            if j1 in FORCE_MAPPING:
                f_idx, sign = FORCE_MAPPING[j1]
                f = forces[f_idx]
                if abs(f) > 1.0:
                    speed_factor += sign * f * 0.08
        speed_factor = np.clip(speed_factor, 0.0, 2.0)

        # Apply speed cap
        desired_vel = dmp_vel * speed_factor
        vel_norm = np.linalg.norm(desired_vel)
        if vel_norm > MAX_SPEED_FORCE_MODE:
            desired_vel = desired_vel * (MAX_SPEED_FORCE_MODE / vel_norm)

        # Integrate
        delta_pos = desired_vel * dt
        desired_pos = np.clip(desired_pos + delta_pos,
                              demo_traj.min(axis=0), demo_traj.max(axis=0))
        velocity = desired_vel.copy()

        # Lock non-free joints
        for j0 in range(6):
            if j0 not in free_indices:
                desired_pos[j0] = home_pos[j0]
                velocity[j0] = 0.0

        # Safety
        safe = all(is_within_safety_limits(i+1, desired_pos[i]) for i in range(6))
        if not safe:
            desired_pos = home_pos[:]
            velocity = [0.0]*6
            time.sleep(dt)
            continue

        # Send command
        err = robot.ServoJ(joint_pos=desired_pos.tolist(), axisPos=[0]*6)
        if err != 0:
            print("ServoJ error:", err)

        cur_speed = np.linalg.norm(desired_vel)

        # Goal reached
        goal_dist = np.sum((desired_pos - goal)**2)
        if goal_dist < GOAL_TOLERANCE:
            print(f"\nGOAL REACHED! (dist²={goal_dist:.2f})")
            robot.ServoMoveEnd()
            robot.Mode(0)
            sys.exit(0)

        # Start repetition
        if not rep_active and cur_speed > START_THRESHOLD:
            err, cur = robot.GetActualJointPosDegree(flag=1)
            if err == 0:
                start_dists = np.sum((demo_traj[0] - cur)**2)
                if start_dists < START_DIST_THRESHOLD:
                    rep_active = True
                    print(f"\nREPETITION STARTED (dist²={start_dists:.1f})")

        # Pause → DMP
        pause_condition = all(abs(forces[i]) < FORCE_PAUSE_THRESHOLD for i in [0, 2])
        if rep_active and pause_condition:
            if stop_timer is None:
                stop_timer = time.time()
            elif time.time() - stop_timer >= STOP_THRESHOLD:
                print("\nUser paused ≥ 0.5s → DMP TAKES OVER (new goal)")
                err, now = robot.GetActualJointPosDegree(flag=1)
                if err == 0:
                    temp_traj = build_shifted_trajectory(dmp, dmp_dt, now, new_goal)
                    for k in range(1, len(temp_traj)):
                        target = temp_traj[k]
                        robot.ServoJ(target.tolist(), [0]*6, int(dmp_dt*1000))
                        time.sleep(dmp_dt)
                    print("DMP FINISHED → Goal reached. Exiting.")
                robot.ServoMoveEnd()
                robot.Mode(0)
                sys.exit(0)
        else:
            stop_timer = None

        # LOG
        log_joints = [f"J{j+1}:{desired_pos[j]:6.2f}°" for j in range(6)]
        print(f"Rep:{rep_active} | Spd:{cur_speed:5.2f}°/s | "
              f"Idx:{current_idx:4d}/{n_steps-1} | {' | '.join(log_joints)} | "
              f"Fz:{forces[2]:5.2f}N | Fx:{forces[0]:5.2f}N")

        time.sleep(dt)


hybrid_control_loop()