# this code is generated on 28th Oct 2025
# new_dmp_trial6.py
# FINAL FIXED VERSION – START FROM CURRENT POSE, NEW GOAL = CURRENT + DEMO_OFFSET
# -------------------------------------------------------------------------
#   • Learns the *shape* from a CSV demonstration (filtered + safe limits)
#   • At execution time:
#        – reads the *current* joint angles → new y0 for every DMP
#        – computes a new goal = current + (demo_goal – demo_start)
#        – replays the **same** velocity/acceleration profile
#   • No overshoot, stops exactly at the **new** goal
# -------------------------------------------------------------------------

import sys
sys.path.append('/home/um/fairino-python-sdk-main/linux/fairino')
import Robot
import time
import numpy as np
import pandas as pd
import os

# Connect to robot
robot = Robot.RPC('192.168.58.2')


class DMP1D:
    def __init__(self, n_basis=50, alpha_y=25.0, beta_y=6.25, alpha_x=1.0):
        self.n_basis = n_basis
        self.alpha_y = alpha_y
        self.beta_y = beta_y
        self.alpha_x = alpha_x

        self.c = np.exp(-self.alpha_x * np.linspace(0, 1, n_basis))
        self.h = np.ones(n_basis) * n_basis / (self.c ** 2)

        self.w = np.zeros(n_basis)
        self.y0 = 0.0          # start (will be overwritten at runtime)
        self.g = 0.0           # goal (will be overwritten at runtime)
        self.tau = 1.0

    def basis_functions(self, x):
        psi = np.exp(-self.h * (x - self.c) ** 2)
        return psi / (np.sum(psi) + 1e-10)

    def forcing_term(self, x):
        psi = self.basis_functions(x)
        return np.dot(psi, self.w) * x * (self.g - self.y0)

    def learn_from_trajectory(self, positions, velocities, accelerations, dt):
        """Learn weights from filtered trajectory"""
        positions = np.array(positions, dtype=float)
        velocities = np.array(velocities, dtype=float)
        accelerations = np.array(accelerations, dtype=float)

        self.y0 = positions[0]                # demo start (will be overridden later)
        self.g = positions[-1]                # demo goal  (will be overridden later)
        self.tau = len(positions) * dt

        n_steps = len(positions)
        x = np.exp(-self.alpha_x * np.linspace(0, 1, n_steps))

        # Target forcing term
        f_target = (self.tau**2 * accelerations -
                    self.alpha_y * (self.beta_y * (self.g - positions) - self.tau * velocities))

        # Regression matrix
        X = np.zeros((n_steps, self.n_basis))
        for i in range(n_steps):
            psi = self.basis_functions(x[i])
            X[i, :] = psi * x[i] * (self.g - self.y0)

        # Solve with regularization
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

        # NEW: store the *relative* offset learned from the demo
        self.goal_offset = np.zeros(n_joints)

    # --------------------------------------------------------------------- #
    #  FILTERING (keeps demo safe)
    # --------------------------------------------------------------------- #
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

    # --------------------------------------------------------------------- #
    #  LEARNING FROM CSV
    # --------------------------------------------------------------------- #
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

            # NEW: remember the *relative* motion of this joint
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
            # Keep demo start/goal only for printing – will be overwritten at runtime
            self.dmps[j].y0 = data['original_start']
            self.dmps[j].g = data['original_goal']

            print(f"  Joint {j+1}: Start={self.dmps[j].y0:.2f}°, Goal={self.dmps[j].g:.2f}°, "
                  f"Offset={self.goal_offset[j]:.2f}°, Duration={self.dmps[j].tau:.2f}s")

        self.trained = True
        print(f"\n{'='*60}")
        print("ALL 6 DMPs TRAINED SUCCESSFULLY!")
        print(f"{'='*60}\n")
        return dt

    # --------------------------------------------------------------------- #
    #  EXECUTION – START FROM CURRENT POSE, NEW GOAL = CURRENT + DEMO_OFFSET
    # --------------------------------------------------------------------- #
    def execute_trajectory(self, dt):
        if not self.trained:
            raise RuntimeError("Train DMPs first!")

        print(f"\n{'='*60}")
        print("STEP 3: EXECUTING TRAJECTORY")
        print(f"{'='*60}")

        # ----- 1. READ CURRENT ROBOT POSE -----
        error, current = robot.GetActualJointPosDegree(flag=1)
        if error != 0:
            print(f"Failed to read joint positions: {error}")
            return

        # ----- 2. UPDATE y0 (start) AND g (goal) FOR EVERY DMP -----
        for i, dmp in enumerate(self.dmps):
            dmp.y0 = current[i]                                 # new start
            dmp.g  = current[i] + self.goal_offset[i]           # new goal = start + demo_offset

        print(f"Current position (new start): {[f'{x:.2f}°' for x in current]}")
        goals = [dmp.g for dmp in self.dmps]
        print(f"Target goals (start + demo offset): {[f'{x:.2f}°' for x in goals]}")

        # ----- 3. OPTIONAL: move to start if far away (kept for safety) -----
        start_errors = [abs(current[i] - self.dmps[i].y0) for i in range(6)]
        max_start_err = max(start_errors)
        if max_start_err > 2.0:
            print(f"Robot far from recorded start (error: {max_start_err:.2f}°). Moving to start...")
            robot.MoveJ([dmp.y0 for dmp in self.dmps], 0, 0, 15.0, 0, 100, -1, -1, 0, 0)
            time.sleep(3.0)
            error, current = robot.GetActualJointPosDegree(flag=1)

        # ----- 4. INITIALISE INTEGRATION -----
        y = np.array(current, dtype=float)   # start from real pose
        dy = np.zeros(6)
        tau = self.dmps[0].tau
        x = 1.0
        t = 0.0
        log = []
        vel_hits = 0
        accel_hits = 0

        movement = max(abs(goals[i] - y[i]) for i in range(6))
        goal_tolerance = max(0.5, movement * 0.02)
        print(f"Goal tolerance: {goal_tolerance:.2f}° (2% of max movement)")
        print(f"Planned duration: {tau:.2f}s | ServoJ dt: {int(dt*1000)}ms\n")

        # ----- 5. MAIN LOOP -----
        try:
            while x > 0.01:
                loop_start = time.time()

                ddy = np.zeros(6)
                for i in range(6):
                    y[i], dy[i], ddy[i] = self.dmps[i].step(
                        y[i], dy[i], x, goals[i], dt, self.max_vel, self.max_accel
                    )
                    if abs(dy[i]) >= self.max_vel - 0.1:
                        vel_hits += 1
                    if abs(ddy[i]) >= self.max_accel - 1.0:
                        accel_hits += 1

                errors = np.abs(y - goals)
                max_vel = np.max(np.abs(dy))
                if (np.all(errors < goal_tolerance) and
                    max_vel < 1.0 and
                    x < 0.95):
                    print(f"\nGOAL REACHED! Max error: {np.max(errors):.2f}°")
                    break

                robot.ServoJ(list(y), [0]*6, int(dt*1000))

                x = x - self.dmps[0].alpha_x * x * dt / tau
                t += dt
                log.append(y.copy())

                if len(log) % 100 == 0:
                    progress = (1 - x) * 100
                    print(f"[{progress:5.1f}%] J1:{y[0]:.1f}° J2:{y[1]:.1f}° J3:{y[2]:.1f}° "
                          f"| V:{max_vel:.2f}°/s | A:{np.max(np.abs(ddy)):.1f}°/s²")

                elapsed = time.time() - loop_start
                if elapsed < dt:
                    time.sleep(dt - elapsed)

        except KeyboardInterrupt:
            print("\nExecution stopped by user.")

        # ----- 9. SUMMARY -----
        print(f"\n{'='*60}")
        print("TRAJECTORY EXECUTION FINISHED")
        print(f"{'='*60}")
        print(f"Total time: {t:.2f}s (planned: {tau:.2f}s)")
        print(f"Steps: {len(log)}")
        print(f"Velocity limit hits: {vel_hits}")
        print(f"Acceleration limit hits: {accel_hits}")
        print(f"Final position: {[f'{x:.2f}°' for x in y]}")
        print(f"Target goals:   {[f'{x:.2f}°' for x in goals]}")
        print(f"Final errors:   {[f'{abs(y[i]-goals[i]):.2f}°' for i in range(6)]}")
        print(f"{'='*60}\n")

        return log


# --------------------------------------------------------------------- #
#  MAIN
# --------------------------------------------------------------------- #
def main():
    print("\n" + "="*60)
    print("DMP REPLAY – START FROM CURRENT POSE – NEW GOAL = START + DEMO_OFFSET")
    print("="*60)
    print("Max Velocity: 15.0°/s | Max Acceleration: 1000.0°/s²")
    print("Goal = current position + (demo_goal – demo_start) → SAME SHAPE")
    print("="*60)

    csv_file = input("\nEnter CSV filename: ").strip()
    if not csv_file:
        print("No file entered. Exiting.")
        return

    dmp_system = MultiJointDMP(
        n_joints=6,
        n_basis=50,
        max_vel=15.0,
        max_accel=1000.0
    )

    try:
        dt = dmp_system.learn_from_csv(csv_file)

        input("\nPress Enter to execute trajectory...")

        print("\nEnabling servo mode...")
        robot.Mode(1)
        time.sleep(0.5)

        log = dmp_system.execute_trajectory(dt)

        save = input("\nSave execution log? (y/n): ").strip().lower()
        if save == 'y':
            filename = input("Filename (default: filtered_dmp_log.csv): ").strip()
            if not filename:
                filename = "filtered_dmp_log.csv"
            log_df = pd.DataFrame(log, columns=[f'joint{i+1}' for i in range(6)])
            log_df.to_csv(filename, index=False)
            print(f"Log saved: {filename}")

        print("\nProcess completed successfully!")

    except Exception as e:
        print(f"\nERROR: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("\nReturning to normal mode...")
        try:
            robot.Mode(0)
            time.sleep(0.5)
        except:
            pass


if __name__ == "__main__":
    main()