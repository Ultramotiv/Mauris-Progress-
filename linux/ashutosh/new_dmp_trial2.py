# This code is developed on 24th oct 2025
#so in this code the dmp is implemented 
# this code take a csv file saved from new_dmp_trial.py file 
# calculated the velocity and acceleration and trains the dmp
# then simply executed that trajectory 


"""
DMP-Based Trajectory Replay with Real-Time Goal Adaptation
Uses Dynamic Movement Primitives to learn from recorded trajectories
and adapts the goal based on Force/Torque sensor feedback
"""

import sys
sys.path.append('/home/um/fairino-python-sdk-main/linux/fairino')
import Robot
import time
import numpy as np
import pandas as pd
import os

# Establish connection with robot controller
robot = Robot.RPC('192.168.58.2')

class DMP1D:
    """
    1D Dynamic Movement Primitive implementation
    Based on Ijspeert et al. 2013 formulation
    """
    
    def __init__(self, n_basis=50, alpha_y=25.0, beta_y=6.25, alpha_x=1.0):
        """
        Initialize DMP parameters
        
        Args:
            n_basis: Number of basis functions (Gaussian RBFs)
            alpha_y: Transformation system gain (stiffness)
            beta_y: Transformation system damping
            alpha_x: Canonical system decay rate
        """
        self.n_basis = n_basis
        self.alpha_y = alpha_y
        self.beta_y = beta_y
        self.alpha_x = alpha_x
        
        # Basis function centers (evenly spaced in phase space)
        self.c = np.exp(-self.alpha_x * np.linspace(0, 1, n_basis))
        
        # Basis function widths
        self.h = np.ones(n_basis) * n_basis / (self.c ** 2)
        
        # Learned weights
        self.w = np.zeros(n_basis)
        
        # Initial and goal positions
        self.y0 = 0.0
        self.g = 0.0
        
        # Duration
        self.tau = 1.0
        
    def basis_functions(self, x):
        """
        Compute Gaussian basis functions
        
        Args:
            x: Phase variable (0 to 1)
            
        Returns:
            psi: Basis function activations [n_basis]
        """
        psi = np.exp(-self.h * (x - self.c) ** 2)
        return psi / (np.sum(psi) + 1e-10)  # Normalized
    
    def forcing_term(self, x):
        """
        Compute forcing term f(x)
        
        Args:
            x: Phase variable
            
        Returns:
            f: Forcing term value
        """
        psi = self.basis_functions(x)
        return np.dot(psi, self.w) * x * (self.g - self.y0)
    
    def learn_from_trajectory(self, positions, velocities, accelerations, dt):
        """
        Learn DMP weights from demonstration trajectory
        
        Args:
            positions: Joint positions over time [n_timesteps]
            velocities: Joint velocities over time [n_timesteps]
            accelerations: Joint accelerations over time [n_timesteps]
            dt: Time step between samples
        """
        self.y0 = positions[0]
        self.g = positions[-1]
        self.tau = len(positions) * dt
        
        # Compute phase variable over trajectory
        n_steps = len(positions)
        x = np.exp(-self.alpha_x * np.linspace(0, 1, n_steps))
        
        # Compute target forcing term from demonstration
        # tau^2 * ddot_y = alpha_y * (beta_y * (g - y) - tau * dot_y) + f
        # f = tau^2 * ddot_y - alpha_y * (beta_y * (g - y) - tau * dot_y)
        
        f_target = (self.tau**2 * accelerations - 
                   self.alpha_y * (self.beta_y * (self.g - positions) - 
                                  self.tau * velocities))
        
        # Solve for weights using linear regression
        # f_target = sum(psi_i * w_i) * x * (g - y0)
        
        X = np.zeros((n_steps, self.n_basis))
        for i in range(n_steps):
            psi = self.basis_functions(x[i])
            X[i, :] = psi * x[i] * (self.g - self.y0)
        
        # Ridge regression for stability
        lambda_reg = 1e-10
        A = X.T @ X + lambda_reg * np.eye(X.shape[1])
        b = X.T @ f_target
        self.w = np.linalg.solve(A, b)
    
    def step(self, y, dy, x, g_current, dt):
        """
        Execute one integration step of the DMP
        
        Args:
            y: Current position
            dy: Current velocity
            x: Current phase
            g_current: Current goal (can change over time)
            dt: Integration time step
            
        Returns:
            y_new: New position
            dy_new: New velocity
            ddy: Acceleration
        """
        # Update goal
        self.g = g_current
        
        # Compute forcing term
        f = self.forcing_term(x)
        
        # Transformation system dynamics
        ddy = (self.alpha_y * (self.beta_y * (self.g - y) - self.tau * dy) + f) / (self.tau ** 2)
        
        # Integrate
        dy_new = dy + ddy * dt
        y_new = y + dy_new * dt
        
        return y_new, dy_new, ddy


class MultiJointDMP:
    """
    Multi-joint DMP controller - synchronizes 6 DMPs for robot control
    """
    
    def __init__(self, n_joints=6, n_basis=50):
        """
        Initialize multi-joint DMP system
        
        Args:
            n_joints: Number of robot joints
            n_basis: Number of basis functions per joint
        """
        self.n_joints = n_joints
        self.dmps = [DMP1D(n_basis=n_basis) for _ in range(n_joints)]
        self.trained = False
        
    def learn_from_csv(self, csv_filename):
        """
        Complete workflow: Load CSV -> Calculate velocities/accelerations -> Train DMPs
        
        Args:
            csv_filename: Path to CSV file with recorded trajectory
            
        Returns:
            dt: Time step in seconds
        """
        print(f"\n{'='*60}")
        print(f"STEP 1: LOADING CSV FILE")
        print(f"{'='*60}")
        print(f"File: {csv_filename}")
        
        # Check if file exists
        if not os.path.exists(csv_filename):
            raise FileNotFoundError(f"CSV file not found: {csv_filename}")
        
        # Load CSV data
        df = pd.read_csv(csv_filename)
        print(f"✓ CSV loaded successfully")
        print(f"  - Rows: {len(df)}")
        print(f"  - Columns: {list(df.columns)}")
        
        # Validate columns
        required_cols = ['timestamp', 'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        missing_cols = [col for col in required_cols if col not in df.columns]
        if missing_cols:
            raise ValueError(f"Missing required columns: {missing_cols}")
        
        # Extract data
        timestamps = df['timestamp'].values
        joint_positions = df[['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']].values
        
        # Calculate dt (average time step)
        dt = np.mean(np.diff(timestamps))
        print(f"  - Average dt: {dt*1000:.2f} ms")
        print(f"  - Duration: {timestamps[-1] - timestamps[0]:.2f} seconds")
        print(f"  - Samples: {len(timestamps)}")
        
        print(f"\n{'='*60}")
        print(f"STEP 2: CALCULATING VELOCITIES AND ACCELERATIONS")
        print(f"{'='*60}")
        
        # Calculate velocities and accelerations for each joint
        self.training_data = []
        for joint_idx in range(self.n_joints):
            positions = joint_positions[:, joint_idx]
            
            # Calculate velocity using central differences
            velocities = np.gradient(positions, timestamps)
            
            # Calculate acceleration using central differences
            accelerations = np.gradient(velocities, timestamps)
            
            # Apply smoothing (moving average filter)
            window_size = 5
            if len(positions) > window_size:
                velocities = np.convolve(velocities, np.ones(window_size)/window_size, mode='same')
                accelerations = np.convolve(accelerations, np.ones(window_size)/window_size, mode='same')
            
            print(f"Joint {joint_idx+1}:")
            print(f"  - Start: {positions[0]:.2f}° | End: {positions[-1]:.2f}°")
            print(f"  - Max velocity: {np.max(np.abs(velocities)):.2f}°/s")
            print(f"  - Max acceleration: {np.max(np.abs(accelerations)):.2f}°/s²")
            
            # Store for training
            self.training_data.append({
                'positions': positions,
                'velocities': velocities,
                'accelerations': accelerations
            })
        
        print(f"\n{'='*60}")
        print(f"STEP 3: TRAINING 1D DMP FOR EACH JOINT")
        print(f"{'='*60}")
        
        # Train each DMP independently
        for joint_idx in range(self.n_joints):
            data = self.training_data[joint_idx]
            
            print(f"Training DMP for Joint {joint_idx+1}...")
            self.dmps[joint_idx].learn_from_trajectory(
                data['positions'],
                data['velocities'],
                data['accelerations'],
                dt
            )
            
            # Verify training
            print(f"  ✓ DMP trained successfully")
            print(f"    - y0 (start): {self.dmps[joint_idx].y0:.2f}°")
            print(f"    - g (goal): {self.dmps[joint_idx].g:.2f}°")
            print(f"    - tau (duration): {self.dmps[joint_idx].tau:.2f}s")
            print(f"    - Learned weights: {len(self.dmps[joint_idx].w)} basis functions")
        
        self.trained = True
        print(f"\n{'='*60}")
        print(f"✓ ALL 6 DMPs TRAINED SUCCESSFULLY!")
        print(f"{'='*60}\n")
        
        return dt
    
    def execute_trajectory(self, dt=0.002, ft_adaptation=False, ft_threshold=10.0):
        """
        STEP 4: Execute learned trajectory with synchronized multi-joint control
        Starts from CURRENT robot position (not DMP start position)
        
        Args:
            dt: Integration time step in SECONDS (will be converted to ms for ServoJ)
            ft_adaptation: Enable FT sensor goal adaptation
            ft_threshold: Force threshold (N) to trigger goal change
        """
        if not self.trained:
            raise RuntimeError("DMPs not trained! Call learn_from_csv() first.")
        
        print(f"\n{'='*60}")
        print(f"STEP 4: EXECUTING SYNCHRONIZED TRAJECTORY")
        print(f"{'='*60}")
        print(f"Mode: {'FT-Adaptive' if ft_adaptation else 'Standard'}")
        if ft_adaptation:
            print(f"FT Threshold: {ft_threshold} N")
        print(f"Integration dt: {dt*1000:.2f} ms")
        print(f"{'='*60}\n")
        
        # Get CURRENT joint positions from robot
        error, current_joints = robot.GetActualJointPosDegree(flag=1)
        if error != 0:
            print(f"Error reading initial joint positions: {error}")
            return
        
        print(f"Current robot position: {[f'{j:.2f}°' for j in current_joints]}")
        
        # Get DMP learned start/goal positions (for reference only)
        dmp_start = [dmp.y0 for dmp in self.dmps]
        dmp_goals = [dmp.g for dmp in self.dmps]
        print(f"DMP learned start:     {[f'{j:.2f}°' for j in dmp_start]}")
        print(f"DMP learned goal:      {[f'{j:.2f}°' for j in dmp_goals]}")
        
        # IMPORTANT: Start from CURRENT position, not DMP start position
        # This allows execution from any initial configuration
        y = np.array(current_joints, dtype=float)  # Start from current position
        dy = np.zeros(self.n_joints)  # Zero initial velocity
        
        # Keep the learned goal positions
        goals = np.array(dmp_goals, dtype=float)
        
        print(f"\n⚠ Starting trajectory from CURRENT position")
        print(f"   (DMP will adapt to reach learned goals from wherever robot is now)\n")
        
        # Get trajectory duration (same for all DMPs)
        tau = self.dmps[0].tau
        alpha_x = self.dmps[0].alpha_x
        
        # Convert dt to milliseconds for ServoJ (must be integer)
        dt_ms = int(dt * 1000)
        if dt_ms < 1:
            dt_ms = 1
            print(f"⚠ Warning: dt too small, using {dt_ms}ms")
        
        print(f"Starting synchronized trajectory execution...")
        print(f"Duration: {tau:.2f}s")
        print(f"ServoJ time step: {dt_ms}ms")
        print("Press Ctrl+C to stop\n")
        
        # Canonical system (shared phase variable for synchronization)
        x = 1.0
        t = 0.0
        
        trajectory_log = []
        
        try:
            while x > 0.01:  # Execute until phase decays
                loop_start = time.time()
                
                # Read FT sensor data (optional)
                if ft_adaptation:
                    error, ft_data = robot.GetActualTCPForce(flag=1)
                    
                    if error == 0:
                        fx, fy, fz = ft_data[0], ft_data[1], ft_data[2]
                        force_magnitude = np.sqrt(fx**2 + fy**2 + fz**2)
                        
                        # Adapt goals based on force feedback
                        if force_magnitude > ft_threshold:
                            # Hold current position as new goal
                            for i in range(self.n_joints):
                                goals[i] = y[i]
                            
                            print(f"[t={t:.3f}s] FT triggered! Force={force_magnitude:.1f}N - Goals adapted")
                
                # Execute one synchronized DMP step for ALL joints
                ddy = np.zeros(self.n_joints)
                for i in range(self.n_joints):
                    # All DMPs use the SAME phase variable (x) for synchronization
                    y[i], dy[i], ddy[i] = self.dmps[i].step(y[i], dy[i], x, goals[i], dt)
                
                # Convert to list and ensure proper format for ServoJ
                # ServoJ expects: ServoJ(joint_pos_list, tool, user, time_ms, gain1, gain2, lookahead_time)
                y_cmd = [float(val) for val in y]  # Ensure list of floats
                
                # Send synchronized command to ALL joints
                # Use keyword args and axisPos list so ServoJ receives correct types
                # cmdT expects seconds in other usages, so pass dt (seconds)
                error = robot.ServoJ(joint_pos=y_cmd, axisPos=[0]*6, cmdT=dt)
                
                if error != 0:
                    print(f"Warning: ServoJ error {error} at t={t:.3f}s")
                
                # Update canonical system (shared phase for synchronization)
                x = x - alpha_x * x * dt / tau
                t += dt
                
                # Log data
                trajectory_log.append({
                    'time': t,
                    'phase': x,
                    'joints': y.copy(),
                    'velocities': dy.copy(),
                    'accelerations': ddy.copy(),
                    'goals': goals.copy()
                })
                
                # Print progress every 100ms
                if len(trajectory_log) % 50 == 0:
                    progress = (1 - x) * 100
                    joints_str = ' | '.join([f"J{i+1}:{y[i]:.1f}°" for i in range(3)])
                    print(f"[{progress:5.1f}%] Phase:{x:.3f} | {joints_str}")
                
                # Timing control
                elapsed = time.time() - loop_start
                if elapsed < dt:
                    time.sleep(dt - elapsed)
        
        except KeyboardInterrupt:
            print("\n\nTrajectory execution interrupted!")
        
        print(f"\n{'='*60}")
        print("✓ TRAJECTORY EXECUTION COMPLETED!")
        print(f"{'='*60}")
        print(f"Total time: {t:.2f}s")
        print(f"Total steps: {len(trajectory_log)}")
        print(f"Final position: {[f'{j:.2f}°' for j in y]}")
        print(f"Target goals:   {[f'{j:.2f}°' for j in goals]}")
        print(f"{'='*60}\n")
        
        return trajectory_log


def main():
    """
    Main function - Sequential workflow:
    1. Ask for CSV file
    2. Calculate velocities and accelerations
    3. Train 1D DMP for each joint
    4. Execute synchronized trajectory FROM CURRENT POSITION
    """
    
    manual_interrupt = False
    try:
        print("\n" + "=" * 60)
        print("DMP-BASED TRAJECTORY REPLAY SYSTEM")
        print("=" * 60)
        print("\nWorkflow:")
        print("  1. Load CSV trajectory file")
        print("  2. Calculate velocities and accelerations")
        print("  3. Train 1D DMP for each of 6 joints")
        print("  4. Execute trajectory from CURRENT position")
        print("=" * 60)
        
        # STEP 1: Get CSV filename from user
        csv_file = input("\nEnter CSV filename (e.g., trajectory.csv): ").strip()
        
        if not csv_file:
            print("❌ No filename provided. Exiting.")
            return
        
        # Initialize multi-joint DMP system
        print("\nInitializing DMP system (6 joints, 50 basis functions per joint)...")
        dmp_system = MultiJointDMP(n_joints=6, n_basis=50)
        
        # STEPS 2-3: Load CSV, calculate derivatives, and train DMPs
        dt = dmp_system.learn_from_csv(csv_file)
        
        # Ask user if ready to execute
        input("\nPress Enter to execute the learned trajectory...")
        
        # Enable servo mode
        print("\nEnabling servo mode...")
        robot.Mode(1)
        time.sleep(0.5)
        
        # STEP 4: Execute synchronized trajectory (starts from current position)
        log = dmp_system.execute_trajectory(dt=dt, ft_adaptation=False)
        
        # Ask to save log
        save_choice = input("\nSave execution log? (y/n): ").strip().lower()
        if save_choice == 'y':
            log_filename = input("Enter log filename (default: dmp_execution_log.csv): ").strip()
            if not log_filename:
                log_filename = "dmp_execution_log.csv"
            
            # Convert log to DataFrame and save
            log_data = []
            for entry in log:
                row = {'time': entry['time'], 'phase': entry['phase']}
                for i in range(6):
                    row[f'joint{i+1}'] = entry['joints'][i]
                    row[f'vel{i+1}'] = entry['velocities'][i]
                    row[f'acc{i+1}'] = entry['accelerations'][i]
                    row[f'goal{i+1}'] = entry['goals'][i]
                log_data.append(row)
            
            df_log = pd.DataFrame(log_data)
            df_log.to_csv(log_filename, index=False)
            print(f"✓ Log saved to: {log_filename}")
        
        print("\n✓ Process completed successfully!")
        
    except FileNotFoundError as e:
        print(f"\n❌ Error: {e}")

    except KeyboardInterrupt:
        # User pressed Ctrl+C - mark manual interrupt so we close RPC
        manual_interrupt = True
        print("\nInterrupted by user (Ctrl+C).")

    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()

    finally:
        # Return to normal mode
        print("\nReturning robot to normal mode...")
        try:
            robot.Mode(0)
            time.sleep(0.5)
        except Exception:
            pass

        # Close connection ONLY if user manually interrupted with Ctrl+C
        if manual_interrupt:
            try:
                robot.CloseRPC()
            except Exception:
                pass
            print("Connection closed (manual interrupt).")
        else:
            print("RPC connection left open (not closed).")


if __name__ == "__main__":
    main()