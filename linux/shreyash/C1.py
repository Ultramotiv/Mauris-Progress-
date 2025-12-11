import numpy as np
import Robot
import time

# --- DH Transformation Function ---
def dh_transformation(theta, d, a, alpha):
    return np.array([
        [np.cos(theta), -np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
        [np.sin(theta), np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
        [0, np.sin(alpha), np.cos(alpha), d],
        [0, 0, 0, 1]
    ])

# --- Jacobian Calculation ---
def calculate_jacobian(dh_params, joint_angles):
    n = len(joint_angles)
    J = np.zeros((6, n))
    T = np.eye(4)
    origins = [np.array([0.0, 0.0, 0.0])]
    z_axes = [np.array([0.0, 0.0, 1.0])]

    # Forward kinematics
    for i in range(n):
        theta, d, a, alpha = dh_params[i]
        T_i = dh_transformation(joint_angles[i] + theta, d, a, alpha)
        T = T @ T_i
        origins.append(T[:3, 3])
        z_axes.append(T[:3, 2])

    end_effector_pos = origins[-1]

    for i in range(n):
        J[:3, i] = np.cross(z_axes[i], end_effector_pos - origins[i])
        J[3:, i] = z_axes[i]

    return J

# --- Force Estimation ---
def estimate_end_effector_force(joint_torques, dh_params, joint_angles):
    J = calculate_jacobian(dh_params, joint_angles)
    # Regularized pseudo-inverse to avoid instability
    JT = J.T
    pseudo_inv = np.linalg.pinv(JT @ J + 1e-6 * np.eye(J.shape[1])) @ JT
    wrench = pseudo_inv @ joint_torques
    return wrench

# --- DH Parameters (adjust according to your robot) ---
dh_params = [
    (0, 0.180, 0, np.pi/2),
    (0, 0.0, -0.700, 0),
    (0, 0.0, -0.586, 0),
    (0, 0.159, 0, np.pi/2),
    (0, 0.114, 0, -np.pi/2),
    (0, 0.106, 0, 0)
]

# --- Main Loop ---
robot = Robot.RPC('192.168.58.2')

while True:
    # Get joint data
    joint_torque_status = robot.GetJointTorques()
    joint_angle_status = robot.GetActualJointPosRadian()
    sensor_data = robot.FT_GetForceTorqueOrigin()

    if joint_torque_status and joint_angle_status and sensor_data:
        joint_torques = np.array(joint_torque_status[1])   # in Nm
        joint_angles = np.array(joint_angle_status[1])     # in radians
        sensor_values = sensor_data[1]                     # [Fx, Fy, Fz, Mx, My, Mz]

        # Estimate end-effector wrench
        estimated_wrench = estimate_end_effector_force(joint_torques, dh_params, joint_angles)

        # Extract individual components
        Fx_e, Fy_e, Fz_e, Mx_e, My_e, Mz_e = estimated_wrench
        Fx_s, Fy_s, Fz_s, Mx_s, My_s, Mz_s = sensor_values

        # Display comparison
        print("\n=== Force & Moment Comparison ===")
        print(f"Fx: Estimated = {Fx_e:7.2f} N\tSensor = {Fx_s:7.2f} N")
        print(f"Fy: Estimated = {Fy_e:7.2f} N\tSensor = {Fy_s:7.2f} N")
        print(f"Fz: Estimated = {Fz_e:7.2f} N\tSensor = {Fz_s:7.2f} N")
        print(f"Mx: Estimated = {Mx_e:7.2f} Nm\tSensor = {Mx_s:7.2f} Nm")
        print(f"My: Estimated = {My_e:7.2f} Nm\tSensor = {My_s:7.2f} Nm")
        print(f"Mz: Estimated = {Mz_e:7.2f} Nm\tSensor = {Mz_s:7.2f} Nm")
        print("=================================")

    else:
        print("Error: Failed to read data from robot.")

    time.sleep(0.1)
