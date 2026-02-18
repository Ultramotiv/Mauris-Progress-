#Failed TRial

import math
import os
import signal
import sys
import time

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

sys.path.append('/home/um/fairino-python-sdk-main/linux/fairino')
import Robot


# -----------------------------------------------------------------------------
# Robot connection
# -----------------------------------------------------------------------------
robot = Robot.RPC('192.168.58.2')
print("Robot connected.")


# -----------------------------------------------------------------------------
# Cartesian impedance parameters (new_mapping style)
# -----------------------------------------------------------------------------
M_cart = [5.0, 5.0, 5.0, 2.0, 2.0, 2.0]
B_cart = [20.0, 20.0, 20.0, 10.0, 10.0, 10.0]
K_cart = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

force_to_translation = 120.0   # N -> mm/s^2
force_to_rotation = 6.0        # Nm -> deg/s^2

force_deadband = 0.5
force_threshold = 1.0

max_linear_vel = 200.0
max_angular_vel = 26.0

max_joint_step = 15.0          # deg per control cycle
joint_filter_alpha = [0.4, 0.4, 0.4, 0.3, 0.15, 0.3]

# Conservative joint velocity caps (deg/s) for passive playback
max_joint_speed_deg_s = [20.0, 8.0, 20.0, 30.0, 30.0, 30.0]

# Pre-position control before exact playback
preposition_timeout_s = 20.0

dt = 0.008
startup_delay = 1.0
gravity_compensation_samples = 200


# -----------------------------------------------------------------------------
# Tunnel parameters (dmp_Trial 1 style)
# -----------------------------------------------------------------------------
TUNNEL_RADIUS_MM = 100.0
TUNNEL_DAMPING_WHEN_CLAMPED = 0.25


# -----------------------------------------------------------------------------
# Global state
# -----------------------------------------------------------------------------
baseline_forces = None
shutdown_requested = False


# -----------------------------------------------------------------------------
# Utility: trajectory loading (CSV)
# -----------------------------------------------------------------------------
def load_csv_joint_trajectory(csv_path, n_joints=6):
	if not os.path.exists(csv_path):
		raise FileNotFoundError(f"CSV not found: {csv_path}")

	df = pd.read_csv(csv_path)
	joint_cols = [f'joint{i+1}' for i in range(n_joints)]

	if all(c in df.columns for c in joint_cols):
		traj = df[joint_cols].values.astype(float)
	else:
		numeric = df.select_dtypes(include=[np.number])
		if numeric.shape[1] < n_joints:
			raise ValueError(
				"CSV must have joint1..joint6 columns or at least 6 numeric columns."
			)
		print("CSV without joint1..joint6 headers. Using first 6 numeric columns.")
		traj = numeric.iloc[:, :n_joints].values.astype(float)

	if traj.ndim != 2 or traj.shape[1] != n_joints:
		raise ValueError(f"Trajectory must be shape (N, {n_joints}). Got {traj.shape}")
	if len(traj) < 2:
		raise ValueError("Trajectory must contain at least 2 points.")

	return traj


def move_to_start_pose(start_joints):
	"""Move robot from current pose to first trajectory point before exact playback."""
	err, current_joints = robot.GetActualJointPosDegree()
	if err != 0:
		print("Error getting current joints for pre-position.")
		return False

	current = np.array(current_joints[:6], dtype=float)
	target = np.array(start_joints[:6], dtype=float)
	step_limit = np.minimum(np.full(6, max_joint_step, dtype=float), np.array(max_joint_speed_deg_s, dtype=float) * dt)

	print("Pre-positioning to first trajectory point...")
	start_time = time.time()
	while not shutdown_requested:
		delta = target - current
		if np.max(np.abs(delta)) < 0.2:
			print("Pre-position complete.")
			return True

		step = np.clip(delta, -step_limit, step_limit)
		current = current + step
		robot.ServoJ(list(current), [0] * 6, int(dt * 1000))

		if (time.time() - start_time) > preposition_timeout_s:
			print("Pre-position timeout.")
			return False

		time.sleep(dt)

	return False


# -----------------------------------------------------------------------------
# Utility: FK tunnel build (dmp_Trial 1 style)
# -----------------------------------------------------------------------------
def get_fk_xyz(joint_pos):
	try:
		err, pose = robot.GetForwardKin(list(joint_pos))
		if err == 0 and pose is not None and len(pose) >= 3:
			return np.array([float(pose[0]), float(pose[1]), float(pose[2])], dtype=float)
	except Exception:
		pass
	return None


def build_tcp_trajectory(joint_trajectory):
	tcp_pts = []
	for jp in joint_trajectory:
		xyz = get_fk_xyz(jp)
		if xyz is None:
			raise RuntimeError("FK failed while building TCP trajectory tunnel.")
		tcp_pts.append(xyz)
	return np.array(tcp_pts, dtype=float)


def clamp_xyz_to_tunnel(candidate_xyz, tcp_tunnel_xyz, radius_mm):
	d2 = np.sum((tcp_tunnel_xyz - candidate_xyz) ** 2, axis=1)
	nearest_idx = int(np.argmin(d2))
	nearest = tcp_tunnel_xyz[nearest_idx]

	vec = candidate_xyz - nearest
	dist = float(np.linalg.norm(vec))

	if dist <= radius_mm or dist < 1e-9:
		return candidate_xyz, nearest_idx, dist, 0.0, False

	clamped_xyz = nearest + (vec / dist) * radius_mm
	excess = dist - radius_mm
	return clamped_xyz, nearest_idx, dist, excess, True


def _orthonormal_basis_from_tangent(tangent):
	t = np.array(tangent, dtype=float)
	tn = np.linalg.norm(t)
	if tn < 1e-9:
		t = np.array([1.0, 0.0, 0.0], dtype=float)
	else:
		t = t / tn

	ref = np.array([0.0, 0.0, 1.0], dtype=float) if abs(t[2]) < 0.9 else np.array([0.0, 1.0, 0.0], dtype=float)
	u = np.cross(t, ref)
	un = np.linalg.norm(u)
	if un < 1e-9:
		u = np.array([1.0, 0.0, 0.0], dtype=float)
	else:
		u = u / un
	v = np.cross(t, u)
	vn = np.linalg.norm(v)
	if vn < 1e-9:
		v = np.array([0.0, 1.0, 0.0], dtype=float)
	else:
		v = v / vn
	return u, v


def visualize_trajectory_with_tunnel(tcp_trajectory, tunnel_radius_mm):
	"""Open a 3D GUI: trajectory centerline + tunnel rings."""
	if tcp_trajectory is None or len(tcp_trajectory) < 2:
		print("Visualization skipped: insufficient TCP points.")
		return None

	fig = plt.figure("Trajectory + Tunnel", figsize=(9, 7))
	ax = fig.add_subplot(111, projection='3d')

	xs, ys, zs = tcp_trajectory[:, 0], tcp_trajectory[:, 1], tcp_trajectory[:, 2]
	ax.plot(xs, ys, zs, color='tab:blue', linewidth=2.2, label='TCP Trajectory')
	ax.scatter([xs[0]], [ys[0]], [zs[0]], color='green', s=45, label='Start')
	ax.scatter([xs[-1]], [ys[-1]], [zs[-1]], color='red', s=45, label='Goal')

	n_points = len(tcp_trajectory)
	stride = max(1, n_points // 80)
	theta = np.linspace(0, 2 * np.pi, 24)

	for i in range(0, n_points, stride):
		if i == 0:
			tangent = tcp_trajectory[1] - tcp_trajectory[0]
		elif i == n_points - 1:
			tangent = tcp_trajectory[-1] - tcp_trajectory[-2]
		else:
			tangent = tcp_trajectory[min(i + 1, n_points - 1)] - tcp_trajectory[max(i - 1, 0)]

		u, v = _orthonormal_basis_from_tangent(tangent)
		c = tcp_trajectory[i]
		ring = c + tunnel_radius_mm * (np.outer(np.cos(theta), u) + np.outer(np.sin(theta), v))
		ax.plot(ring[:, 0], ring[:, 1], ring[:, 2], color='orange', alpha=0.28, linewidth=0.9)

	ax.set_xlabel('X (mm)')
	ax.set_ylabel('Y (mm)')
	ax.set_zlabel('Z (mm)')
	ax.set_title(f'Trajectory with Tunnel Radius = {tunnel_radius_mm:.1f} mm')
	ax.legend(loc='best')
	ax.grid(True)

	x_range = np.ptp(xs) if np.ptp(xs) > 1e-6 else 1.0
	y_range = np.ptp(ys) if np.ptp(ys) > 1e-6 else 1.0
	z_range = np.ptp(zs) if np.ptp(zs) > 1e-6 else 1.0
	max_range = max(x_range, y_range, z_range)
	x_mid, y_mid, z_mid = np.mean(xs), np.mean(ys), np.mean(zs)
	half = max_range / 2.0
	ax.set_xlim(x_mid - half, x_mid + half)
	ax.set_ylim(y_mid - half, y_mid + half)
	ax.set_zlim(z_mid - half, z_mid + half)

	plt.tight_layout()
	plt.ion()
	plt.show(block=False)
	plt.pause(0.001)
	return fig, ax


# -----------------------------------------------------------------------------
# Utility: FT mapping (new_mapping style)
# -----------------------------------------------------------------------------
def euler_to_rotation_matrix(rx, ry, rz):
	rx_rad = math.radians(rx)
	ry_rad = math.radians(ry)
	rz_rad = math.radians(rz)

	rx_m = np.array([
		[1, 0, 0],
		[0, math.cos(rx_rad), -math.sin(rx_rad)],
		[0, math.sin(rx_rad), math.cos(rx_rad)]
	])
	ry_m = np.array([
		[math.cos(ry_rad), 0, math.sin(ry_rad)],
		[0, 1, 0],
		[-math.sin(ry_rad), 0, math.cos(ry_rad)]
	])
	rz_m = np.array([
		[math.cos(rz_rad), -math.sin(rz_rad), 0],
		[math.sin(rz_rad), math.cos(rz_rad), 0],
		[0, 0, 1]
	])
	return rz_m @ ry_m @ rx_m


R_FT_TO_TCP = np.array([
	[1.0, 0.0, 0.0],
	[0.0, 1.0, 0.0],
	[0.0, 0.0, 1.0]
], dtype=float)


def ft_to_world(ft_forces, tcp_orientation):
	rx, ry, rz = tcp_orientation
	r_tcp_to_world = euler_to_rotation_matrix(rx, ry, rz)
	r_ft_to_world = r_tcp_to_world @ R_FT_TO_TCP

	ft_force_vec = np.array(ft_forces[:3], dtype=float)
	ft_moment_vec = np.array(ft_forces[3:6], dtype=float)

	world_force_vec = r_ft_to_world @ ft_force_vec
	world_moment_vec = r_ft_to_world @ ft_moment_vec

	return np.array([
		world_force_vec[0], world_force_vec[1], world_force_vec[2],
		world_moment_vec[0], world_moment_vec[1], world_moment_vec[2]
	], dtype=float)


# -----------------------------------------------------------------------------
# FT setup
# -----------------------------------------------------------------------------
def init_ft_sensor():
	company = 24
	device = 0
	robot.FT_SetConfig(company, device)
	robot.FT_Activate(0)
	time.sleep(0.5)
	robot.FT_Activate(1)
	time.sleep(1.0)

	robot.SetLoadWeight(0, 0.0)
	robot.SetLoadCoord(0.0, 0.0, 0.0)

	robot.FT_SetZero(0)
	time.sleep(0.3)
	robot.FT_SetZero(1)
	time.sleep(1.0)
	robot.FT_SetZero(0)
	time.sleep(0.3)
	robot.FT_SetZero(1)
	time.sleep(1.0)

	print("FT sensor initialized and zeroed.")


def calibrate_baseline_forces():
	global baseline_forces
	print("Calibrating baseline FT forces...")

	robot.FT_SetZero(0)
	time.sleep(0.3)
	robot.FT_SetZero(1)
	time.sleep(1.5)

	for _ in range(50):
		robot.FT_GetForceTorqueRCS()
		time.sleep(0.01)

	samples = []
	for _ in range(gravity_compensation_samples):
		ft_data = robot.FT_GetForceTorqueRCS()
		if ft_data[0] == 0:
			samples.append([float(ft_data[1][i]) for i in range(6)])
		time.sleep(0.01)

	if not samples:
		baseline_forces = [0.0] * 6
		print("Warning: baseline sample collection failed, using zeros.")
		return

	baseline_forces = [sum(s[i] for s in samples) / len(samples) for i in range(6)]
	print("Baseline:", [round(v, 4) for v in baseline_forces])


# -----------------------------------------------------------------------------
# Signal handling
# -----------------------------------------------------------------------------
def shutdown(sig, frame):
	global shutdown_requested
	shutdown_requested = True
	print("\n[Ctrl+C] Shutdown requested...")


signal.signal(signal.SIGINT, shutdown)


# -----------------------------------------------------------------------------
# Main control loop
# -----------------------------------------------------------------------------
def control_loop(joint_trajectory):
	if len(joint_trajectory) == 0:
		print("Empty trajectory. Exiting.")
		return

	if not move_to_start_pose(joint_trajectory[0]):
		print("Could not pre-position to start point. Exiting.")
		return

	print("\n" + "=" * 72)
	print("PASSIVE_TRAJECTORY: direct joint ServoJ playback")
	print("=" * 72)
	print("FT sensor input: disabled")
	print("Tunnel constraint/visualization: disabled")
	print("Trajectory smoothing/filtering: disabled (exact CSV points)")
	print("Press Ctrl+C to stop.")
	print("=" * 72 + "\n")

	for idx, target_joints in enumerate(joint_trajectory):
		if shutdown_requested:
			break

		loop_start = time.time()
		target = np.array(target_joints[:6], dtype=float)
		servo_err = robot.ServoJ(list(target), [0] * 6, int(dt * 1000))
		if idx % 125 == 0:
			print(f"Point {idx+1}/{len(joint_trajectory)} | ServoErr={servo_err}")

		elapsed = time.time() - loop_start
		if elapsed < dt:
			time.sleep(dt - elapsed)

	print("\nStopping servo mode...")
	time.sleep(0.05)
	try:
		robot.ServoMoveEnd()
	except Exception as e:
		print(f"ServoMoveEnd warning: {e}")
	print("Exited cleanly.")


def main():
	csv_path = input("Enter trajectory CSV file path: ").strip()
	if not csv_path:
		print("No CSV provided. Exiting.")
		return

	if not os.path.isabs(csv_path):
		csv_path = os.path.abspath(csv_path)

	try:
		joint_traj = load_csv_joint_trajectory(csv_path, n_joints=6)
	except Exception as e:
		print(f"Failed to load CSV trajectory: {e}")
		return

	print(f"Trajectory loaded: {len(joint_traj)} joint points")

	if robot.ServoMoveStart() != 0:
		print("Failed to start servo mode.")
		return

	print(f"Waiting {startup_delay:.1f}s before control starts...")
	time.sleep(startup_delay)

	control_loop(joint_traj)


if __name__ == '__main__':
	main()
