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

dt = 0.008
startup_delay = 1.0
gravity_compensation_samples = 200


# -----------------------------------------------------------------------------
# Tunnel parameters (dmp_Trial 1 style)
# -----------------------------------------------------------------------------
TUNNEL_RADIUS_MM = 100.0
TUNNEL_DAMPING_WHEN_CLAMPED = 0.251

# Gradual resistance when TCP deviates from trajectory
RESIST_SOFT_START_RATIO = 0.0   # start resistance immediately from centerline
RESIST_INNER_GAIN = 2.0         # gain at tunnel boundary (inside profile)
RESIST_INNER_POWER = 1.15       # inner profile curvature (lower => more noticeable early)
RESIST_MIN_MISMATCH_MM = 1.0    # apply minimum resistance once mismatch exceeds this
RESIST_MIN_NONZERO_GAIN = 0.45  # noticeable resistance even inside tunnel
RESIST_OUTER_K = 0.10           # gain growth per mm outside tunnel
RESIST_OUTER_POWER = 1.45       # nonlinear growth outside tunnel
RESIST_MAX_GAIN = 12.0          # cap resistance gain


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
def control_loop(tcp_tunnel_trajectory, viz_handles=None):
	loop_count = 0

	err, actual_tcp_list = robot.GetActualTCPPose()
	if err != 0:
		print("Error getting current TCP pose. Exiting.")
		return

	err, current_joints = robot.GetActualJointPosDegree()
	if err != 0:
		print("Error getting current joint pose. Exiting.")
		return

	desired_tcp = np.array(actual_tcp_list, dtype=float)
	cart_velocity = np.zeros(6, dtype=float)
	filtered_joints = list(current_joints)

	# Optional initial clamp (if robot starts outside tunnel)
	clamped_start_xyz, _, start_dist, _, was_start_clamped = clamp_xyz_to_tunnel(
		desired_tcp[:3], tcp_tunnel_trajectory, TUNNEL_RADIUS_MM
	)
	if was_start_clamped:
		desired_tcp[:3] = clamped_start_xyz
		print(
			f"Start TCP is outside tunnel by {start_dist - TUNNEL_RADIUS_MM:.1f} mm. "
			"Initial position clamped to tunnel boundary."
		)

	print("\n" + "=" * 72)
	print("DMP_TRIAL3: FT -> World mapping -> IK -> ServoJ with tunnel constraint")
	print("=" * 72)
	print(f"Tunnel radius: {TUNNEL_RADIUS_MM:.1f} mm")
	print("Gradual resistance increases as TCP moves away from trajectory centerline.")
	print("Press Ctrl+C to stop.")
	print("=" * 72 + "\n")

	while not shutdown_requested:
		loop_start = time.time()
		loop_count += 1
		verbose = (loop_count % 125 == 0)

		# 1) Read current TCP + joints
		err, actual_tcp_list = robot.GetActualTCPPose()
		if err != 0:
			time.sleep(dt)
			continue
		actual_tcp = np.array(actual_tcp_list, dtype=float)

		err, current_joints = robot.GetActualJointPosDegree()
		if err != 0:
			time.sleep(dt)
			continue

		# 2) FT read + compensation
		ft_data = robot.FT_GetForceTorqueRCS()
		if ft_data[0] != 0:
			time.sleep(dt)
			continue

		raw_forces = [float(ft_data[1][i]) for i in range(6)]
		comp_forces = [raw_forces[i] - baseline_forces[i] for i in range(6)] if baseline_forces else raw_forces

		for i in range(6):
			if abs(comp_forces[i]) < force_deadband:
				comp_forces[i] = 0.0

		# 3) FT -> World
		world_forces = ft_to_world(comp_forces, actual_tcp[3:6])

		# 4) Cartesian impedance update
		f_scaled = np.zeros(6, dtype=float)
		f_scaled[:3] = world_forces[:3] * force_to_translation
		f_scaled[3:6] = world_forces[3:6] * force_to_rotation

		# Distance of current TCP from trajectory (for gradual resistance)
		d2_actual = np.sum((tcp_tunnel_trajectory - actual_tcp[:3]) ** 2, axis=1)
		nearest_idx_actual = int(np.argmin(d2_actual))
		nearest_xyz_actual = tcp_tunnel_trajectory[nearest_idx_actual]
		radial_vec = actual_tcp[:3] - nearest_xyz_actual
		radial_dist = float(np.linalg.norm(radial_vec))
		if radial_dist > 1e-9:
			radial_dir = radial_vec / radial_dist
		else:
			radial_dir = np.zeros(3, dtype=float)

		if TUNNEL_RADIUS_MM > 1e-9:
			ratio = radial_dist / TUNNEL_RADIUS_MM
		else:
			ratio = 0.0

		if ratio <= RESIST_SOFT_START_RATIO:
			resist_gain = 0.0
		elif ratio <= 1.0:
			s = (ratio - RESIST_SOFT_START_RATIO) / max(1e-9, (1.0 - RESIST_SOFT_START_RATIO))
			inner_target = max(RESIST_INNER_GAIN, RESIST_MIN_NONZERO_GAIN)
			resist_gain = RESIST_MIN_NONZERO_GAIN + (inner_target - RESIST_MIN_NONZERO_GAIN) * (s ** RESIST_INNER_POWER)
			if radial_dist < RESIST_MIN_MISMATCH_MM:
				resist_gain = 0.0
		else:
			resist_gain = RESIST_INNER_GAIN + RESIST_OUTER_K * ((radial_dist - TUNNEL_RADIUS_MM) ** RESIST_OUTER_POWER)
		resist_gain = float(min(RESIST_MAX_GAIN, max(0.0, resist_gain)))

		lin_force_mag = float(np.linalg.norm(world_forces[:3]))
		ang_force_mag = float(np.linalg.norm(world_forces[3:6]))

		for i in range(6):
			spring_force = -K_cart[i] * (desired_tcp[i] - actual_tcp[i])
			damping_force = -B_cart[i] * cart_velocity[i]

			if (i < 3 and lin_force_mag > force_threshold) or (i >= 3 and ang_force_mag > force_threshold):
				ext_force = f_scaled[i]
			else:
				ext_force = 0.0

			total_force = ext_force + spring_force + damping_force
			acc = total_force / M_cart[i] if M_cart[i] > 0 else 0.0
			cart_velocity[i] += acc * dt

		# Apply gradual radial resistance on linear velocity
		if radial_dist > 1e-9 and resist_gain > 0.0:
			v_lin = cart_velocity[:3]
			v_rad = float(np.dot(v_lin, radial_dir))
			v_tan = v_lin - v_rad * radial_dir
			v_rad_resisted = v_rad / (1.0 + resist_gain)
			cart_velocity[:3] = v_tan + v_rad_resisted * radial_dir

		cart_velocity[:3] = np.clip(cart_velocity[:3], -max_linear_vel, max_linear_vel)
		cart_velocity[3:6] = np.clip(cart_velocity[3:6], -max_angular_vel, max_angular_vel)

		candidate_tcp = desired_tcp + cart_velocity * dt

		# Keep orientation stable from actual TCP (no orientation drift)
		candidate_tcp[3:6] = actual_tcp[3:6]

		# 5) Tunnel hard clamp on XYZ
		clamped_xyz, nearest_idx, path_dist, tunnel_excess, was_clamped = clamp_xyz_to_tunnel(
			candidate_tcp[:3], tcp_tunnel_trajectory, TUNNEL_RADIUS_MM
		)
		candidate_tcp[:3] = clamped_xyz

		if was_clamped:
			# Dampen linear velocity strongly when user pushes outside tunnel
			cart_velocity[:3] *= TUNNEL_DAMPING_WHEN_CLAMPED

		desired_tcp = candidate_tcp

		# 6) IK
		ik_result = robot.GetInverseKin(0, list(desired_tcp), -1)
		ik_ok = isinstance(ik_result, (list, tuple)) and len(ik_result) >= 2 and ik_result[0] == 0

		if not ik_ok:
			if verbose:
				ik_err = ik_result if isinstance(ik_result, int) else ik_result[0]
				print(f"IK failed (err={ik_err}), holding current joints.")
			desired_tcp[:3] = actual_tcp[:3]
			cart_velocity *= 0.3
			filtered_joints = list(current_joints)
			robot.ServoJ(current_joints, [0] * 6, int(dt * 1000))

			elapsed = time.time() - loop_start
			if elapsed < dt:
				time.sleep(dt - elapsed)
			continue

		target_joints = list(ik_result[1][:6])

		# 7) Joint jump safety
		safe = True
		for j in range(6):
			if abs(target_joints[j] - current_joints[j]) > max_joint_step:
				safe = False
				if verbose:
					print(
						f"Joint jump rejected J{j+1}: "
						f"{current_joints[j]:.2f} -> {target_joints[j]:.2f}"
					)
				break

		if not safe:
			desired_tcp[:3] = actual_tcp[:3]
			cart_velocity *= 0.3
			filtered_joints = list(current_joints)
			robot.ServoJ(current_joints, [0] * 6, int(dt * 1000))

			elapsed = time.time() - loop_start
			if elapsed < dt:
				time.sleep(dt - elapsed)
			continue

		# 8) Joint EMA smoothing
		for j in range(6):
			alpha = joint_filter_alpha[j]
			filtered_joints[j] = alpha * target_joints[j] + (1.0 - alpha) * filtered_joints[j]

		# 9) ServoJ
		servo_err = robot.ServoJ(filtered_joints, [0] * 6, int(dt * 1000))

		if verbose:
			print(
				f"idx={nearest_idx:4d} | d_path={path_dist:6.2f} mm | d_actual={radial_dist:6.2f} mm | "
				f"excess={tunnel_excess:6.2f} mm | clamped={was_clamped} | "
				f"R={resist_gain:4.2f} | "
				f"ServoErr={servo_err}"
			)

		# 10) Timing
		elapsed = time.time() - loop_start
		if elapsed < dt:
			time.sleep(dt - elapsed)

	print("\nStopping servo mode...")
	time.sleep(0.05)
	try:
		robot.ServoMoveEnd()
	except Exception as e:
		print(f"ServoMoveEnd warning: {e}")
	if viz_handles is not None:
		try:
			plt.ioff()
			plt.close(viz_handles[0])
		except Exception:
			pass
	print("Exited cleanly.")


def main():
	init_ft_sensor()
	calibrate_baseline_forces()

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

	print("Building TCP tunnel trajectory from FK...")
	try:
		tcp_tunnel_trajectory = build_tcp_trajectory(joint_traj)
	except Exception as e:
		print(f"Failed to build TCP trajectory tunnel: {e}")
		return

	print(f"Trajectory loaded: {len(joint_traj)} joint points")
	print(f"Tunnel centerline points: {len(tcp_tunnel_trajectory)}")

	viz_handles = None
	try:
		viz_handles = visualize_trajectory_with_tunnel(tcp_tunnel_trajectory, TUNNEL_RADIUS_MM)
		if viz_handles is not None:
			print("Visualization window opened: trajectory + tunnel + live TCP marker.")
	except Exception as e:
		print(f"Visualization could not be started: {e}")

	if robot.ServoMoveStart() != 0:
		print("Failed to start servo mode.")
		return

	print(f"Waiting {startup_delay:.1f}s before control starts...")
	time.sleep(startup_delay)

	control_loop(tcp_tunnel_trajectory, viz_handles=viz_handles)


if __name__ == '__main__':
	main()
