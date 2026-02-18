import math
import os
import signal
import sys
import threading
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
TUNNEL_DAMPING_WHEN_CLAMPED = 0.25

# Inside-tunnel guidance behavior:
# - Tangential force should move freely along the trajectory
# - Normal/radial force should feel resistant (softly allowed, strongly damped)
TANGENTIAL_FORCE_RATIO = 1.0
NORMAL_FORCE_RATIO = 0.05
CENTERLINE_STIFFNESS = 0.08
NORMAL_VELOCITY_DAMPING = 0.35

# TCP measurement smoothing (to avoid jitter when updating current TCP in-loop)
TCP_FILTER_ALPHA = 0.22
TCP_POS_DEADBAND_MM = 0.12
TCP_ANG_DEADBAND_DEG = 0.06
TCP_MAX_POS_STEP_MM = 2.0
TCP_MAX_ANG_STEP_DEG = 0.35


# -----------------------------------------------------------------------------
# Global state
# -----------------------------------------------------------------------------
baseline_forces = None
shutdown_requested = False
gui_stop_event = threading.Event()
latest_tcp_pose_xyz = None
tcp_pose_lock = threading.Lock()


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


def get_tunnel_tangent(tcp_tunnel_xyz, idx):
	n = len(tcp_tunnel_xyz)
	if n < 2:
		return np.array([1.0, 0.0, 0.0], dtype=float)

	if idx <= 0:
		tangent = tcp_tunnel_xyz[1] - tcp_tunnel_xyz[0]
	elif idx >= n - 1:
		tangent = tcp_tunnel_xyz[-1] - tcp_tunnel_xyz[-2]
	else:
		tangent = tcp_tunnel_xyz[idx + 1] - tcp_tunnel_xyz[idx - 1]

	tn = float(np.linalg.norm(tangent))
	if tn < 1e-9:
		return np.array([1.0, 0.0, 0.0], dtype=float)
	return tangent / tn


def smooth_tcp_measurement(filtered_tcp, measured_tcp):
	"""EMA + deadband + slew-rate limiting for stable TCP updates."""
	if filtered_tcp is None:
		return np.array(measured_tcp, dtype=float)

	measured = np.array(measured_tcp, dtype=float)
	prev = np.array(filtered_tcp, dtype=float)
	delta = measured - prev

	# Deadband to suppress encoder/estimation micro-noise
	for i in range(3):
		if abs(delta[i]) < TCP_POS_DEADBAND_MM:
			delta[i] = 0.0
	for i in range(3, 6):
		if abs(delta[i]) < TCP_ANG_DEADBAND_DEG:
			delta[i] = 0.0

	# Slew limit per control cycle to prevent jumps
	delta[:3] = np.clip(delta[:3], -TCP_MAX_POS_STEP_MM, TCP_MAX_POS_STEP_MM)
	delta[3:6] = np.clip(delta[3:6], -TCP_MAX_ANG_STEP_DEG, TCP_MAX_ANG_STEP_DEG)

	return prev + TCP_FILTER_ALPHA * delta


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
	(live_tcp_marker,) = ax.plot([xs[0]], [ys[0]], [zs[0]], marker='o', markersize=7, linestyle='None', color='magenta', label='Live TCP')

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
	return fig, ax, live_tcp_marker


def start_tcp_gui_update_thread(viz_handles, poll_dt=0.03):
	"""Start a separate thread that polls CurrentTCP and updates GUI marker."""
	if viz_handles is None or len(viz_handles) < 3:
		return None

	fig, _ax, live_tcp_marker = viz_handles
	global latest_tcp_pose_xyz

	def _gui_tick():
		if gui_stop_event.is_set() or shutdown_requested:
			return False
		with tcp_pose_lock:
			xyz = None if latest_tcp_pose_xyz is None else latest_tcp_pose_xyz.copy()
		if xyz is not None:
			live_tcp_marker.set_data_3d([xyz[0]], [xyz[1]], [xyz[2]])
			fig.canvas.draw_idle()
		return True

	# GUI updates must run on GUI thread; timer callback reads latest polled TCP.
	timer = fig.canvas.new_timer(interval=max(10, int(poll_dt * 1000)))
	timer.add_callback(_gui_tick)
	fig._tcp_gui_timer = timer
	timer.start()

	def _worker():
		global latest_tcp_pose_xyz
		while not gui_stop_event.is_set() and not shutdown_requested:
			try:
				err, actual_tcp_list = robot.GetActualTCPPose(flag=1)
				if err == 0 and actual_tcp_list is not None and len(actual_tcp_list) >= 3:
					xyz = np.array([
						float(actual_tcp_list[0]),
						float(actual_tcp_list[1]),
						float(actual_tcp_list[2])
					], dtype=float)
					with tcp_pose_lock:
						latest_tcp_pose_xyz = xyz
			except Exception:
				pass
			time.sleep(poll_dt)

	t = threading.Thread(target=_worker, name="tcp-gui-updater", daemon=True)
	t.start()
	return t


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
	gui_stop_event.set()
	print("\n[Ctrl+C] Shutdown requested...")


signal.signal(signal.SIGINT, shutdown)


# -----------------------------------------------------------------------------
# Main control loop
# -----------------------------------------------------------------------------
def control_loop(tcp_tunnel_trajectory, viz_handles=None, joint_ref_trajectory=None):
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
	filtered_actual_tcp = np.array(actual_tcp_list, dtype=float)

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
		filtered_actual_tcp = smooth_tcp_measurement(filtered_actual_tcp, actual_tcp)

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
		world_forces = ft_to_world(comp_forces, filtered_actual_tcp[3:6])

		# Build local trajectory frame at nearest point to current TCP
		d2_actual = np.sum((tcp_tunnel_trajectory - filtered_actual_tcp[:3]) ** 2, axis=1)
		nearest_idx_actual = int(np.argmin(d2_actual))
		centerline_xyz = tcp_tunnel_trajectory[nearest_idx_actual]
		tangent_hat = get_tunnel_tangent(tcp_tunnel_trajectory, nearest_idx_actual)

		# Decompose world linear force into tangential + normal components
		f_lin_world = np.array(world_forces[:3], dtype=float)
		f_tan = float(np.dot(f_lin_world, tangent_hat)) * tangent_hat
		f_norm = f_lin_world - f_tan

		# 4) Cartesian impedance update
		f_scaled = np.zeros(6, dtype=float)
		f_scaled[:3] = (
			f_tan * (force_to_translation * TANGENTIAL_FORCE_RATIO)
			+ f_norm * (force_to_translation * NORMAL_FORCE_RATIO)
		)
		f_scaled[3:6] = world_forces[3:6] * force_to_rotation

		# Add centerline restoring force so off-trajectory motion feels resistant
		radial_vec = filtered_actual_tcp[:3] - centerline_xyz
		f_scaled[:3] += -CENTERLINE_STIFFNESS * radial_vec

		lin_force_mag = float(np.linalg.norm(world_forces[:3]))
		ang_force_mag = float(np.linalg.norm(world_forces[3:6]))

		for i in range(6):
			spring_force = -K_cart[i] * (desired_tcp[i] - filtered_actual_tcp[i])
			damping_force = -B_cart[i] * cart_velocity[i]

			if (i < 3 and lin_force_mag > force_threshold) or (i >= 3 and ang_force_mag > force_threshold):
				ext_force = f_scaled[i]
			else:
				ext_force = 0.0

			total_force = ext_force + spring_force + damping_force
			acc = total_force / M_cart[i] if M_cart[i] > 0 else 0.0
			cart_velocity[i] += acc * dt

		# Keep linear velocity mostly tangential; damp normal velocity strongly
		v_lin = np.array(cart_velocity[:3], dtype=float)
		v_tan = float(np.dot(v_lin, tangent_hat)) * tangent_hat
		v_norm = v_lin - v_tan
		cart_velocity[:3] = v_tan + (1.0 - NORMAL_VELOCITY_DAMPING) * v_norm

		cart_velocity[:3] = np.clip(cart_velocity[:3], -max_linear_vel, max_linear_vel)
		cart_velocity[3:6] = np.clip(cart_velocity[3:6], -max_angular_vel, max_angular_vel)

		candidate_tcp = desired_tcp + cart_velocity * dt

		# Keep orientation stable from actual TCP (no orientation drift)
		candidate_tcp[3:6] = desired_tcp[3:6]

		# 5) Tunnel hard clamp on XYZ
		clamped_xyz, nearest_idx, path_dist, tunnel_excess, was_clamped = clamp_xyz_to_tunnel(
			candidate_tcp[:3], tcp_tunnel_trajectory, TUNNEL_RADIUS_MM
		)
		candidate_tcp[:3] = clamped_xyz

		if was_clamped:
			# Dampen linear velocity strongly when user pushes outside tunnel
			cart_velocity[:3] *= TUNNEL_DAMPING_WHEN_CLAMPED

		desired_tcp = candidate_tcp

		# 6) IK (use trajectory joint reference when available to preserve joint behavior)
		ik_result = None
		if joint_ref_trajectory is not None and len(joint_ref_trajectory) > nearest_idx:
			joint_ref = [float(v) for v in joint_ref_trajectory[nearest_idx][:6]]
			ik_result = robot.GetInverseKinRef(0, list(desired_tcp), joint_ref)
			if isinstance(ik_result, int) or (isinstance(ik_result, (list, tuple)) and ik_result[0] != 0):
				# Fallback to standard IK if reference-based IK fails
				ik_result = robot.GetInverseKin(0, list(desired_tcp), -1)
		else:
			ik_result = robot.GetInverseKin(0, list(desired_tcp), -1)
		ik_ok = isinstance(ik_result, (list, tuple)) and len(ik_result) >= 2 and ik_result[0] == 0

		if not ik_ok:
			if verbose:
				ik_err = ik_result if isinstance(ik_result, int) else ik_result[0]
				print(f"IK failed (err={ik_err}), holding current joints.")
			desired_tcp[:3] = filtered_actual_tcp[:3]
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
			desired_tcp[:3] = filtered_actual_tcp[:3]
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
				f"idx={nearest_idx:4d} | d_path={path_dist:6.2f} mm | "
				f"excess={tunnel_excess:6.2f} mm | clamped={was_clamped} | "
				f"|F_tan|={np.linalg.norm(f_tan):5.2f}N |F_norm|={np.linalg.norm(f_norm):5.2f}N | "
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
			gui_stop_event.set()
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
	tcp_gui_thread = None
	try:
		gui_stop_event.clear()
		viz_handles = visualize_trajectory_with_tunnel(tcp_tunnel_trajectory, TUNNEL_RADIUS_MM)
		if viz_handles is not None:
			tcp_gui_thread = start_tcp_gui_update_thread(viz_handles)
			print("Visualization window opened: trajectory + tunnel + live TCP marker.")
	except Exception as e:
		print(f"Visualization could not be started: {e}")

	if robot.ServoMoveStart() != 0:
		print("Failed to start servo mode.")
		return

	print(f"Waiting {startup_delay:.1f}s before control starts...")
	time.sleep(startup_delay)

	control_loop(tcp_tunnel_trajectory, viz_handles=viz_handles, joint_ref_trajectory=joint_traj)


if __name__ == '__main__':
	main()
