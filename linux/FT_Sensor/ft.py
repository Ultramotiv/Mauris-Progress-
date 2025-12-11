
import Robot
import time
import math

try:
    robot = Robot.RPC('192.168.58.2')
    print("Successfully connected to the robot.")
except Exception as e:
    print(f"ERROR: Failed to connect to robot: {e}")
    exit()

try:
    company = 24
    device = 0
    robot.FT_SetConfig(company, device)
    robot.FT_Activate(0)
    time.sleep(0.5)
    robot.FT_Activate(1)
    time.sleep(0.5)
    robot.SetLoadWeight(0, 0.0)
    robot.SetLoadCoord(0.0, 0.0, 0.0)
    print("Zeroing FT sensor...")
    robot.FT_SetZero(0)
    time.sleep(0.5)
    robot.FT_SetZero(1)
    time.sleep(0.5)
    print("FT Sensor initialized and zeroed.")
except Exception as e:
    print(f"ERROR: Could not initialize FT Sensor: {e}")
    if 'robot' in locals():
        robot.close()
    exit()

try:
    initial_pos_data = robot.GetActualToolFlangePose()
    if initial_pos_data[0] != 0:
        print(f"ERROR: Failed to get initial pose, code: {initial_pos_data[0]}")
        raise Exception("Failed to get initial pose")
    initial_pos = list(initial_pos_data[1])
    print(f"Initial Pose (mm, rad): {initial_pos}")
except Exception as e:
    print(f"CRITICAL ERROR: {e}")
    if 'robot' in locals():
        robot.close()
    exit()

M_unused = 0.4
B_unused = 150.0
K_unused = 0.3

dt = 0.01
tool = 0
user = 0

position_threshold_for_move = 0.2
force_activate_threshold = 1.6
min_disp_on_force_activation = 5.0
force_to_disp_scale_factor = 10.0

max_total_disp_mm = 1000.0

Kp = 25.0
Kd = 8.0

print(f"Control Params: Kp={Kp}, Kd={Kd}, ForceThreshold={force_activate_threshold}N, ScaleFactor={force_to_disp_scale_factor}mm/N")
print(f"Loop dt={dt}s")

current_disp_mm = [0.0, 0.0, 0.0]
current_vel_mm_s = [0.0, 0.0, 0.0]
prev_sent_disp_mm = [0.0, 0.0, 0.0]

try:
    while True:
        loop_start_realtime = time.perf_counter()
        ft_raw_values = robot.FT_GetForceTorqueRCS()
        if ft_raw_values[0] != 0:
            print(f"Warning: FT Sensor read error (Code: {ft_raw_values[0]}). Skipping cycle.")
            time.sleep(dt)
            continue

        external_forces_N = [-ft_raw_values[1][0], ft_raw_values[1][1], -ft_raw_values[1][2]]

        for i in range(3):
            F_ext_axis = external_forces_N[i]
            target_disp_for_pd = 0.0

            # Force range for admittance activation
            force_activate_min = 1.6   # N: below this, considered noise or idle
            force_activate_max = 15.0 # N: above this, considered too strong (e.g., impact)

            if force_activate_min < abs(F_ext_axis) < force_activate_max:
                force_over_threshold = abs(F_ext_axis) - force_activate_min
                calculated_disp_from_force = min_disp_on_force_activation + (force_over_threshold * force_to_disp_scale_factor)
                target_disp_for_pd = math.copysign(calculated_disp_from_force, F_ext_axis)
            else:
                # Hold current compliant displacement if outside force activation range
                target_disp_for_pd = current_disp_mm[i]


            error_disp = target_disp_for_pd - current_disp_mm[i]
            acceleration_disp = Kp * error_disp - Kd * current_vel_mm_s[i]

            current_vel_mm_s[i] += acceleration_disp * dt
            current_disp_mm[i] += current_vel_mm_s[i] * dt
            current_disp_mm[i] = max(min(current_disp_mm[i], max_total_disp_mm), -max_total_disp_mm)

        vel_mag_disp = math.sqrt(sum(v**2 for v in current_vel_mm_s))
        base_move_vel = 10.0
        max_move_vel = 100.0
        robot_command_velocity = base_move_vel + min(vel_mag_disp * 10.0, max_move_vel - base_move_vel)

        target_robot_pose = [
            initial_pos[0] + current_disp_mm[0],
            initial_pos[1] + current_disp_mm[1],
            initial_pos[2] + current_disp_mm[2],
            initial_pos[3], initial_pos[4], initial_pos[5]
        ]

        send_new_command = any(abs(current_disp_mm[i] - prev_sent_disp_mm[i]) > position_threshold_for_move for i in range(3))

        if send_new_command:
            blend_time_param = 0.0
            move_accel_param = 100.0

            err_code = robot.MoveCart(target_robot_pose, tool, user, vel=robot_command_velocity, acc=move_accel_param, ovl=100.0, blendT=blend_time_param)
            if err_code != 0:
                print(f"MoveCart Error Code: {err_code}")
            prev_sent_disp_mm = list(current_disp_mm)

        loop_exec_time = time.perf_counter() - loop_start_realtime
        if loop_exec_time < dt:
            time.sleep(dt - loop_exec_time)

except KeyboardInterrupt:
    print("\nAdmittance control stopped by user (KeyboardInterrupt).")
except Exception as e:
    print(f"\nAn unhandled error occurred: {e}")
finally:
    print("Exiting: Sending StopMotion command to robot...")
    if 'robot' in locals() and robot is not None:
        try:
            stop_err = robot.StopMotion()
            print(f"StopMotion command sent, response: {stop_err}")
            time.sleep(0.1)
            print("Robot should be stopped. Please verify.")
        except Exception as e_stop:
            print(f"ERROR during robot stop/disconnect: {e_stop}")
    else:
        print("Robot object not available. Manual check of robot state is advised.")
    print("Program terminated.")