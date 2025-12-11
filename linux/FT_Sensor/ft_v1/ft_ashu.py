import Robot
import time
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.cm as cm

# === Connect to Robot ===
print("Connecting to robot...")
robot = Robot.RPC('192.168.58.2')

# === FT Sensor Setup ===
print("Initializing FT sensor...")
config = robot.FT_GetConfig()
print("Sensor config:", config)
time.sleep(1)

robot.FT_Activate(0)
time.sleep(1)
robot.FT_Activate(1)
time.sleep(1)

robot.SetLoadCoord(0.0, 0.0, 0.0)
robot.FT_SetZero(0)
time.sleep(1)

# === Get Initial Pose ===
ret = robot.GetActualTCPPose()
pose = ret[1]  # [X, Y, Z, RX, RY, RZ]
tool = 0
user = 0

# === Parameters ===
force_to_mm = 1             # mm per N
torque_to_deg = 1            # deg per Nm
min_force_threshold = 0.2 #0.3  # Ignore forces smaller than 0.3N (noise filtering)
max_move_mm =  10           # max linear move per axis (mm)
max_rot_deg = 8              # max rotation per axis (deg)

prev_F_res = 0.0

# === 3D Plot Setup ===
plt.ion()
fig = plt.figure(figsize=(8, 8))
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim([-50, 50])
ax.set_ylim([-50, 50])
ax.set_zlim([-50, 50])
ax.set_xlabel('X (←Fz)')
ax.set_ylabel('Y (Fy)')
ax.set_zlabel('Z (Fx)')
cmap = cm.get_cmap('coolwarm')

# === Main Loop ===
while True:
    data = robot.FT_GetForceTorqueOrigin()

    if data and len(data[1]) == 6:
        Fx, Fy, Fz, Mx, My, Mz = data[1]

        # --- Map Forces to Translation ---
        dx = -Fz * force_to_mm   # Fz → -X
        dy = Fy * force_to_mm    # Fy → Y
        dz = Fx * force_to_mm    # Fx → Z

        dx = np.clip(dx, -max_move_mm, max_move_mm)
        dy = np.clip(dy, -max_move_mm, max_move_mm)
        dz = np.clip(dz, -max_move_mm, max_move_mm)

        # --- Map Torques to Rotation (in degrees) ---
        dRx = Mx * torque_to_deg
        dRy = My * torque_to_deg
        dRz = Mz * torque_to_deg

        dRx = np.clip(dRx, -max_rot_deg, max_rot_deg)
        dRy = np.clip(dRy, -max_rot_deg, max_rot_deg)
        dRz = np.clip(dRz, -max_rot_deg, max_rot_deg)

        # --- Calculate Force Magnitude ---
        F_res = np.sqrt(Fx**2 + Fy**2 + Fz**2)

        if abs(F_res - prev_F_res) > min_force_threshold:
            # Update position
            pose[0] += dx
            pose[1] += dy
            pose[2] += dz

            # Update orientation
            pose[3] += dRx
            pose[4] += dRy
            pose[5] += dRz

            ret = robot.MoveL(pose, tool, user, vel=10)
            print(f"Moved: ΔX={dx:.2f}mm ΔY={dy:.2f}mm ΔZ={dz:.2f}mm | "
                  f"ΔRx={dRx:.2f}° ΔRy={dRy:.2f}° ΔRz={dRz:.2f}° | Code: {ret}")

        prev_F_res = F_res

        # === Visualization ===
        ax.cla()
        ax.set_xlim([-50, 50])
        ax.set_ylim([-50, 50])
        ax.set_zlim([-50, 50])
        ax.set_xlabel('X (←Fz)')
        ax.set_ylabel('Y (Fy)')
        ax.set_zlabel('Z (Fx)')

        # Force Vector
        force_color = cmap(plt.Normalize(vmin=0, vmax=100)(F_res))
        ax.quiver(0, 0, 0, -Fz, Fy, Fx, color=force_color, length=F_res/10, normalize=False, linewidth=2)
        ax.text(-Fz, Fy, Fx, f"F({Fx:.1f}, {Fy:.1f}, {Fz:.1f})", color='black')

        # Torque Vector
        torque_mag = np.sqrt(Mx**2 + My**2 + Mz**2)
        ax.quiver(0, 0, 0, Mx, My, Mz, color='green', length=torque_mag/10, normalize=False, linewidth=1.5)
        ax.text(Mx, My, Mz, f"M({Mx:.1f}, {My:.1f}, {Mz:.1f})", color='green')

        ax.set_title(f"Force Mag: {F_res:.2f} N | Torque Mag: {torque_mag:.2f} Nm")
        plt.pause(0.01)
    else:
        print("Invalid FT data.")

    time.sleep(0.3)
