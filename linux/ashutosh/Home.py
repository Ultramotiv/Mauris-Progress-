# this code created on 31st oct 2025
# used to return to home position at 4.277% of 100% speed i.e 7.70 deg/sec speed 

import sys
import time
import threading
import tkinter as tk
from tkinter import messagebox

sys.path.append('/home/um/fairino-python-sdk-main/linux/fairino')
import Robot

ROBOT_IP = '192.168.58.2'

TARGET_JOINTS = [
    2.608,    # J1
    -88.384,  # J2
    127.473,  # J3
    -137.27,  # J4
    -92.275,  # J5
    -90.118   # J6
]

# The SDK works with percentage of the robot's maximum joint speed.
# The maximum joint speed for a Fairino arm is typically 180 deg/s
# (check the robot manual if a different value applies).
MAX_JOINT_SPEED = 180.0          # deg/s   <-- adjust if your model differs
DESIRED_SPEED = 7.70             # deg/s

robot = None


def connect_robot():
    global robot
    if robot is None:
        robot = Robot.RPC(ROBOT_IP)
    return robot


def compute_velocity_percent():
    vel_percent = (DESIRED_SPEED / MAX_JOINT_SPEED) * 100.0
    return round(vel_percent, 3)


def move_home():
    bot = connect_robot()

    current_pose = bot.GetActualJointPosDegree(flag=1)
    print("\nCurrent Joint Position [J1, J2, J3, J4, J5, J6]:")
    print(current_pose)

    vel_percent = compute_velocity_percent()
    ovl_percent = 100.0

    print(f"\nMoving to joint position with {DESIRED_SPEED} deg/s")
    print(f" -> vel = {vel_percent}%   ovl = {ovl_percent}%")

    ret = bot.MoveJ(
        joint_pos=TARGET_JOINTS,   # mandatory
        tool=0,                    # default tool
        user=0,                    # default user frame
        desc_pos=[0.0] * 7,        # default (positive kinematics)
        vel=vel_percent,           # speed cap
        acc=0.0,                   # not implemented yet
        ovl=ovl_percent,           # extra scaling (100 % = no reduction)
        exaxis_pos=[0.0] * 4,      # no external axes
        blendT=-1.0,               # -1 -> blocking motion
        offset_flag=0,
        offset_pos=[0.0] * 6
    )

    current_pose = bot.GetActualJointPosDegree(flag=1)
    print("\nCurrent Joint Position [J1, J2, J3, J4, J5, J6]:")
    print(current_pose)

    if ret == 0:
        print("MoveJ command succeeded – robot reached the target joint position.")
    else:
        print(f"MoveJ failed with error code: {ret}")

    time.sleep(0.5)
    return ret


def create_gui():
    root = tk.Tk()
    root.title("Home Position")
    root.geometry("360x200")

    status_var = tk.StringVar(value="Ready")

    title = tk.Label(root, text="Robot Home Position", font=("Segoe UI", 14, "bold"))
    title.pack(pady=10)

    status_label = tk.Label(root, textvariable=status_var, font=("Segoe UI", 10))
    status_label.pack(pady=5)

    def run_move():
        try:
            ret = move_home()
            if ret == 0:
                status_var.set("Home position reached")
            else:
                status_var.set(f"MoveJ failed (code: {ret})")
                messagebox.showerror("MoveJ Failed", f"MoveJ failed with error code: {ret}")
        except Exception as exc:
            status_var.set("Error")
            messagebox.showerror("Error", str(exc))
        finally:
            home_btn.config(state=tk.NORMAL)

    def on_home_click():
        status_var.set("Moving to home...")
        home_btn.config(state=tk.DISABLED)
        threading.Thread(target=run_move, daemon=True).start()

    home_btn = tk.Button(root, text="Home", font=("Segoe UI", 11, "bold"), command=on_home_click)
    home_btn.pack(pady=15, ipadx=20, ipady=6)

    root.mainloop()


if __name__ == "__main__":
    create_gui()