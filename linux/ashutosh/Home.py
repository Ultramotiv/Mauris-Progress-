# this code created on 31st oct 2025
# used to return to home position at 4.277% of 100% speed i.e 7.70 deg/sec speed 

import sys
sys.path.append('/home/um/fairino-python-sdk-main/linux/fairino')
import Robot
import time

robot = Robot.RPC('192.168.58.2')

current_pose = robot.GetActualJointPosDegree(flag=1)
print("\nCurrent Joint Position [J1, J2, J3, J4, J5, J6]:")
print(current_pose)


target_joints = [
    2.608,    # J1
    -88.384,  # J2
    127.473,  # J3
    -137.27,  # J4
    -92.275,  # J5
    -90.118   # J6
]

# The SDK works with **percentage of the robot's maximum joint speed**.
# The maximum joint speed for a Fairino arm is typically 180 deg/s
# (check the robot manual if a different value applies).
MAX_JOINT_SPEED = 180.0          # deg/s   <-- adjust if your model differs

desired_speed = 7.70             # deg/s
vel_percent   = (desired_speed / MAX_JOINT_SPEED) * 100.0   # -> 4.277...
ovl_percent   = 100.0            # keep full scaling (no extra reduction)

# Round to a sensible number of decimals (the SDK expects float)
vel_percent = round(vel_percent, 3)   # 4.278

print(f"\nMoving to joint position with {desired_speed} deg/s")
print(f" -> vel = {vel_percent}%   ovl = {ovl_percent}%")

ret = robot.MoveJ(
    joint_pos = target_joints,   # mandatory
    tool      = 0,               # default tool
    user      = 0,               # default user frame
    desc_pos  = [0.0]*7,         # default (positive kinematics)
    vel       = vel_percent,     # **speed cap**
    acc       = 0.0,             # not implemented yet
    ovl       = ovl_percent,     # extra scaling (100 % = no reduction)
    exaxis_pos= [0.0]*4,         # no external axes
    blendT    = -1.0,            # -1 -> blocking motion
    offset_flag=0,
    offset_pos=[0.0]*6
)

current_pose = robot.GetActualJointPosDegree(flag=1)
print("\nCurrent Joint Position [J1, J2, J3, J4, J5, J6]:")
print(current_pose)

if ret == 0:
    print("MoveJ command succeeded – robot reached the target joint position.")
else:
    print(f"MoveJ failed with error code: {ret}")

# give the controller a moment to settle before the next command
time.sleep(0.5)