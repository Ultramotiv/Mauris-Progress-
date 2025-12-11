import Robot
import time

# Connect to the robot controller
robot = Robot.RPC('192.168.58.2')

# Get current TCP pose
current_pose = robot.GetActualTCPPose()
print("\nCurrent Tool Position [X, Y, Z, RX, RY, RZ]:")
print(current_pose)

# Extract pose from the returned tuple
pose_list = current_pose[1]  # [X, Y, Z, RX, RY, RZ]

# Ask user for movement direction and distance
direction = input("\nEnter direction to move (X/Y/Z): ").strip().upper()
distance_cm = float(input("Enter distance to move in cm: "))

# Convert cm to mm
distance_mm = distance_cm * 10

# Copy current pose to target
target_pose = pose_list[:]  # Proper list copy

# Apply offset to selected axis
if direction == "X":
    target_pose[0] += distance_mm
elif direction == "Y":
    target_pose[1] += distance_mm
elif direction == "Z":
    target_pose[2] += distance_mm
else:
    print("Invalid direction entered. Use X, Y, or Z.")
    exit()

# Move in Cartesian space
tool = 0
user = 0
ret = robot.MoveCart(target_pose, tool, user, vel=50)
print("\nMoving robot in", direction, "direction by", distance_cm, "cm. Error code:", ret)

# Get and print new position
new_pose = robot.GetActualTCPPose()
print("\nNew Tool Position [X, Y, Z, RX, RY, RZ]:")
print(new_pose)
