import Robot
import time

# Establish a connection with the robot controller and return a robot object if the connection is successful

robot = Robot.RPC('192.168.58.2')


def get_current_joint_angles():
    ret = robot.GetActualJointPosDegree()
    print("current joint angles;",ret)
    return ret[1] if ret[0] == 0 else []


def get_joint_soft_limits():
    ret = robot.GetJointSoftLimitDeg()
    if ret[0] != 0:
        return []
    # Convert flat list to tuple pairs (min, max)
    soft_limits = list(zip(ret[1][::2], ret[1][1::2]))
    return soft_limits

def monitor_joint_angles(threshold=10.0):
    """ Continuously monitors joint angles and prints directions to avoid exceeding soft limits. """
    soft_limits = get_joint_soft_limits()
    if not soft_limits:
        print("Failed to retrieve soft limits.")
        return

    while True:
        joint_angles = get_current_joint_angles()
        if not joint_angles:
            print("Failed to retrieve joint angles.")
            return

        for i, (angle, (min_limit, max_limit)) in enumerate(zip(joint_angles, soft_limits)):
            # Check proximity to soft limits
            if angle <= min_limit + threshold:
                print(f"Joint {i+1} is near the minimum limit. Rotate positively to avoid exceeding.")
            elif angle >= max_limit - threshold:
                print(f"Joint {i+1} is near the maximum limit. Rotate negatively to avoid exceeding.")
        
        time.sleep(1.0)  # Adjust polling frequency as needed

if __name__ == "__main__":
    monitor_joint_angles()