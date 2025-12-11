import Robot
import time

# Establish a connection with the robot controller
robot = Robot.RPC('192.168.58.2')

try:
    while True:
        ret_torque = robot.GetJointTorques()
        print("Current joint torques:", ret_torque[1])

        angle = robot.GetActualJointPosRadian()
        print("Current Robot Joint Angles",angle[1])

        ret = robot.GetActualJointSpeedsDegree()
        print("Getting joint feedback speed -deg/s", ret[1])

        ret = robot.GetActualTCPSpeed()
        print("Getting TCP feedback speed", ret[1])

        print("-" * 50)
        time.sleep(0.01)  # Adjust the interval as needed

except KeyboardInterrupt:
    print("Process interrupted. Exiting...")
