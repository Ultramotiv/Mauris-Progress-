import Robot

import time

# Establish a connection with the robot controller and return a robot object if the connection is successful


robot = Robot.RPC('192.168.58.2')

while True:
    ft_data = robot.FT_GetForceTorqueRCS()
    print(ft_data)

    time.sleep(1.0)