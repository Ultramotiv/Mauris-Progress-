import Robot

import time

# Establish a connection with the robot controller and return a robot object if the connection is successful

robot = Robot.RPC('192.168.58.2')

joint_pos4 = [-83.24, -96.476, 93.688, -114.079, -62, -100]


tool = 0 #Tool coordinate system number

user = 0 #Workpiece coordinate system number

ret = robot.MoveJ(joint_pos4, tool, user, vel=10) #joint space motion
