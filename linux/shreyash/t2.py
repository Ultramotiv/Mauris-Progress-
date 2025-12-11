import Robot

# Establish a connection with the robot controller and return a robot object if the connection is successful

robot = Robot.RPC('192.168.58.2')

rcs = robot.FT_GetForceTorqueRCS() #Query data in sensor coordinate system

print(rcs)