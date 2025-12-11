import Robot

import time

# Establish a connection with the robot controller and return a robot object if the connection is successful

robot = Robot.RPC('192.168.58.2')

# P1=[-321.821, 125.694, 282.556, 174.106, -15.599, 152.669]

name = 'tpd2023' #track name

blend = 1 # whether to smooth, 1-smooth, 0-not smooth

ovl = 200.0 #speed scaling

ret = robot.LoadTPD(name) # track preloading

print("track preload error code",ret)

ret,P1 = robot.GetTPDStartPose(name) #Get trajectory start pose

print ("Get trajectory start position error code",ret, "start position",P1)

ret = robot.MoveL(P1,0,0) #move to start point

print("Movement to start point error code",ret)

time.sleep(10)

ret = robot.MoveTPD(name, blend, ovl) # trajectory replication

print("Trajectory reproduction error code",ret)