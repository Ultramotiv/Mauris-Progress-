import Robot
import subprocess
import time

# Establish a connection with the robot controller and return a robot object if the connection is successful

robot = Robot.RPC('192.168.58.2')

type = 1 # data type, 1-joint position

name = 'E4' # track name

period = 2 # sampling period, 2ms or 4ms or 8ms

di = 0 # di input configuration

do = 0 # do output configuration

ret = robot.SetTPDParam(name, period, di_choose=di) #configure TPD parameters

print("Configuration TPD parameter error code", ret)

robot.Mode(1) # robot cut to manual mode

time.sleep(1)

error = robot.FrictionCompensationOnOff(1)

print("Joint friction compensation switch error code:",error)

lcoeff = [0.001]*6

error = robot.SetFrictionValue_level(lcoeff)

print("Setting Joint Friction Compensation Coefficient - Genuine Error Code:",error)

# fcoeff = [2.0,2.0,2.0,2.0,2.0,2.0,2.0]

# error =robot.SetFrictionValue_freedom(fcoeff)

# print("Setting the joint friction compensation factor - free loading error code:",error)

robot.DragTeachSwitch(1) # robot cuts to drag teach mode

ret = robot.GetActualTCPPose()

print("Get current tool position", ret)

time.sleep(1)

ret = robot.SetTPDStart(name, period, do_choose=do) # start logging the demonstration trajectory

print("Starting to record the demonstration track error code", ret)

time.sleep(10)

ret = robot.SetWebTPDStop() # stop logging the demonstration trajectory

print("Stopped recording of the demonstration track error code", ret)

robot.DragTeachSwitch(0) # robot cuts to non-drag teach mode

# robot.SetTPDDelete('tpd2023') # Delete TPD tracks
