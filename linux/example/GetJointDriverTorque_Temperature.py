import Robot

import time

# Establish a connection with the robot controller and return a robot object if the connection is successful

robot = Robot.RPC('192.168.58.2')

company = 17 #Sensor Manufacturer, 17-Kunwit Technology

device = 0 # sensor device number

error = robot.FT_SetConfig(company, device) #Configure force sensors

print("Configuring force sensor error code",error)

config = robot.FT_GetConfig() #Get force sensor configuration information

print('Get force sensor configuration information',config)

time.sleep(1)

error = robot.FT_Activate(0) # sensor reset

print("Sensor reset error code",error)

time.sleep(1)

error = robot.FT_Activate(1) #sensor activation

print("Sensor activation error code",error)

time.sleep(1)

error = robot.SetLoadWeight(0.0) # end load set to zero

print("End load set to zero error code",error)

time.sleep(1)

error = robot.SetLoadCoord(0.0,0.0,0.0) # end load center of mass set to zero

print("End center of mass set to zero error code",error)

time.sleep(1)

error = robot.FT_SetZero(0) #sensor de-zeroing

print("Sensor Remove Zero Error Code",error)

time.sleep(1)

error = robot.FT_GetForceTorqueOrigin() #Query sensor raw data

print("Querying sensor raw data",error)

error = robot.FT_SetZero(1) #Sensor zero correction, note that at this time the end can not be installed tools, only the force sensor

print("Sensor zero correction",error)

time.sleep(1)

error = robot.FT_GetForceTorqueRCS() #Query data in sensor coordinate system

print("Querying data in sensor coordinate system",error)