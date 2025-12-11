import Robot

# Establish a connection with the robot controller and return a robot object if the connection is successful

robot = Robot.RPC('192.168.58.2')



level = [20.0,20.0,20.0,20.0,20.0,20.0]

error = robot.SetAnticollision(1,level,1)

print("Setting collision level error code:",error)

error = robot.SetCollisionStrategy(strategy=2)

print("Setting post-collision policy error code:",error)

error = robot.FrictionCompensationOnOff(1)

print("Joint friction compensation switch error code:",error)



fcoeff = [0.1,0.1,0.1,0.1,0.1,0.1]

error =robot.SetFrictionValue_freedom(fcoeff)

print("Setting the joint friction compensation factor - free loading error code:",error)