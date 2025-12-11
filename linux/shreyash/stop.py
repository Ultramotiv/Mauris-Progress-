import Robot

robot = Robot.RPC('192.168.58.2')

err = robot.StopMotion()
ret = robot.ResetAllError()
print("Error:",ret)

print("Stop:",err)
