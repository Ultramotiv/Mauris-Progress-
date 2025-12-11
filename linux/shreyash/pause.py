import Robot

robot = Robot.RPC('192.168.58.2')

err = robot.PauseMotion()
print("Pause:",err)