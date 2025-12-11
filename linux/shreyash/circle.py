import Robot

# Establish a connection with the robot controller and return a robot object if the connection is successful

robot = Robot.RPC('192.168.58.2')

desc_pos2 = [236.794,-475.119, 65.379, -176.938, 2.535, -179.829]

desc_posc3 = [256.794,-435.119, 65.379, -176.938, 2.535, -179.829] #Circle path points

desc_posc4 = [286.794,-475.119, 65.379, -176.938, 2.535, -179.829] #Circle target points

tool = 0#Tool coordinate system number

user = 0 #Workpiece coordinate system number

robot.MoveL(desc_pos2, tool, user, vel=40, acc=100)



ret = robot.Circle(desc_posc3, tool, user, desc_posc4, tool, user, vel_t=40, offset_flag=1, offset_pos=[5,10,15,0,0,1]) #Cartesian space circular motion

print("Circular motion in Cartesian space: error code", ret) #Circular motion in Cartesian space
