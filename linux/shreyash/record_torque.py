import Robot
import time
import csv

def record_torques(robot, filename='joint_torques.csv', interval=0.1):
    print("Monitoring drag teach mode. Waiting to start recording...")

    while True:
        ret, state = robot.IsInDragTeach()

        # Start recording when in drag teach mode
        if ret == 0 and state == 1:
            print("Drag teach mode detected. Starting to record torques.")
            with open(filename, mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(['Timestamp', 'Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6'])

                start_time = time.time()
                while True:
                    ret, state = robot.IsInDragTeach()
                    if ret != 0 or state != 1:
                        print("Exited drag teach mode. Stopping recording.")
                        break

                    ret_torque = robot.GetJointTorques()
                    current_torques = [ret_torque[1][i] for i in range(6)]
                    timestamp = time.time() - start_time

                    print(f"Time: {timestamp:.2f}s, Torques: {current_torques}")
                    writer.writerow([timestamp] + current_torques)
                    time.sleep(interval)

                print(f"Torques recorded to {filename}")
        else:
            time.sleep(0.1)

# Connect to the robot
robot = Robot.RPC('192.168.58.2')
record_torques(robot, filename='joint_torques.csv', interval=0.1)
