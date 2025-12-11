import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
import time
import Robot

# Load CSV Data
csv_path = '/home/um/robot_torque_data_log.csv'
data = pd.read_csv(csv_path)
baseline_torques = data[['j1_cur_tor', 'j2_cur_tor', 'j3_cur_tor', 'j4_cur_tor', 'j5_cur_tor', 'j6_cur_tor']].mean().values

# Connect to robot
robot = Robot.RPC('192.168.58.2')

# Parameters
thresholds = [50.0] * 6
slope_thresholds = [250.0] * 6
previous_torques = [None] * 6
previous_times = [None] * 6

# Data for plotting
time_data = []
torque_data = [[] for _ in range(6)]
spike_data = [[] for _ in range(6)]
derivatives = [[] for _ in range(6)]

plt.ion()
fig, axs = plt.subplots(6, 1, figsize=(10, 15))
slider_fig, slider_axs = plt.subplots(6, 1, figsize=(6, 10))
sensitivity_sliders, slope_sliders = [], []

for i in range(6):
    ax_slider = slider_axs[i]
    slider = Slider(ax_slider, f"Joint {i+1} Torque", 10.0, 200.0, valinit=thresholds[i])
    sensitivity_sliders.append(slider)
    slope_slider = Slider(ax_slider, f"Joint {i+1} Slope", 5.0, 500.0, valinit=slope_thresholds[i])
    slope_sliders.append(slope_slider)

# Main Loop
try:
    start_time = time.time()

    while True:
        current_time = time.time() - start_time
        time_data.append(current_time)

        for i in range(6):
            thresholds[i] = sensitivity_sliders[i].val
            slope_thresholds[i] = slope_sliders[i].val

        # Get current torques
        ret_torque = robot.GetJointTorques()
        current_torques = [ret_torque[1][i] for i in range(6)]
        print("Current joint torques:", current_torques)

        for i in range(6):
            current_torque = current_torques[i]
            torque_data[i].append(current_torque)

            if previous_torques[i] is not None and previous_times[i] is not None:
                delta_t = current_time - previous_times[i]
                if delta_t > 0:
                    torque_derivative = (current_torque - previous_torques[i]) / delta_t
                    derivatives[i].append(torque_derivative)

                    # Collision detection using baseline and slope
                    deviation = abs(current_torque - baseline_torques[i])
                    if deviation > thresholds[i] or abs(torque_derivative) > slope_thresholds[i]:
                        robot.PauseMotion()
                        print(f"⚠️ Collision detected on Joint {i+1}! Deviation: {deviation:.2f}, Slope: {torque_derivative:.2f}")
                        input("Press Enter to resume...")
                        robot.ResumeMotion()
                        print("✅ Resumed")
                        spike_data[i].append(current_torque)
                    else:
                        spike_data[i].append(None)
            else:
                spike_data[i].append(None)

            previous_torques[i] = current_torque
            previous_times[i] = current_time

            # Plot data
            axs[i].cla()
            axs[i].plot(time_data, torque_data[i], label='Torque', color='blue')
            axs[i].scatter(time_data, spike_data[i], color='red', label='Spike', marker='x')
            axs[i].axhline(thresholds[i], color='orange', linestyle='--', label='Torque Threshold')
            axs[i].axhline(slope_thresholds[i], color='purple', linestyle='--', label='Slope Threshold')
            axs[i].set_title(f"Joint {i+1}")
            axs[i].legend(loc='upper right')

        plt.tight_layout()
        plt.pause(0.01)
        time.sleep(0.01)

except KeyboardInterrupt:
    print("Process interrupted. Exiting...")
