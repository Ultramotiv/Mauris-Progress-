# DMP_Traject.py (to create CSV file of the demo trajectory)
# this code is developed on 24th oct 2025
# drag mode with custom impedence control + gravity compensation + joint recording
# trial 1 for storing the csv file

import sys
sys.path.append('/home/um/fairino-python-sdk-main/linux/fairino')
import Robot
import time
import threading
import csv
from datetime import datetime

# Establish connection with robot controller
robot = Robot.RPC('192.168.58.2')

# SINGLE SET OF IMPEDANCE PARAMETERS - MODIFY THESE VALUES ONLY
IMPEDANCE_PARAMS = {
    'lamde_dain' : [2.5, 2.0, 2.0, 2.0, 2.0, 2.0],
    'b_gain' : [20.0, 10.0, 10.0, 5.0, 5.0, 1.0],
    'k_gain' : [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    'max_tcp_vel': 500,      # Maximum linear velocity limit (mm/s)
    'max_tcp_ori_vel': 90    # Maximum angular velocity limit (deg/s)
}

# GRAVITY COMPENSATION PARAMETERS FOR JOINTS 1 AND 2
GRAVITY_COMPENSATION = {
    'joint1': 600,
    'joint2': 400,
    'joint3': 150,
    'joint4': 100,
    'joint5': 100,
    'joint6': 100
}

# Recording control variables
recording_active = False
recording_data = []
recording_thread = None

def custom_drag_teach_mode(enable=True):
    """
    Custom impedance control with gravity compensation for smooth joint movement
    
    Args:
        enable (bool): True to enable, False to disable
    """
    
    if enable:
        print("Enabling custom impedance control with gravity compensation...")
        print(f"Using impedance parameters: {IMPEDANCE_PARAMS}")
        print(f"Using gravity compensation: {GRAVITY_COMPENSATION}")
        
        # First enable drag teach mode (this is required)
        robot.DragTeachSwitch(1)
        time.sleep(0.5)
        
        # Apply gravity compensation for joints 1 and 2
        gravity_values = [
            GRAVITY_COMPENSATION['joint1'],
            GRAVITY_COMPENSATION['joint2'], 
            GRAVITY_COMPENSATION['joint3'],
            GRAVITY_COMPENSATION['joint4'],
            GRAVITY_COMPENSATION['joint5'],
            GRAVITY_COMPENSATION['joint6']
        ]
        
        try:
            # Enable gravity compensation
            rtn_gravity = robot.SetGravityComp(1, gravity_values)
            if rtn_gravity == 0:
                print("Gravity compensation enabled successfully")
            else:
                print(f"Warning: Gravity compensation error: {rtn_gravity}")
        except Exception as e:
            print(f"Gravity compensation not available or error: {e}")
            print("Continuing without gravity compensation...")
        
        # Apply custom impedance parameters
        rtn = robot.ForceAndJointImpedanceStartStop(
            status=1,
            impedanceFlag=1,
            lamdeDain=IMPEDANCE_PARAMS['lamde_dain'],
            KGain=IMPEDANCE_PARAMS['k_gain'], 
            BGain=IMPEDANCE_PARAMS['b_gain'],
            dragMaxTcpVel=IMPEDANCE_PARAMS['max_tcp_vel'],
            dragMaxTcpOriVel=IMPEDANCE_PARAMS['max_tcp_ori_vel']
        )
        
        if rtn == 0:
            print("Custom impedance control enabled successfully")
            print("Joints 1 and 2 should now feel smoother with gravity compensation")
        else:
            print(f"Error enabling custom impedance control: {rtn}")
            
    else:
        print("Disabling custom impedance control and gravity compensation...")
        
        # Turn off gravity compensation first
        try:
            rtn_gravity = robot.SetGravityComp(0, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            if rtn_gravity == 0:
                print("Gravity compensation disabled successfully")
            else:
                print(f"Warning: Gravity compensation disable error: {rtn_gravity}")
        except Exception as e:
            print(f"Gravity compensation disable error: {e}")
        
        # Turn off custom impedance parameters
        rtn = robot.ForceAndJointImpedanceStartStop(
            status=0,
            impedanceFlag=0,
            lamdeDain=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            KGain=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            BGain=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            dragMaxTcpVel=1000,
            dragMaxTcpOriVel=180
        )
        
        # Then disable drag teach mode
        robot.DragTeachSwitch(0)
        time.sleep(0.5)
        
        if rtn == 0:
            print("Custom impedance control disabled successfully")
        else:
            print(f"Error disabling custom impedance control: {rtn}")

def record_joint_positions():
    """
    Background thread function that records joint positions every 2ms
    """
    global recording_active, recording_data
    
    start_time = time.time()
    
    while recording_active:
        try:
            # Get current joint positions
            error, joint_pos = robot.GetActualJointPosDegree(flag=1)
            
            if error == 0:
                # Calculate timestamp relative to start
                timestamp = time.time() - start_time
                
                # Store data: [timestamp, j1, j2, j3, j4, j5, j6]
                data_point = [timestamp] + joint_pos
                recording_data.append(data_point)
            else:
                print(f"Warning: Error reading joint positions: {error}")
            
            # Sleep for 2ms (0.002 seconds)
            time.sleep(0.002)
            
        except Exception as e:
            print(f"Error in recording thread: {e}")
            break

def start_recording():
    """
    Start recording joint positions in background thread
    """
    global recording_active, recording_data, recording_thread
    
    if recording_active:
        print("Recording is already active!")
        return
    
    # Reset recording data
    recording_data = []
    recording_active = True
    
    # Start recording thread
    recording_thread = threading.Thread(target=record_joint_positions)
    recording_thread.daemon = True
    recording_thread.start()
    
    print("\n[RECORDING STARTED]")
    print("Recording joint positions every 2ms...")
    print("Press 'g' to stop recording\n")

def stop_recording():
    """
    Stop recording and save data to CSV file
    """
    global recording_active, recording_data, recording_thread
    
    if not recording_active:
        print("No active recording to stop!")
        return
    
    # Stop recording
    recording_active = False
    
    # Wait for thread to finish
    if recording_thread:
        recording_thread.join(timeout=1.0)
    
    print(f"\n[RECORDING STOPPED]")
    print(f"Total data points recorded: {len(recording_data)}")
    
    if len(recording_data) == 0:
        print("No data recorded!")
        return
    
    # Calculate recording duration
    duration = recording_data[-1][0] if recording_data else 0
    print(f"Recording duration: {duration:.3f} seconds")
    
    # Ask user for filename
    filename = input("\nEnter filename to save (without extension): ").strip()
    
    if not filename:
        filename = f"joint_recording_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        print(f"Using default filename: {filename}")
    
    # Add .csv extension if not present
    if not filename.endswith('.csv'):
        filename += '.csv'
    
    # Save to CSV
    try:
        with open(filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            
            # Write header
            writer.writerow(['timestamp', 'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'])
            
            # Write data
            writer.writerows(recording_data)
        
        print(f"\n✓ Data saved successfully to: {filename}")
        print(f"  Total rows: {len(recording_data)}")
        print(f"  Sampling rate: ~{len(recording_data)/duration:.1f} Hz" if duration > 0 else "")
        
    except Exception as e:
        print(f"Error saving file: {e}")

def main():
    """Main function - uses impedance control with gravity compensation and recording"""
    
    try:
        # Display current configuration
        print("=" * 60)
        print("Fairino10 Robot Drag Mode with Recording")
        print("=" * 60)
        print(f"\nRobot Configuration:")
        print(f"- Impedance control: Enabled")
        print(f"- Gravity compensation: Enabled")
        print(f"- Recording sampling rate: 2ms (500 Hz)")
        print()
        
        # Enable custom impedance mode with gravity compensation
        custom_drag_teach_mode(enable=True)
        
        # Check if in drag mode
        error, state = robot.IsInDragTeach()
        print(f"Drag state: {state}")
        print()
        
        # Instructions
        print("=" * 60)
        print("CONTROLS:")
        print("  's' - Start recording joint positions")
        print("  'g' - Stop recording and save to file")
        print("  'q' - Quit program")
        print("=" * 60)
        print("\nRobot is ready. Move it manually and control recording.\n")
        
        # Main control loop
        while True:
            user_input = input("Enter command (s/g/q): ").strip().lower()
            
            if user_input == 's':
                start_recording()
                
            elif user_input == 'g':
                stop_recording()
                
            elif user_input == 'q':
                print("\nExiting program...")
                break
                
            else:
                print("Invalid command! Use 's' to start, 'g' to stop, 'q' to quit")
        
    except KeyboardInterrupt:
        print("\n\nKeyboardInterrupt detected!")
        
        # If recording is active, stop it
        if recording_active:
            print("Stopping active recording...")
            stop_recording()
        
    except Exception as e:
        print(f"Error: {e}")
        
    finally:
        # Stop any active recording
        if recording_active:
            recording_active = False
            if recording_thread:
                recording_thread.join(timeout=1.0)
        
        # Always ensure we disable drag mode and gravity compensation on exit
        print("\nDisabling drag mode...")
        custom_drag_teach_mode(enable=False)
        
        # Verify it's disabled
        error, state = robot.IsInDragTeach()
        print(f"Drag state after disable: {state}")
        
        # Close connection
        robot.CloseRPC()
        print("Connection closed.")

# Run the main function
if __name__ == "__main__":
    main()