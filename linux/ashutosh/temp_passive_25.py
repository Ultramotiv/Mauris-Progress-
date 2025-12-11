import sys
sys.path.append('/home/um/fairino-python-sdk-main/linux/fairino')
import Robot
import time

# Establish connection with robot controller
robot = Robot.RPC('192.168.58.2')

# SINGLE SET OF IMPEDANCE PARAMETERS - MODIFY THESE VALUES ONLY
IMPEDANCE_PARAMS = {
    'lamde_dain' : [2.5, 2.0, 2.0, 2.0, 2.0, 2.0],
    'b_gain' : [10.0, 10.0, 10.0, 5.0, 5.0, 1.0],
    'k_gain' : [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    'max_tcp_vel': 500,      # Maximum linear velocity limit (mm/s)
    'max_tcp_ori_vel': 90    # Maximum angular velocity limit (deg/s)
}

# Force sensor configuration - MODIFY THESE BASED ON YOUR SENSOR
FORCE_SENSOR_CONFIG = {
    'company': 24,      # XJC force sensor (modify based on your actual sensor)
    'device': 0,        # Device number
    'softversion': 0,   # Software version
    'bus': 0           # Bus location
}

def setup_force_sensor():
    """Configure and activate the force sensor"""
    print("Setting up force sensor...")
    
    try:
        # Configure force sensor
        rtn = robot.FT_SetConfig(
            FORCE_SENSOR_CONFIG['company'],
            FORCE_SENSOR_CONFIG['device'], 
            FORCE_SENSOR_CONFIG['softversion'],
            FORCE_SENSOR_CONFIG['bus']
        )
        
        if rtn != 0:
            print(f"Warning: Force sensor config failed: {rtn}")
            return False
            
        time.sleep(1)
        
        # Get configuration to verify
        error, config = robot.FT_GetConfig()
        if error == 0:
            print(f"Force sensor config: Company={config[1]}, Device={config[2]}")
        
        # Reset and activate force sensor
        robot.FT_Activate(0)  # Reset first
        time.sleep(1)
        robot.FT_Activate(1)  # Then activate
        time.sleep(1)
        
        # Set zero (remove existing zero first, then apply new zero)
        robot.FT_SetZero(0)   # Remove zero
        time.sleep(1)
        robot.FT_SetZero(1)   # Apply zero
        time.sleep(1)
        
        # Set reference coordinate system to tool coordinate system
        rtn = robot.FT_SetRCS(0)  # 0 = tool coordinate system
        if rtn == 0:
            print("Force sensor setup completed successfully")
            return True
        else:
            print(f"Warning: Setting reference coordinate system failed: {rtn}")
            return False
            
    except Exception as e:
        print(f"Force sensor setup error: {e}")
        return False

def get_speed_data():
    """Get current force/torque data from the sensor"""
    try:
        # Get force data in reference coordinate system
        error,jointSpeed = robot.GetActualJointSpeedsDegree(0)
        if error == 0:
            return error, jointSpeed
        
        # If RCS fails, try getting raw data
        error,jointSpeed = robot.GetActualJointSpeedsDegree(0)
        return error, jointSpeed
        
    except Exception as e:
        print(f"Error getting force data: {e}")
        return -1, None

def get_joint_torques():
    """Get current joint torques - using SDK documented function"""
    try:
        # Using the documented function from the SDK
        error, joint_torques = robot.GetJointTorques(0)
        return error, joint_torques
    except Exception as e:
        print(f"Error getting joint torques: {e}")
        return -1, None

def print_force_data(jointSpeed, joint_torques=None):
    """Print force and torque data in a readable format"""
    if jointSpeed is not None:
        # Calculate force and moment magnitudes
        #force_magnitude = (force_data[0]**2 + force_data[1]**2 + force_data[2]**2)**0.5
        #moment_magnitude = (force_data[3]**2 + force_data[4]**2 + force_data[5]**2)**0.5
        
        print(f"joint speeds deg: J1: {jointSpeed[0]},  J2:  {jointSpeed[1]},  J3:  {jointSpeed[2]},  J4:  {jointSpeed[3]},  J5:  {jointSpeed[4]}  ,  J6:  {jointSpeed[5]}")
        # print(f"TCP Forces:  Fx={force_data[0]:6.2f}N  Fy={force_data[1]:6.2f}N  Fz={force_data[2]:6.2f}N  |Total: {force_magnitude:6.2f}N|")
        # print(f"TCP Moments: Mx={force_data[3]:6.2f}Nm My={force_data[4]:6.2f}Nm Mz={force_data[5]:6.2f}Nm |Total: {moment_magnitude:6.2f}Nm|")
        
    if joint_torques is not None:
        print(f"Joint Torques: J1={joint_torques[0]:5.1f} J2={joint_torques[1]:5.1f} J3={joint_torques[2]:5.1f} J4={joint_torques[3]:5.1f} J5={joint_torques[4]:5.1f} J6={joint_torques[5]:5.1f} Nm")
    error,torque = robot.GetJointDriverTorque()
    print(f"torque:{torque[0]},{torque[1]},{torque[2]},{torque[3]},{torque[4]},{torque[5]}")
    print("-" * 90)

def custom_drag_teach_mode(enable=True):
    """
    Custom impedance control using the single IMPEDANCE_PARAMS set above
    
    Args:
        enable (bool): True to enable, False to disable
    """
    
    if enable:
        # print("Enabling custom impedance control...")
        # print(f"Using parameters: {IMPEDANCE_PARAMS}")
        
        # First enable drag teach mode (this is required)
        robot.DragTeachSwitch(1)
        time.sleep(0.5)
        
        # Then apply your custom impedance parameters
        rtn = robot.ForceAndJointImpedanceStartStop(
            status=1,  # Turn on
            impedanceFlag=1,  # Enable impedance control
            lamdeDain=IMPEDANCE_PARAMS['lamde_dain'],
            KGain=IMPEDANCE_PARAMS['k_gain'], 
            BGain=IMPEDANCE_PARAMS['b_gain'],
            dragMaxTcpVel=IMPEDANCE_PARAMS['max_tcp_vel'],
            dragMaxTcpOriVel=IMPEDANCE_PARAMS['max_tcp_ori_vel']
        )
        
        if rtn == 0:
            #print("Custom impedance control enabled successfully")
            print("You can control the robot now ")
        else:
            print(f"Error enabling custom impedance control: {rtn}")
            
    else:
        print("Disabling custom impedance control...")
        
        # Turn off custom impedance parameters
        rtn = robot.ForceAndJointImpedanceStartStop(
            status=0,  # Turn off
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

def monitor_forces_during_drag():
    """Monitor and display force data during drag teaching"""
    print("Monitoring forces during drag teaching...")
    print("Move the robot manually to see applied forces and joint torques")
    print("Press Ctrl+C to exit...")
    print("=" * 90)
    
    force_threshold = 1.0  # Minimum force magnitude to display (N)
    torque_threshold = 1.0  # Minimum joint torque change to display (Nm)
    previous_joint_torques = None
    
    while True:
        try:
            # Get force data
            force_error, force_data = get_speed_data()
            
            # Get joint torque data  
            torque_error, joint_torques = get_joint_torques()
            
            # Check if there's significant force being applied
            display_data = False
            
            if force_error == 0 and force_data is not None:
                force_magnitude = (force_data[0]**2 + force_data[1]**2 + force_data[2]**2)**0.5
                if force_magnitude > force_threshold:
                    display_data = True
            
            # Check if there's significant change in joint torques
            if torque_error == 0 and joint_torques is not None:
                if previous_joint_torques is not None:
                    for i in range(len(joint_torques)):
                        if abs(joint_torques[i] - previous_joint_torques[i]) > torque_threshold:
                            display_data = True
                            break
                else:
                    display_data = True  # First reading
                    
                previous_joint_torques = joint_torques[:]
            
            # Display data if there's activity
            if display_data:
                print(f"Timestamp: {time.strftime('%H:%M:%S')}")
                if force_error == 0 and force_data is not None:
                    print_force_data(force_data, joint_torques if torque_error == 0 else None)
                else:
                    print("Force sensor data not available - showing joint torques only")
                    if torque_error == 0 and joint_torques is not None:
                        print_force_data(None, joint_torques)
                    print("-" * 90)
            
            time.sleep(0.1)  # Check every 100ms
            
        except KeyboardInterrupt:
            break
        except Exception as e:
            print(f"Error in force monitoring: {e}")
            time.sleep(1)

def main():
    """Main function with force monitoring"""
    
    force_sensor_available = False
    
    try:
        # Try to setup force sensor (optional - will work without it)
        print("Attempting to setup force sensor...")
        force_sensor_available = setup_force_sensor()
        
        if not force_sensor_available:
            print("Force sensor setup failed or not available.")
            print("Continuing with joint torque monitoring only...")
        
        # Enable custom impedance mode
        custom_drag_teach_mode(enable=True)
        
        # Check if in drag mode
        error, state = robot.IsInDragTeach()
        print(f"Drag state: {state}")
        
        if state != 1:
            print("Warning: Drag mode not properly activated!")
        
        # Start force monitoring during drag teaching
        monitor_forces_during_drag()
        
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt detected! Disabling custom impedance mode...")
        
    except Exception as e:
        print(f"Error: {e}")
        
    finally:
        # Always ensure we disable drag mode on exit
        custom_drag_teach_mode(enable=False)
        
        # Verify it's disabled
        error, state = robot.IsInDragTeach()
        print(f"Drag state after disable: {state}")
        
        # Deactivate force sensor if it was setup
        if force_sensor_available:
            try:
                robot.FT_Activate(0)
                print("Force sensor deactivated")
            except:
                pass
        
        # Close connection
        robot.CloseRPC()
        print("Connection closed.")

# Run the main function
if __name__ == "__main__":
    main()