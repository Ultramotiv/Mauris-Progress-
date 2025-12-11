import sys
sys.path.append('/home/um/fairino-python-sdk-main/linux/fairino')
import Robot
import time

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

def custom_drag_teach_mode(enable=True):
    """
    Custom impedance control using the single IMPEDANCE_PARAMS set above
    
    Args:
        enable (bool): True to enable, False to disable
    """
    
    if enable:
        print("Enabling custom impedance control...")
        print(f"Using parameters: {IMPEDANCE_PARAMS}")
        
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
            print("Custom impedance control enabled successfully")
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

def main():
    """Main function - uses only the single IMPEDANCE_PARAMS set"""
    
    try:
        # Enable custom impedance mode with the single parameter set
        custom_drag_teach_mode(enable=True)
        
        # Check if in drag mode
        error, state = robot.IsInDragTeach()
        print(f"Drag state: {state}")
        
        # Keep robot in drag mode until user presses Ctrl+C
        print("Robot is now in custom impedance mode. Try moving it manually...")
        error,jtorque = robot.GetJointTorques(0)
        print(f"torques:{jtorque[0]},{jtorque[1]},{jtorque[2]},{jtorque[3]},{jtorque[4]},{jtorque[5]}")
        print("Press Ctrl+C to exit...")
        
        while True:
            time.sleep(1)
        
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
        
        # Close connection
        robot.CloseRPC()
        print("Connection closed.")

# Run the main function
if __name__ == "__main__":
    main()

