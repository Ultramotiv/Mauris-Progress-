#drag mode with custom impedence control + gravity compensation

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

# GRAVITY COMPENSATION PARAMETERS FOR JOINTS 1 AND 2
# These values are calculated based on the mass distribution from your URDF
GRAVITY_COMPENSATION = {
    'joint1': 450, #600, #450.0,  # Joint 1 supports entire arm weight (~55kg total) during horizontal rotation
    'joint2': 500, #400, #500.0,  # Joint 2 supports heavy upperarm + forearm (~42kg) against gravity
    'joint3': 200, #150, #200.0,   # Joint 3 supports forearm + wrist assembly (~17kg)
    'joint4': 100,    # Wrist joints have minimal gravity effect
    'joint5': 100,
    'joint6': 100
}

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

def fine_tune_gravity_compensation():
    """
    Helper function to fine-tune gravity compensation values
    Call this if joints still don't feel smooth enough
    """
    print("\n=== Gravity Compensation Fine-Tuning ===")
    print("Current values:")
    print(f"Joint 1: {GRAVITY_COMPENSATION['joint1']} (supports entire arm ~55kg)")
    print(f"Joint 2: {GRAVITY_COMPENSATION['joint2']} (supports upperarm+forearm ~42kg)")
    print(f"Joint 3: {GRAVITY_COMPENSATION['joint3']} (supports forearm+wrist ~17kg)")
    print("\nTo fine-tune:")
    print("1. If Joint 1 feels heavy during horizontal rotation: increase joint1 (150-220)")
    print("2. If Joint 2 feels heavy when lifting arm up/down: increase joint2 (300-400)")
    print("3. If Joint 3 feels heavy during elbow movement: increase joint3 (60-120)")
    print("4. If any joint feels too bouncy/light: decrease the respective value")
    print("5. Test incrementally: adjust by ±20-30 and retest")
    print("\nTotal arm mass breakdown:")
    print("- Shoulder link: 9.81 kg")
    print("- Upper arm: 29.76 kg (heaviest single link)")
    print("- Forearm: 12.68 kg") 
    print("- All wrist links: ~4.67 kg")
    print("- Total moving mass for J1: ~55+ kg")

def main():
    """Main function - uses impedance control with gravity compensation"""
    
    try:
        # Display current configuration
        print("=== Fairino10 Robot Drag Mode with Gravity Compensation ===")
        print(f"Robot mass distribution from URDF:")
        print(f"- Base link: 0.74 kg")
        print(f"- Shoulder link: 9.81 kg") 
        print(f"- Upper arm link: 29.76 kg (heaviest)")
        print(f"- Forearm link: 12.68 kg")
        print(f"- Wrist links: ~2.23 kg each")
        print()
        
        # Enable custom impedance mode with gravity compensation
        custom_drag_teach_mode(enable=True)
        
        # Check if in drag mode
        error, state = robot.IsInDragTeach()
        print(f"Drag state: {state}")
        
        # Keep robot in drag mode until user presses Ctrl+C
        print("Robot is now in custom impedance mode with gravity compensation.")
        print("Try moving joints 1 and 2 manually - they should feel much smoother now!")
        print("Press Ctrl+C to exit...")
        
        while True:
            time.sleep(1)
        
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt detected! Disabling custom impedance mode...")
        
    except Exception as e:
        print(f"Error: {e}")
        
    finally:
        # Always ensure we disable drag mode and gravity compensation on exit
        custom_drag_teach_mode(enable=False)
        
        # Verify it's disabled
        error, state = robot.IsInDragTeach()
        print(f"Drag state after disable: {state}")
        
        # Close connection
        robot.CloseRPC()
        print("Connection closed.")
        
        # Show fine-tuning tips
        print("\nIf joints still don't feel smooth enough, you can modify")
        print("the GRAVITY_COMPENSATION values at the top of this script.")

# Run the main function
if __name__ == "__main__":
    main()