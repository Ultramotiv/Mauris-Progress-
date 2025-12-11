# this code was developed on 6th NOV 2025 
#and it goes to the Y_Movement after user says ok 
import sys
sys.path.append('/home/um/fairino-python-sdk-main/linux/fairino')
import Robot
import time

robot = Robot.RPC('192.168.58.2')

# Display current joint position for reference
current_pose = robot.GetActualJointPosDegree(flag=1)
print("\nCurrent Joint Position [J1, J2, J3, J4, J5, J6]:")
print(current_pose)

# ============================================================================
# LINEAR MOTION: Move specified distance in Y-axis from CURRENT TCP position
# ============================================================================

# SET THE DESIRED Y-AXIS MOVEMENT HERE (in mm)
Y_MOVEMENT = 100.0  # Change this to 50.0 or 150.0 as needed

print("\n" + "="*60)
print(f"Starting Linear Motion in Y-axis (+{Y_MOVEMENT}mm)")
print("="*60)

# Get current TCP (Tool Center Point) Cartesian position
# Returns tuple: (error_code, [X, Y, Z, Rx, Ry, Rz]) in mm and degrees
tcp_result = robot.GetActualTCPPose(flag=1)
print(f"\nRaw TCP Result: {tcp_result}")

# Extract error code and TCP position
error_code = tcp_result[0]
current_tcp = tcp_result[1] if len(tcp_result) > 1 else None

if error_code == 0 and current_tcp and len(current_tcp) >= 6:
    print(f"\nCurrent TCP Position [X, Y, Z, Rx, Ry, Rz]: {current_tcp}")
    
    # Extract current position
    x_current = current_tcp[0]
    y_current = current_tcp[1]
    z_current = current_tcp[2]
    rx_current = current_tcp[3]
    ry_current = current_tcp[4]
    rz_current = current_tcp[5]
    
    print(f"\nCurrent Position:")
    print(f"  X  = {x_current:.2f} mm")
    print(f"  Y  = {y_current:.2f} mm")
    print(f"  Z  = {z_current:.2f} mm")
    print(f"  Rx = {rx_current:.2f}°")
    print(f"  Ry = {ry_current:.2f}°")
    print(f"  Rz = {rz_current:.2f}°")
    
    # Calculate target position: move Y_MOVEMENT mm in Y-axis
    y_target = y_current + Y_MOVEMENT  # Away
    #y_target = y_current - Y_MOVEMENT  # close
    
    # Target Cartesian position (keep X, Z, and orientation the same)
    target_desc_pos = [
        x_current,   # X [mm]
        y_target,    # Y [mm] - moved by Y_MOVEMENT
        z_current,   # Z [mm] 
        rx_current,  # Rx [°]
        ry_current,  # Ry [°]
        rz_current   # Rz [°]
    ]
    
    print(f"\n{'='*60}")
    print("CALCULATED TARGET POSITION")
    print(f"{'='*60}")
    print(f"  X  = {x_current:.2f} mm (no change)")
    print(f"  Y  = {y_target:.2f} mm (+{Y_MOVEMENT}mm)")
    print(f"  Z  = {z_current:.2f} mm (no change)")
    print(f"  Rx = {rx_current:.2f}° (no change)")
    print(f"  Ry = {ry_current:.2f}° (no change)")
    print(f"  Rz = {rz_current:.2f}° (no change)")
    print(f"{'='*60}")
    
    # Ask for user confirmation
    print(f"\nRobot will move Y-axis by +{Y_MOVEMENT}mm")
    user_input = input("Type 'ok' to proceed with the movement: ").strip().lower()
    
    if user_input == 'ok':
        print(f"\n✓ Confirmation received. Executing linear motion...")
        
        # Execute linear motion with MoveL
        # Using a moderate speed for linear motion (20% default)
        linear_vel = 5.0  # % of max Cartesian speed
        
        ret_linear = robot.MoveL(
            desc_pos    = target_desc_pos,  # Target Cartesian position [X,Y,Z,Rx,Ry,Rz]
            tool        = 0,                # Tool number
            user        = 0,                # User/workpiece frame
            joint_pos   = [0.0]*7,          # Let IK solve for joint positions
            vel         = linear_vel,       # Speed percentage
            acc         = 0.0,              # Acceleration (not open)
            ovl         = 100.0,            # Velocity scaling factor
            blendR      = -1.0,             # -1.0 = blocking motion (move in place)
            exaxis_pos  = [0.0]*4,          # No external axes
            search      = 0,                # No wire search
            offset_flag = 0,                # No offset
            offset_pos  = [0.0]*6           # No offset position
        )
        
        # Check result
        if ret_linear == 0:
            print(f"\n✓ MoveL command succeeded – TCP moved +{Y_MOVEMENT}mm in Y-axis")
            
            # Verify final position
            time.sleep(0.5)
            final_tcp_result = robot.GetActualTCPPose(flag=1)
            final_error = final_tcp_result[0]
            final_tcp = final_tcp_result[1] if len(final_tcp_result) > 1 else None
            
            if final_error == 0 and final_tcp:
                print(f"\nFinal TCP Position [X, Y, Z, Rx, Ry, Rz]: {final_tcp}")
                actual_y_movement = final_tcp[1] - y_current
                print(f"\nY-axis movement verification:")
                print(f"  Initial Y: {y_current:.2f} mm")
                print(f"  Final Y:   {final_tcp[1]:.2f} mm")
                print(f"  Movement:  {actual_y_movement:.2f} mm (target: +{Y_MOVEMENT}mm)")
                
                # Check if movement is within acceptable tolerance (±1mm)
                movement_error = abs(actual_y_movement - Y_MOVEMENT)
                if movement_error < 1.0:
                    print(f"  ✓ Movement accurate (error: {movement_error:.2f}mm)")
                else:
                    print(f"  ⚠ Movement error: {movement_error:.2f}mm")
        else:
            print(f"\n✗ MoveL failed with error code: {ret_linear}")
    else:
        print(f"\n✗ Movement cancelled by user (you entered: '{user_input}')")
        print("  Movement was NOT executed. Robot remains at current position.")
else:
    print(f"\n✗ Failed to get current TCP position (error code: {error_code})")

print("\n" + "="*60)
print("Motion sequence completed")
print("="*60)