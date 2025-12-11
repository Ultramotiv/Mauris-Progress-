# this code created on 31st oct 2025
# Move Z_MOVEMENT mm in Z-axis from CURRENT robot position using MoveL

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
# OPTION 1: Comment out this entire section if you want to move from current position
# ============================================================================
"""
target_joints = [
    2.608,    # J1
    -88.384,  # J2
    127.473,  # J3
    -137.27,  # J4
    -92.275,  # J5
    -90.118   # J6
]

MAX_JOINT_SPEED = 180.0          # deg/s
desired_speed = 7.70             # deg/s
vel_percent   = (desired_speed / MAX_JOINT_SPEED) * 100.0
ovl_percent   = 100.0
vel_percent = round(vel_percent, 3)

print(f"\nMoving to home position with {desired_speed} deg/s")
print(f" -> vel = {vel_percent}%   ovl = {ovl_percent}%")

ret = robot.MoveJ(
    joint_pos = target_joints,
    tool      = 0,
    user      = 0,
    desc_pos  = [0.0]*7,
    vel       = vel_percent,
    acc       = 0.0,
    ovl       = ovl_percent,
    exaxis_pos= [0.0]*4,
    blendT    = -1.0,
    offset_flag=0,
    offset_pos=[0.0]*6
)

current_pose = robot.GetActualJointPosDegree(flag=1)
print("\nJoint Position after MoveJ [J1, J2, J3, J4, J5, J6]:")
print(current_pose)

if ret == 0:
    print("MoveJ command succeeded – robot reached the target joint position.")
else:
    print(f"MoveJ failed with error code: {ret}")

time.sleep(0.5)
"""
# ============================================================================
# END OF OPTIONAL SECTION
# ============================================================================

# ============================================================================
# LINEAR MOTION: Move specified distance in Z-axis from CURRENT TCP position
# ============================================================================

# SET THE DESIRED Z-AXIS MOVEMENT HERE (in mm)
Z_MOVEMENT = 50.0  # Change this to 50.0 or 150.0 as needed

print("\n" + "="*60)
print(f"Starting Linear Motion in Z-axis (+{Z_MOVEMENT}mm)")
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
    
    # Calculate target position: move Z_MOVEMENT mm in Z-axis
    z_target = z_current - Z_MOVEMENT
    #y_target  = y_current + Z_MOVEMENT # Away
    #y_target  = y_current - Z_MOVEMENT # close
    # Target Cartesian position (keep X, Y, and orientation the same)
    target_desc_pos = [
        x_current,   # X [mm]
        y_current,   # Y [mm]
        z_target,    # Z [mm] - moved by Z_MOVEMENT
        rx_current,  # Rx [°]
        ry_current,  # Ry [°]
        rz_current   # Rz [°]
    ]
    
    print(f"\nTarget Position:")
    print(f"  X  = {x_current:.2f} mm (no change)")
    print(f"  Y  = {y_current:.2f} mm (+{Z_MOVEMENT}mm)")
    print(f"  Z  = {z_target:.2f} mm (no change)")
    print(f"  Rx = {rx_current:.2f}° (no change)")
    print(f"  Ry = {ry_current:.2f}° (no change)")
    print(f"  Rz = {rz_current:.2f}° (no change)")
    
    print(f"\nExecuting linear motion...")
    
    # Execute linear motion with MoveL
    # Using a moderate speed for linear motion (20% default)
    linear_vel = 20.0  # % of max Cartesian speed
    
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
        print(f"\n✓ MoveL command succeeded – TCP moved +{Z_MOVEMENT}mm in Z-axis")
        
        # Verify final position
        time.sleep(0.5)
        final_tcp_result = robot.GetActualTCPPose(flag=1)
        final_error = final_tcp_result[0]
        final_tcp = final_tcp_result[1] if len(final_tcp_result) > 1 else None
        
        if final_error == 0 and final_tcp:
            print(f"\nFinal TCP Position [X, Y, Z, Rx, Ry, Rz]: {final_tcp}")
            actual_z_movement = final_tcp[2] - z_current
            print(f"\nZ-axis movement verification:")
            print(f"  Initial Z: {z_current:.2f} mm")
            print(f"  Final Z:   {final_tcp[2]:.2f} mm")
            print(f"  Movement:  {actual_z_movement:.2f} mm (target: +{Z_MOVEMENT}mm)")
            
            # Check if movement is within acceptable tolerance (±1mm)
            movement_error = abs(actual_z_movement - Z_MOVEMENT)
            if movement_error < 1.0:
                print(f"  ✓ Movement accurate (error: {movement_error:.2f}mm)")
            else:
                print(f"  ⚠ Movement error: {movement_error:.2f}mm")
    else:
        print(f"\n✗ MoveL failed with error code: {ret_linear}")
else:
    print(f"\n✗ Failed to get current TCP position (error code: {error_code})")

print("\n" + "="*60)
print("Motion sequence completed")
print("="*60)