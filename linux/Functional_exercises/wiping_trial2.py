# Arc Drawing with Specified End Point and Radius
# Developed: Nov 2025
import sys
sys.path.append('/home/um/fairino-python-sdk-main/linux/fairino')
import Robot
import time
import math

robot = Robot.RPC('192.168.58.2')

# Display SDK version
error, version = robot.GetSDKVersion()
print(f"SDK version: {version}")

# Display current joint position
current_pose = robot.GetActualJointPosDegree(flag=1)
print("\nCurrent Joint Position [J1, J2, J3, J4, J5, J6]:")
print(current_pose)

# ============================================================================
# ARC DRAWING: Draw arc from current position to specified end point
# ============================================================================

# CONFIGURABLE PARAMETERS
# Specify your desired END POINT here (relative to current position)
END_POINT_OFFSET = [500.0, 300.0, 0.0]  # [ΔX, ΔY, ΔZ] in mm from current position
# Example: [200, 300, 0] means end point is 200mm right, 300mm forward, same height

ARC_RADIUS = 300.0              # Radius of the arc in mm
ARC_DIRECTION = 1               # 1 = "left" side arc, -1 = "right" side arc
ARC_PLANE = "XY"                # Plane for arc: "XY", "XZ", or "YZ"
ARC_VELOCITY = 5.0              # Speed percentage (1-100)

print("\n" + "="*70)
print(f"ARC DRAWING PARAMETERS")
print("="*70)
print(f"  End Point Offset: X={END_POINT_OFFSET[0]:+.1f}mm, Y={END_POINT_OFFSET[1]:+.1f}mm, Z={END_POINT_OFFSET[2]:+.1f}mm")
print(f"  Arc Radius:       {ARC_RADIUS} mm")
print(f"  Arc Direction:    {'Left side' if ARC_DIRECTION > 0 else 'Right side'}")
print(f"  Plane:            {ARC_PLANE}")
print(f"  Velocity:         {ARC_VELOCITY}%")
print("="*70)

# Get current TCP position
tcp_result = robot.GetActualTCPPose(flag=1)
print(f"\nRaw TCP Result: {tcp_result}")

error_code = tcp_result[0]
current_tcp = tcp_result[1] if len(tcp_result) > 1 else None

if error_code == 0 and current_tcp and len(current_tcp) >= 6:
    
    x_start = current_tcp[0]
    y_start = current_tcp[1]
    z_start = current_tcp[2]
    rx_current = current_tcp[3]
    ry_current = current_tcp[4]
    rz_current = current_tcp[5]
    
    print(f"\nCurrent TCP Position (START POINT):")
    print(f"  X  = {x_start:.2f} mm")
    print(f"  Y  = {y_start:.2f} mm")
    print(f"  Z  = {z_start:.2f} mm")
    print(f"  Rx = {rx_current:.2f}°")
    print(f"  Ry = {ry_current:.2f}°")
    print(f"  Rz = {rz_current:.2f}°")
    
    # Calculate end point
    x_end = x_start + END_POINT_OFFSET[0]
    y_end = y_start + END_POINT_OFFSET[1]
    z_end = z_start + END_POINT_OFFSET[2]
    
    print(f"\nCalculated END POINT:")
    print(f"  X  = {x_end:.2f} mm")
    print(f"  Y  = {y_end:.2f} mm")
    print(f"  Z  = {z_end:.2f} mm")
    
    # Select the two coordinates based on plane
    if ARC_PLANE == "XY":
        coord1_start, coord2_start = x_start, y_start
        coord1_end, coord2_end = x_end, y_end
        const_coord = z_start
        coord_names = ("X", "Y", "Z")
    elif ARC_PLANE == "XZ":
        coord1_start, coord2_start = x_start, z_start
        coord1_end, coord2_end = x_end, z_end
        const_coord = y_start
        coord_names = ("X", "Z", "Y")
    elif ARC_PLANE == "YZ":
        coord1_start, coord2_start = y_start, z_start
        coord1_end, coord2_end = y_end, z_end
        const_coord = x_start
        coord_names = ("Y", "Z", "X")
    else:
        print(f"Invalid plane: {ARC_PLANE}")
        exit(1)
    
    # Calculate chord (straight-line distance between start and end in the plane)
    chord_length = math.sqrt((coord1_end - coord1_start)**2 + (coord2_end - coord2_start)**2)
    
    print(f"\n" + "="*70)
    print(f"ARC GEOMETRY CALCULATION")
    print("="*70)
    print(f"  Chord Length: {chord_length:.2f} mm")
    
    # Check if arc is possible with given radius
    if chord_length > 2 * ARC_RADIUS:
        print(f"\n❌ ERROR: Cannot create arc!")
        print(f"  Chord length ({chord_length:.2f}mm) is longer than diameter ({2*ARC_RADIUS:.2f}mm)")
        print(f"  Either:")
        print(f"    - Increase ARC_RADIUS to at least {chord_length/2:.2f}mm")
        print(f"    - Reduce the distance to end point")
        exit(1)
    
    if chord_length < 1.0:
        print(f"\n❌ ERROR: Start and end points are too close!")
        print(f"  Distance: {chord_length:.2f}mm")
        exit(1)
    
    # Calculate arc geometry
    # Using the formula for arc through two points with given radius
    
    # Midpoint of chord
    mid1 = (coord1_start + coord1_end) / 2.0
    mid2 = (coord2_start + coord2_end) / 2.0
    
    # Distance from midpoint to center (using Pythagorean theorem)
    half_chord = chord_length / 2.0
    center_to_mid = math.sqrt(ARC_RADIUS**2 - half_chord**2)
    
    # Perpendicular direction to chord (normalized)
    chord_dir1 = (coord1_end - coord1_start) / chord_length
    chord_dir2 = (coord2_end - coord2_start) / chord_length
    
    # Perpendicular vector (rotate 90 degrees)
    perp_dir1 = -chord_dir2
    perp_dir2 = chord_dir1
    
    # Center of the arc (two possible centers, choose based on ARC_DIRECTION)
    center1 = mid1 + (perp_dir1 * center_to_mid * ARC_DIRECTION)
    center2 = mid2 + (perp_dir2 * center_to_mid * ARC_DIRECTION)
    
    print(f"  Arc Center ({coord_names[0]}, {coord_names[1]}): ({center1:.2f}, {center2:.2f})")
    print(f"  Distance from midpoint to center: {center_to_mid:.2f} mm")
    
    # Calculate path point (midpoint of arc, not chord)
    # This is at 50% of the arc angle
    
    # Angle from center to start point
    angle_start = math.atan2(coord2_start - center2, coord1_start - center1)
    angle_end = math.atan2(coord2_end - center2, coord1_end - center1)
    
    # Calculate arc angle (handle wraparound)
    arc_angle = angle_end - angle_start
    
    # Normalize to [-π, π]
    while arc_angle > math.pi:
        arc_angle -= 2 * math.pi
    while arc_angle < -math.pi:
        arc_angle += 2 * math.pi
    
    # If direction doesn't match the natural direction, go the long way
    if (ARC_DIRECTION > 0 and arc_angle < 0) or (ARC_DIRECTION < 0 and arc_angle > 0):
        if arc_angle > 0:
            arc_angle -= 2 * math.pi
        else:
            arc_angle += 2 * math.pi
    
    arc_angle_deg = math.degrees(arc_angle)
    arc_length = ARC_RADIUS * abs(arc_angle)
    
    # Path point at 50% of arc
    path_angle = angle_start + (arc_angle / 2.0)
    
    # Calculate path point coordinates
    path_coord1 = center1 + ARC_RADIUS * math.cos(path_angle)
    path_coord2 = center2 + ARC_RADIUS * math.sin(path_angle)
    
    print(f"  Arc Angle: {arc_angle_deg:.2f}° ({arc_angle:.3f} radians)")
    print(f"  Arc Length: {arc_length:.2f} mm")
    print("="*70)
    
    # Map back to X, Y, Z coordinates
    if ARC_PLANE == "XY":
        path_x, path_y, path_z = path_coord1, path_coord2, const_coord
        target_x, target_y, target_z = x_end, y_end, z_end
    elif ARC_PLANE == "XZ":
        path_x, path_z, path_y = path_coord1, path_coord2, const_coord
        target_x, target_y, target_z = x_end, y_end, z_end
    elif ARC_PLANE == "YZ":
        path_y, path_z, path_x = path_coord1, path_coord2, const_coord
        target_x, target_y, target_z = x_end, y_end, z_end
    
    # Create position arrays
    path_pos = [path_x, path_y, path_z, rx_current, ry_current, rz_current]
    target_pos = [target_x, target_y, target_z, rx_current, ry_current, rz_current]
    
    print(f"\nCALCULATED WAYPOINTS:")
    print(f"  Start Point (current):")
    print(f"    X={x_start:.2f}, Y={y_start:.2f}, Z={z_start:.2f}")
    print(f"  Path Point (arc midpoint):")
    print(f"    X={path_x:.2f}, Y={path_y:.2f}, Z={path_z:.2f}")
    print(f"  Target Point (end):")
    print(f"    X={target_x:.2f}, Y={target_y:.2f}, Z={target_z:.2f}")
    
    # Calculate movements
    dx_path = path_x - x_start
    dy_path = path_y - y_start
    dz_path = path_z - z_start
    
    dx_target = target_x - x_start
    dy_target = target_y - y_start
    dz_target = target_z - z_start
    
    print(f"\n  Movement to Path Point:")
    print(f"    ΔX={dx_path:+.2f}mm, ΔY={dy_path:+.2f}mm, ΔZ={dz_path:+.2f}mm")
    print(f"  Total Movement to Target:")
    print(f"    ΔX={dx_target:+.2f}mm, ΔY={dy_target:+.2f}mm, ΔZ={dz_target:+.2f}mm")
    
    # Ask for user confirmation
    print("\n" + "="*70)
    user_input = input("Type 'ok' to draw the arc: ").strip().lower()
    
    if user_input == 'ok':
        print(f"\n✓ Confirmation received. Drawing arc...")
        print(f"⚠ Moving at {ARC_VELOCITY}% speed...")
        
        # Execute arc motion with MoveC
        ret_arc = robot.MoveC(
            desc_pos_p=path_pos,     # Path point (midpoint of arc)
            tool_p=0,
            user_p=0,
            desc_pos_t=target_pos,   # Target point (end of arc)
            tool_t=0,
            user_t=0,
            vel_p=ARC_VELOCITY,      # Velocity at path point
            vel_t=ARC_VELOCITY,      # Velocity at target point
            ovl=100.0,               # Velocity scaling factor
            blendR=-1.0              # Blocking motion (wait until complete)
        )
        
        print(f"MoveC error code: {ret_arc}")
        
        # Check result
        if ret_arc == 0:
            print(f"\n✓ Arc drawing successful!")
            
            # Verify final position
            time.sleep(0.5)
            final_tcp_result = robot.GetActualTCPPose(flag=1)
            final_error = final_tcp_result[0]
            final_tcp = final_tcp_result[1] if len(final_tcp_result) > 1 else None
            
            if final_error == 0 and final_tcp:
                print(f"\nFinal TCP Position:")
                print(f"  X  = {final_tcp[0]:.2f} mm")
                print(f"  Y  = {final_tcp[1]:.2f} mm")
                print(f"  Z  = {final_tcp[2]:.2f} mm")
                
                actual_dx = final_tcp[0] - x_start
                actual_dy = final_tcp[1] - y_start
                actual_dz = final_tcp[2] - z_start
                
                error_x = actual_dx - dx_target
                error_y = actual_dy - dy_target
                error_z = actual_dz - dz_target
                position_error = math.sqrt(error_x**2 + error_y**2 + error_z**2)
                
                print(f"\nMovement Verification:")
                print(f"  Expected: ΔX={dx_target:+.2f}mm, ΔY={dy_target:+.2f}mm, ΔZ={dz_target:+.2f}mm")
                print(f"  Actual:   ΔX={actual_dx:+.2f}mm, ΔY={actual_dy:+.2f}mm, ΔZ={actual_dz:+.2f}mm")
                print(f"  Position error: {position_error:.2f}mm")
                print(f"  Arc length: {arc_length:.2f}mm (radius: {ARC_RADIUS:.2f}mm)")
                
                if position_error < 2.0:
                    print(f"  ✓ Target reached accurately!")
                else:
                    print(f"  ⚠ Position error: {position_error:.2f}mm")
        else:
            print(f"\n✗ Arc drawing failed with error code: {ret_arc}")
    else:
        print(f"\n✗ Arc drawing cancelled by user")
        print("  Robot remains at current position.")
else:
    print(f"\n✗ Failed to get current TCP position (error code: {error_code})")

print("\n" + "="*70)
print("Arc drawing sequence completed")
print("="*70)