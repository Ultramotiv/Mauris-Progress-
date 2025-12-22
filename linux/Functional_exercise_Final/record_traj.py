# recorded trajetory for the code 

import sys
sys.path.append('/home/um/fairino-python-sdk-main/linux/fairino')
import Robot
import time
import threading
import json
from datetime import datetime

# Establish connection with robot controller
robot = Robot.RPC('192.168.58.2')

# Recording control variables
recording_active = False
recorded_trajectory = []
recording_thread = None

def custom_drag_teach_mode(enable=True):
    """Enable/disable drag teach mode with custom impedance"""
    
    IMPEDANCE_PARAMS = {
        'lamde_dain': [2.5, 2.0, 2.0, 2.0, 2.0, 2.0],
        'b_gain': [20.0, 10.0, 10.0, 5.0, 5.0, 1.0],
        'k_gain': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        'max_tcp_vel': 500,
        'max_tcp_ori_vel': 90
    }
    
    GRAVITY_COMP = [600, 400, 150, 100, 100, 100]
    
    if enable:
        print("Enabling drag mode...")
        robot.DragTeachSwitch(1)
        time.sleep(0.5)
        
        try:
            robot.SetGravityComp(1, GRAVITY_COMP)
        except:
            pass
        
        robot.ForceAndJointImpedanceStartStop(
            status=1,
            impedanceFlag=1,
            lamdeDain=IMPEDANCE_PARAMS['lamde_dain'],
            KGain=IMPEDANCE_PARAMS['k_gain'], 
            BGain=IMPEDANCE_PARAMS['b_gain'],
            dragMaxTcpVel=IMPEDANCE_PARAMS['max_tcp_vel'],
            dragMaxTcpOriVel=IMPEDANCE_PARAMS['max_tcp_ori_vel']
        )
        print("✓ Drag mode enabled - You can now move the robot manually")
        
    else:
        print("Disabling drag mode...")
        try:
            robot.SetGravityComp(0, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        except:
            pass
        
        robot.ForceAndJointImpedanceStartStop(
            status=0,
            impedanceFlag=0,
            lamdeDain=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            KGain=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            BGain=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            dragMaxTcpVel=1000,
            dragMaxTcpOriVel=180
        )
        robot.DragTeachSwitch(0)
        time.sleep(0.5)
        print("✓ Drag mode disabled")

def recording_loop():
    """Background thread that records TCP positions"""
    global recording_active, recorded_trajectory
    
    record_rate = 0.002  # 2ms sampling (500 Hz)
    
    while recording_active:
        err, tcp = robot.GetActualTCPPose()
        if err == 0:
            timestamp = time.time()
            recorded_trajectory.append({
                'timestamp': timestamp,
                'tcp_position': tcp.copy()
            })
        time.sleep(record_rate)

def start_recording():
    """Start recording TCP positions"""
    global recording_active, recorded_trajectory, recording_thread
    
    if recording_active:
        print("⚠ Recording already in progress!")
        return
    
    recorded_trajectory = []
    recording_active = True
    
    recording_thread = threading.Thread(target=recording_loop, daemon=True)
    recording_thread.start()
    
    print("\n" + "="*60)
    print("🔴 RECORDING STARTED")
    print("="*60)
    print("Move the robot to the desired positions")
    print("Type 'g' and press ENTER when done")
    print("="*60 + "\n")

def stop_recording():
    """Stop recording and save to file"""
    global recording_active, recording_thread
    
    if not recording_active:
        print("⚠ No active recording to stop!")
        return
    
    recording_active = False
    
    if recording_thread:
        recording_thread.join(timeout=1.0)
    
    print("\n" + "="*60)
    print("⏹ RECORDING STOPPED")
    print("="*60)
    
    if len(recorded_trajectory) == 0:
        print("⚠ No data recorded!")
        return
    
    # Calculate duration
    start_time = recorded_trajectory[0]['timestamp']
    end_time = recorded_trajectory[-1]['timestamp']
    duration = end_time - start_time
    
    print(f"✓ Recorded {len(recorded_trajectory)} TCP positions")
    print(f"✓ Duration: {duration:.2f} seconds")
    print(f"✓ Average rate: {len(recorded_trajectory)/duration:.1f} Hz")
    
    # Save to JSON file
    filename = f"tcp_trajectory_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
    
    # Normalize timestamps to start from 0
    for point in recorded_trajectory:
        point['timestamp'] = point['timestamp'] - start_time
    
    data = {
        'metadata': {
            'total_points': len(recorded_trajectory),
            'duration_seconds': duration,
            'sampling_rate_hz': len(recorded_trajectory)/duration,
            'recorded_date': datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        },
        'trajectory': recorded_trajectory
    }
    
    with open(filename, 'w') as f:
        json.dump(data, f, indent=2)
    
    print(f"✓ Saved to: {filename}")
    print("="*60 + "\n")

def main():
    """Main function"""
    
    try:
        print("\n" + "="*60)
        print("TCP POSITION RECORDER")
        print("="*60)
        print("Sampling Rate: 500 Hz (2ms)")
        print("="*60 + "\n")
        
        # Enable drag mode
        custom_drag_teach_mode(enable=True)
        
        print("COMMANDS:")
        print("  's' - Start recording TCP positions")
        print("  'g' - Stop recording and save to file")
        print("  'q' - Quit program")
        print("\nReady to record!\n")
        
        # Main control loop
        while True:
            user_input = input("Enter command: ").strip().lower()
            
            if user_input == 's':
                start_recording()
                
            elif user_input == 'g':
                stop_recording()
                
            elif user_input == 'q':
                if recording_active:
                    print("Stopping recording before exit...")
                    stop_recording()
                print("\nExiting program...")
                break
                
            else:
                print("Invalid command! Use 's' to start, 'g' to stop, 'q' to quit\n")
        
    except KeyboardInterrupt:
        print("\n\nInterrupted by user!")
        if recording_active:
            stop_recording()
        
    except Exception as e:
        print(f"Error: {e}")
        
    finally:
        # Always disable drag mode on exit
        if recording_active:
            recording_active = False
            if recording_thread:
                recording_thread.join(timeout=1.0)
        
        custom_drag_teach_mode(enable=False)
        robot.CloseRPC()
        print("Connection closed.\n")

if __name__ == "__main__":
    main()