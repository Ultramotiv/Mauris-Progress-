import sys
sys.path.append('/home/um/fairino-python-sdk-main/linux/fairino')
import Robot
import time
import signal
import numpy as np
import threading
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
import uvicorn
from datetime import datetime

# ============================== PARAMETERS ==============================
M = [1.8, 1.8, 1.8, 1.0, 1.0, 1.8]
B = [2.0, 2.0, 2.0, 2.5, 2.5, 2.0]
K = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

JOINT4_MIN, JOINT4_MAX = -175.0, -105.0
JOINT5_MIN, JOINT5_MAX = 60.0, 120.0
MAX_SPEED_DEG_S = 70.0
force_to_deg = 4.0
dt = 0.008
startup_delay = 1.5

# ============================== GLOBALS ==============================
robot = Robot.RPC('192.168.58.2')
baseline_forces = None
home_pos = None
desired_pos = None
velocity = [0.0] * 6
current_actual_pos = None

current_j4 = 0.0
current_j5 = 0.0
is_connected = False
last_error = None
update_counter = 0
state_lock = threading.Lock()

# ============================== FASTAPI SETUP ==============================
app = FastAPI(title="Fairino Robot API", version="1.0.0")

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Allows all origins
    allow_credentials=True,
    allow_methods=["*"],  # Allows all methods
    allow_headers=["*"],  # Allows all headers
)

# Response model
class RobotData(BaseModel):
    j4: float
    j5: float
    timestamp: float
    connected: bool
    update_count: int
    error: str | None = None

@app.get("/robot_data", response_model=RobotData)
async def get_robot_data():
    """
    Get current robot joint positions and status.
    Returns real-time J4 and J5 values with anti-caching headers.
    """
    with state_lock:
        j4_value = current_j4
        j5_value = current_j5
        connected = is_connected
        error = last_error
        count = update_counter
    
    return RobotData(
        j4=round(j4_value, 2),
        j5=round(j5_value, 2),
        timestamp=time.time(),
        connected=connected,
        update_count=count,
        error=error
    )

@app.get("/health")
async def health_check():
    """Health check endpoint"""
    return {
        "status": "ok",
        "timestamp": time.time(),
        "connected": is_connected
    }

@app.get("/")
async def root():
    """Root endpoint with API information"""
    return {
        "message": "Fairino Robot API",
        "endpoints": {
            "/robot_data": "Get current robot joint positions",
            "/health": "Health check",
            "/docs": "Interactive API documentation"
        }
    }

# Custom response headers middleware for aggressive anti-caching
@app.middleware("http")
async def add_no_cache_headers(request, call_next):
    response = await call_next(request)
    if request.url.path == "/robot_data":
        response.headers["Cache-Control"] = "no-store, no-cache, must-revalidate, max-age=0"
        response.headers["Pragma"] = "no-cache"
        response.headers["Expires"] = "0"
        response.headers["Last-Modified"] = datetime.utcnow().strftime('%a, %d %b %Y %H:%M:%S GMT')
        response.headers["ETag"] = f'"{update_counter}-{time.time()}"'
    return response

def run_fastapi_server():
    """Run FastAPI server in a separate thread"""
    print("[FastAPI] Starting server at http://0.0.0.0:8080")
    print("    → Unity should connect to: http://YOUR_PC_IP:8080/robot_data")
    print("    → API docs available at: http://YOUR_PC_IP:8080/docs")
    uvicorn.run(
        app, 
        host="0.0.0.0", 
        port=8080, 
        log_level="warning",  # Reduce console spam
        access_log=False  # Disable access logs
    )

# ============================== UTILITIES ==============================
def update_http_state(j4, j5, connected=True, error=None):
    global current_j4, current_j5, is_connected, last_error, update_counter
    
    with state_lock:
        current_j4 = float(j4)
        current_j5 = float(j5)
        is_connected = connected
        last_error = error
        update_counter += 1

def limit_speed(current, target, max_speed):
    max_step = max_speed * dt
    return np.clip(target, current - max_step, current + max_step)

def shutdown(sig, frame):
    update_http_state(0.0, 0.0, connected=False, error="Shutdown")
    robot.ServoMoveEnd()
    print("\nRobot stopped. Goodbye!")
    sys.exit(0)

signal.signal(signal.SIGINT, shutdown)

def init_ft_sensor():
    company = 24; device = 0
    robot.FT_SetConfig(company, device)
    robot.FT_Activate(1)
    time.sleep(1.0)
    robot.SetLoadWeight(0, 0.0)
    robot.SetLoadCoord(0.0, 0.0, 0.0)
    robot.FT_SetZero(1)
    time.sleep(0.5)
    print("FT Sensor initialized and zeroed.")

def calibrate_baseline_forces():
    global baseline_forces
    print("Calibrating gravity compensation...")
    samples = []
    for _ in range(100):
        ret = robot.FT_GetForceTorqueRCS()
        if ret[0] == 0:
            samples.append([ret[1][0], -ret[1][1], ret[1][2], ret[1][3], ret[1][4], ret[1][5]])
        time.sleep(0.01)
    if samples:
        baseline_forces = np.mean(samples, axis=0).tolist()
        print(f"Baseline forces: Fx={baseline_forces[0]:.2f}, Fy={baseline_forces[1]:.2f}, Fz={baseline_forces[2]:.2f}")
    else:
        baseline_forces = [0]*6

# ============================== MAIN ROBOT CONTROL LOOP ==============================
def robot_control_loop():
    global desired_pos, velocity, current_actual_pos, home_pos

    print("SDK连接机器人")
    print(robot)
    print("Robot connected.")
    
    try:
        init_ft_sensor()

        err, pos = robot.GetActualJointPosDegree()
        if err != 0:
            print("Failed to get joint positions!")
            update_http_state(0.0, 0.0, connected=False, error="Failed to get joint positions")
            return

        home_pos = pos.copy()
        desired_pos = pos.copy()
        current_actual_pos = pos.copy()

        update_http_state(desired_pos[3], desired_pos[4], connected=True)
        print(f"Initial position - J4: {desired_pos[3]:.2f}°, J5: {desired_pos[4]:.2f}°")

        calibrate_baseline_forces()

        if robot.ServoMoveStart() != 0:
            print("ServoMoveStart failed!")
            update_http_state(desired_pos[3], desired_pos[4], connected=False, error="ServoMoveStart failed")
            return

        print(f"Waiting {startup_delay} seconds...")
        time.sleep(startup_delay)

        print("\n" + "="*60)
        print("   FAIRINO PLANE SIMULATION ACTIVE")
        print("   Joint 4 ← Fx (Left/Right force)")
        print("   Joint 5 ← Fy (Forward/Backward force)")
        print("   FastAPI Server → http://YOUR_PC_IP:8080/robot_data")
        print("="*60 + "\n")

        loop_count = 0
        last_print_time = time.time()
        
        while True:
            start = time.time()

            # Read FT sensor
            ft = robot.FT_GetForceTorqueRCS()
            if ft[0] != 0:
                time.sleep(dt)
                continue

            raw = ft[1]
            forces = [raw[0], -raw[1], raw[2], raw[3], raw[4], raw[5]]

            if baseline_forces:
                forces = [f - b for f, b in zip(forces, baseline_forces)]

            forces = [0.0 if abs(f) < 0.7 else f for f in forces]

            err, actual_pos = robot.GetActualJointPosDegree()
            if err == 0:
                current_actual_pos = actual_pos

            # JOINT 4 → Fx
            fx = -forces[0]
            acc4 = (fx - B[3] * velocity[3]) / M[3]
            velocity[3] += acc4 * dt
            desired_pos[3] += velocity[3] * dt * force_to_deg

            # JOINT 5 → Fy
            fy = -forces[1]
            acc5 = (fy - B[4] * velocity[4]) / M[4]
            velocity[4] += acc5 * dt
            desired_pos[4] += velocity[4] * dt * force_to_deg

            desired_pos[3] = limit_speed(current_actual_pos[3], desired_pos[3], MAX_SPEED_DEG_S)
            desired_pos[4] = limit_speed(current_actual_pos[4], desired_pos[4], MAX_SPEED_DEG_S)
            desired_pos[3] = np.clip(desired_pos[3], JOINT4_MIN, JOINT4_MAX)
            desired_pos[4] = np.clip(desired_pos[4], JOINT5_MIN, JOINT5_MAX)

            for j in [0,1,2,5]:
                desired_pos[j] = home_pos[j]
                velocity[j] = 0.0

            err = robot.ServoJ(desired_pos, [0]*6)
            
            # Update HTTP state immediately after computing positions
            if err != 0:
                update_http_state(desired_pos[3], desired_pos[4], connected=True, error=f"ServoJ error: {err}")
            else:
                update_http_state(desired_pos[3], desired_pos[4], connected=True, error=None)

            loop_count += 1
            current_time = time.time()
            if current_time - last_print_time >= 0.5:
                print(f"Loop:{loop_count:5d} J4:{desired_pos[3]:7.2f}° J5:{desired_pos[4]:7.2f}°  "
                      f"Fx:{forces[0]:+5.1f}N Fy:{forces[1]:+5.1f}N  [Updates:{update_counter}]")
                last_print_time = current_time

            elapsed = time.time() - start
            if elapsed < dt:
                time.sleep(dt - elapsed)

    except Exception as e:
        print(f"\nError in robot control loop: {e}")
        import traceback
        traceback.print_exc()
        update_http_state(0.0, 0.0, connected=False, error=str(e))
        raise

# ============================== START EVERYTHING ==============================
if __name__ == "__main__":
    print("\n" + "="*70)
    print("  FAIRINO → UNITY FASTAPI BRIDGE")
    print("="*70 + "\n")
    
    # Start FastAPI server in daemon thread
    server_thread = threading.Thread(target=run_fastapi_server, daemon=True)
    server_thread.start()
    time.sleep(2.0)  # Give server time to start

    try:
        robot_control_loop()
    except KeyboardInterrupt:
        shutdown(None, None)
    except Exception as e:
        print(f"Fatal error: {e}")
        update_http_state(0.0, 0.0, connected=False, error=str(e))
        sys.exit(1)