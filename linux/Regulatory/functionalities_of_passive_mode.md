# Passive Mode Functionalities

## Overview
Passive mode in the unified therapy backend allows therapists to record robot trajectories through manual manipulation (drag teach) and play them back repeatedly for patient therapy sessions.

---

## Core Functionalities

### 1. **Start Recording**
**Endpoint:** `POST /start_recording`
**Parameters:** 
- `name` (string, required) - Name for the recording

**Description:**
- Enables drag teach mode for manual robot manipulation
- Initializes TPD (Teaching Point Data) recording on the robot
- Activates impedance control with predefined parameters
- Logs the recording start event

**Process:**
1. Validates recording is not already active
2. Calls `custom_drag_teach_mode(enable=True)` to enable impedance-controlled manipulation
3. Executes `SetTPDParam()` to initialize TPD with drag teaching parameters
4. Executes `SetTPDStart()` to begin recording TCP trajectory
5. Logs trajectory information to `trajectory_log.csv`

**Returns:** Success/failure status with recording name

**Error Handling:**
- Rejects if already recording
- Rejects if name is not provided
- Automatically disables drag mode if setup fails

---

### 2. **Stop Recording**
**Endpoint:** `POST /stop_recording`
**Parameters:** None

**Description:**
- Stops the active trajectory recording
- Disables impedance-controlled drag mode
- Finalizes the recorded trajectory data

**Process:**
1. Validates recording is active
2. Executes `SetWebTPDStop()` to finalize recording
3. Calls `custom_drag_teach_mode(enable=False)` to disable impedance control
4. Returns the name of the recorded trajectory

**Returns:** Success/failure status with trajectory name

**Error Handling:**
- Rejects if not currently recording
- Safely disables drag mode on failure

---

### 3. **List Recordings**
**Endpoint:** `GET /list_recordings`
**Parameters:** None

**Description:**
- Retrieves all previously recorded trajectories
- Reads from trajectory_log.csv file
- Returns sorted, deduplicated list of recording names

**Process:**
1. Opens `trajectory_log.csv` file
2. Parses CSV entries (format: Timestamp, TrajectoryName, Notes)
3. Extracts trajectory names from column 2
4. Returns sorted unique list

**Returns:** JSON list of available trajectory names

**File Format:**
```
Timestamp,TrajectoryName,Notes
2026-01-14 10:30:45,Therapy_Session_1,Recorded via Flask
2026-01-14 10:35:20,Therapy_Session_2,Recorded via Flask
```

---

### 4. **Start Playback**
**Endpoint:** `POST /start_playback`
**Parameters:**
- `filename` (string, required) - Name of trajectory file to play
- `repetitions` (integer, optional, default=1) - Number of times to repeat playback

**Description:**
- Loads and plays back a previously recorded trajectory
- Executes the trajectory multiple times based on repetitions parameter
- Runs asynchronously in a background thread
- Moves to start pose before each repetition

**Process:**
1. Validates playback is not already active
2. Launches async playback thread
3. Loads trajectory using `LoadTPD(filename)`
4. For each repetition:
   - Gets trajectory start pose with `GetTPDStartPose()`
   - Moves robot to start pose with `MoveL()`
   - Executes trajectory with `MoveTPD()` at 100% speed
   - Waits for motion completion with `GetRobotMotionDone()`
   - Waits 0.5s between repetitions
5. Tracks playback repetition count globally

**Returns:** Success/failure status

**Error Handling:**
- Rejects if playback already active
- Stops gracefully if `stop_playback()` called mid-execution
- Breaks on `LoadTPD()` or `GetTPDStartPose()` failures

**Global Tracking:**
- `playback_repetition_count` - Tracks which repetition is currently playing
- Updated in telemetry response

---

### 5. **Stop Playback**
**Endpoint:** `POST /stop_playback`
**Parameters:** None

**Description:**
- Immediately stops active trajectory playback
- Stops all robot motion
- Sets playback state to inactive

**Process:**
1. Sets `playback_active` flag to False
2. Calls `robot.StopMotion()` to halt any ongoing motion
3. Thread-safe state management

**Returns:** Success status

---

### 6. **Drag Mode Control**
**Endpoint:** `POST /drag_mode`
**Parameters:**
- `enable` (boolean, optional) - Enable/disable drag mode
  - If not provided, toggles current state

**Description:**
- Enables or disables impedance-controlled drag teaching
- Activates force/joint impedance for manual robot manipulation
- Used during recording to allow therapist to manually move robot

**Impedance Parameters Used:**
```
lamde_dain: [2.5, 2.0, 2.0, 2.0, 2.0, 2.0]
b_gain: [20.0, 10.0, 10.0, 5.0, 5.0, 1.0]
k_gain: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
max_tcp_vel: 500 mm/s
max_tcp_ori_vel: 90 deg/s
```

**Process (Enable):**
1. Calls `DragTeachSwitch(1)` to enable drag mode
2. Waits 0.5s for stabilization
3. Executes `ForceAndJointImpedanceStartStop()` with impedance parameters
4. Updates global `drag_mode_enabled` flag

**Process (Disable):**
1. Calls `ForceAndJointImpedanceStartStop()` with zero parameters
2. Calls `DragTeachSwitch(0)` to disable drag mode
3. Waits 0.5s for stabilization
4. Sets `drag_mode_enabled` to False

**Returns:** Success/failure status with current mode and enable state

**Error Handling:**
- Returns failure if impedance control fails
- Validates return codes from robot SDK

---

### 7. **Mode Switching**
**Endpoint:** `POST /set_mode`
**Parameters:**
- `mode` (string, required) - 'passive' or 'active'

**Description:**
- Switches system between active and passive therapy modes
- Initializes appropriate therapy class instance
- Cleans up previous mode resources

**Passive Mode Initialization:**
1. Validates mode is 'passive'
2. Stops any active playback from previous mode
3. Creates new `PassiveTherapy()` instance
4. Sets `current_mode = 'passive'`

**Returns:** Success status with new mode

---

### 8. **Telemetry & Status**
**Endpoint:** `GET /get_telemetry`
**Parameters:** None

**Description:**
- Returns real-time system status for passive mode
- Includes playback progress and robot state

**Response Data (Passive Mode):**
```json
{
  "fz": -0.25,                        // Force in Z-axis (N)
  "tcp_z": 245.63,                    // TCP Z position (mm)
  "mode": "passive",                  // Current mode
  "active_movement": false,           // Always false in passive
  "playback_active": true,            // Is trajectory playing
  "playback_repetition_count": 2,     // Current repetition number
  "drag_mode_enabled": false,         // Is drag teach active
  "z_limits": null                    // Always null in passive
}
```

**Update Rate:** 100ms intervals (telemetry thread)

---

## Passive Mode Functions

### Shared Helper Function

#### `custom_drag_teach_mode(enable=True)` 
*Enable/disable impedance-controlled drag teaching mode*

```python
def custom_drag_teach_mode(enable=True):
    global drag_mode_enabled, robot
    if enable:
        robot.DragTeachSwitch(1)
        time.sleep(0.5)
        rtn = robot.ForceAndJointImpedanceStartStop(
            status=1, impedanceFlag=1,
            lamdeDain=IMPEDANCE_PARAMS['lamde_dain'],
            KGain=IMPEDANCE_PARAMS['k_gain'],
            BGain=IMPEDANCE_PARAMS['b_gain'],
            dragMaxTcpVel=IMPEDANCE_PARAMS['max_tcp_vel'],
            dragMaxTcpOriVel=IMPEDANCE_PARAMS['max_tcp_ori_vel']
        )
        drag_mode_enabled = (rtn == 0)
        return rtn == 0
    else:
        robot.ForceAndJointImpedanceStartStop(
            status=0, impedanceFlag=0,
            lamdeDain=[0]*6, KGain=[0]*6, BGain=[0]*6,
            dragMaxTcpVel=1000, dragMaxTcpOriVel=180
        )
        robot.DragTeachSwitch(0)
        time.sleep(0.5)
        drag_mode_enabled = False
        return True
```

---

### PassiveTherapy Class Functions

#### `__init__(self, robot_instance)`
*Initialize passive therapy instance with robot connection*

```python
def __init__(self, robot_instance):
    self.robot = robot_instance
    self.is_recording = False
    self.current_recording_name = None
    self.playback_active = False
```

---

#### `start_recording(self, name)`
*Begin recording robot trajectory with impedance control enabled*

```python
def start_recording(self, name):
    if self.is_recording:
        return False, "Already recording"
    if not name:
        return False, "Name is required"
    try:
        success = custom_drag_teach_mode(enable=True)
        if not success:
            return False, "Failed to enable drag mode"
        ret1 = self.robot.SetTPDParam(name, 4, 0)
        if ret1 != 0:
            custom_drag_teach_mode(enable=False)
            return False, f"SetTPDParam failed (code: {ret1})"
        ret2 = self.robot.SetTPDStart(name, 4, 0)
        if ret2 != 0:
            custom_drag_teach_mode(enable=False)
            return False, f"SetTPDStart failed (code: {ret2})"
        self.is_recording = True
        self.current_recording_name = name
        return True, f"Recording started: {name}"
    except Exception as e:
        custom_drag_teach_mode(enable=False)
        return False, str(e)
```

---

#### `stop_recording(self)`
*Finalize trajectory recording and disable impedance control*

```python
def stop_recording(self):
    if not self.is_recording:
        return False, "Not recording"
    try:
        ret = self.robot.SetWebTPDStop()
        custom_drag_teach_mode(enable=False)
        name = self.current_recording_name
        self.is_recording = False
        self.current_recording_name = None
        if ret != 0:
            return False, f"SetWebTPDStop failed (code: {ret})"
        return True, name
    except Exception as e:
        return False, str(e)
```

---

#### `list_recordings(self)`
*Retrieve all recorded trajectory names from trajectory_log.csv*

```python
def list_recordings(self):
    log_file = 'trajectory_log.csv'
    names = []
    if os.path.exists(log_file):
        with open(log_file, 'r') as f:
            for line in f:
                if ',' in line:
                    parts = line.strip().split(',')
                    if len(parts) >= 2:
                        names.append(parts[1])
    return sorted(set(names))
```

---

#### `_log_trajectory(self, name)`
*Log trajectory metadata to trajectory_log.csv file*

```python
def _log_trajectory(self, name):
    log_file = 'trajectory_log.csv'
    file_exists = os.path.exists(log_file)
    with open(log_file, 'a') as f:
        if not file_exists:
            f.write('Timestamp,TrajectoryName,Notes\n')
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
        f.write(f'{timestamp},{name},Recorded via Flask\n')
```

---

#### `start_playback(self, filename, repetitions=1)`
*Load and play recorded trajectory in background thread with repetition support*

```python
def start_playback(self, filename, repetitions=1):
    global playback_repetition_count
    if self.playback_active:
        return False, "Playback already active"
    def playback_task():
        global playback_repetition_count
        try:
            self.playback_active = True
            playback_repetition_count = 0
            ret = self.robot.LoadTPD(filename)
            if ret != 0:
                print(f"LoadTPD failed: {ret}")
                return
            for rep in range(repetitions):
                if not self.playback_active:
                    break
                playback_repetition_count = rep + 1
                err, start_pose = self.robot.GetTPDStartPose(filename)
                if err != 0:
                    print(f"GetTPDStartPose failed: {err}")
                    break
                self.robot.MoveL(start_pose, 0, 0)
                time.sleep(0.5)
                self.robot.MoveTPD(filename, 1, 100)
                while self.playback_active:
                    err, done = self.robot.GetRobotMotionDone()
                    if err == 0 and done == 1:
                        break
                    time.sleep(0.1)
                if not self.playback_active:
                    break
                time.sleep(0.5)
            playback_repetition_count = repetitions
        except Exception as e:
            print(f"Playback error: {e}")
        finally:
            self.playback_active = False
    threading.Thread(target=playback_task, daemon=True).start()
    return True, "Playback started"
```

---

#### `stop_playback(self)`
*Immediately stop trajectory playback and halt robot motion*

```python
def stop_playback(self):
    self.playback_active = False
    try:
        self.robot.StopMotion()
    except:
        pass
    return True, "Playback stopped"
```

---

#### `is_playback_active(self)`
*Check if trajectory playback is currently active*

```python
def is_playback_active(self):
    return self.playback_active
```

---

## Key Global Variables in Passive Mode

| Variable | Type | Purpose |
|----------|------|---------|
| `current_mode` | str | Set to 'passive' when in passive mode |
| `passive_therapy` | PassiveTherapy | Instance of passive therapy controller |
| `playback_active` | bool | Tracks if trajectory is playing |
| `playback_repetition_count` | int | Current repetition number in playback |
| `drag_mode_enabled` | bool | Tracks impedance drag teach state |
| `current_fz` | float | Real-time Z-axis force (N) |
| `current_tcp_z` | float | Real-time TCP Z position (mm) |
| `baseline_forces` | list[6] | Force sensor calibration baseline |

---

## Workflow: Recording + Playback Cycle

### Recording Phase:
1. POST `/set_mode` → `{"mode": "passive"}`
2. POST `/drag_mode` → `{"enable": true}` (optional, auto-enabled by recording)
3. POST `/start_recording` → `{"name": "Therapy_Session_1"}`
4. [Therapist manually moves robot]
5. POST `/stop_recording`
6. Recording saved, trajectory_log.csv updated

### Playback Phase:
1. GET `/list_recordings` → Get available recordings
2. POST `/start_playback` → `{"filename": "Therapy_Session_1", "repetitions": 3}`
3. [Robot plays trajectory 3 times]
4. GET `/get_telemetry` (polled to show progress)
5. POST `/stop_playback` (if early stop needed)

---

## File Dependencies

- **trajectory_log.csv** - Stores metadata about recorded sessions
- **Robot SDK** - `/home/um/fairino-python-sdk-main/linux/fairino/Robot.py`
- **TPD Files** - Robot internal storage for trajectory data

---

## Safety Features

1. **Impedance Limitations** - Drag mode limits TCP velocity and torque
2. **Mode Isolation** - Only one mode active at a time
3. **State Validation** - Rejects commands in wrong mode
4. **Graceful Shutdown** - Disables drag mode and stops motion on exit
5. **Error Recovery** - Catches exceptions and disables drag mode safely

---

## Thread Management

- **Recording Thread** - Main thread (blocking during record)
- **Playback Thread** - Daemon thread (non-blocking, can be interrupted)
- **Telemetry Thread** - Continuous background updates (100ms)

---

## API Response Examples

### Start Recording Success
```json
{"status": "success", "name": "Hip_Extension_Exercise_1"}
```

### Start Playback Success
```json
{"status": "success"}
```

### List Recordings
```json
{"files": ["Therapy_Session_1", "Therapy_Session_2", "Hip_Extension_1"]}
```

### Passive Mode Telemetry
```json
{
  "fz": 1.25,
  "tcp_z": 248.5,
  "mode": "passive",
  "active_movement": false,
  "playback_active": true,
  "playback_repetition_count": 1,
  "drag_mode_enabled": false,
  "z_limits": null
}
```
