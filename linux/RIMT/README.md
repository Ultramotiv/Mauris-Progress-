# Robotic Isometric Muscle Testing (RIMT) System

## Overview

The **Robotic Isometric Muscle Testing (RIMT) System** is a sophisticated clinical rehabilitation platform that leverages industrial robotic technology (FaiRino 6-axis robot arm) to perform precise, reproducible, and quantifiable muscle strength assessments. The system integrates a **Flask web server**, **real-time sensor monitoring**, **bilateral testing capabilities**, and **automated data analysis** to support physical therapists in evaluating patient muscle function during rehabilitation.

### Key Capabilities
- **Bilateral Testing:** Simultaneous assessment of left and right limbs for comparative analysis
- **Real-time Monitoring:** Live force/torque and joint angle streaming
- **Automated Analysis:** Peak force, RFD (rate of force development), and temporal metrics
- **Drag Mode (Free-Drive):** Manual robot positioning with baseline force calibration
- **Data Persistence:** CSV-based patient history with progress tracking
- **Web-Based Access:** No desktop installation required; cross-device compatible

### Technology Stack
- **Frontend:** HTML5, CSS3, JavaScript (Vanilla JS)
- **Backend:** Python 3.x, Flask, Threading
- **Robotics:** FaiRino Robot SDK (RPC Protocol)
- **Sensors:** 6-axis Force/Torque (F/T) Sensor 
- **Visualization:** Matplotlib, Base64 image embedding
- **Data Storage:** CSV (results.csv)

## System Architecture
┌─────────────────────────────────────────────────────────────┐
│                    WEB BROWSER (Frontend)                   │
│  ┌──────────────────────────────────────────────────────┐   │
│  │  Interactive HTML/CSS/JavaScript Dashboard           │   │
│  │  - Patient Information Form                          │   │
│  │  - Exercise Selection (Upper/Lower Limb)             │   │
│  │  - Live Data Monitoring (Fz, Joint Angles)           │   │
│  │  - Trial Recording & Visualization                   │   │
│  │  - Progress Report Viewer                            │   │
│  └──────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────┘ 
                           │
                    HTTP REST API (JSON)
                           │
┌────────────────────────────────────────────────────────────┐
│              FLASK WEB SERVER (Backend)                    │
│  ┌─────────────────────────────────────────────────────┐   │
│  │  RobotController Class                              │   │
│  │  ├─ Robot Connection Management                     │   │
│  │  ├─ Force/Torque Sensor Monitoring                  │   │
│  │  ├─ Drag Mode (Free-Drive) Control                  │   │
│  │  ├─ Trial Recording & Analysis                      │   │
│  │  ├─ Data Visualization (Matplotlib)                 │   │
│  │  └─ Patient Data Management                         │   │
│  ├─ Flask Routes (REST API Endpoints)                  │   │
│  │  ├─ /api/status                                     │   │
│  │  ├─ /api/trial/start                                │   │
│  │  ├─ /api/trial/save                                 │   │
│  │  ├─ /api/patient                                    │   │
│  │  ├─ /api/drag_mode/*                                │   │
│  │  └─ /api/exercise/*                                 │   │
│  └─ Background Threads                                 │   │
│     └─ FT Sensor Monitoring (200ms poll)               │   │
│     └─ Plot Generation Thread                          │   │
└────────────────────────────────────────────────────────────┘
                           │
                   TCP/IP Socket (RPC)
                           │
┌─────────────────────────────────────────────────────────────┐
│           INDUSTRIAL ROBOT (FaiRino Robot Arm)              │
│  ├─ Force/Torque (F/T) Sensor (6-axis)                      │
│  ├─ 6-axis robotic arm (6 joints)                           │
│  ├─ Control Unit                                            │
│  └─ Drag Mode (Move Robot freely)                           │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│                 DATA STORAGE & LOGGING                      │
│  ├─ results.csv (Trial results)                             │
│  ├─ images/ (Exercise reference images)                     │
│  └─ Browser console (Debug logs)                            │
└─────────────────────────────────────────────────────────────┘
## Core Components

### 1. **Frontend (HTML/CSS/JavaScript)**
Embedded in Python as `HTML_TEMPLATE` string

#### Pages & Components:

##### **Main Dashboard Window**
The entry point for the system. Displays the patient intake form/ patient information and exercise selection interface.
**Components/Card on main window:**

- **Robot Connection Status Card**
  - Real-time indicator: 
        Connecting → (orange ●) 
        Connected → (green ●) 
        Active → (green ●)
  - Status message and connection feedback
  - "Enable Drag Mode" button (disabled if not connected)
  - API endpoint: `/api/status` (polls every 1000ms)

- **Patient Information Card**
  - Full Name: Text input with auto-save
  - Age: Numeric input
  - Gender: Dropdown (Male/Female/Other)
  - Diagnosis: Text input for clinical notes
  - API endpoint: `/api/patient` (POST on change)
  - Data persists in Flask backend memory

- **Exercise Grid (Upper Limb)**
  - 3-column grid layout of 8 exercise buttons
  - Exercises: Shoulder Flexion/Extension/Abduction/Adduction, Elbow Flexion/Extension, Wrist Flexion/Extension
  - Click handler: `startExercise(exerciseName)` → redirects to Exercise Testing Panel

- **Exercise Grid (Lower Limb)**
  - 3-column grid layout of 8 exercise buttons
  - Exercises: Hip Flexion/Extension/Abduction/Adduction, Ankle Plantarflexion/Dorsiflexion, Knee Extension/Flexion
  - Click handler: `startExercise(exerciseName)` → redirects to Exercise Testing Panel
---

##### **Exercise Testing Panel**
Full-screen overlay where isometric muscle testing occurs. Displays bilateral (left/right) side testing capability.

**Layout Structure:**
```
'''

**Left & Right Side Panels (Identical Layout):**
- **Header:** "Left Side" or "Right Side" with colored border
- **Live Metrics Box:**
  - Fz: Force (N) - Updated 200ms
  - J4: Joint 4 angle (°) - Updated 200ms
  - J5: Joint 5 angle (°) - Updated 200ms
  - J6: Joint 6 angle (°) - Updated 200ms
  - API: `/api/live` polling
  
- **Graph Area (520px height):**
  - Displays matplotlib-generated PNG image (base64 encoded)
  - Shows force vs. time curve with annotations:
    - Peak force marker (green ●)
    - Average force line (dashed orange)
    - RFD window (50-100ms) highlighted in blue
  - Placeholder: "Graph will appear when trial is completed"

- **Results Box (once trial is completed):**
  - 5-column grid:
    - Peak Force (kg + N)
    - Average Force (kg + N)
    - Start Force (kg + N)
    - Time to Peak (seconds)
    - Early RFD (N/s)
  - Each metric in colored box with icon (💪)

- **Control Buttons:**
  - "Start Trial" (blue) → Initiates 5-second recording
  - "Reset" (orange) → Clears graph and results
  - Click handler: `startTrial('left')` or `startTrial('right')`

- **Progress Bar:**
  - 100% width, 28px height
  - Animates from 0% to 100% over 5 seconds
  - Linear animation (5s duration)

**Live Data Polling JavaScript:**
```javascript
// Polls every 200ms during exercise page active
```

<!-- **Trial Recording Animation:**
```javascript
async function startTrial(side = 'left') {
    // 1. Disable button and show countdown
    startBtn.disabled = true;
    startBtn.textContent = 'Recording...';
    
    // 2. Animate progress bar over 5 seconds
    progressBar.style.transition = 'width 5s linear';
    progressBar.style.width = '100%';
    
    // 3. Update button text each second
    let secondsLeft = 5;
    const countdownInterval = setInterval(() => {
        secondsLeft--;
        if (secondsLeft > 0) {
            startBtn.textContent = `${secondsLeft}s left...`;
        }
    }, 1000);
    
    // 4. POST to backendp
    const response = await fetch(`${API_BASE}/trial/start`, {
        method: 'POST',
        body: JSON.stringify({ 
            exercise_name: currentExerciseName,
            duration: 5.0,
            side: side
        })
    });
    
    // 5. Display graph and metrics
    const data = await response.json();
    if (data.success) {
        plotTrialResults(data.results, side);
    }
}
``` -->

---

##### **Progress Report Page**
Historical data viewer and patient progress analytics.

**Features:**
- **Search Bar:** Patient name lookup (case-insensitive)
- **Grouped By Exercise:** Each exercise section shows left/right trial history
- **Visual Analytics:**
  - Horizontal bar charts for each trial (Peak Force in kg)
  - Color-coded: Left = blue (#3498db), Right = orange (#e67e22)
  - Gradient fills with shimmer animation
  - Trial date/time stamps
  
- **Expandable Details:**
  - Click "View All Data" to see detailed metrics
  - Columns: Trial #, Date, Time, Peak (kg), Avg (kg), Start (kg), RFD (N/s)
  - Separated into Left/Right columns

- **Print Functionality:**
  - `printProgressReport()` opens new window with printable version
  - Styled for A4 paper output

**CSS Classes:**
- `.progress-report-page` - Main container
- `.exercise-section` - Grouped exercise container
- `.side-bars` - Left/right comparison
- `.trial-bar` - Individual trial visualization

### **Notifications System**
Fixed position toast (top-right, 20px offset):
```javascript
showNotification(message, type) // type: 'success' | 'error' | 'info'
```
Styling:
- Success: Green left border (4px, #27ae60)
- Error: Red left border (4px, #e74c3c)
- Info: Blue left border (4px, #3498db)
- Auto-hide after 3 seconds
- Slide-in animation from right
---

### 2. **Backend (Flask + RobotController)**

#### **RobotController Class**
Manages all robot interactions and data processing.
**Key Attributes:**
- `robot` - Robot RPC connection instance (Robot.RPC object)
- `current_forces` - Array of 6-axis F/T sensor readings: [Fx, Fy, Fz, Tx, Ty, Tz] (Newtons / Newton-meters)
- `current_robot_joints` - Joint angle positions: [J1, J2, J3, J4, J5, J6] (degrees)
- `drag_mode_active` - Boolean state (True if free-drive enabled)
- `baseline_forces` - Calibration reference values (100-sample average for offset removal)
- `patient_data` - Dictionary with demographics (name, age, gender, diagnosis)
- `exercise_data` - Storage dictionary keyed by exercise name
- `monitoring_active` - Boolean state (True if FT monitoring thread running)
- `monitoring_thread` - Thread object reference for FT polling
- `last_trial_results` - Previous trial data (stored for potential re-analysis)
- `last_plot_image` - Base64-encoded PNG from previous trial (used for polling)
- `connection_attempted` - Boolean (prevents repeated connection attempts)

**Key Methods:**

| Method | Signature | Purpose | Returns |
|--------|-----------|---------|---------|
| `connect_robot()` | `def connect_robot(self)` | Establish RPC connection to robot at 192.168.58.2. Tries once if already attempted. | `bool` (True if successful) |
| `init_ft_sensor()` | `def init_ft_sensor(self)` | Initialize F/T sensor: SetConfig, Activate channels, SetZero, SetLoadWeight. Called before drag mode. | `bool` |
| `calibrate_baseline_forces()` | `def calibrate_baseline_forces(self)` | Collect 100 F/T samples @ 100Hz, compute mean for each axis. Used for zero-offset subtraction. | `bool` |
| `enable_drag_mode()` | `def enable_drag_mode(self)` | Activate robot free-drive (teach mode). Calls: connect_robot() → init_ft_sensor() → calibrate_baseline() → DragTeachSwitch(1) → start_ft_monitoring() | `bool` |
| `disable_drag_mode()` | `def disable_drag_mode(self)` | Deactivate free-drive. Calls DragTeachSwitch(0). | `bool` |
| `start_ft_monitoring()` | `def start_ft_monitoring(self)` | Spawn daemon thread to continuously poll F/T sensor (8ms loop). Sets `monitoring_active = True` | `None` |
| `ft_monitoring_loop()` | `def ft_monitoring_loop(self)` | Background thread target. Runs while `monitoring_active == True`. Polls F/T, applies baseline subtraction, updates `current_forces`. Logs to terminal. | `None` |
| `record_trial(duration=5.0)` | `def record_trial(self, duration=5.0)` | Capture force samples for N seconds. Loops until `time.time() - start_time > duration`. Calls FT_GetForceTorqueRCS() every 8ms. Computes peak, avg, RFD. | `dict` or `None` |
| `plot_results_in_thread(results)` | `def plot_results_in_thread(self, results)` | Generate matplotlib figure: force vs time, annotations for peak/avg/RFD, convert to base64 PNG string. Stores in `last_plot_image`. Called in async thread. | `str` (base64) or `None` |
| `calculate_early_rfd(times, forces, threshold=0.5)` | `def calculate_early_rfd(...)` | Detect force onset (threshold > 0.5N for 10 consecutive samples). Find 50ms and 100ms post-onset indices. Compute (F[100ms] - F[50ms]) / Δt. | `float` (N/s) |
| `start_exercise(exercise_name)` | `def start_exercise(self, exercise_name)` | Set `current_exercise` context. Initialize exercise storage dict if new. Log to console. | `bool` |
| `save_trial_to_csv(exercise_name, side='left', results=None)` | `def save_trial_to_csv(...)` | Append row to `results.csv` with trial metrics, patient info, side designation. Creates file if missing. | `bool` |
| `stop_monitoring()` | `def stop_monitoring(self)` | Graceful shutdown: stop FT thread, disable drag mode. | `None` |
| `log_message(msg)` | `def log_message(self, msg)` | Print timestamped console output: `[HH:MM:SS] message` | `None` |
| `print_current_joint_positions_deg_and_force_in_Fz()` | `def print_...()` | Query robot for current joint positions (degrees) and Fz force. Print to console. | `None` |

---

### 3. **Flask API Endpoints**

| Endpoint | Method | Purpose | Request | Response |
|----------|--------|---------|---------|----------|
| `/` | GET | Serve main page | - | HTML_TEMPLATE |
| `/api/status` | GET | Robot & system status | - | `{connected, drag_mode_active, monitoring_active, current_forces, current_exercise}` |
| `/api/live` | GET | Live Fz + joint angles | - | `{fz, joints[]}` |
| `/api/drag_mode/toggle` | POST | Toggle free-drive | - | `{success, active}` |
| `/api/drag_mode/enable` | POST | Enable free-drive | - | `{success, active}` |
| `/api/drag_mode/disable` | POST | Disable free-drive | - | `{success, active}` |
| `/api/patient` | GET | Fetch patient data | - | Patient dict |
| `/api/patient` | POST | Save patient data | `{name, age, gender, diagnosis}` | `{success, data}` |
| `/api/patient/progress` | GET | Search patient history | `?name=<string>` | `{success, records[]}` |
| `/api/exercise/start` | POST | Start exercise context | `{exercise_name}` | `{success, exercise}` |
| `/api/exercise/stop` | POST | End exercise context | - | `{success}` |
| `/api/exercise/image/<filename>` | GET | Serve exercise images | - | Image file (JPG/PNG) |
| `/api/trial/start` | POST | Record trial data | `{exercise_name, duration, side}` | `{success, results}` |
| `/api/trial/save` | POST | Save to CSV | `{exercise_name, side, results}` | `{success, message}` |
| `/api/trial/plot-status` | GET | Poll plot status | - | `{ready, plot_image_base64}` |
| `/api/forces` | GET | Raw sensor data | - | `{forces[], timestamp}` |

## Detailed API Reference

### Status & Connection Endpoints

#### **GET /api/status**
Returns current system state (robot connection, drag mode, monitoring, forces).
**Request:**
```
GET http://localhost:5000/api/status
```
**Response (200 OK):**
```json
{
    "connected": true,
    "drag_mode_active": false,
    "monitoring_active": true,
    "current_forces": [0.5, -0.3, 45.2, 0.1, 0.2, 0.0],
    "current_exercise": "Shoulder Flexion"
}
```

#### **GET /api/live**
Returns live Fz force and joint angles (updated 200ms intervals from frontend).
**Request:**
```
GET http://localhost:5000/api/live
```
**Response (200 OK):**
```json
{
    "fz": 45.23,
    "joints": [10.5, 20.3, -15.2, 32.1, 18.4, 5.6]
}
```
**Notes:**
- `fz`: Vertical force (Newton) from sensor. Can be negative (pulling upward).
- `joints`: [J1°, J2°, J3°, J4°, J5°, J6°] - all degrees
- J4, J5, J6 are typically wrist joints; J1-J3 are shoulder/arm
- Frontend updates display every 200ms using setInterval(fetchLiveData, 200)
```

#### **POST /api/drag_mode/toggle**
Toggle free-drive mode on/off (determines if robot can be manually moved).
**Request:**
```
POST http://localhost:5000/api/drag_mode/toggle
Content-Type: application/json
```
**Request Body:** (empty)
```json
{}
```
**Response (200 OK):**
```json
{
    "success": true,
    "active": true
}
```
**Logic:**
```
if drag_mode_active:
    disable_drag_mode()  → DragTeachSwitch(0)
else:
    enable_drag_mode()   → init hardware → calibrate → DragTeachSwitch(1)
```
**Side Effects:**
- Spawns FT monitoring thread (if enable)
- Performs sensor calibration (baseline)
- Stores calibration in `baseline_forces` array


#### **POST /api/drag_mode/enable**
Explicitly enable drag mode (redundant if toggle exists; for explicit control).
**Response:**
```json
{
    "success": true,
    "active": true
}
```

#### **POST /api/drag_mode/disable**
Explicitly disable drag mode and stop FT monitoring.
**Response:**
```json
{
    "success": true,
    "active": false
}
```

### Patient Data Endpoints

#### **GET /api/patient**
Retrieve current patient demographic information.
**Request:**
```
GET http://localhost:5000/api/patient
```
**Response (200 OK):**
```json
{
    "name": "Ashutosh babras",
    "age": "25",
    "gender": "Male",
    "diagnosis": "Rotator Cuff Tear"
}
```

#### **POST /api/patient**
Save/update patient information (called on form field change).
**Request:**
```
POST http://localhost:5000/api/patient
Content-Type: application/json
{
    "name": "Ashutosh babras",
    "age": "25",
    "gender": "Male",
    "diagnosis": "Rotator Cuff Tear"
}
```
**Response (200 OK):**
```json
{
    "success": true,
    "data": {
        "name": "Ashutosh babras",
        "age": "25",
        "gender": "Male",
        "diagnosis": "Rotator Cuff Tear"
    }
}
```
**Notes:**
- Data stored in `robot_controller.patient_data` (in-memory, not persisted between server restarts)
- Could be enhanced to save to database or JSON file
- Called by `savePatientData()` JS function on form input change

#### **GET /api/patient/progress?name=<string>**
Retrieve patient's trial history from results.csv (grouped by exercise).
**Request:**
```
GET http://localhost:5000/api/patient/progress?name=John%20Doe
```
**Response (200 OK):**
```json
{
    "success": true,
    "patient_name":" Ashutosh babras",
    "records": [
        {
            "timestamp": "2025-02-06 14:23:15",
            "exercise": "Shoulder Flexion",
            "side": "Left",
            "mode": "Isometric",
            "peak_force": "125.50",
            "avg_force": "95.23",
            "start_force": "0.50",
            "time_to_peak": "2.145",
            "early_rfd": "450"
        },
        {
            "timestamp": "2025-02-06 14:28:30",
            "exercise": "Shoulder Flexion",
            "side": "Right",
            "mode": "Isometric",
            "peak_force": "128.75",
            "avg_force": "98.50",
            "start_force": "0.45",
            "time_to_peak": "2.089",
            "early_rfd": "465"
        }
    ]
}
```
**Response (200 OK - No Records):**
```json
{
    "success": false,
    "records": [],
    "patient_name": "Ashutosh babras"
}
```
**Notes:**
- Case-insensitive name matching
- Returns records sorted by timestamp (newest first)
- Each record includes `side` designation (Left/Right)
- CSV file: `/home/um/fairino-python-sdk-main/results.csv`


### Exercise Endpoints

#### **POST /api/exercise/start**
Initialize an exercise session (set context for upcoming trials).
**Request:**
```
POST http://localhost:5000/api/exercise/start
Content-Type: application/json
{
    "exercise_name": "Shoulder Flexion"
}
```
**Response (200 OK):**
```json
{
    "success": true,
    "exercise": "Shoulder Flexion"
}
```
**Notes:**
- Sets `robot_controller.current_exercise` variable
- Creates storage dict if exercise is new
- Logs to console with timestamp

#### **POST /api/exercise/stop**
End exercise session (clear context).
**Request:**
```
POST http://localhost:5000/api/exercise/stop
```
**Response:**
```json
{
    "success": true
}
```

#### **GET /api/exercise/image/<filename>**
Serve exercise reference images (fallback with multiple extensions).
**Request:**
```
GET http://localhost:5000/api/exercise/image/Shoulder_flexion.jpeg
```
**Response:** 
- 200 OK: Image file (application/jpeg)
- 404 Not Found: "Image not found: filename" (text/plain)

**Image Directory:** `/home/um/fairino-python-sdk-main/images/`

**Fallback Logic:**
1. Try exact filename match
2. Try with .jpg, .jpeg, .png, .JPG, .JPEG, .PNG extensions
3. Return 404 if not found
**Supported Filenames:**
```
Shoulder_flexion.jpeg
Shoulder_extension.jpeg
Shoulder_adduction.jpeg
Elbow_flexion.jpeg
Elbow_extension.jpeg
Wrist_flexion.jpeg
Wrist_extension.jpeg
Hip_flexion.jpeg
Hip_extension.jpeg
Hip_abduction.jpeg
Hip_adduction.jpeg
Ankle_plantarflexion.jpeg
Ankle_dorsiflexion.jpeg
Knee_extension.jpeg
Knee_flexion.jpeg
```

### Trial Recording Endpoints

#### **POST /api/trial/start**
Initiate force recording for 5 seconds. Samples F/T sensor at 125 Hz. Returns results immediately (plot generated async in background).
**Request:**
```
POST http://localhost:5000/api/trial/start
Content-Type: application/json
{
    "exercise_name": "Shoulder Flexion",
    "duration": 5.0,
    "side": "left"
}
```
**Response (200 OK):**
```json
{
    "success": true,
    "results": {
        "peak_force": 125.50,
        "avg_force": 95.23,
        "start_force": 0.50,
        "time_to_peak": 2.145,
        "early_rfd": 450,
        "times": [0.0, 0.008, 0.016, 0.024, ...],
        "forces": [0.0, 0.2, 1.5, 3.2, ...],
        "plot_image_base64": null
    }
}
```
**Note:** 
- `plot_image_base64` usually null on immediate response (generated in background thread)
- Frontend polls `/api/trial/plot-status` every 1000ms to retrieve image when ready

**Asynchronous Flow:**
```
1. POST /api/trial/start
2. Backend: record_trial(5.0) → sleeps 5 seconds collecting samples
3. Compute metrics immediately → return JSON response
4. Spawn async thread: plot_results_in_thread()
5. Thread generates matplotlib → base64 PNG → stores in last_plot_image
6. Frontend polls /api/trial/plot-status until image ready
7. Display image when received
```

#### **POST /api/trial/save**
Save trial data to results.csv (called when user clicks "Save Both Sides").
**Request (Save Both):**
```
POST http://localhost:5000/api/trial/save
Content-Type: application/json
{
    "exercise_name": "Shoulder Flexion",
    "side": "both",
    "results_left": {
        "peak_force": 125.50,
        "avg_force": 95.23,
        ...
    },
    "results_right": {
        "peak_force": 128.75,
        "avg_force": 98.50,
        ...
    }
}
```
**Request (Save Single):**
```
POST http://localhost:5000/api/trial/save
Content-Type: application/json
{
    "exercise_name": "Shoulder Flexion",
    "side": "left",
    "results": {
        "peak_force": 125.50,
        ...
    }
}
```
**Response (Both):**
```json
{
    "success": true,
    "message": "Left side saved and Right side saved",
    "left_saved": true,
    "right_saved": true
}
```
**Response (Single):**
```json
{
    "success": true,
    "message": "Left side saved"
}
```
**CSV Output (results.csv):**
```
Timestamp,Exercise,Side,Name,Age,Gender,Diagnosis,Mode,Peak_Force_N,Average_Force_N,Start_Force_N,Time_to_Peak_s,Early_RFD_N_s
2025-02-06 14:23:15,Shoulder Flexion,Left,Ashutosh babras,25,Male,Rotator Cuff,Isometric,125.50,95.23,0.50,2.145,450
2025-02-06 14:23:45,Shoulder Flexion,Right,Ashutosh babras,25,Male,Rotator Cuff,Isometric,128.75,98.50,0.45,2.089,465
```

#### **GET /api/trial/plot-status**
Poll endpoint for plot image availability (used by frontend during async generation).
**Request:**
```
GET http://localhost:5000/api/trial/plot-status
```
**Response (Ready):**
```json
{
    "ready": true,
    "plot_image_base64": "iVBORw0KGgoAAAANSUhEUgAAA..."
}
```
**Response (Not Ready):**
```json
{
    "ready": false
}
```

**Usage Pattern:**
```javascript
// Frontend polling loop
const pollInterval = setInterval(async () => {
    const statusResponse = await fetch(`${API_BASE}/trial/plot-status`);
    const statusData = await statusResponse.json();
    
    if (statusData.ready && statusData.plot_image_base64) {
        clearInterval(pollInterval);
        // Display image: <img src="data:image/png;base64,...">
    }
}, 1000);  // Poll every second
```

#### **GET /api/forces**
Get raw 6-axis force/torque sensor readings with timestamp.
**Request:**
```
GET http://localhost:5000/api/forces
```
**Response:**
```json
{
    "forces": [0.5, -0.3, 45.2, 0.1, 0.2, 0.0],
    "timestamp": "2025-02-06T14:23:15.123456"
}
```

## Calculation Methods & Formulas

### **Force Baseline Calibration**
During `enable_drag_mode()`, 100 F/T sensor readings are collected and averaged:
$$\text{baseline}[i] = \frac{1}{N} \sum_{k=0}^{N-1} \text{raw}_k[i], \quad N = 100$$
Then, all subsequent readings are corrected:
$$\text{corrected}[i] = \text{raw}[i] - \text{baseline}[i]$$
This removes sensor drift, gravitational offset, and tool weight from measurements.

### **Fz Extraction**
From the 6-axis reading [Fx, Fy, Fz, Tx, Ty, Tz], only the vertical component is used:
$$F_z = \max(-\text{corrected}[2], 0)$$
The negation accounts for sensor axis orientation (negative Z = upward push).

### **Peak Force**
Maximum force during trial (from valid samples > 0.5 N):
$$F_{\text{peak}} = \max(F_z) \quad \text{where} \quad F_z > 0.5 \text{ N}$$

### **Average Force**
Mean of valid samples:
$$F_{\text{avg}} = \frac{1}{M} \sum_{i=1}^{M} F_{z,i} \quad \text{where} \quad F_{z,i} > 0.5 \text{ N}, \quad M = \text{count of valid samples}$$

### **Rate of Force Development (RFD) - Early Window**
RFD measures how quickly force is generated, computed over the 50-100 ms window after force onset:

**Step 1: Detect Onset**
Find first time point where F > 0.5 N for at least 10 consecutive samples (to filter noise):
$$\text{onset\_idx} = \min\{i : F_z[i] > 0.5 \text{ and } F_z[j] > 0.4 \text{ for all } j \in [i, i+10)\}$$

**Step 2: Find Window Boundaries**
Given sampling interval Δt = 0.008 s:
$$\text{idx}_{50ms} = \text{onset\_idx} + \left\lceil \frac{0.05}{\Delta t} \right\rceil = \text{onset\_idx} + 6$$
$$\text{idx}_{100ms} = \text{onset\_idx} + \left\lceil \frac{0.10}{\Delta t} \right\rceil = \text{onset\_idx} + 12$$

**Step 3: Compute Slope**
$$\text{RFD}_{\text{early}} = \frac{F_z[\text{idx}_{100ms}] - F_z[\text{idx}_{50ms}]}{t[\text{idx}_{100ms}] - t[\text{idx}_{50ms}]}$$
Units: Newtons/second (N/s)

### **Time to Peak**
Timestamp of maximum force:
$$t_{\text{peak}} = t[\arg\max(F_z)]$$
Units: seconds

### **Force Conversion: Newtons to Kilograms**
For clinical interpretation, force is converted to equivalent mass:
$$F_{\text{kg}} = \frac{F_{\text{N}}}{9.81 \text{ m/s}^2}$$
Example: 125.5 N ≈ 12.8 kg

### **Trial Recording Workflow**
```
1. User clicks "Start Trial" (Left/Right side)
   │
2. Frontend POST /api/trial/start
   │
3. Backend: record_trial(duration=5.0)
   ├─ Loop: time.time() < 5.0 seconds
   │  ├─ Read F/T sensor: robot.FT_GetForceTorqueRCS()
   │  ├─ Apply baseline subtraction (calibration offset)
   │  ├─ Extract Fz (vertical force) → only positive values
   │  ├─ Store {time, force} samples
   │  └─ Sleep 8ms (125 Hz sampling)
   │
4. Compute metrics from samples:
   ├─ peak_force = max(valid_forces)
   ├─ avg_force = mean(valid_forces > 0.5N)
   ├─ time_to_peak = timestamp of max
   ├─ early_rfd = (F[100ms] - F[50ms]) / Δt
   └─ start_force = first sample
   │
5. Generate plot in async thread:
   ├─ Create matplotlib figure
   ├─ Plot force vs time curve
   ├─ Mark peak, average, RFD window
   ├─ Convert to PNG → base64 string
   └─ Store in last_plot_image
   │
6. Return to frontend:
   ├─ JSON: {success, results{peak_force, avg_force, ...}}
   └─ Plot base64 embedded in response
   │
7. Frontend displays:
   ├─ Force graph image
   ├─ Metrics table (Peak, Avg, RFD, etc.)
   └─ "Save to CSV" button
   │
8. User clicks "Save Both Sides"
   │
9. Backend: save_trial_to_csv('left') + save_trial_to_csv('right')
   ├─ Append row to results.csv
   │  └─ Columns: Timestamp, Exercise, Side, Name, Age, Gender, ...
   └─ Return success
```

## Data Structures
### **Trial Results Object**
```python
{
    'peak_force': float,        # Newton (N)
    'avg_force': float,         # Newton (N)
    'start_force': float,       # Newton (N)
    'time_to_peak': float,      # seconds
    'early_rfd': int,           # N/s (50-100ms window)
    'times': [float],           # timestamps (seconds)
    'forces': [float],          # force samples (Newton)
    'plot_image_base64': str    # base64-encoded PNG image
}
```
### **Patient Data Object**
```python
{
    'name': str,
    'age': str,
    'gender': str,              # 'Male' | 'Female' | 'Other'
    'diagnosis': str
}
```
### **CSV Row (results.csv)**
```
Timestamp, Exercise, Side, Name, Age, Gender, Diagnosis, Mode, Peak_Force_N, 
Average_Force_N, Start_Force_N, Time_to_Peak_s, Early_RFD_N_s
Example:
2025-02-06 14:23:15, Shoulder Flexion, Left, Ashutosh babras, 25, Male, Rotator Cuff, 
Isometric, 125.50, 95.23, 0.50, 2.145, 450
```

## Threading Model
### **Main Thread**
- Flask WSGI server
- Handles HTTP requests
- Non-blocking API responses
### **Background Threads**
| Thread | Start | Function | Frequency |
|--------|-------|----------|-----------|
| **FT Monitoring** | `enable_drag_mode()` | Poll F/T sensor, update `current_forces` | 8ms (125 Hz) |
| **Plot Generation** | `POST /api/trial/start` | Generate matplotlib graph async | One-shot per trial |
| **Browser Opening** | `if __name__ == '__main__'` | Auto-launch web browser | 1.5s delay |
| **Robot Auto-Connect** | `if __name__ == '__main__'` | Initial robot connection attempt | 2s delay |
---

## **User Workflow & Code Execution**
Below section will provide detailed, step-by-step explanations of the ACTUAL user workflow and what happens in the code at each stage.
Users must follow this sequence:

1. **Page Load** → System attempts auto-connection, polls robot status every 1000ms
2. **Wait for Robot Connection** → Status indicator shows "Connecting..." until robot responds (green when ready)
3. **Enter Patient Information** → User fills name, age, gender, diagnosis (auto-saves per field)
4. **Select Exercise** → User picks exercise from grid (Shoulder Flexion, Knee Extension, etc.)
5. **Enable Drag Mode** → User clicks button to calibrate baseline forces and enable free-drive to move robot at perticular position shown in the diagram
6. **Start Trial Recording** → User clicks "Start Trial" to record 5 seconds of force data
---

### **Phase 1: Page Load & Automatic Connection Status Polling**
#### **Frontend Initialization (JavaScript)**

**On Page Load:**
```javascript
document.addEventListener('DOMContentLoaded', function() {
    // Step 1: Initialize global variables
    dragModeActive = false;
    isConnected = false;
    monitoring_active = false;
    currentExerciseName = null;
    currentPatientData = {};
    trialResults = { left: null, right: null };
    // Step 2: Load any stored patient data from backend
    loadPatientData();  // GET /api/patient
    // Step 3: Start automatic status polling (every 1 second)
    const statusPollInterval = setInterval(async () => {
        try {
            const response = await fetch(`${API_BASE}/api/status`);
            const data = await response.json();
            // Step 4: Update global connection state
            isConnected = data.connected;
            dragModeActive = data.drag_mode_active;
            monitoring_active = data.monitoring_active;
            // Step 5: Update UI to reflect current state
            updateStatusUI(isConnected, dragModeActive);
            // Step 6: Update "Enable Drag Mode" button enable/disable state
            updateDragModeButtonState(isConnected, dragModeActive);
        } catch (error) {
            console.log('Status poll error:', error);
            isConnected = false;
            updateStatusUI(false, false);
        }
    }, 1000);  // Poll every 1 second
});
```

#### **Status Display (HTML & JavaScript)**
**HTML Structure (Dashboard Header):**
```html
<div id="statusCard" class="card">
    <div class="status-container">
        <div id="statusIndicator" class="status-indicator connecting"></div>
        <div id="statusInfo">
            <h2 id="statusTitle">Status: Connecting...</h2>
            <p id="statusMessage">Waiting for robot connection...</p>
        </div>
    </div>
    <!-- Drag Mode Button -->
    <button class="btn btn-primary drag-toggle" 
            id="dragModeBtn" 
            onclick="toggleDragMode()" 
            disabled>
        Enable Drag Mode
    </button>
</div>
```

**updateStatusUI() - Visual Feedback (Line ~1371):**
```javascript
function updateStatusUI(connected, active) {
    const indicator = document.getElementById('statusIndicator');
    const title = document.getElementById('statusTitle');
    const message = document.getElementById('statusMessage');
    if (connected && !active) {
        // CONNECTED BUT DRAG MODE OFF (Ready for patient setup)
        indicator.className = 'status-indicator active';
        indicator.style.color = '#2ecc71';  // Green
        title.textContent = 'Status: Connected';
        message.textContent = 'Robot ready - Enter patient info and select exercise';
    } else if (connected && active) {
        // CONNECTED AND DRAG MODE ON (Ready for trial)
        indicator.className = 'status-indicator active';
        indicator.style.color = '#2ecc71';  // Green
        title.textContent = 'Status: Active (Free-Drive Enabled)';
        message.textContent = 'Drag mode enabled - Ready to start trial'; 
    } else {
        // NOT CONNECTED YET (Connecting...)
        indicator.className = 'status-indicator connecting';
        indicator.style.color = '#f39c12';  // Orange (pulsing)
        title.textContent = 'Status: Connecting...';
        message.textContent = 'Waiting for robot connection... Please wait';
    }
}

function updateDragModeButtonState(connected, active) {
    const btn = document.getElementById('dragModeBtn');
    if (!connected) {
        // Cannot enable drag mode if robot not connected
        btn.disabled = true;
        btn.textContent = 'Enable Drag Mode';
        btn.className = 'btn btn-primary drag-toggle';
    } else if (active) {
        // Drag mode is ON - show disable button
        btn.disabled = false;
        btn.textContent = 'Disable Drag Mode';
        btn.className = 'btn btn-danger drag-toggle';
    } else {
        // Drag mode is OFF - show enable button
        btn.disabled = false;
        btn.textContent = 'Enable Drag Mode';
        btn.className = 'btn btn-primary drag-toggle';
    }
}
```

**Result on Screen:**
```
BEFORE Connection (0-2 seconds after page load):
┌───────────────────────────────────────────┐
│ ● Status: Connecting...  (orange pulsing) │
│ "Waiting for robot connection..."         │
│                                           │
│ [Enable Drag Mode] (DISABLED/GRAYED)      │
└───────────────────────────────────────────┘
AFTER Connection Succeeds (2+ seconds):
┌───────────────────────────────────────────┐
│ ● Status: Connected  (green, solid)       │
│ "Robot ready - Enter patient info..."     │
│                                           │
│ [Enable Drag Mode] (BLUE, ENABLED)        │
└───────────────────────────────────────────┘
```

#### **Backend Auto-Connection (Python)**
**On Server Startup (Lines ~2415-2420):**
```python
if __name__ == '__main__':
    print("="*66)
    print("  ROBOT REHABILITATION CONTROL SYSTEM - WEB SERVER")
    print("="*66)
    print("")
    print("  🚀 Server starting...")
    print("  🌐 URL: http://localhost:5000")
    print("  🤖 Auto-connecting to robot at 192.168.58.2")
    print("")
    print("  Press Ctrl+C to stop the server")
    print("="*66)
    print("")
    # Step 1: Auto-open browser after 1.5 seconds
    import threading
    def open_browser():
        time.sleep(1.5)
        try:
            webbrowser.open('http://localhost:5000')
        except:
            pass
    browser_thread = threading.Thread(target=open_browser, daemon=True)
    browser_thread.start()
    # Step 2: Auto-attempt robot connection after 2 seconds
    def connect_robot_thread():
        time.sleep(2.0)
        robot_controller.connect_robot()
    robot_thread = threading.Thread(target=connect_robot_thread, daemon=True)
    robot_thread.start()
    # Step 3: Start Flask web server
    app.run(debug=False, host='0.0.0.0', port=5000, 
            use_reloader=False, threaded=True)
```

**GET /api/status Endpoint (Line ~2120):**
```python
@app.route('/api/status', methods=['GET'])
def status():
    """Return current system status to frontend"""
    return jsonify({
        'connected': robot_controller.robot is not None,
        'drag_mode_active': robot_controller.drag_mode_active,
        'monitoring_active': robot_controller.monitoring_active,
        'current_forces': robot_controller.current_forces,
        'current_robot_joints': robot_controller.current_robot_joints,
        'current_exercise': robot_controller.current_exercise
    })
```

**Timeline:**
```
Time  0.0s: User opens http://localhost:5000
Time  1.5s: Browser auto-opens (Firefox/Chrome)
Time  2.0s: Backend attempts RPC connection
       ├─ robot = Robot.RPC('192.168.58.2')
       └─ Log: "✓ Connected to robot at 192.168.58.2" or "✗ Connection failed"
Time  2.0s: Frontend's first /api/status poll receives response
       ├─ connected: true/false
       └─ Updates statusUI() → indicator turns green or stays orange
```

### **Phase 2: User Enters Patient Information**
#### **Patient Form (Frontend HTML)**
**Location: Main Dashboard (Lines ~650-700):**
```html
<div class="card">
    <h3 class="card-title">Patient Information</h3>
    <div class="form-group">
        <label for="patientName">Full Name:</label>
        <input id="patientName" type="text" 
               placeholder="Enter patient name"
               onchange="savePatientData()">
    </div>
    <div class="form-group">
        <label for="patientAge">Age:</label>
        <input id="patientAge" type="number" 
               placeholder="Enter age"
               onchange="savePatientData()">
    </div>
    <div class="form-group">
        <label for="patientGender">Gender:</label>
        <select id="patientGender" onchange="savePatientData()">
            <option value="">Select Gender</option>
            <option value="Male">Male</option>
            <option value="Female">Female</option>
            <option value="Other">Other</option>
        </select>
    </div>
    <div class="form-group">
        <label for="patientDiagnosis">Diagnosis:</label>
        <input id="patientDiagnosis" type="text" 
               placeholder="Enter clinical diagnosis"
               onchange="savePatientData()">
    </div>
</div>
```

**JavaScript: savePatientData() (Line ~1450):**
```javascript
function savePatientData() {
    // Step 1: Collect form values
    const patientData = {
        name: document.getElementById('patientName').value || '',
        age: document.getElementById('patientAge').value || '',
        gender: document.getElementById('patientGender').value || '',
        diagnosis: document.getElementById('patientDiagnosis').value || ''
    };
    // Step 2: Validate - name is required
    if (!patientData.name.trim()) {
        console.log('Patient name required');
        return;
    }
    // Step 3: Store in global variable (used when saving trials)
    currentPatientData = patientData;
    // Step 4: Send to backend for storage
    fetch(`${API_BASE}/api/patient`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(patientData)
    })
    .then(response => response.json())
    .then(data => {
        console.log('Patient data saved:', data.data);
        // Show optional toast notification
        showNotification('Patient info saved', 'success');
    })
    .catch(e => console.error('Error saving patient:', e));
}
```

#### **Backend: POST /api/patient**
**Endpoint (Line ~2180):**
```python
@app.route('/api/patient', methods=['POST'])
def save_patient_data():
    """Save patient demographic information"""
    data = request.json
    # Step 1: Extract fields
    name = data.get('name', '').strip()
    age = data.get('age', '')
    gender = data.get('gender', '')
    diagnosis = data.get('diagnosis', '')
    # Step 2: Validate name required
    if not name:
        return jsonify({'success': False, 'error': 'Name required'})
    # Step 3: Store in RobotController instance
    robot_controller.patient_data = {
        'name': name,
        'age': age,
        'gender': gender,
        'diagnosis': diagnosis
    }
    # Step 4: Log the action
    robot_controller.log_message(
        f"[PATIENT] Data saved: Name={name}, Age={age}, Gender={gender}, Diagnosis={diagnosis}"
    )
    # Step 5: Return confirmation
    return jsonify({
        'success': True,
        'data': robot_controller.patient_data
    })
```

**Example User Interaction:**
```
User enters in form:
  Name: Ashutosh babras
  Age:  25
  Gender: Male
  Diagnosis: Rotator Cuff Tear
Result (visible to user):
  Toast notification (top-right): "Patient info saved" ✓
Result (backend console):
  [14:23:15] [PATIENT] Data saved: Name=Ashutosh babras, Age=25, Gender=Male, Diagnosis=Rotator Cuff Tear
```

### **Phase 3: User Selects Exercise**
#### **Exercise Grid (Frontend HTML)**
**Location: Main Dashboard (Lines ~750-850):**
```html
<div class="card">
    <h3 class="card-title">Select Exercise</h3>
    <!-- Upper Limb Exercises (Blue buttons) -->
    <div class="exercise-section">
        <h4>Upper Limb</h4>
        <div class="exercise-grid">
            <button class="exercise-btn upper-btn" 
                    onclick="startExercise('Shoulder Flexion')">
                Shoulder Flexion
            </button>
            <button class="exercise-btn upper-btn" 
                    onclick="startExercise('Shoulder Extension')">
                Shoulder Extension
            </button>
            <!-- ... 6 more upper limb exercises ... -->
        </div>
    </div>
    <!-- Lower Limb Exercises (Orange buttons) -->
    <div class="exercise-section">
        <h4>Lower Limb</h4>
        <div class="exercise-grid">
            <button class="exercise-btn lower-btn" 
                    onclick="startExercise('Hip Flexion')">
                Hip Flexion
            </button>
            <!-- ... 7 more lower limb exercises ... -->
        </div>
    </div>
</div>
```

**JavaScript: startExercise() (Line ~1500):**
```javascript
async function startExercise(exerciseName) {
    // Step 1: Validate connection
    if (!isConnected) {
        showNotification('Robot not connected yet', 'error');
        return;
    }
    // Step 2: Validate patient name entered
    const patientName = document.getElementById('patientName').value.trim();
    if (!patientName) {
        showNotification('Please enter patient name first', 'error');
        return;
    }
    // Step 3: Store exercise name globally (used in trial recording)
    currentExerciseName = exerciseName;
    // Step 4: Update page title
    document.getElementById('exerciseTitle').textContent = 
        `${exerciseName} | Axis: Fz (Vertical Force)`;
    // Step 5: Download and display exercise reference image
    const imgElement = document.getElementById('exerciseImg');
    imgElement.src = `/api/exercise/image/${exerciseName.replace(/ /g, '_').toLowerCase()}.jpeg`;
    // Makes GET /api/exercise/image/shoulder_flexion.jpeg
    // Step 6: Hide main dashboard, show exercise testing panel
    document.getElementById('mainDashboard').style.display = 'none';
    document.getElementById('exercisePage').classList.add('active');
    // Step 7: Clear previous results
    document.getElementById('graphAreaLeft').innerHTML = 
        '<div style="text-align:center; padding:80px 20px; color:rgba(255,255,255,0.6);">Graph will appear here when you start a trial</div>';
    document.getElementById('graphAreaRight').innerHTML = 
        '<div style="text-align:center; padding:80px 20px; color:rgba(255,255,255,0.6);">Graph will appear here when you start a trial</div>';
    // Step 8: Clear previous trial results from memory
    trialResults.left = null;
    trialResults.right = null;
    // Step 9: Notify backend that exercise session started
    try {
        await fetch(`${API_BASE}/api/exercise/start`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ exercise_name: exerciseName })
        });
        robot_controller.log_message(`[EXERCISE] Started session: ${exerciseName}`);
    } catch (e) {
        console.error('Error starting exercise:', e);
    }
    // Step 10: Show notification
    showNotification(`Exercise loaded: ${exerciseName}`, 'success');
    // Step 11: Begin live data polling (every 200ms)
    // Updates left/right panel with current Fz and joint angles
    const liveDataInterval = setInterval(async () => {
        if (!isConnected || !currentExerciseName) return;
        try {
            const response = await fetch(`${API_BASE}/api/live`);
            const data = await response.json();
            // Step 12: Update left side live values
            document.getElementById('liveFzLeft').textContent = 
                (data.fz || 0).toFixed(2) + ' N';
            document.getElementById('j4Left').textContent = 
                (data.joints?.[3] || 0).toFixed(1) + '°';
            document.getElementById('j5Left').textContent = 
                (data.joints?.[4] || 0).toFixed(1) + '°';
            document.getElementById('j6Left').textContent = 
                (data.joints?.[5] || 0).toFixed(1) + '°';
            // Step 13: Update right side live values (same sensor data)
            document.getElementById('liveFzRight').textContent = 
                (data.fz || 0).toFixed(2) + ' N';
            document.getElementById('j4Right').textContent = 
                (data.joints?.[3] || 0).toFixed(1) + '°';
            document.getElementById('j5Right').textContent = 
                (data.joints?.[4] || 0).toFixed(1) + '°';
            document.getElementById('j6Right').textContent = 
                (data.joints?.[5] || 0).toFixed(1) + '°';   
        } catch (e) {
            console.log('Live data poll error');
        }
    }, 200);  // Poll every 200ms
}
```

**Result on Screen:**
```
EXERCISE TESTING PANEL (Full-Screen)
───────────────────────────────────────────────────────────────
SHOULDER FLEXION | Axis: Fz (Vertical Force)

┌──────────────────────────────────────────────────────────┐
│           [Exercise Reference Image]                     │
│     Shows correct shoulder flexion position              │
│     (200 x 250 pixels)                                   │
└──────────────────────────────────────────────────────────┘

LEFT SIDE PANEL              RIGHT SIDE PANEL
─────────────────────────────────────────────────────────
Fz: 0.00 N                   Fz: 0.00 N
J4: 0.0°                     J4: 0.0°
J5: 0.0°                     J5: 0.0°
J6: 0.0°                     J6: 0.0°

[Graph Area]                 [Graph Area]
(empty)                      (empty)

[Start Trial] [Reset]        [Start Trial] [Reset]
─────────────────────────────────────────────────────────
[💾 Save Both Sides to CSV]
[Back to Dashboard]
```

**Live Data Updates (Every 200ms):**
- Fz value updates from current sensor reading (either 0.0N or whatever is being pushed)
- J4, J5, J6 update from current robot joint positions
---

### **Phase 4: User Enables Drag Mode (Critical Pre-Trial Step)**
#### **Prerequisites:**
- ✓ Robot is connected
- ✓ Patient info entered
- ✓ Exercise selected
- ✓ Exercise testing panel displayed
#### **Frontend: toggleDragMode() Button Click (Line ~1430)**
**Event Handler:**
```html
<button class="btn btn-primary drag-toggle" 
        id="dragModeBtn"
        onclick="toggleDragMode()">
    Enable Drag Mode
</button>
```
**JavaScript Function:**
```javascript
async function toggleDragMode() {
    // Step 1: Get all drag mode buttons (appears in multiple places)
    const btns = document.querySelectorAll('.drag-toggle');
    // Step 2: Save original button text (for error recovery)
    const originalTexts = Array.from(btns).map(b => b.textContent);
    // Step 3: Show "Processing..." feedback to user
    btns.forEach(b => {
        b.disabled = true;
        b.textContent = 'Processing...';
    });
    // Step 4: Send toggle request to backend
    try {
        const response = await fetch(`${API_BASE}/api/drag_mode/toggle`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' }
        });
        // Step 5: Parse backend response
        const data = await response.json();
        if (data.success) {
            // SUCCESS: Drag mode toggled
            showNotification(
                data.active ? 
                    'Drag mode ENABLED - Robot in free-drive mode' : 
                    'Drag mode DISABLED', 
                'success'
            );
            // Step 6a: Refresh UI to reflect new state
            await updateStatus();  // Makes GET /api/status
            
        } else {
            // FAILURE: Operation failed
            showNotification(
                'Failed to toggle drag mode: ' + (data.error || 'Unknown error'), 
                'error'
            );
            // Step 6b: Restore buttons to original state
            btns.forEach((b, i) => {
                b.textContent = originalTexts[i];
                b.disabled = false;
            });
        }
        
    } catch (error) {
        // NETWORK ERROR
        showNotification(
            'Connection error: ' + error.message, 
            'error'
        );
        btns.forEach((b, i) => {
            b.textContent = originalTexts[i];
            b.disabled = false;
        });
    }
}
```

#### **Backend: POST /api/drag_mode/toggle (Line ~2041)**
**Flask Route:**
```python
@app.route('/api/drag_mode/toggle', methods=['POST'])
def toggle_drag():
    """Toggle drag mode on/off"""
    global robot_controller
    # Step 1: Check current state
    if robot_controller.drag_mode_active:
        # Currently ON → Disable it
        success = robot_controller.disable_drag_mode()
        # Calls robot.DragTeachSwitch(0)
    else:
        # Currently OFF → Enable it (multi-step initialization)
        success = robot_controller.enable_drag_mode()
    # Step 2: Return new state
    return jsonify({
        'success': success,
        'active': robot_controller.drag_mode_active
    })
```

**Inside enable_drag_mode() (Lines ~1676-1730):**
```python
def enable_drag_mode(self):
    """Enable drag/teach mode with full sensor initialization"""
    try:
        # STEP 1: Connect to robot (if not already connected)
        if not self.robot:
            if not self.connect_robot():
                self.log_message("✗ Cannot enable drag mode: Robot not connected")
                return False
        # STEP 2: Initialize F/T sensor hardware
        if not self.init_ft_sensor():
            self.log_message("⚠ Sensor initialization may have issues")
        # STEP 3: CRITICAL CALIBRATION - Collect baseline forces
        # This step removes gravitational bias, tool weight, and sensor offset
        self.calibrate_baseline_forces()
        # Inside calibrate_baseline_forces():
        # ├─ samples = []
        # ├─ Loop 100 times:
        // │  ├─ Call: robot.FT_GetForceTorqueRCS()
        // │  │  Returns: (error_code, (Fx_raw, Fy_raw, Fz_raw, Tx, Ty, Tz))
        // │  │  Example: (0, (5.2, -3.1, 127.5, 0.05, -0.02, 0.1))
        // │  ├─ Append [5.2, -(-3.1), 127.5, ...] to samples
        // │  └─ sleep(0.01)  # 10ms between samples
        // └─ Compute mean of all 100 samples:
        //    baseline_forces[0] = mean(Fx values) ≈ 5.0
        //    baseline_forces[1] = mean(Fy values) ≈ 2.8
        //    baseline_forces[2] = mean(Fz values) ≈ 2.1  ← Gravity!
        //    ... etc for all 6 axes
        # STEP 4: Enable robot free-drive mode (low mechanical resistance)
        ret = self.robot.DragTeachSwitch(1)
        if ret != 0:
            self.log_message(f"✗ DragTeachSwitch(1) failed with code {ret}")
            return False
        # STEP 5: Start continuous background F/T monitoring thread
        self.start_ft_monitoring()
        
        # Inside start_ft_monitoring():
        # ├─ monitoring_active = True
        // ├─ Spawn background daemon thread: ft_monitoring_loop()
        // │  └─ Loop continuously while monitoring_active:
        // │     ├─ Read: robot.FT_GetForceTorqueRCS() every 8ms
        // │     │  Returns: [Fx_raw, Fy_raw, Fz_raw, Tx, Ty, Tz]
        // │     │  Example: [5.2, -3.1, 127.5, 0.05, -0.02, 0.1]
        // │     ├─ Apply baseline subtraction:
        // │     │  forces[i] = raw[i] - baseline_forces[i]
        // │     │  Example: forces[2] = 127.5 - 2.1 = 125.4 (gravity removed!)
        // │     ├─ Update global: current_forces = forces
        // │     ├─ Print to terminal (live feedback):
        // │     │  "[14:23:15] Forces: Fx=0.23N, Fy=-0.15N, Fz=0.12N, ..."
        // │     └─ sleep(0.008)  ← 8ms = 125 Hz sampling
        // └─ Return immediately (thread continues in background)
        
        # STEP 6: Mark as active
        self.drag_mode_active = True
        self.log_message("✓ FREE-DRIVE ACTIVE - Ready for trial")
        return True
        
    except Exception as e:
        self.log_message(f"✗ Error enabling drag mode: {e}")
        import traceback
        traceback.print_exc()
        return False
```

#### **Frontend Response Processing**
**updateStatus() Refresh (Line ~1371):**
```javascript
async function updateStatus() {
    try {
        // Step 1: Fetch status from backend
        const response = await fetch(`${API_BASE}/api/status`);
        const data = await response.json();
        
        // Step 2: Update global variables
        isConnected = data.connected;
        dragModeActive = data.drag_mode_active;
        monitoring_active = data.monitoring_active;
        
        // Step 3: Update UI display
        updateStatusUI(isConnected, dragModeActive);
        updateDragModeButtonState(isConnected, dragModeActive);
        
    } catch (e) {
        console.log('Status update failed:', e);
    }
}
```

**UI Updates After Enable Success:**

```
BEFORE clicking "Enable Drag Mode":
Status: Connected  (green)
Button: [Enable Drag Mode] (blue)

DURING click (0.5-1.5 seconds):
Status: Connected  (green)
Button: [Processing...] (grayed)

Backend is doing:
├─ Initializing sensor (0.1s)
├─ Calibrating baseline (1.0s) ← Collecting 100 samples
├─ Starting monitoring thread (0.1s)
└─ Total: ~1.2 seconds

AFTER enable succeeds:
Status: Active (Free-Drive Enabled)  (green)
Button: [Disable Drag Mode] (red, enabled)

Terminal Output:
✓ FREE-DRIVE ACTIVE - Ready for trial
Forces: Fx=0.23, Fy=-0.15, Fz=0.12, ...  (prints every 8ms)
Forces: Fx=0.22, Fy=-0.14, Fz=0.10, ...
Forces: Fx=0.24, Fy=-0.16, Fz=0.11, ...
```

### **Phase 5: User Clicks "Start Trial" (Left Side)**
#### **Prerequisites Now Met:**
- ✓ Robot connected
- ✓ Patient info saved
- ✓ Exercise selected
- ✓ Drag mode enabled + baseline calibrated
- ✓ Live polling active (200ms)
- ✓ F/T monitoring thread running (125 Hz background)
#### **Frontend: startTrial('left') (Line ~1548)**

**Event Handler:**
```html
<button class="side-control-btn start" 
        onclick="startTrial('left')">
    Start Trial
</button>
```
**JavaScript Function (Detailed):**
```javascript
async function startTrial(side = 'left') {
    // Step 1: Validate pre-conditions
    if (!isConnected) {
        showNotification('Robot not connected', 'error');
        return;
    }
    const sideUpper = side.charAt(0).toUpperCase() + side.slice(1);
    // Result: sideUpper = "Left"
    const startBtn = document.querySelector(`.start[onclick="startTrial('${side}')"]`);
    const progressBar = document.getElementById(`progressBar${sideUpper}`);
    if (!startBtn || !progressBar) {
        console.error(`UI elements not found for side: ${side}`);
        return;
    }
    // Step 2: Disable button to prevent double-clicking
    startBtn.disabled = true;
    startBtn.textContent = 'Recording...';
    // Step 3: Show user notification + start progress animation
    showNotification(`🔴 Recording ${side} side - 5 seconds`, 'info');
    progressBar.style.width = '0%';
    progressBar.style.transition = 'width 5s linear';
    progressBar.style.width = '100%';
    // Progress bar will animate from 0% to 100% over 5 seconds
    // Step 4: Add countdown text (updates every second)
    let secondsLeft = 5;
    const countdownInterval = setInterval(() => {
        secondsLeft--;
        if (secondsLeft > 0) {
            startBtn.textContent = `${secondsLeft}s left...`;
        }
    }, 1000);
    // Step 5: Send request to backend
    // WARNING: This will BLOCK for ~5 seconds while recording
    try {
        const response = await fetch(`${API_BASE}/api/trial/start`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ 
                exercise_name: currentExerciseName,  // "Shoulder Flexion"
                duration: 5.0,                        // seconds
                side: side                            // "left"
            })
        });
        // Step 6: Wait for response (backend is recording for 5 seconds)
        const data = await response.json();
        clearInterval(countdownInterval);
        if (data.success && data.results) {
            // Step 7: SUCCESS - Save results to global
            trialResults[side] = data.results;
            // Example data.results:
            // {
            //   peak_force: 125.50,
            //   avg_force: 95.23,
            //   start_force: 0.50,
            //   time_to_peak: 2.145,
            //   early_rfd: 450,
            //   times: [0.0, 0.008, 0.016, ..., 4.992],
            //   forces: [0.0, 0.2, 1.5, ..., 125.5, 120.0, ...]
            // }
            // Step 8: Display results immediately
            plotTrialResults(data.results, side);
            // Step 9: Check if plot image is included
            if (!data.results?.plot_image_base64) {
                // Plot not ready yet (still generating in backend thread)
                // Start polling for it
                let pollAttempts = 0;
                const pollInterval = setInterval(async () => {
                    pollAttempts++;
                    try {
                        const statusResponse = await fetch(
                            `${API_BASE}/api/trial/plot-status`
                        );
                        const statusData = await statusResponse.json();
                        
                        if (statusData.ready && statusData.plot_image_base64) {
                            // Plot is ready!
                            clearInterval(pollInterval);
                            
                            // Update results with image
                            data.results.plot_image_base64 = 
                                statusData.plot_image_base64;
                            trialResults[side] = data.results;
                            
                            // Display graph
                            plotTrialResults(data.results, side);
                            
                            showNotification(
                                '✓ Graph ready!', 
                                'success'
                            );
                        }
                    } catch (e) {
                        console.log('Plot status poll error');
                    }
                    
                    // Stop polling after 30 attempts (30 seconds max)
                    if (pollAttempts >= 30) {
                        clearInterval(pollInterval);
                        showNotification(
                            'Plot took too long to generate', 
                            'warning'
                        );
                    }
                }, 1000);  // Poll every 1 second
            }          
            // Step 10: Show success notification
            showNotification(
                `✓ ${sideUpper} side trial completed!`, 
                'success'
            );
            // Step 11: Re-enable button for next trial
            startBtn.disabled = false;
            startBtn.textContent = 'Start Trial'; 
        } else {
            // Failed
            showNotification(
                'Trial recording failed: ' + (data.error || 'Unknown error'), 
                'error'
            );
            startBtn.disabled = false;
            startBtn.textContent = 'Start Trial';
        }
    } catch (error) {
        // Network error or timeout
        showNotification(
            'Connection error: ' + error.message, 
            'error'
        );
        clearInterval(countdownInterval);
        startBtn.disabled = false;
        startBtn.textContent = 'Start Trial';
        progressBar.style.width = '0%';
    }
}
```

#### **Backend: POST /api/trial/start (Lines ~2231-2280)**
**Flask Route:**
```python
@app.route('/api/trial/start', methods=['POST'])
def start_trial():
    """Start trial recording: capture 5 seconds of force data"""
    data = request.json
    exercise_name = data.get('exercise_name')  # "Shoulder Flexion"
    duration = data.get('duration', 5.0)       # 5.0
    side = data.get('side', 'left')            # "left"
    # Step 1: Validate robot is ready
    if not robot_controller.robot:
        return jsonify({
            'success': False, 
            'error': 'Robot not connected'
        })
    # Step 2: Call record_trial() to collect 5 seconds of force data
    # THIS BLOCKS FOR ~5 SECONDS
    results = robot_controller.record_trial(duration)
    # Step 3: If recording successful, spawn async plot generation
    if results:
        def generate_plot_async():
            """Generate matplotlib graph in background thread"""
            try:
                robot_controller.log_message(
                    "[ASYNC] Starting plot generation..."
                )
                image_base64 = robot_controller.plot_results_in_thread(results)
                
                if image_base64:
                    # Store in controller for later polling
                    results['plot_image_base64'] = image_base64
                    robot_controller.last_plot_image = image_base64
                    robot_controller.log_message(
                        f"[ASYNC] Plot generated: {len(image_base64)} chars"
                    )
            except Exception as e:
                robot_controller.log_message(f"[ASYNC] Plot error: {e}")
        # Start plotting in background (non-blocking)
        import threading
        plot_thread = threading.Thread(target=generate_plot_async, daemon=True)
        plot_thread.start()
        # Note: Thread continues while we return response immediately
    # Step 4: Return results immediately (plot may still be generating)
    return jsonify({
        'success': True,
        'results': results
    })
```

**Inside record_trial(5.0) (Lines ~1760-1820):**
```python
def record_trial(self, duration=5.0):
    """Record F/T sensor data for N seconds at 125 Hz (8ms interval)"""
    samples = []  # List of force values (Fz)
    times = []    # List of timestamps
    start_time = time.time()
    self.log_message(
        f"[TRIAL] Recording {duration}s trial for {self.current_exercise}..."
    )
    # MAIN RECORDING LOOP: Run for exactly 5 seconds
    while time.time() - start_time < duration:
        elapsed = time.time() - start_time
        # elapsed progresses: 0.0, 0.008, 0.016, 0.024, ..., 4.992, 5.000
        try:
            # Step 1: Read raw F/T sensor data from robot
            status, raw_data = self.robot.FT_GetForceTorqueRCS()
            # status: error code (0 = success)
            # raw_data: tuple (Fx_raw, Fy_raw, Fz_raw, Tx, Ty, Tz)
            # Example: (5.2, -3.1, 127.5, 0.05, -0.02, 0.1)
            if status == 0:  # Check for read success
                # Step 2: Extract and flip Y axis (by convention)
                fx, fy, fz, tx, ty, tz = raw_data
                fy = -fy  # Flip Y axis
                # Step 3: Apply baseline subtraction (remove calibration offset)
                if self.baseline_forces:
                    forces = [
                        fx - self.baseline_forces[0],    # Remove Fx offset
                        fy - self.baseline_forces[1],    # Remove Fy offset
                        fz - self.baseline_forces[2]     # Remove Fz offset (gravity!)
                    ]
                else:
                    forces = [fx, fy, fz, tx, ty, tz]
                # Step 4: Extract Fz (vertical force)
                # Negate because sensor axis is inverted
                # Only keep positive values (pushing down, not pulling up)
                fz_value = max(-forces[2], 0)
                # Example: max(-(-125.4), 0) = 125.4
                # Step 5: Record sample
                samples.append(round(fz_value, 2))  # [0.0, 0.2, 1.5, ...]
                times.append(round(elapsed, 3))    # [0.0, 0.008, 0.016, ...]
                # Step 6: Print to terminal for live feedback
                print(f"[{elapsed:.3f}s] Fz: {fz_value:.2f}N")
        except Exception as e:
            self.log_message(f"[TRIAL] Read error: {e}")
        # Step 7: Sleep 8ms before next read
        # 125 Hz = 1/125 = 0.008s = 8ms
        time.sleep(0.008)
    
    # ========== POST-PROCESSING (after 5 seconds) ==========
    
    self.log_message(
        f"[TRIAL] Complete - Collected {len(samples)} samples in {duration}s"
    )
    # Step 8: Filter out noise (only keep forces > 0.5N)
    valid = [f for f in samples if f > 0.5]
    # Step 9: Compute performance metrics
    if valid:
        peak_force = max(valid)                              # Max force
        avg_force = sum(valid) / len(valid)                 # Mean force
        time_to_peak = times[samples.index(peak_force)]     # Time at max
        early_rfd = self.calculate_early_rfd(times, samples)  # RFD
        start_force = samples[0] if samples else 0.0
    else:
        # No valid data (very weak effort)
        peak_force = avg_force = start_force = time_to_peak = early_rfd = 0.0
    # Step 10: Return results dictionary
    return {
        'peak_force': round(peak_force, 2),
        'avg_force': round(avg_force, 2),
        'start_force': round(start_force, 2),
        'time_to_peak': round(time_to_peak, 3),
        'early_rfd': int(early_rfd),
        'times': times,     # For graph X-axis
        'forces': samples   # For graph Y-axis
    }
```

#### **Async Plot Generation (Backend Thread)**

**Inside plot_results_in_thread(results) (Lines ~2078-2130):**
```python
def plot_results_in_thread(self, results):
    """Generate matplotlib force vs time graph"""
    times = results.get('times', [])
    forces = results.get('forces', [])
    self.log_message(f"[PLOT] Starting matplotlib generation with {len(times)} points")
    if not times or not forces:
        return None
    try:
        # Step 1: Create figure (hidden, no display)
        fig, ax = plt.subplots(figsize=(12, 5), dpi=100)
        
        # Step 2: Plot main force curve (red, 2.5pt line width)
        ax.plot(times, forces, color='#e74c3c', linewidth=2.5, label='Force')
        
        # Step 3: Mark peak force (green circle)
        peak = results.get('peak_force')
        if peak:
            try:
                p_idx = forces.index(peak)
                ax.plot(times[p_idx], forces[p_idx], 'o', 
                        color='#27ae60', markersize=10, 
                        label='Peak Force', zorder=5)
                ax.text(times[p_idx], forces[p_idx] + 1, 
                        f'Peak\n{peak:.1f}N', 
                        fontsize=9, ha='center', 
                        color='#27ae60', fontweight='bold')
            except:
                pass
        
        # Step 4: Add average force line (orange dashed)
        avg_force = results.get('avg_force', 0)
        ax.axhline(y=avg_force, color='#f39c12', 
                   linestyle='--', linewidth=2, 
                   label='Average Force')
        
        # Step 5: Highlight RFD window (50-100ms post-onset)
        onset_idx = None
        threshold = 0.5
        for i in range(len(forces)):
            if forces[i] > threshold:
                if all(forces[j] > threshold * 0.8 
                       for j in range(i, min(i+10, len(forces)))):
                    onset_idx = i
                    break
        
        if onset_idx and len(times) > 1:
            dt = times[1] - times[0]  # 0.008
            idx_50 = onset_idx + int(0.05 / dt)   # +6 samples
            idx_100 = onset_idx + int(0.10 / dt)  # +12 samples
            idx_50 = max(0, min(len(forces)-1, idx_50))
            idx_100 = max(0, min(len(forces)-1, idx_100))
            
            if idx_50 < len(forces) and idx_100 < len(forces):
                ax.plot([times[idx_50], times[idx_100]], 
                        [forces[idx_50], forces[idx_100]], 
                        color='#3498db', linewidth=4, 
                        marker='o', markersize=8,
                        label='Early RFD (50-100ms)', zorder=4)
                
                ax.text(times[idx_50], forces[idx_50] - 1.5, 
                        'RFD Start\n50ms', fontsize=8, 
                        ha='center', color='#3498db', fontweight='bold')
                ax.text(times[idx_100], forces[idx_100] + 1, 
                        'RFD End\n100ms', fontsize=8, 
                        ha='center', color='#3498db', fontweight='bold')
        
        # Step 6: Configure axes
        ax.set_title('Force vs Time (Isometric Trial)', 
                     fontsize=14, fontweight='bold')
        ax.set_xlabel('Time (s)', fontsize=12)
        ax.set_ylabel('Force (N)', fontsize=12)
        ax.grid(True, alpha=0.3, color='gray')
        ax.set_facecolor('white')
        fig.patch.set_facecolor('white')
        
        # Step 7: Add legend
        ax.legend(loc='upper left', fontsize=10, 
                  facecolor='white', edgecolor='black', 
                  labelcolor='black', framealpha=0.95)
        
        # Step 8: Save to buffer (PNG format)
        plt.tight_layout()
        buffer = io.BytesIO()
        fig.savefig(buffer, format='png', facecolor='white', 
                    dpi=100, bbox_inches='tight')
        buffer.seek(0)
        
        # Step 9: Convert to base64 string
        image_data = buffer.read()
        image_base64 = base64.b64encode(image_data).decode('utf-8')
        # Result: "iVBORw0KGgoAAAANSUhEUgAAA..." (100+ KB)
        
        # Step 10: Clean up
        plt.close(fig)
        
        self.log_message(f"[PLOT] Done: {len(image_base64)} char base64 string")
        return image_base64
        
    except Exception as e:
        self.log_message(f"[PLOT] ERROR: {e}")
        import traceback
        traceback.print_exc()
        return None
```

#### **Frontend: Display Results**
**plotTrialResults(results, 'left') (Lines ~1641-1710):**
```javascript
function plotTrialResults(results, side = 'left') {
    const sideUpper = side.charAt(0).toUpperCase() + side.slice(1);
    const graphArea = document.getElementById(`graphArea${sideUpper}`);
    // Step 1: Check if plot image available
    let graphHTML = '';
    if (results?.plot_image_base64) {
        // YES: Plot is ready
        graphHTML = `<img src="data:image/png;base64,${results.plot_image_base64}" 
                          style="max-width: 100%; height: auto; border-radius: 6px;">`;
    } else {
        // NO: Plot not ready yet
        graphHTML = `<div style="height: 350px; background: rgba(0,0,0,0.2); 
                                border: 1px dashed rgba(255,255,255,0.3); 
                                display: flex; align-items: center; 
                                justify-content: center; border-radius: 6px;">
                        <div style="text-align: center; color: rgba(255,255,255,0.6);">
                            <div style="font-size: 14px; margin-bottom: 10px;">⏳ Generating graph...</div>
                            <div style="font-size: 12px;">This usually takes 1-2 seconds</div>
                        </div>
                     </div>`;
    }
    // Step 2: Build complete results display
    graphArea.innerHTML = `
        <div style="width: 100%;">
            <!-- Graph Image Section -->
            <div style="margin-bottom: 20px;">
                ${graphHTML}
            </div>
            <!-- Results Metrics Section -->
            <div style="background: rgba(255,255,255,0.08); 
                        border: 1px solid rgba(255,255,255,0.15); 
                        border-radius: 6px; padding: 15px;">
                <h4 style="color: white; font-size: 14px; font-weight: bold; 
                          margin-bottom: 12px; text-align: center;">Trial Results</h4>
                <!-- 5-column grid of metrics -->
                <div style="display: grid; grid-template-columns: 1fr 1fr; gap: 10px;">
                    
                    <!-- Peak Force Box -->
                    <div style="background: rgba(255,255,255,0.08); 
                                border: 1px solid rgba(255,255,255,0.15); 
                                border-radius: 4px; padding: 10px;">
                        <div style="font-size: 11px; color: rgba(255,255,255,0.7);">
                            Peak Force
                        </div>
                        <div style="font-size: 16px; font-weight: bold; 
                                   color: #27ae60; margin-top: 2px;">
                            ${results?.peak_force || '0.00'} N
                        </div>
                        <div style="font-size: 10px; color: rgba(255,255,255,0.5);">
                            (${(parseFloat(results?.peak_force || 0) / 9.81).toFixed(2)} kg)
                        </div>
                    </div>
                    <!-- Avg Force Box -->
                    <div style="background: rgba(255,255,255,0.08); 
                                border: 1px solid rgba(255,255,255,0.15); 
                                border-radius: 4px; padding: 10px;">
                        <div style="font-size: 11px; color: rgba(255,255,255,0.7);">
                            Avg Force
                        </div>
                        <div style="font-size: 16px; font-weight: bold; 
                                   color: #3498db; margin-top: 2px;">
                            ${results?.avg_force || '0.00'} N
                        </div>
                        <div style="font-size: 10px; color: rgba(255,255,255,0.5);">
                            (${(parseFloat(results?.avg_force || 0) / 9.81).toFixed(2)} kg)
                        </div>
                    </div>
                    <!-- Time to Peak Box -->
                    <div style="background: rgba(255,255,255,0.08); 
                                border: 1px solid rgba(255,255,255,0.15); 
                                border-radius: 4px; padding: 10px;">
                        <div style="font-size: 11px; color: rgba(255,255,255,0.7);">
                            Time to Peak
                        </div>
                        <div style="font-size: 16px; font-weight: bold; 
                                   color: #f39c12; margin-top: 2px;">
                            ${results?.time_to_peak || '0.00'} s
                        </div>
                    </div>
                    <!-- Early RFD Box -->
                    <div style="background: rgba(255,255,255,0.08); 
                                border: 1px solid rgba(255,255,255,0.15); 
                                border-radius: 4px; padding: 10px;">
                        <div style="font-size: 11px; color: rgba(255,255,255,0.7);">
                            Early RFD
                        </div>
                        <div style="font-size: 16px; font-weight: bold; 
                                   color: #3498db; margin-top: 2px;">
                            ${results?.early_rfd || '0'} N/s
                        </div>
                    </div>
                </div>
            </div>
        </div>
    `;
}
```

**Result on Screen:**
```
LEFT SIDE PANEL (After 5-second trial)
────────────────────────────────────────────────────
[Graph Loading...]
OR (when plot ready):
┌────────────────────────────────────────────────┐
│ [Force vs Time Graph - Matplotlib PNG]         │
│  - Red curve showing force progression         │
│  - Green circle at peak (125.5N @ 2.145s)     │
│  - Orange dashed line at avg (95.23N)          │
│  - Blue segment for RFD window (50-100ms)     │
│                                                │
│  Peak Force    Avg Force                      │
│  125.50 N      95.23 N                        │
│  (12.8 kg)     (9.7 kg)                       │
│                                                │
│  Time to Peak  Early RFD                      │
│  2.145 s       450 N/s                        │
└────────────────────────────────────────────────┘

[Start Trial] (re-enabled) [Reset]
```

### **Phase 6: User Takes Trial of Other limb (Right Side)**
#### **Prerequisites:**
- ✓ Left side trial completed and results displayed
- ✓ Graph generated and shown
- ✓ Metrics calculated (peak force, RFD, etc.)
- ✓ Still on Exercise Testing Panel (same exercise)
- ✓ Drag mode still enabled from before
#### **Frontend: User Clicks "Start Trial" for Right Side (Line ~1548)**

**This is essentially the same as Phase 5, but for the right side:**

```javascript
async function startTrial(side = 'right') {  // Now side = 'right'
    // Steps 1-11: IDENTICAL to left side trial
    // ├─ Validate connection
    // ├─ Disable button ("Recording...")
    // ├─ Show countdown (5s → 0s)
    // ├─ Progress bar animates
    // ├─ Wait for backend response (5 second blocking)
    // ├─ Receive results
    // └─ Display metrics immediately
    
    // Display shows:
    // LEFT SIDE:                      RIGHT SIDE:
    // Peak: 125.50 N                  Fz: 0.00 N
    // Avg: 95.23 N                    J4: 0.0°
    // [Graph already displayed]       [Start Trial] (about to be clicked)
}
```

#### **Backend: Recording Right Side (Same as Left)**
**POST /api/trial/start (called again with side='right'):**
```python
@app.route('/api/trial/start', methods=['POST'])
def start_trial():
    data = request.json
    # This time: side='right' instead of 'left'
    
    # Steps identical to Phase 5:
    # 1. Validate connection ✓
    # 2. Call record_trial(5.0) → Records for 5 seconds
    # 3. Collects 625 samples at 125 Hz
    # 4. Applies baseline subtraction
    # 5. Computes metrics:
    #    - Peak force: 118.75 N (right might be stronger/weaker)
    #    - Avg force: 88.45 N
    #    - RFD: 425 N/s
    #    - Time to Peak: 2.301 s
    # 6. Return results dict
    # 7. Start async plot generation
    
    return jsonify({
        'success': True,
        'results': {
            'peak_force': 118.75,
            'avg_force': 88.45,
            'start_force': 0.45,
            'time_to_peak': 2.301,
            'early_rfd': 425,
            'times': [0.0, 0.008, 0.016, ..., 4.992],
            'forces': [0.0, 0.15, 1.2, ..., 118.75, ...]
        }
    })
```

#### **Frontend Response & Display**
**Timeline for Right Side (Same as Left):**
```
Time  X.Xs: User clicks "Start Trial" for RIGHT side
            ├─ Progress bar animates (0% → 100% over 5s)
            ├─ Backend records 625 samples
            └─ Metrics returned immediately

Time (X+5)s: Right side results appear
            ├─ Metrics display:
            │  ├─ Peak: 118.75 N
            │  ├─ Avg: 88.45 N
            │  ├─ Time to Peak: 2.301 s
            │  └─ Early RFD: 425 N/s
            └─ Plot generation starts in background

Time (X+6)s: Graph appears when ready
            └─ Displays force curve for right side

SCREEN AFTER RIGHT SIDE COMPLETE:
────────────────────────────────────────────────────────────
LEFT SIDE PANEL              RIGHT SIDE PANEL
─────────────────────────────────────────────────────────
Peak: 125.50 N               Peak: 118.75 N
Avg: 95.23 N                 Avg: 88.45 N
Time to Peak: 2.145 s        Time to Peak: 2.301 s
Early RFD: 450 N/s           Early RFD: 425 N/s

[Graph - Force vs Time]      [Graph - Force vs Time]
(Both graphs displayed        (Both graphs displayed
 side by side)                 side by side)

[Start Trial] [Reset]         [Start Trial] [Reset]
────────────────────────────────────────────────────────────
[💾 Save Both Sides to CSV]  [⬅️ Back to Dashboard]
```

**Global Variable State After Right Trial:**
```javascript
trialResults = {
    left: {
        peak_force: 125.50,
        avg_force: 95.23,
        time_to_peak: 2.145,
        early_rfd: 450,
        forces: [...625 samples...],
        times: [...625 timestamps...],
        plot_image_base64: "iVBORw0KGgo..." // PNG data
    },
    right: {
        peak_force: 118.75,
        avg_force: 88.45,
        time_to_peak: 2.301,
        early_rfd: 425,
        forces: [...625 samples...],
        times: [...625 timestamps...],
        plot_image_base64: "iVBORw0KGgo..." // PNG data
    }
}
```

### **Phase 7: User Clicks "Save Both Sides to CSV"**
#### **Frontend: Save Button Handler (Line ~1850)**
**HTML Structure:**
```html
<button id="saveBothBtn" 
        class="btn btn-success save-both-btn"
        onclick="saveBothSidesToCSV()">
    💾 Save Both Sides to CSV
</button>
```
**JavaScript Function:**
```javascript
async function saveBothSidesToCSV() {
    // Step 1: Validate both trials completed
    if (!trialResults.left || !trialResults.right) {
        showNotification(
            'Error: Both sides must be tested before saving', 
            'error'
        );
        return;
    }
    // Step 2: Validate patient data entered
    const patientName = document.getElementById('patientName').value.trim();
    if (!patientName) {
        showNotification('Error: Patient name required', 'error');
        return;
    }
    // Step 3: Show loading state
    const btn = document.getElementById('saveBothBtn');
    const originalText = btn.textContent;
    btn.disabled = true;
    btn.textContent = 'Saving...';
    // Step 4: Prepare data payload
    const saveData = {
        // Patient information
        patient_name: patientName,
        age: document.getElementById('patientAge').value || '',
        gender: document.getElementById('patientGender').value || '',
        diagnosis: document.getElementById('patientDiagnosis').value || '',
        // Exercise info
        exercise_name: currentExerciseName,  // "Shoulder Flexion"
        // Left side results
        left_side: {
            peak_force: trialResults.left.peak_force,
            avg_force: trialResults.left.avg_force,
            time_to_peak: trialResults.left.time_to_peak,
            early_rfd: trialResults.left.early_rfd
        },
        // Right side results
        right_side: {
            peak_force: trialResults.right.peak_force,
            avg_force: trialResults.right.avg_force,
            time_to_peak: trialResults.right.time_to_peak,
            early_rfd: trialResults.right.early_rfd
        }
    };
    
    // Step 5: Send to backend
    try {
        const response = await fetch(`${API_BASE}/api/trial/save`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify(saveData)
        });
        const data = await response.json();
        if (data.success) {
            // SUCCESS
            showNotification(
                '✓ Both sides saved to CSV successfully!', 
                'success'
            );
            // Step 6: Re-enable button
            btn.disabled = false;
            btn.textContent = originalText;
            // Step 7: Enable back button
            document.getElementById('backBtn').disabled = false;
        } else {
            // FAILURE
            showNotification(
                'Failed to save: ' + (data.error || 'Unknown error'), 
                'error'
            );
            btn.disabled = false;
            btn.textContent = originalText;
        }
    } catch (error) {
        showNotification('Connection error: ' + error.message, 'error');
        btn.disabled = false;
        btn.textContent = originalText;
    }
}
```

#### **Backend: POST /api/trial/save (Line ~2300)**
**Flask Route:**
```python
@app.route('/api/trial/save', methods=['POST'])
def save_trial_data():
    """Save trial results for both sides to CSV"""
    data = request.json
    # Step 1: Extract all fields
    patient_name = data.get('patient_name', '').strip()
    age = data.get('age', '')
    gender = data.get('gender', '')
    diagnosis = data.get('diagnosis', '')
    exercise_name = data.get('exercise_name', '')
    left_side = data.get('left_side', {})
    right_side = data.get('right_side', {})
    # Step 2: Validate required fields
    if not patient_name or not exercise_name:
        return jsonify({
            'success': False,
            'error': 'Patient name and exercise required'
        })
    # Step 3: Get current timestamp
    from datetime import datetime
    timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    # Step 4: Create two CSV rows (one for left, one for right)
    csv_file = '/home/um/fairino-python-sdk-main/linux/RIMT/results.csv'
    # Check if file exists to determine if headers needed
    import os
    file_exists = os.path.isfile(csv_file)
    # CSV columns:
    # Timestamp, Exercise, Side, Name, Age, Gender, Diagnosis, Mode, 
    # Peak_Force_N, Average_Force_N, Start_Force_N, Time_to_Peak_s, Early_RFD_N_s
    try:
        import csv
        with open(csv_file, 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            # Step 5a: Write header if file is new
            if not file_exists:
                writer.writerow([
                    'Timestamp', 'Exercise', 'Side', 'Name', 'Age', 'Gender', 
                    'Diagnosis', 'Mode', 'Peak_Force_N', 'Average_Force_N', 
                    'Start_Force_N', 'Time_to_Peak_s', 'Early_RFD_N_s'
                ])
            # Step 5b: Write LEFT SIDE row
            writer.writerow([
                timestamp,
                exercise_name,
                'Left',                                    # Side
                patient_name,
                age,
                gender,
                diagnosis,
                'Isometric',                               # Mode (always isometric in RIMT)
                left_side.get('peak_force', 0),
                left_side.get('avg_force', 0),
                left_side.get('start_force', 0),
                left_side.get('time_to_peak', 0),
                left_side.get('early_rfd', 0)
            ])
            # Step 5c: Write RIGHT SIDE row
            writer.writerow([
                timestamp,
                exercise_name,
                'Right',                                   # Side
                patient_name,
                age,
                gender,
                diagnosis,
                'Isometric',
                right_side.get('peak_force', 0),
                right_side.get('avg_force', 0),
                right_side.get('start_force', 0),
                right_side.get('time_to_peak', 0),
                right_side.get('early_rfd', 0)
            ])
        # Step 6: Log success
        robot_controller.log_message(
            f"[CSV] Saved trial for {patient_name}: "
            f"Shoulder Flexion (L:{left_side.get('peak_force')}N, "
            f"R:{right_side.get('peak_force')}N)"
        )
        # Step 7: Return success
        return jsonify({
            'success': True,
            'message': 'Trial data saved to results.csv',
            'rows_added': 2,
            'file': csv_file
        })
    except Exception as e:
        robot_controller.log_message(f"[CSV] Error: {e}")
        return jsonify({
            'success': False,
            'error': str(e)
        })
```

#### **CSV File Structure After Save**
**results.csv (Example with 2 trials):**
```
Timestamp,Exercise,Side,Name,Age,Gender,Diagnosis,Mode,Peak_Force_N,Average_Force_N,Start_Force_N,Time_to_Peak_s,Early_RFD_N_s
2026-02-06 14:23:45,Shoulder Flexion,Left,Ashutosh babras,25,Male,Rotator Cuff Tear,Isometric,125.50,95.23,0.50,2.145,450
2026-02-06 14:23:52,Shoulder Flexion,Right,Ashutosh babras,25,Male,Rotator Cuff Tear,Isometric,118.75,88.45,0.45,2.301,425
2026-02-06 15:10:30,Hip Flexion,Left,Jane Smith,38,Female,Hip Osteoarthritis,Isometric,145.20,108.50,0.75,1.890,520
2026-02-06 15:10:37,Hip Flexion,Right,Jane Smith,38,Female,Hip Osteoarthritis,Isometric,142.80,105.30,0.70,1.950,510
```

#### **Frontend Response**
**After Save Success:**
```
Toast Notification (top-right):
"✓ Both sides saved to CSV successfully!"
Screen State After:
- "Save Both Sides to CSV" button: Re-enabled
- "Back to Dashboard" button: Now clickable
- Exercise Testing Panel: Still displayed
- User can:
  ├─ Click "Back to Dashboard" to select new exercise
  ├─ Click "Reset" to clear graphs and re-test
  └─ Click "Start Trial" again for same exercise
```

**Terminal Output (Backend):**
```
[14:23:52] [CSV] Saved trial for Ashutosh babras: Shoulder Flexion (L:125.50N, R:118.75N)
```

### **Phase 8: Progress Report - Patient History View**
#### **Prerequisites:**
- ✓ Trial data saved to results.csv
- ✓ One or more patients have completed sessions
- ✓ User clicks "Back to Dashboard"
- ✓ User is on main menu
#### **Frontend: Progress Report Button (Line ~850)**
**HTML Structure:**
```html
<div class="card">
    <h3 class="card-title">Reports & Analysis</h3>
    <button class="btn btn-info" onclick="openProgressReport()">
        📊 Progress Report
    </button>
    <button class="btn btn-secondary" onclick="exportData()">
        📥 Export to Excel
    </button>
</div>
```

**JavaScript: openProgressReport() (Line ~1900)**
```javascript
async function openProgressReport() {
    // Step 1: Hide main dashboard
    document.getElementById('mainDashboard').style.display = 'none';
    // Step 2: Show progress report section
    document.getElementById('progressReportPage').classList.add('active');
    // Step 3: Load list of all patients from CSV
    try {
        const response = await fetch(`${API_BASE}/api/patients/list`);
        const data = await response.json();
        // data.patients = ['Ashutosh babras', 'ARchana Makhija', 'Ninad saraf', ...]
        // Step 4: Display patient list in dropdown
        const patientSelect = document.getElementById('patientSearchSelect');
        patientSelect.innerHTML = '<option value="">-- Select Patient --</option>';
        data.patients.forEach(name => {
            const option = document.createElement('option');
            option.value = name;
            option.textContent = name;
            patientSelect.appendChild(option);
        });
        // Step 5: Also show search input for manual entry
        document.getElementById('patientSearchInput').placeholder = 
            'Or type patient name...';
    } catch (e) {
        console.error('Error loading patients:', e);
        showNotification('Failed to load patient list', 'error');
    }
}
```

**HTML Progress Report Page:**
```html
<div id="progressReportPage" class="page">
    <h2>Patient Progress Report</h2>
    <!-- Patient Search Section -->
    <div class="card">
        <h3>Search Patient</h3>
        <div class="form-group">
            <label>Select from list:</label>
            <select id="patientSearchSelect" onchange="loadPatientHistory(this.value)">
                <option value="">-- Select Patient --</option>
            </select>
        </div>
        <div class="form-group">
            <label>Or enter name:</label>
            <input id="patientSearchInput" 
                   type="text" 
                   placeholder="Type patient name..."
                   onchange="loadPatientHistory(this.value)">
        </div>
    </div>
    <!-- Results Display Section -->
    <div id="patientHistoryContainer" style="display: none;">
        <!-- Populated by loadPatientHistory() -->
    </div>
    <!-- Back Button -->
    <button class="btn btn-secondary" onclick="backToMainMenu()">
        ⬅️ Back to Dashboard
    </button>
</div>
```

#### **Frontend: Load Patient History (Line ~1950)**
```javascript
async function loadPatientHistory(patientName) {
    // Step 1: Validate input
    patientName = patientName.trim();
    if (!patientName) {
        showNotification('Please enter or select a patient name', 'error');
        return;
    }
    // Step 2: Show loading state
    showNotification(`Loading history for ${patientName}...`, 'info');
    // Step 3: Request patient history from backend
    try {
        const response = await fetch(
            `${API_BASE}/api/patient/history?name=${encodeURIComponent(patientName)}`
        );
        const data = await response.json();
        if (!data.success || !data.history || data.history.length === 0) {
            showNotification(`No records found for ${patientName}`, 'warning');
            return;
        }
        // Step 4: Build history table
        let html = `
            <div class="card">
                <h3>Patient History: ${patientName}</h3>
                
                <div style="overflow-x: auto;">
                    <table class="history-table">
                        <thead>
                            <tr>
                                <th>Date & Time</th>
                                <th>Exercise</th>
                                <th>Side</th>
                                <th>Peak Force (N)</th>
                                <th>Avg Force (N)</th>
                                <th>Time to Peak (s)</th>
                                <th>Early RFD (N/s)</th>
                                <th>Age</th>
                                <th>Gender</th>
                                <th>Diagnosis</th>
                            </tr>
                        </thead>
                        <tbody>
        `;
        // Step 5: Add row for each trial
        data.history.forEach((record, index) => {
            const rowClass = index % 2 === 0 ? 'even' : 'odd';
            const sideColor = record.Side === 'Left' ? '#3498db' : '#e74c3c';
            html += `
                <tr class="${rowClass}">
                    <td>${record.Timestamp}</td>
                    <td><strong>${record.Exercise}</strong></td>
                    <td>
                        <span style="background: ${sideColor}; 
                                    color: white; 
                                    padding: 3px 8px; 
                                    border-radius: 3px;">
                            ${record.Side}
                        </span>
                    </td>
                    <td style="color: #27ae60; font-weight: bold;">
                        ${parseFloat(record.Peak_Force_N).toFixed(2)}
                    </td>
                    <td>${parseFloat(record.Average_Force_N).toFixed(2)}</td>
                    <td>${parseFloat(record.Time_to_Peak_s).toFixed(3)}</td>
                    <td>${parseInt(record.Early_RFD_N_s)}</td>
                    <td>${record.Age || '-'}</td>
                    <td>${record.Gender || '-'}</td>
                    <td>${record.Diagnosis || '-'}</td>
                </tr>
            `;
        });
        html += `
                        </tbody>
                    </table>
                </div>
                
                <!-- Summary Statistics -->
                <div style="margin-top: 20px; padding: 15px; 
                           background: rgba(255,255,255,0.08); 
                           border-radius: 6px;">
                    <h4>Summary Statistics</h4>
        `;
        // Step 6: Calculate and display summary stats
        const exercises = {};
        data.history.forEach(record => {
            const exercise = record.Exercise;
            if (!exercises[exercise]) {
                exercises[exercise] = {
                    count: 0,
                    peak_forces: [],
                    avg_forces: [],
                    rfds: []
                };
            }
            exercises[exercise].count++;
            exercises[exercise].peak_forces.push(parseFloat(record.Peak_Force_N));
            exercises[exercise].avg_forces.push(parseFloat(record.Average_Force_N));
            exercises[exercise].rfds.push(parseInt(record.Early_RFD_N_s));
        });
        // Step 7: Display stats per exercise
        html += '<div style="display: grid; grid-template-columns: 1fr 1fr; gap: 15px;">';
        Object.entries(exercises).forEach(([exercise, stats]) => {
            const avgPeak = (stats.peak_forces.reduce((a,b) => a+b) / stats.count).toFixed(2);
            const avgRfd = (stats.rfds.reduce((a,b) => a+b) / stats.count).toFixed(0);
            
            html += `
                <div style="background: rgba(52, 152, 219, 0.1); 
                           border: 1px solid #3498db; 
                           border-radius: 6px; 
                           padding: 12px;">
                    <h5>${exercise}</h5>
                    <div style="font-size: 12px; color: rgba(255,255,255,0.7);">
                        <div>Tests: <strong>${stats.count}</strong></div>
                        <div>Avg Peak Force: <strong style="color: #27ae60;">${avgPeak}N</strong></div>
                        <div>Avg RFD: <strong>${avgRfd}N/s</strong></div>
                    </div>
                </div>
            `;
        });
        html += '</div></div>';
        // Step 8: Display complete history
        const container = document.getElementById('patientHistoryContainer');
        container.innerHTML = html;
        container.style.display = 'block';
        showNotification(
            `✓ Loaded ${data.history.length} records for ${patientName}`, 
            'success'
        );
    } catch (error) {
        showNotification('Error loading patient history: ' + error.message, 'error');
    }
}
```

#### **Backend: GET /api/patient/history (Line ~2350)**
**Flask Route:**
```python
@app.route('/api/patient/history', methods=['GET'])
def get_patient_history():
    """Retrieve all trial records for a specific patient"""
    # Step 1: Get patient name from query parameter
    patient_name = request.args.get('name', '').strip()
    if not patient_name:
        return jsonify({
            'success': False,
            'error': 'Patient name required'
        })
    # Step 2: Read CSV file
    csv_file = '/home/um/fairino-python-sdk-main/linux/RIMT/results.csv'
    try:
        import csv
        history = []
        # Step 3: Open and read CSV
        if os.path.isfile(csv_file):
            with open(csv_file, 'r') as f:
                reader = csv.DictReader(f)
                # Step 4: Filter rows matching patient name (case-insensitive)
                for row in reader:
                    if row['Name'].lower() == patient_name.lower():
                        history.append(row)
        # Step 5: Sort by timestamp (oldest first)
        history.sort(key=lambda x: x['Timestamp'])
        # Step 6: Return results
        if not history:
            return jsonify({
                'success': False,
                'error': f'No records found for patient: {patient_name}',
                'history': []
            })
        robot_controller.log_message(
            f"[HISTORY] Retrieved {len(history)} records for {patient_name}"
        )
        return jsonify({
            'success': True,
            'patient_name': patient_name,
            'history': history,
            'record_count': len(history)
        })
    except Exception as e:
        robot_controller.log_message(f"[HISTORY] Error: {e}")
        return jsonify({
            'success': False,
            'error': str(e)
        })
```

#### **Backend: GET /api/patients/list (Line ~2330)**
**Flask Route (for dropdown):**
```python
@app.route('/api/patients/list', methods=['GET'])
def get_patients_list():
    """Get unique list of all patients in CSV"""
    csv_file = '/home/um/fairino-python-sdk-main/linux/RIMT/results.csv'
    try:
        import csv
        patients = set()
        if os.path.isfile(csv_file):
            with open(csv_file, 'r') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    if row.get('Name'):
                        patients.add(row['Name'])
        # Sort alphabetically
        patients_list = sorted(list(patients))
        return jsonify({
            'success': True,
            'patients': patients_list,
            'count': len(patients_list)
        })
    except Exception as e:
        return jsonify({
            'success': False,
            'error': str(e),
            'patients': []
        })
```

#### **Progress Report Display Example**
**After user searches for "Ashutosh babras":**

```
┌──────────────────────────────────────────────────────────┐
│        Patient Progress Report: Ashutosh babras          │
├──────────────────────────────────────────────────────────┤
│                                                          │
│  Date & Time         │ Exercise      │ Side  │ Peak (N)  │
│  ─────────────────────────────────────────────────────── │
│  2026-02-06 14:23:45 │ Shoulder Flex │ Left  │  125.50   │
│  2026-02-06 14:23:52 │ Shoulder Flex │ Right │  118.75   │
│  2026-02-06 15:10:30 │ Hip Flexion   │ Left  │  145.20   │
│  2026-02-06 15:10:37 │ Hip Flexion   │ Right │  142.80   │
│  2026-02-07 09:45:12 │ Shoulder Flex │ Left  │  128.35   │
│  2026-02-07 09:45:19 │ Shoulder Flex │ Right │  121.50   │
│  2026-02-07 10:20:44 │ Knee Ext      │ Left  │  156.75   │
│  2026-02-07 10:20:51 │ Knee Ext      │ Right │  159.40   │
│                                                          │
│  Summary Statistics:                                     │
│  ┌────────────────────┬────────────────────┐             │
│  │ Shoulder Flexion   │ Hip Flexion        │             │
│  │ Tests: 4           │ Tests: 2           │             │
│  │ Avg Peak: 123.53N  │ Avg Peak: 144.00N  │             │
│  │ Avg RFD: 440 N/s   │ Avg RFD: 515 N/s   │             │
│  └────────────────────┴────────────────────┘             │
│  ┌────────────────────┐                                  │
│  │ Knee Extension     │                                  │
│  │ Tests: 2           │                                  │
│  │ Avg Peak: 158.08N  │                                  │
│  │ Avg RFD: 530 N/s   │                                  │
│  └────────────────────┘                                  │
│                                                          │
│  [⬅️ Back to Dashboard]                                  │
└──────────────────────────────────────────────────────────┘
```

#### **Key Features Demonstrated:**
1. **Complete Patient History:**
   - All exercises performed
   - All dates and times
   - Results for each side (left/right)
   - Patient demographics (age, gender, diagnosis)
2. **Metrics Displayed:**
   - Peak Force (with color highlighting)
   - Average Force
   - Time to Peak
   - Early RFD (Rate of Force Development)
3. **Summary Statistics:**
   - Tests per exercise
   - Average peak force per exercise
   - Average RFD per exercise
   - Trend visualization
4. **Progress Tracking:**
   - See improvement over multiple days
   - Compare left vs right side performance
   - Track rehabilitation progress


### **Summary: User Workflow Timeline**
```
TIMELINE:
─────────────────────────────────────────────────────────────────────

Time  0.0s: User opens browser to http://localhost:5000
            └─ Main Dashboard appears with "Connecting..." status

Time  1.0s: Robot auto-connection occurs
            ├─ Backend tries: robot = Robot.RPC('192.168.58.2')
            ├─ Success: Status changes to "Connected" (green)
            └─ "Enable Drag Mode" button becomes enabled

Time  1.5s: User enters patient information
            ├─ Name: Ashutosh babras
            ├─ Age: 25
            ├─ Gender: Male
            ├─ Diagnosis: Rotator Cuff Tear
            └─ Each field triggers savePatientData() → POST /api/patient

Time  2.5s: User clicks "Shoulder Flexion" button
            ├─ Dashboard hides
            ├─ Exercise Testing Panel shows
            ├─ Exercise image displays
            ├─ Live polling starts (200ms updates)
            └─ Status updates to show exercise name

Time  3.0s: User clicks "Enable Drag Mode"
            ├─ POST /api/drag_mode/toggle
            ├─ Backend:
            │  ├─ Initializes F/T sensor (0.1s)
            │  ├─ Calibrates baseline (1.0s) ← 100 samples
            │  ├─ Enables free-drive: DragTeachSwitch(1)
            │  └─ Starts monitoring thread (8ms loop)
            │
            ├─ Button changes to "Disable Drag Mode" (red)
            └─ Status: "Active (Free-Drive Enabled)"

Time  4.2s: USER IS READY TO START TRIAL
            ├─ Patient info ✓
            ├─ Exercise selected ✓
            ├─ Drag mode enabled ✓
            ├─ Baseline calibrated ✓
            └─ Monitoring thread running ✓

Time  4.5s: User clicks "Start Trial" (Left side)
            ├─ Progress bar animates (0% → 100% over 5s)
            ├─ Button shows countdown: "4s left...", "3s left..."
            │
            ├─ Backend recording loop (5 seconds):
            │  ├─ Read F/T sensor every 8ms (125 Hz)
            │  ├─ Collect 625 samples total (5s ÷ 0.008s)
            │  ├─ Apply baseline subtraction
            │  ├─ Extract Fz values
            │  └─ Compute metrics
            │
            └─ Async plot generation starts in background

Time  9.5s: Recording complete
            ├─ Results returned immediately:
            │  ├─ Peak: 125.50 N
            │  ├─ Avg: 95.23 N
            │  ├─ RFD: 450 N/s
            │  └─ Time to Peak: 2.145 s
            │
            ├─ Metrics display on screen
            ├─ Graph still generating (backend thread)
            └─ Frontend polls /api/trial/plot-status every 1 second

Time 11.0s: Plot generation completes
            ├─ Frontend receives base64 image
            ├─ Graph displays with annotations
            └─ Button "Start Trial" re-enabled

Time 12.0s: User can click "Start Trial" again (Right side)
            └─ Repeats same 5-second recording flow

Time 17.0s: User clicks "💾 Save Both Sides to CSV"
            ├─ POST /api/trial/save
            ├─ Append 2 rows to results.csv:
            │  ├─ Left side data
            │  └─ Right side data
            └─ Notification: "Both sides saved successfully!"

Time 18.0s: User can:
            ├─ Click "Back to Dashboard" → Return to main menu
            ├─ Click "Start Trial" (again) → Re-test same exercise
            ├─ Click "Reset" → Clear graphs and re-test
            └─ Click "Select New Exercise" → Choose different exercise

Time 19.0s: USER COMPLETES REHABILITATION SESSION
            ├─ Returns to main dashboard
            ├─ Patient data saved in CSV
            ├─ CSV file updated: results.csv
            └─ 2 rows added (left & right for same exercise)

OPTIONAL: USER VIEWS PROGRESS REPORT (Time ~20s+):

Time 20.0s: User clicks "📊 Progress Report"
            ├─ Progress Report Page opens
            ├─ Patient dropdown loads all unique patient names
            └─ User can select or type patient name

Time 21.0s: User selects "Ashutosh babras"
            ├─ Backend queries results.csv for matching rows
            ├─ Retrieves ALL historical data for Ashutosh babras
            ├─ Calculates summary statistics per exercise
            └─ Frontend builds comprehensive history table

Time 22.0s: Progress Report displays
            ├─ Complete history table showing:
            │  ├─ Date & Time of each test
            │  ├─ Exercise name
            │  ├─ Side (Left or Right)
            │  ├─ Peak Force, Avg Force
            │  ├─ Time to Peak, RFD
            │  ├─ Patient age, gender, diagnosis
            │  └─ ALL records from first test to current date
            └─ Summary statistics below table:
               ├─ Tests per exercise (e.g., Shoulder Flexion: 6 tests)
               ├─ Average peak force per exercise
               ├─ Average RFD per exercise
               └─ Progress trend visualization

EXAMPLE PROGRESS REPORT DATA FOR "ASHUTOSH BABRAS":
────────────────────────────────────────────────────────────────────

Exercise               Tests    Avg Peak    Avg RFD
─────────────────────────────────────────────────────
Shoulder Flexion      6        125.40 N    445 N/s
Hip Flexion           4        143.75 N    510 N/s
Knee Extension        2        157.50 N    530 N/s

Date/Time             Exercise           Side  Peak(N)  RFD(N/s)
─────────────────────────────────────────────────────────────────
2026-02-06 14:23:45   Shoulder Flexion   Left  125.50   450
2026-02-06 14:23:52   Shoulder Flexion   Right 118.75   425
2026-02-06 15:10:30   Hip Flexion        Left  145.20   520
2026-02-06 15:10:37   Hip Flexion        Right 142.80   510
2026-02-07 09:45:12   Shoulder Flexion   Left  128.35   460
2026-02-07 09:45:19   Shoulder Flexion   Right 121.50   430
2026-02-07 10:20:44   Knee Extension     Left  156.75   540
2026-02-07 10:20:51   Knee Extension     Right 159.40   520
2026-02-08 13:30:10   Shoulder Flexion   Left  122.10   440
2026-02-08 13:30:17   Shoulder Flexion   Right 119.20   420
2026-02-08 14:15:35   Hip Flexion        Left  142.30   500
2026-02-08 14:15:42   Hip Flexion        Right 143.20   515

✓ Progress tracked across MULTIPLE DAYS with COMPLETE HISTORY
```

## Configuration & Constants
| Constant | Value | Purpose |
|----------|-------|---------|
| `ROBOT_IP` | `192.168.58.2` | RPC connection address |
| `TRIAL_DURATION` | `5.0` seconds | Default recording length |
| `SAMPLING_RATE` | `125 Hz` (8ms) | F/T sensor poll interval |
| `FORCE_THRESHOLD` | `0.5 N` | Minimum valid force |
| `BASELINE_SAMPLES` | `100` | Calibration measurements |
| `PORT` | `5000` | Flask server port |
| `HOST` | `0.0.0.0` | Listen on all interfaces |

## Installation & Setup
### **Prerequisites**

- **Python 3.7+** with pip
- **FaiRino Robot SDK** (Robot.py module, available in `/home/um/fairino-python-sdk-main/linux/fairino/`)
- **Robot Hardware:** FaiRino 6-axis industrial robot arm with 6-axis F/T sensor
- **Network:** Robot accessible at `192.168.58.2` (or update IP in line ~2010)
- **Modern Web Browser:** Chrome, Firefox, Safari, Edge (ES6 JavaScript support)

### **Python Dependencies**

Install required packages:

```bash
pip install flask flask-cors matplotlib
```

### **File Structure Setup**

```bash
# Ensure images directory exists
mkdir -p /home/um/fairino-python-sdk-main/images

# Copy exercise reference images into directory
# Supported formats: .jpg, .jpeg, .png

# Working directory for data files
cd /home/um/fairino-python-sdk-main
```

### **Running the Application**

```bash
# Navigate to RIMT folder
cd /home/um/fairino-python-sdk-main/linux/RIMT

# Start server
python Final_RIMT_Code.py

# Expected output:
# ======================================================================
#   ROBOT REHABILITATION CONTROL SYSTEM - WEB SERVER
# ======================================================================
#
#   🚀 Server starting...
#   🌐 URL: http://localhost:5000
#   🤖 Auto-connecting to robot at 192.168.58.2
#
#   Press Ctrl+C to stop the server
# ======================================================================
```

**Browser Auto-Opens:** After 1.5 seconds, Firefox/default browser opens to `http://localhost:5000`

**Robot Auto-Connect:** After 2 seconds, backend attempts connection (may show "Connecting..." if unreachable)

### **Troubleshooting Startup**

| Issue | Solution |
|-------|----------|
| Port 5000 already in use | Change `port=5000` in line ~2427 to unused port (e.g., 5001) |
| Robot not connecting | Verify IP address matches (default: 192.168.58.2). Update line ~1980 if different. |
| Matplotlib not found | `pip install matplotlib` |
| Images not loading | Place .jpg/.jpeg/.png files in `/home/um/fairino-python-sdk-main/images/` |
| JavaScript errors in browser console | Check browser DevTools (F12) - ensure Flask server is running |
| Module Robot not found | Ensure `Robot.py` is in `/home/um/fairino-python-sdk-main/linux/fairino/` |

---

## Usage Guide

### **Basic Workflow: Isometric Muscle Testing Session**

#### **1. Start Application**
```bash
python Final_RIMT_Code.py
```
Wait for browser to open and robot status to update.

#### **2. Patient Registration**
On Main Dashboard:
- Enter: Full Name, Age, Gender, Diagnosis
- Data saves to backend automatically (onChange event)
- Data persists during session (lost on server restart)

#### **3. Enable Drag Mode** (if manual positioning needed)
- Click "Enable Drag Mode" button (blue, top-right)
- System initializes F/T sensor and calibrates baseline
- Button changes to "Disable Drag Mode" (red)
- Therapist can now manually move robot with low resistance

#### **4. Select Exercise**
- Click exercise button (e.g., "Shoulder Flexion")
- Exercise Testing Panel opens
- Reference image displays on left
- Right side shows Left & Right panels for bilateral testing

#### **5. Record Left Side Trial**
- Click "Start Trial" on Left Side panel
- 5-second recording begins
- Progress bar animates
- Button shows countdown: "4s left...", "3s left...", etc.
- Force graph appears when complete
- Metrics display: Peak (kg), Avg (kg), Start (kg), RFD (N/s)

#### **6. Record Right Side Trial**
- Repeat step 5 for Right Side panel
- Compare left vs. right metrics on screen

#### **7. Save Results**
- Click "💾 Save Both Sides to CSV" button
- Confirmation message: "Both sides saved to CSV successfully!"
- Data appended to `results.csv` in working directory

#### **8. View Progress Report**
- Click "Progress Report" on Dashboard
- Enter patient name and search
- View historical comparison charts
- Analyze trends over time

#### **9. Exit Session**
- Press Ctrl+C in terminal to stop server
- Graceful shutdown: "Shutting down gracefully..."

---

### **Advanced Features (with Technical Details)**

#### **Bilateral Comparison**
- Record both sides in same session
- Side-by-side graphs visible
- Save both simultaneously
- Compare metrics (asymmetry detection)

#### **Drag Mode (Free-Drive) Positioning**
- Enable before exercise to manually position limb
- Robot enters low-resistance mode (therapist can move freely)
- Baseline calibration removes tool weight from readings
- Disable before trial recording

#### **Progress Tracking**
- Search patient by name
- View all exercises for patient
- Grouped by exercise type
- Animated bar charts showing peak force trends
- Export-ready data (CSV can be opened in Excel)

---

## Sensor Specifications & Calibration

### **F/T Sensor Hardware**

Typical FaiRino robotic system uses 6-axis F/T sensor (e.g., ATI Industrial Automation Mini45):

| Axis | Measurement | Range | Resolution |
|------|-------------|-------|------------|
| Fx | Side force (X) | ±1000 N | 0.25 N |
| Fy | Side force (Y) | ±1000 N | 0.25 N |
| Fz | Vertical force (Z) | ±2500 N | 0.6 N |
| Tx | Roll torque | ±50 N·m | 0.05 N·m |
| Ty | Pitch torque | ±50 N·m | 0.05 N·m |
| Tz | Yaw torque | ±50 N·m | 0.05 N·m |

### **Calibration Procedure**

**Manual Calibration (Before Session):**

1. Place robot end-effector in neutral position
2. Ensure no external load on sensor
3. Click "Enable Drag Mode"
4. System automatically:
   - Sends `FT_SetConfig(24, 0)` → Configure sensor
   - Sends `FT_Activate(0)` → Activate channel 0
   - Sends `FT_Activate(1)` → Activate channel 1
   - Sends `FT_SetZero(0)` → Zero channel 0
   - Sends `FT_SetZero(1)` → Zero channel 1
   - Collects 100 samples @ 100 Hz → Averages → Stores as `baseline_forces`

**Why Calibration Matters:**
- Removes gravitational offset (tool weight)
- Removes sensor thermal drift
- Ensures accurate force measurements
- Baseline subtraction: `corrected = raw - baseline`

**Recalibration Scenarios:**
- Before each patient session
- If tool/payload changes
- If sensor appears to drift (readings not returning to zero when unloaded)
- After approximately 1 hour of continuous use (thermal effects)

---

## Data Management

### **CSV File Structure (results.csv)**

Located in working directory: `/home/um/fairino-python-sdk-main/results.csv`

**Columns:**
```
Timestamp           | ISO format: YYYY-MM-DD HH:MM:SS
Exercise            | String: "Shoulder Flexion", "Knee Extension", etc.
Side                | String: "Left" or "Right"
Name                | Patient full name (from form)
Age                 | Patient age (from form)
Gender              | "Male", "Female", or "Other"
Diagnosis           | Clinical diagnosis string
Mode                | String: "Isometric" (hardcoded)
Peak_Force_N        | Float: Maximum force (Newtons)
Average_Force_N     | Float: Mean force of valid samples (Newtons)
Start_Force_N       | Float: First sample force (Newtons)
Time_to_Peak_s      | Float: Time when peak occurs (seconds)
Early_RFD_N_s       | Integer: RFD slope 50-100ms window (N/s)
```

**Example Row:**
```
2025-02-06 14:23:15 | Shoulder Flexion | Left | Ashutosh babras | 25 | Male | Rotator Cuff | Isometric | 125.50 | 95.23 | 0.50 | 2.145 | 450
```

**Excel Analysis:**
1. Open results.csv in Excel/LibreOffice
2. Use pivot tables to summarize by exercise, side, patient
3. Create charts: Peak Force trends over time
4. Calculate asymmetry: |Peak_Left - Peak_Right| / max(Peak_Left, Peak_Right)

### **Image Storage**

Directory: `/home/um/fairino-python-sdk-main/images/`

**Adding Exercise Images:**
1. Create or download anatomical reference image
2. Save as JPG/PNG (recommended: 400x600 pixels)
3. Name: `Exercise_name.jpg` (match button text but with underscores)
4. Example: Shoulder Flexion → `Shoulder_flexion.jpeg`
5. API auto-discovers with fallback extension matching

---

## Troubleshooting & Error Recovery

### **Connection Issues**

**Robot Connection Fails**
- Check IP address: 192.168.58.2 (update in line ~1980 if different)
- Verify robot power is on and network is connected
- Test with: `ping 192.168.58.2` from terminal
- Check firewall rules (port RPC may need whitelisting)

**F/T Sensor Not Reading**
- Check sensor power supply
- Verify sensor cable connections
- Recalibrate: Click "Enable Drag Mode" again
- Check terminal output for error messages starting with "✗"

### **Data Recording Issues**

**Trial Returns No Data**
- Ensure robot is connected (`connected: true` in status)
- Check force readings in `/api/live` endpoint via browser
- Verify baseline calibration (should show ✓ in console)
- If Fz always shows 0, check sensor orientation (negation at line ~1925)

**CSV File Grows Large**
- Expected: ~200 bytes per trial
- After 100 patients × 10 sessions × 2 sides = ~40 KB
- Archive old records periodically:
  ```bash
  cp results.csv results_backup_$(date +%Y%m%d).csv
  head -1 results.csv > results_new.csv  # Keep header
  mv results_new.csv results.csv
  ```

### **Browser/UI Issues**

**Blank Page After Opening**
- Press F12 (DevTools) → Console tab
- Check for JavaScript errors
- Verify server is running (should see Flask debug output in terminal)

**Graph Not Displaying**
- Ensure matplotlib installed: `pip install matplotlib`
- Check browser console for image loading errors
- Verify `/api/trial/plot-status` returns base64 string (not null)

**Notifications Not Showing**
- Check CSS: `.notification` should be visible in DOM
- Verify JavaScript `showNotification()` function is not overridden

### **Performance Issues**

**Lag in Live Updates**
- Normal: 200ms refresh (5 Hz frontend polling)
- If >500ms delay: Check system CPU usage
- May need to optimize sensor polling (reduce from 8ms to 10ms)

**Memory Leak (Server Gets Slow Over Time)**
- Matplotlib figure objects not being closed properly
- Fix: Ensure `plt.close(fig)` called after saving (line ~2077)
- Server restart clears memory

---

## Advanced Configuration

### **Changing Robot IP Address**

Line ~1980 in Final_RIMT_Code.py:
```python
self.robot = Robot.RPC('192.168.58.2')  # Change IP here
```

Update to your robot's IP:
```python
self.robot = Robot.RPC('192.168.100.10')  # Example new IP
```

Then restart server.

### **Changing Port Number**

Line ~2427:
```python
app.run(debug=False, host='0.0.0.0', port=5000, ...)
```

Change to:
```python
app.run(debug=False, host='0.0.0.0', port=8080, ...)  # New port
```

Access at: `http://localhost:8080`

### **Modifying Trial Duration**

Default: 5 seconds (line ~1706 in `record_trial()`)

To change to 10 seconds, update HTML trigger:
```javascript
// Frontend: Line ~1320
body: JSON.stringify({ 
    exercise_name: currentExerciseName,
    duration: 10.0,  // Changed from 5.0
    side: side
})
```

And backend default:
```python
def record_trial(self, duration=10.0):  # Changed from 5.0
```

### **RFD Window (Advanced Biomechanics)**

Current: 50-100 ms post-onset (line ~1748)

To change to 0-100 ms window (total RFD):
```python
idx_50ms = onset_idx + int(0.00 / dt)   # Changed from 0.05
idx_100ms = onset_idx + int(0.10 / dt)
```

---

## Startup Sequence (Detailed)

```
1. User: python Final_RIMT_Code.py
   └─ Loads Final_RIMT_Code.py script
   
2. Global: Create RobotController instance
   └─ robot = None
   └─ patient_data = {}
   └─ monitoring_active = False
   
3. if __name__ == '__main__':
   ├─ Print startup banner
   │
   ├─ Thread 1 (browser_thread):
   │  └─ sleep(1.5) → webbrowser.open('http://localhost:5000')
   │
   ├─ Thread 2 (robot_thread):
   │  └─ sleep(2.0) → robot_controller.connect_robot()
   │     ├─ Try: robot = Robot.RPC('192.168.58.2')
   │     └─ Log "✓ Connected" or "✗ Connection error"
   │
   └─ Main Thread (Flask):
      └─ app.run(host='0.0.0.0', port=5000, threaded=True)
         ├─ Bind to 0.0.0.0:5000
         ├─ Start WSGI server
         └─ Listen for HTTP requests

4. User opens browser (auto-opened after 1.5s)
   ├─ GET / → Returns HTML_TEMPLATE (main dashboard)
   ├─ JS: window.addEventListener('load', ...)
   │  ├─ loadPatientData() → GET /api/patient
   │  ├─ updateStatus() → GET /api/status
   │  └─ setInterval(updateStatus, 1000) → Poll every second
   │
   └─ Display main dashboard

5. User clicks "Enable Drag Mode"
   ├─ POST /api/drag_mode/toggle
   ├─ Backend: enable_drag_mode()
   │  ├─ connect_robot() (if not already)
   │  ├─ init_ft_sensor()
   │  ├─ calibrate_baseline_forces() [sleeps 1 second]
   │  ├─ DragTeachSwitch(1)
   │  ├─ start_ft_monitoring() [spawns Thread 3]
   │  └─ Set drag_mode_active = True
   │
   ├─ Thread 3 (FT monitoring):
   │  └─ Loop while monitoring_active:
   │     ├─ FT_GetForceTorqueRCS() every 8ms
   │     ├─ Apply baseline subtraction
   │     ├─ Update current_forces
   │     └─ Print to terminal
   │
   └─ Response: {success: true, active: true}

6. User selects exercise (e.g., "Shoulder Flexion")
   ├─ startExercise() JS function
   ├─ POST /api/exercise/start
   ├─ Switch to Exercise Testing Panel
   └─ Begin polling /api/live every 200ms for live data

7. User clicks "Start Trial" (Left side)
   ├─ startTrial('left')
   ├─ POST /api/trial/start {exercise_name, duration: 5.0, side: 'left'}
   ├─ Backend: record_trial(5.0)
   │  ├─ Loop: for 5.0 seconds
   │  │  ├─ FT_GetForceTorqueRCS() every 8ms
   │  │  ├─ Apply baseline, extract Fz
   │  │  ├─ Append {time, force} to samples
   │  │  └─ sleep(0.008)
   │  │
   │  ├─ Compute metrics: peak, avg, RFD, time_to_peak
   │  ├─ Return immediately with {success, results{}}
   │  │
   │  └─ Spawn Thread 4 (async plot):
   │     ├─ plot_results_in_thread(results)
   │     ├─ Create matplotlib figure
   │     ├─ Plot force vs time
   │     ├─ Annotate peak/avg/RFD
   │     ├─ savefig() to BytesIO buffer
   │     ├─ base64.encode() to string
   │     ├─ Store in last_plot_image
   │     └─ (Thread exits)
   │
   ├─ Frontend receives response (plot_image_base64: null)
   ├─ Display metrics table immediately
   ├─ Start polling /api/trial/plot-status every 1000ms
   │  └─ When ready, display base64 image: <img src="data:image/png;base64,...">
   │
   └─ Show "Start Trial" button re-enabled

8. User clicks "💾 Save Both Sides"
   ├─ saveToCSV('both')
   ├─ POST /api/trial/save
   │  └─ body: {exercise_name, side: 'both', results_left, results_right}
   │
   ├─ Backend: save_trial_to_csv(..., side='left', results=results_left)
   │  ├─ Open results.csv in append mode
   │  ├─ Write row: [timestamp, exercise, 'Left', patient_data, metrics]
   │  └─ Close file
   │
   ├─ Backend: save_trial_to_csv(..., side='right', results=results_right)
   │  ├─ Open results.csv in append mode
   │  ├─ Write row: [timestamp, exercise, 'Right', patient_data, metrics]
   │  └─ Close file
   │
   └─ Response: {success: true, message: "...saved"}

9. User presses Ctrl+C
   └─ KeyboardInterrupt caught (line ~2412)
      ├─ Print "Shutting down gracefully..."
      ├─ robot_controller.stop_monitoring()
      │  ├─ monitoring_active = False (Thread 3 exits)
      │  ├─ disable_drag_mode() → DragTeachSwitch(0)
      │  └─ Print "✓ Server stopped."
      │
      └─ Flask server shuts down
```

---



The system interfaces with the FaiRino robot SDK:

```python
robot = Robot.RPC('192.168.58.2')

# Connection
robot.FT_SetConfig(24, 0)          # Set sensor config
robot.FT_Activate(0)               # Activate channel 0
robot.FT_Activate(1)               # Activate channel 1
robot.FT_SetZero(0)                # Zero sensor channel 0
robot.FT_SetZero(1)                # Zero sensor channel 1
robot.SetLoadWeight(0, 0.0)        # Set tool payload
robot.SetLoadCoord(0, 0, 0)        # Set load coordinate

# Data Acquisition
robot.FT_GetForceTorqueRCS()       # Get 6-axis F/T reading
robot.GetActualJointPosDegree()    # Get joint angles in degrees

# Control
robot.DragTeachSwitch(1)           # Enable drag mode
robot.DragTeachSwitch(0)           # Disable drag mode
```

---

## Exercise Database

### Supported Exercises (16 total)

**Upper Limb (8):**
- Shoulder Flexion
- Shoulder Extension
- Shoulder Abduction
- Shoulder Adduction
- Elbow Flexion
- Elbow Extension
- Wrist Flexion
- Wrist Extension

**Lower Limb (8):**
- Hip Flexion
- Hip Extension
- Hip Abduction
- Hip Adduction
- Ankle Plantarflexion
- Ankle Dorsiflexion
- Knee Extension
- Knee Flexion

Each exercise maps to a reference image in `/home/um/fairino-python-sdk-main/images/`

---

## File Structure

```
/home/um/fairino-python-sdk-main/linux/RIMT/
├── Final_RIMT_Code.py              # Main application (2466 lines)
├── README.md                         # This file
└── (future: config.json)

/home/um/fairino-python-sdk-main/images/
├── Shoulder_flexion.jpeg
├── Elbow_flexion.jpeg
├── Hip_flexion.jpeg
├── Knee_extension.jpeg
└── ... (16 exercise images)

/home/um/fairino-python-sdk-main/
├── results.csv                       # Trial data archive
└── linux/
    ├── fairino/
    │   └── Robot.py                  # Robot SDK module
    └── RIMT/
```

---

## Key Features

### **1. Bilateral Testing**
- Independent left/right side trials
- Comparative force analysis
- Asymmetry detection

### **2. Real-time Monitoring**
- Live Fz force display
- Joint angle streaming (J4, J5, J6)
- 200ms UI refresh rate

### **3. Drag Mode (Free-Drive)**
- Manual robot positioning by therapist
- Low-force sensitivity
- Baseline calibration for accurate offset removal

### **4. Automated Analysis**
- Peak force detection
- RFD calculation (50-100ms window)
- Matplotlib graph generation with annotations

### **5. Data Persistence**
- CSV export with side information
- Patient history tracking
- Progress report visualization

### **6. Web-Based Interface**
- No desktop app required
- Responsive design
- Cross-device compatibility

---

## Error Handling

### Connection Errors
- Attempts 1 auto-reconnect on startup
- Displays "Connecting..." status
- Disables drag mode controls if disconnected

### Sensor Errors
- Catches F/T read exceptions
- Continues monitoring (fault-tolerant)
- Logs to browser console

### File I/O Errors
- CSV write failures logged
- Image not found returns 404 with message
- Creates `images/` directory if missing

---

## Performance Metrics

| Metric | Value |
|--------|-------|
| HTTP Response Time | <50ms |
| Sensor Sampling Rate | 125 Hz (8ms) |
| Live Data Update Rate | 200ms (5 Hz) |
| Plot Generation Time | 1-2 seconds |
| CSV Write Time | <100ms |
| Memory Footprint | ~150 MB (with matplotlib) |

---

## Future Enhancements

- [ ] Database (SQLite/PostgreSQL) instead of CSV
- [ ] Export reports as PDF
- [ ] Multi-user accounts & authentication
- [ ] Machine learning for strength prediction
- [ ] Mobile app (React Native)
- [ ] Real-time EMG integration
- [ ] Advanced analytics dashboard
- [ ] Batch exercise protocols

---

## Dependencies

```python
# Core
flask
flask-cors
threading
json, csv, os

# Robotics
Robot  # FaiRino SDK module

# Visualization
matplotlib
io, base64

# System
datetime, time, webbrowser
```

---

## Startup Sequence

```
1. python Final_RIMT_Code.py
2. Flask server starts (port 5000)
3. Print startup banner
4. Open web browser → http://localhost:5000
5. Auto-attempt robot connection
6. Display dashboard
7. User interacts → triggers API calls → backend processes
```

---

## Robot API Methods Reference

The system interfaces with the FaiRino robot SDK via RPC protocol. All robot commands are synchronous blocking calls:

```python
from Robot import RPC  # FaiRino SDK module

robot = Robot.RPC('192.168.58.2')

# ==================== F/T SENSOR SETUP ====================
robot.FT_SetConfig(24, 0)          # Configure sensor
                                    # Args: mode (24=default), reserved (0)
                                    # Returns: error code (0=success)

robot.FT_Activate(0)                # Activate F/T channel 0
robot.FT_Activate(1)                # Activate F/T channel 1
                                    # Must be called before FT_SetZero()

robot.SetLoadWeight(0, 0.0)         # Set tool payload mass
                                    # Args: channel (0), mass in kg
                                    # Use 0.0 for no load (default)

robot.SetLoadCoord(0, 0, 0)         # Set load center of mass offset
                                    # Args: channel, x (mm), y (mm), z (mm)
                                    # Used for gravity compensation

robot.FT_SetZero(0)                 # Zero sensor channel 0 (remove bias/offset)
robot.FT_SetZero(1)                 # Zero sensor channel 1
                                    # Call after calibration starts
                                    # Typical delay: 0.5 seconds between calls

# ==================== DATA ACQUISITION ====================
d = robot.FT_GetForceTorqueRCS()
# Returns tuple: (error_code, (Fx, Fy, Fz, Tx, Ty, Tz))
# error_code: 0 = success, non-zero = sensor error
# Fx, Fy, Fz: Forces in Newtons [N]
# Tx, Ty, Tz: Torques in Newton-meters [N·m]
# Coordinate frame: Robot Cartesian System (RCS)
# 
# Example return:
# (0, (5.2, -3.1, 125.5, 0.05, -0.02, 0.1))

errorcode, joint_pos = robot.GetActualJointPosDegree(flag=1)
# Returns tuple: (error_code, [J1, J2, J3, J4, J5, J6])
# error_code: 0 = success
# J1-J6: Joint angles in degrees [°]
# flag=1: Real-time mode (immediate return)
# flag=0: Blocking mode (slower)
#
# Example return:
# (0, [0.0, 15.2, -45.3, 32.1, 18.5, 90.0])

# ==================== ROBOT CONTROL ====================
ret = robot.DragTeachSwitch(1)      # Enable drag/free-drive mode (Teach mode)
                                    # Args: 1 = enable, 0 = disable
                                    # Returns: error code (0=success)
                                    # Side effect: Robot enters low-resistance mode
                                    # Use Case: Allow therapist to manually position limb

ret = robot.DragTeachSwitch(0)      # Disable drag mode (return to normal)
                                    # Returns: error code (0=success)
```

**FT_GetForceTorqueRCS() Example Usage:**
```python
d = robot.FT_GetForceTorqueRCS()
if d[0] == 0:  # Check for success
    fx, fy, fz = d[1][0], d[1][1], d[1][2]
    tx, ty, tz = d[1][3], d[1][4], d[1][5]
    print(f"Fx={fx:.2f}N, Fy={fy:.2f}N, Fz={fz:.2f}N")
else:
    print(f"Sensor error: {d[0]}")
```

**GetActualJointPosDegree() Example Usage:**
```python
errorcode, joint_pos = robot.GetActualJointPosDegree(flag=1)
if errorcode == 0:
    for i, angle in enumerate(joint_pos, start=1):
        print(f"Joint {i}: {angle:.1f}°")
    # Typical output:
    # Joint 1: 0.0°
    # Joint 2: 15.2°
    # Joint 3: -45.3°
    # Joint 4: 32.1°  (Wrist - Primary axis for testing)
    # Joint 5: 18.5°  (Wrist - Secondary axis)
    # Joint 6: 90.0°  (Wrist - Rotation)
else:
    print(f"Joint read error: {errorcode}")
```

---

## Matplotlib Graph Generation Details

When a trial completes, the system generates a publication-quality annotated force vs. time graph using matplotlib:

**Graph Generation Code Flow:**
```python
def plot_results_in_thread(self, results):
    times = results['times']    # [0.0, 0.008, 0.016, ..., 4.992]
    forces = results['forces']  # [0.0, 0.2, 1.5, ..., 125.5, 120.3, ...]
    peak = results['peak_force']      # 125.5
    avg_force = results['avg_force']  # 95.23
    start_force = results['start_force']  # 0.5
    time_to_peak = results['time_to_peak']  # 2.145
    early_rfd = results['early_rfd']  # 450
    
    # Create figure: 12 inches wide, 5 inches tall, 100 DPI
    fig, ax = plt.subplots(figsize=(12, 5), dpi=100)
    
    # 1. Plot main force curve (red, 2.5pt line)
    ax.plot(times, forces, color='#e74c3c', linewidth=2.5, label='Force')
    
    # 2. Add peak force marker (green circle)
    peak_idx = forces.index(peak)
    ax.plot(times[peak_idx], forces[peak_idx], 'o', color='#27ae60', 
            markersize=10, label='Peak Force')
    ax.text(times[peak_idx], forces[peak_idx] + 1, f'Peak\n{peak:.1f}N', 
            fontsize=9, ha='center', color='#27ae60', fontweight='bold')
    
    # 3. Add average force line (orange dashed)
    ax.axhline(y=avg_force, color='#f39c12', linestyle='--', 
               linewidth=2, label='Average Force')
    
    # 4. Highlight RFD window (50-100ms post-onset)
    # Find force onset (first point > 0.5N for 10 consecutive samples)
    onset_idx = ...  # [computed]
    idx_50ms = onset_idx + int(0.05 / dt)  # 0.05 / 0.008 = 6.25 ≈ 6 samples
    idx_100ms = onset_idx + int(0.10 / dt)  # 0.10 / 0.008 = 12.5 ≈ 12 samples
    
    ax.plot([times[idx_50ms], times[idx_100ms]], 
            [forces[idx_50ms], forces[idx_100ms]], 
            color='#3498db', linewidth=4, marker='o', markersize=8,
            label='Early RFD (50-100ms)', zorder=4)
    
    ax.text(times[idx_50ms], forces[idx_50ms] - 1.5, 'RFD Start\n50ms', 
            fontsize=8, ha='center', color='#3498db', fontweight='bold')
    ax.text(times[idx_100ms], forces[idx_100ms] + 1, 'RFD End\n100ms', 
            fontsize=8, ha='center', color='#3498db', fontweight='bold')
    
    # 5. Configure axes
    ax.set_title('Force vs Time (Isometric Trial)', fontsize=14, fontweight='bold')
    ax.set_xlabel('Time (s)', fontsize=12)
    ax.set_ylabel('Force (N)', fontsize=12)
    ax.grid(True, alpha=0.3, color='gray')
    ax.set_facecolor('white')
    fig.patch.set_facecolor('white')
    
    # 6. Add legend (upper left, outside plot area)
    ax.legend(loc='upper left', fontsize=10, facecolor='white', 
              edgecolor='black', labelcolor='black', framealpha=0.95)
    
    # 7. Convert to base64 PNG
    buffer = io.BytesIO()
    fig.savefig(buffer, format='png', facecolor='white', 
                dpi=100, bbox_inches='tight')
    buffer.seek(0)
    image_base64 = base64.b64encode(buffer.read()).decode('utf-8')
    plt.close(fig)
    
    # Result: ~100 KB base64-encoded PNG string
    return image_base64
```

**Graph Components Breakdown:**

| Component | Color | Purpose | Example |
|-----------|-------|---------|---------|
| Force Curve | Red (#e74c3c) | Primary data visualization | Smooth red line 0→5s |
| Peak Marker | Green (#27ae60) | Highlight maximum force | Circle at 2.145s, 125.5N |
| Average Line | Orange (#f39c12) | Reference baseline | Horizontal dashed at 95.23N |
| RFD Window | Blue (#3498db) | Early force development | Segment from 50ms to 100ms post-onset |
| Grid | Gray (α=0.3) | Reference lines | Subtle background grid |
| Title | Black, Bold | Context | "Force vs Time (Isometric Trial)" |
| Axis Labels | Black | Measurement units | "Time (s)", "Force (N)" |

**Embedding in HTML:**
```javascript
// Frontend receives base64 from /api/trial/plot-status
const imageSrc = `data:image/png;base64,${plot_image_base64}`;
const imgElement = document.createElement('img');
imgElement.src = imageSrc;
imgElement.style.maxWidth = '100%';
imgElement.style.height = 'auto';
imgElement.style.borderRadius = '6px';

document.getElementById('graphArea').appendChild(imgElement);
```

**HTML Result:**
```html
<img src="data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAA..." 
     style="max-width: 100%; height: auto; border-radius: 6px;">
```

---

## Dependencies

```python
# Web Framework & CORS
flask              # Micro web framework
flask-cors         # Cross-origin resource sharing

# Data Processing & Threading
threading          # Concurrent execution (built-in)
json               # JSON serialization (built-in)
csv                # CSV file reading/writing (built-in)
os                 # Operating system interface (built-in)
datetime           # Timestamp generation (built-in)
time               # Time utilities (built-in)

# Robotics
Robot              # FaiRino SDK module
                   # Location: /home/um/fairino-python-sdk-main/linux/fairino/Robot.py

# Visualization & Image Encoding
matplotlib         # Plotting library
io                 # In-memory file buffers (built-in)
base64             # Base64 encoding (built-in)

# System Integration
webbrowser         # Auto-open browser (built-in)
```

**Installation:**
```bash
pip install flask flask-cors matplotlib
```

---

## Future Enhancements & Roadmap

- [ ] **Database Integration** - Replace CSV with SQLite/PostgreSQL for better querying
- [ ] **PDF Export** - Generate printable clinical reports
- [ ] **Multi-User System** - User accounts with patient assignment
- [ ] **Authentication** - Login/role-based access control
- [ ] **Advanced Analytics** - Machine learning for strength prediction
- [ ] **Mobile App** - React Native client for tablet/phone
- [ ] **EMG Integration** - Real-time electromyography overlay
- [ ] **Advanced Dashboards** - Interactive analytics with Plotly/Chart.js
- [ ] **Batch Protocols** - Pre-defined testing sequences for standard assessments
- [ ] **Cloud Sync** - Backup patient data to cloud storage
- [ ] **Video Recording** - Record trial sessions for documentation
- [ ] **Export Formats** - Excel, JSON, XML output options

---

## Support & Troubleshooting Reference

| Issue | Symptom | Solution |
|-------|---------|----------|
| Port conflict | `Address already in use` | Change port in line 2427 |
| Robot offline | `Connecting...` status persists | Ping 192.168.58.2, verify network |
| Sensor not zeroing | Forces always show offset | Recalibrate: disable/enable drag mode |
| Graphs not rendering | Blank graph area | Check DevTools console, verify matplotlib installed |
| CSV not saving | No results.csv file | Check write permissions in working directory |
| Images missing | 404 errors in console | Place images in `/home/um/fairino-python-sdk-main/images/` |
| Slow performance | UI lag, 1-2 second delays | Reduce polling frequency (line 1318), check CPU usage |

---

## Quick Reference: Common Tasks

**Adding a New Exercise:**
1. Add button to HTML (line ~800): `<button class="exercise-btn" onclick="startExercise('New Exercise')">New Exercise</button>`
2. Add image mapping (line ~1280): `'New Exercise': '/api/exercise/image/new_exercise.jpeg',`
3. Place image: `/home/um/fairino-python-sdk-main/images/new_exercise.jpeg`

**Changing Calibration Points:**
Edit line ~2030: `for _ in range(100):` → Change 100 to desired count (e.g., 50 or 200)

**Adjusting RFD Window:**
Edit line ~1748: `idx_50ms = onset_idx + int(0.05 / dt)` → Change 0.05 to desired start time

**Increasing Sampling Resolution:**
Edit line ~1725: `time.sleep(0.008)` → Change to smaller value (e.g., 0.004 for 250 Hz)

---

**Last Updated:** February 6, 2025  
**Version:** 2.0 (Detailed Documentation)  
**Documentation Status:** Comprehensive - All major components documented

