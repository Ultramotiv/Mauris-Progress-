# MAURIS Passive Mode – Step-by-Step Guide (`Mauris_modes.py`)

This README explains exactly what happens in **Passive Mode only** when you run `Mauris_modes.py`, what you will see on screen, and what options you will have.

---

## 1) Run the App

From the project root, run:

```bash
python3 linux/Meril/Mauris_modes.py
```

The app opens a desktop window titled **Therapy Control Interface**.

---

## 2) First Screen You See (Main Menu)

When the window opens, you see 3 major sections:

1. **Robot Connection**
   - Status indicator dot + text
   - Initially shows connecting status
   - Background thread checks robot connectivity periodically
2. **Patient Information**
   - Full Name
   - Age
   - Gender
   - Diagnosis
3. **Therapy Mode Selection**
   - Buttons: Active, Passive, Guided, Resistive, Game
   - In this code, only **Passive Mode** is enabled
   - Other mode buttons are disabled / not available

### Important behavior on startup
- `PassiveTherapyPage` is created at startup.
- It tries `Robot.RPC(ROBOT_IP)` using IP `192.168.58.2`.
- If successful, main-page status can update to connected.

---

## 3) Select Passive Mode

Click **Passive Mode** button on the main menu.

The app switches to the Passive Therapy page (`PassiveTherapyPage`), which contains these blocks:

1. **Controls**
   - `Move Robot` (drag/assist mode toggle)
   - `Home` (move joints to predefined home position)
2. **Recording**
   - `Start Recording`
   - `Stop Recording`
   - Current recording status label (`Idle`, `Recording: ...`, `Saved as: ...`)
3. **Playback Recorded Exercise**
   - Exercise dropdown
   - Repetitions spinbox (1 to 100)
   - Current Rep label
   - `Start Playback` (changes to `Stop Playback` while running)
   - `Pause`
   - `Play` (resume)
4. **Real-time Telemetry**
   - Status text (`Ready`, `Recording...`, `Playing...`, etc.)
   - `Fz Force` (N)
   - `TCP Z Position` (mm)
5. **Back to Main Menu** button

---

## 4) Passive Page Background Processes (automatic)

As soon as Passive page is loaded:

1. **Telemetry thread starts** (every ~20 ms)
   - Reads TCP pose via `GetActualTCPPose()`
   - Reads force/torque via `FT_GetForceTorqueRCS()`
2. **UI timer starts** (every 50 ms)
   - Updates labels:
     - `Fz Force`
     - `TCP Z Position`
     - `Current Rep`

So you continuously see live force + position values.

---

## 5) Controls Section (what each option does)

## 5.1 `Move Robot` button (Drag mode toggle)

- First click:
  - Calls `custom_drag_teach_mode(enable=True)`
  - Internally:
    1. `DragTeachSwitch(1)`
    2. Starts force/joint impedance (`ForceAndJointImpedanceStartStop(status=1, impedanceFlag=1, ...)`)
  - If success:
    - Internal state `drag_active=True`
    - Button text changes to **Drag Mode (Active)**
    - Status label shows **Drag Mode: Active**
- Next click:
  - Disables impedance + drag mode
  - Button text returns to **Drag Mode**
  - Status label returns to **Ready**

### Restriction
- If drag mode is active, you cannot start recording or playback.

## 5.2 `Home` button

- Moves robot to fixed joint target:
  - `[2.608, -88.384, 127.473, -137.27, -92.275, -90.118]`
- Uses `MoveJ(...)` with computed velocity percent.
- On success, status shows **Moving to Home...**.

### Restriction
- Home move is blocked if recording or playback is active.

---

## 6) Recording Section (exact flow)

## 6.1 Start Recording

When you click **Start Recording**:

1. If drag mode is active -> warning popup: disable drag mode first.
2. Input dialog asks trajectory name.
3. If name is empty/cancelled -> nothing happens.
4. If name entered, app calls `start_recording(name)`:
   - Enables drag/impedance mode
   - `SetTPDParam(name, 4, 0)`
   - `SetTPDStart(name, 4, 0)`
5. If success:
   - Logs entry in `trajectory_log.csv` (`Timestamp, TrajectoryName, Notes`)
   - `Start Recording` button disabled
   - `Stop Recording` button enabled
   - Recording label: **Recording: <name>**
   - Status: **Recording Trajectory...**
6. If failure:
   - Warning popup with SDK error code/message

## 6.2 Stop Recording

When you click **Stop Recording**:

1. Calls `SetWebTPDStop()`
2. Disables drag/impedance mode
3. If success:
   - UI returns to idle recording state
   - Recording label: **Saved as: <name>**
   - Status: **Ready**
   - Recorded exercises list is refreshed from `trajectory_log.csv`
4. If failure:
   - Warning popup with reason

---

## 7) Playback Section (exact flow)

## 7.1 Select recording + repetitions

- Dropdown first item is placeholder: **Select a recorded exercise...**
- Playback button is enabled only when a real recording is selected.
- Repetitions can be set from 1 to 100 (default 5).

## 7.2 Start Playback

When you click **Start Playback**:

1. If drag mode active -> warning popup
2. Reads selected filename and repetitions
3. Calls `start_playback(filename, reps)` (runs in daemon thread)
4. UI changes:
   - `Start Playback` button text becomes **Stop Playback**
   - Button action switches to stop function
   - `Pause` enabled
   - `Play` (resume) disabled
   - Status: **Playing Back Recorded Exercise**

### Playback thread behavior
Inside playback thread:

1. `LoadTPD(filename)`
2. Reads current TCP pose
3. Determines direction (forward/reverse)
4. For each repetition:
   - Move robot to trajectory start/end (`MoveL`)
   - Wait until motion complete (`GetRobotMotionDone` loop)
   - Execute trajectory (`MoveTPD(filename, direction, 100)`)
   - Wait until done
   - Increment `playback_repetition_count`
5. On finish/stop:
   - `playback_active=False`
   - direction cleared

`Current Rep` label updates live from `playback_repetition_count`.

## 7.3 Stop Playback

When you click **Stop Playback**:

1. `playback_active` flag set false
2. `StopMotion()` sent to robot
3. UI resets:
   - button text back to **Start Playback**
   - click action back to start function
   - `Pause` disabled
   - `Play` disabled
   - Status: **Ready**

## 7.4 Pause / Resume

- **Pause**:
  - Calls `PauseMotion()`
  - If success: Pause disabled, Play enabled
- **Play** (resume):
  - Calls `ResumeMotion()`
  - If success: Pause enabled, Play disabled
- If SDK call fails: warning popup with error code/message

---

## 8) Back to Main Menu

Click **Back to Main Menu** to return to first screen. No app restart is needed.

---

## 9) Files/Logs Used by Passive Mode

- `trajectory_log.csv`
  - Used to populate recorded exercise dropdown
  - New recordings are logged here
- TPD data is handled by robot SDK functions (`SetTPD...`, `LoadTPD`, `MoveTPD`)

---

## 10) What You Can Do in Passive Mode (Quick Summary)

You can:

1. Put robot in drag-assist mode (`Move Robot`)
2. Move robot to home joint position (`Home`)
3. Record a therapist-guided trajectory (`Start/Stop Recording`)
4. Replay recorded trajectory for N repetitions (`Start/Stop Playback`)
5. Pause/resume playback (`Pause` / `Play`)
6. Monitor live force and TCP Z telemetry

You cannot (in this file) use Active/Guided/Resistive/Game workflows; those are disabled or not implemented here.
