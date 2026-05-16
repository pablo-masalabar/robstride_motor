# trajectory_tracker

ROS 2 package for recording, replaying, and managing arm trajectories and poses for RobStride motors. Operates on two arms defined as **source** and **target**, with configurable per-motor transforms between them.

---

## Package structure

```
trajectory_tracker/
├── config/
│   ├── left_s_right_t.toml      # Left arm source, right arm target
│   ├── right_s_left_t.toml      # Right arm source, left arm target
│   ├── left_arm_homing.toml     # Homing pose for left arm
│   └── right_arm_homing.toml   # Homing pose for right arm
├── launch/
│   ├── left_s_right_t.launch.py
│   └── right_s_left_t.launch.py
├── recorded_poses/              # Snapshot poses saved by record_arm_pose
├── recorded_trajectories/       # Not used by node; for manual file management
├── trajectory_tracker/
│   ├── trajectory_tracker_node.py
│   └── transforms.py            # Per-motor position transform functions
```

---

## Config files

### Trajectory config (`left_s_right_t.toml` / `right_s_left_t.toml`)

| Field | Description |
|---|---|
| `node_name` | Prefix for all service, action, and topic names |
| `source_node_prefix` | ROS 2 node prefix of the source arm motor node |
| `target_node_prefix` | ROS 2 node prefix of the target arm motor node |
| `active_report_hz` | Hz for active state reporting on both arms at startup |
| `trajectory_record_hz` | Sampling rate when recording a trajectory to CSV |
| `replay_hz` | Fallback replay rate if not set in CSV metadata or action goal |
| `target_motors_mode` | Default command mode: `"pp"` or `"csp"` |
| `export_path` | Directory where trajectory CSVs are written |
| `source_motors` | List of source arm motor names |
| `target_motors` | List of target arm motor names (parallel to `source_motors`) |
| `source_motor_state_topic_pattern` | Topic pattern for source motor states (`{name}` is replaced) |
| `target_pp_topic_pattern` | Topic pattern for target PP commands |
| `target_csp_topic_pattern` | Topic pattern for target CSP commands |
| `[motor_map]` | Explicit source motor → target motor mapping |
| `[transform_map]` | Per-source-motor transform function name (source → target direction) |
| `[inverse_transform_map]` | Per-source-motor transform for target → source direction |
| `[pp_defaults]` | Default PP motion parameters: `speed`, `acceleration`, `deceleration`, `torque_limit` |
| `[csp_defaults]` | Default CSP parameters: `speed_limit`, `current_limit` |

### Homing config (`left_arm_homing.toml` / `right_arm_homing.toml`)

| Field | Description |
|---|---|
| `motor_node_prefix` | ROS 2 node prefix of the arm being homed |
| `motor_mode` | Mode for homing: `"pp"` or `"csp"` |
| `[homing_pos]` | Motor name → target position (rad) |
| `[pp_defaults]` | PP parameters used when `motor_mode = "pp"` |
| `[csp_defaults]` | CSP parameters used when `motor_mode = "csp"` |

---

## Transforms

Transforms are defined in `transforms.py`. Each function takes a `float` position and returns a `float`.

| Name | Formula |
|---|---|
| `passthrough` | `x` |
| `negate` | `-x` |
| `subtract_from_2pi` | `2π − x` |
| `subtract_2pi` | `x − 2π` |

**`[transform_map]`** is applied when going **source → target** (replay, simulate).  
**`[inverse_transform_map]`** is applied when going **target → source** (`set_arm_pose` with a target-recorded pose).

Both maps are keyed by the **source motor name**.

---

## Node initialisation

On startup the node:

1. Loads the config file specified via the `config_path` ROS 2 parameter.
2. Resolves `config_dir` (defaults to the parent directory of `config_path`) — used to resolve short filenames passed to services and actions.
3. Loads motor maps, transforms, and inverse transforms.
4. Creates PP and CSP command publishers for every target motor.
5. Subscribes to the state topic of every source motor (caches latest `MotorState` per motor).
6. Registers all services and action servers under the `node_name` namespace.
7. After a 1 s delay, enables active reporting on both source and target arms.

---

## Topics

All topics are prefixed with `node_name` (e.g. `trajectory_tracker/`).

| Topic | Type | Direction | Description |
|---|---|---|---|
| `joint_command` | `custom_interfaces/JointCommand` | Published | One frame per `simulate_trajectory` tick — position, velocity, effort for all source motors |
| `step_trajectory` | `std_msgs/Bool` | Subscribed | `true` = advance one step in step-through replay; `false` = cancel the replay |

---

## Services

All services are prefixed with `node_name`.

### `capture_homing_pose`
Reads current motor positions from the arm specified in a homing `.toml` file and writes them back to `[homing_pos]` in that file.

**Request:** `config_file` (path or filename relative to `config_dir`)  
**Response:** `success`, `message`, `motors[]`, `positions[]`

**Flow:**
1. Load homing config to get `motor_node_prefix` and `[homing_pos]` motor names.
2. Subscribe temporarily to each motor's state topic; wait up to 3 s per motor.
3. Update `[homing_pos]` values in the file in-place using regex (comments preserved).
4. Destroy temporary subscriptions.

---

### `record_arm_pose`
Captures the current state of either the source or target arm and saves it as a CSV in `{export_path}/recorded_poses/`.

**Request:** `arm` (`"source"` or `"target"`), `name` (filename; empty → `H_M_S_DD_MM_YY`)  
**Response:** `success`, `message`, `file_path`

**CSV columns:** `motor_name`, `position`, `velocity`, `torque`, `temperature`, `mode`, `fault`, `enabled`

**Flow:**
1. Determine motor list and state topic pattern from `arm` field.
2. Subscribe temporarily to each motor's state topic; wait up to 3 s per motor.
3. Write one CSV row per motor.

---

### `set_arm_pose`
Loads a pose CSV from `recorded_poses/`, applies the appropriate transform, sets run mode, and commands the mapped motors.

**Request:** `name`, `target_mode` (`"pp"` or `"csp"`), PP/CSP params (0.0 = config default)  
**Response:** `success`, `message`, `motors_set[]`

**Transform logic:**
- If CSV motor is a **source motor** → apply `[transform_map]` → command the **mapped target motor**
- If CSV motor is a **target motor** → apply `[inverse_transform_map]` → command the **mapped source motor**

**Flow:**
1. Load CSV, parse `motor_name → position`.
2. Resolve each motor's destination and transform.
3. Call `set_run_mode` with `automatic_enable_disable=True` for all destination motors — this disables, changes mode, and re-enables.
4. Publish position commands. Motors are left **enabled and in the specified mode**.

---

### `stop_trajectory_recording`
Stops an active `record_trajectory` action by signalling its stop event.

**Request:** (none)  
**Response:** `success`, `message`, `file_path`, `samples_recorded`

---

### `trim_trajectory`
Removes rows from a trajectory CSV whose timestamp falls inside any of the given `(start_ts, end_ts)` ranges (exclusive). Edits the file in-place.

**Request:** `trajectory_name`, `start_ts[]`, `end_ts[]` (parallel lists)  
**Response:** `success`, `message`, `rows_before`, `rows_after`, `rows_removed`

---

## Actions

All actions are prefixed with `node_name`.

### `homing`
Moves all motors listed in a homing config to their defined home positions.

**Goal:** `config_path` (path or filename relative to `config_dir`)  
**Feedback:** `motor_name`, `motors_done`, `motors_total`  
**Result:** `success`, `message`, `homed_motors[]`

**Constraints:** `motor_mode` must be `"pp"` or `"csp"` — velocity mode is rejected.

**Flow:**
1. Load homing config; validate `motor_mode`.
2. For each motor: call `set_run_mode` (`automatic_enable_disable=True`), then publish the position command with the configured PP/CSP params.
3. Motors are left **enabled and holding** the homing position when the action exits.

---

### `record_trajectory`
Records source motor states to a CSV file at `trajectory_record_hz`, running until `stop_trajectory_recording` is called.

**Goal:** `trajectory_name` (empty → `H_M_S_DD_MM_YY` timestamp)  
**Feedback:** `samples_recorded`, `elapsed_time`  
**Result:** `success`, `message`, `file_path`, `samples_recorded`

**CSV metadata lines (before header):**
```
# recorded_at: H_M_S_DD_MM_YY
# replay_hz: <trajectory_record_hz>
```

**CSV columns:** `timestamp` (seconds from action start, 4 decimal places), then for each source motor: `{motor}_position`, `{motor}_velocity`, `{motor}_torque`, `{motor}_temperature`, `{motor}_mode`, `{motor}_fault`, `{motor}_enabled`

**Flow:**
1. Open CSV file, write metadata and header.
2. Every `1/trajectory_record_hz` seconds: sample `_latest_states` cache, write one row.
3. On `stop_trajectory_recording` service call: exit loop, close file cleanly.
4. On node shutdown: `_recording_stop_event` is set automatically — file is always closed cleanly.

---

### `replay_trajectory`
Loads a trajectory CSV and replays it onto target motors via the configured transforms.

**Goal:**

| Field | Description |
|---|---|
| `trajectory_name` | CSV filename (without extension) |
| `replay_hz` | Playback rate; `0.0` → use `# replay_hz` from CSV metadata, then config fallback |
| `target_mode` | `"pp"` or `"csp"`; `""` → use config `target_motors_mode` |
| `step_through` | If `true`, advance by `step_pct` per trigger instead of continuous playback |
| `step_pct` | Percentage of total frames to advance per `step_trajectory` trigger |
| `pp_speed` / `pp_acceleration` / `pp_deceleration` / `pp_torque_limit` | PP overrides; `0.0` → config default |
| `csp_speed_limit` / `csp_current_limit` | CSP overrides; `0.0` → config default |

**Feedback:** `frames_published`, `frames_total`, `elapsed_time`, `progress_pct`  
**Result:** `success`, `message`, `frames_published`

**Flow:**
1. Load CSV via `_read_csv` (skips `#` metadata lines); parse `replay_hz` from metadata.
2. Resolve Hz, mode, and PP/CSP params.
3. For each source motor present in `[motor_map]`: call `set_run_mode` on its target.
4. Enable all target motors.
5. Replay loop: for each row, apply `[transform_map]` to position, publish to target motor's command topic.
6. **Step-through mode:** wait on `step_trajectory` topic before each batch. Publish `true` to advance, `false` to cancel.
7. After last frame: hold last position for 1 s (PP/CSP), then disable all target motors.
8. On node shutdown: target motors are disabled by `shutdown_cleanup`.

**Hz resolution order:** goal `replay_hz` → CSV `# replay_hz` → config `replay_hz`

---

### `simulate_trajectory`
Loads a trajectory CSV and publishes each frame as a `JointCommand` message (does not command motors directly).

**Goal:** `trajectory_name`, `replay_hz` (`0.0` → use original inter-frame timestamps from CSV)  
**Feedback:** `frames_published`, `frames_total`, `elapsed_time`  
**Result:** `success`, `message`, `frames_published`

**Flow:**
1. Load CSV, parse motor names from header.
2. Publish one `JointCommand` per row: `names[]`, `positions[]`, `velocities[]`, `efforts[]` with `header.stamp` set to current ROS time.
3. Timing: fixed interval if `replay_hz > 0`, else sleep the exact inter-frame delta from the CSV `timestamp` column.

---

## Shutdown behaviour

`shutdown_cleanup` is called on SIGINT and performs the following in order:

1. If recording is active: sets the stop event → recording loop exits and closes the CSV cleanly.
2. Disables all target motors via `enable_motor all=False`.
3. Disables active reporting on both source and target arms.

---

## Step-through replay

Step-through mode allows advancing through a trajectory one batch at a time, useful for inspection or manual validation.

```bash
# Start a step-through replay (10% per step)
ros2 action send_goal /trajectory_tracker/replay_trajectory \
  custom_interfaces/action/ReplayTrajectory \
  "{trajectory_name: 'my_traj', replay_hz: 0.0, target_mode: '', \
    step_through: true, step_pct: 10.0, \
    pp_speed: 0.0, pp_acceleration: 0.0, pp_deceleration: 0.0, pp_torque_limit: 0.0, \
    csp_speed_limit: 0.0, csp_current_limit: 0.0}"

# Advance one step
ros2 topic pub --once /trajectory_tracker/step_trajectory std_msgs/msg/Bool "{data: true}"

# Cancel
ros2 topic pub --once /trajectory_tracker/step_trajectory std_msgs/msg/Bool "{data: false}"
```

---

## Typical workflows

### Record and replay a trajectory

```bash
# 1. Start recording (left arm source)
ros2 action send_goal /trajectory_tracker/record_trajectory \
  custom_interfaces/action/RecordTrajectory "{trajectory_name: 'demo'}"

# 2. Move the left arm manually or via teleop

# 3. Stop recording
ros2 service call /trajectory_tracker/stop_trajectory_recording \
  custom_interfaces/srv/StopTrajectoryRecording

# 4. Replay on right arm
ros2 action send_goal /trajectory_tracker/replay_trajectory \
  custom_interfaces/action/ReplayTrajectory \
  "{trajectory_name: 'demo', replay_hz: 0.0, target_mode: 'pp', \
    step_through: false, step_pct: 0.0, \
    pp_speed: 0.0, pp_acceleration: 0.0, pp_deceleration: 0.0, pp_torque_limit: 0.0, \
    csp_speed_limit: 0.0, csp_current_limit: 0.0}"
```

### Homing

```bash
# Home the left arm
ros2 action send_goal /trajectory_tracker/homing \
  custom_interfaces/action/Homing "{config_path: 'left_arm_homing.toml'}"
```

### Capture and restore a pose

```bash
# Capture current left arm pose
ros2 service call /trajectory_tracker/record_arm_pose \
  custom_interfaces/srv/RecordArmPose "{arm: 'source', name: 'rest_pose'}"

# Restore it (applied to right arm via transform)
ros2 service call /trajectory_tracker/set_arm_pose \
  custom_interfaces/srv/SetArmPose \
  "{name: 'rest_pose', target_mode: 'pp', \
    pp_speed: 0.0, pp_acceleration: 0.0, pp_deceleration: 0.0, pp_torque_limit: 0.0, \
    csp_speed_limit: 0.0, csp_current_limit: 0.0}"
```

### Update homing positions from current pose

```bash
ros2 service call /trajectory_tracker/capture_homing_pose \
  custom_interfaces/srv/CaptureHomingPose "{config_file: 'left_arm_homing.toml'}"
```
