# trajectory_tracker

ROS 2 package for recording, replaying, and managing arm trajectories and poses for RobStride motors. Supports two physical arms (`left_arm` / `right_arm`) with configurable per-joint transforms, flexible recording/replay direction, fault detection, and pause/resume control.

---

## Package structure

```
trajectory_tracker/
├── config/
│   └── config.toml              # Single unified config
├── launch/
│   └── trajectory_tracker.launch.py
├── recorded_poses/              # Snapshot poses saved by record_arm_pose / capture_homing_pose
├── recorded_trajectories/       # CSVs written by record_trajectory
├── trajectory_tracker/
│   ├── trajectory_tracker_node.py
│   └── transforms.py            # Per-joint position transform functions
```

---

## Config (`config.toml`)

| Field | Description |
|---|---|
| `node_name` | Prefix for all service, action, and topic names |
| `package_path` | Absolute path to the package root; used to resolve `recorded_poses/` and `recorded_trajectories/` |
| `left_arm_node_prefix` | ROS 2 node prefix of the physical left arm (e.g. `"/left_arm"`) |
| `right_arm_node_prefix` | ROS 2 node prefix of the physical right arm (e.g. `"/right_arm"`) |
| `left_arm_motors` | Physical left arm motor names (e.g. `["SpL", "SrL", "SwL", "EpL", "WwL", "WpL", "WrL"]`) |
| `right_arm_motors` | Physical right arm motor names (e.g. `["SpR", "SrR", "SwR", "EpR", "WwR", "WpR", "WrR"]`) |
| `active_report_hz` | Active state reporting rate requested from both arms at startup (Hz) |
| `trajectory_record_hz` | Sampling rate when recording a trajectory to CSV (Hz) |
| `replay_hz` | Fallback replay rate if not set in CSV metadata or action goal |
| `replay_motor_mode` | Default command mode for replay: `"pp"` or `"csp"` |
| `[motor_map]` | Recording motor → replay motor mapping. Keys determine which arm records; values determine which replays. |
| `[transform_map]` | Per-joint transform applied going **left → right**. Keys are **base names** (no L/R suffix, e.g. `Sp`). |
| `[inverse_transform_map]` | Per-joint transform applied going **right → left**. Keys are base names. |
| `[pp_defaults]` | Default PP parameters: `speed`, `acceleration`, `deceleration`, `torque_limit` |
| `[csp_defaults]` | Default CSP parameters: `speed_limit`, `current_limit` |

### Direction detection

The node auto-detects recording and replay arms from `[motor_map]`:
- If motor_map keys ⊆ `left_arm_motors` → left arm records, right arm replays
- If motor_map keys ⊆ `right_arm_motors` → right arm records, left arm replays

### Transform keys

Transform map keys are **base motor names** (strip the trailing `L` or `R`). This makes them direction-independent — the same config works regardless of which arm records.

---

## Transforms

Defined in `transforms.py`. Each function: `(position: float) -> float`.

| Name | Formula |
|---|---|
| `passthrough` | `x` |
| `negate` | `-x` |
| `subtract_from_2pi` | `2π − x` |
| `subtract_2pi` | `x − 2π` |

`[transform_map]` is used when going **left → right**.  
`[inverse_transform_map]` is used when going **right → left**.  
**Same-arm** replay always uses passthrough regardless of the maps.

---

## Node initialisation

On startup the node:

1. Loads `config.toml` from the `config_path` ROS 2 parameter.
2. Derives recording/replay arms from `[motor_map]`.
3. Creates PP and CSP command publishers for **all** motors on both arms (enables any replay direction without dynamic publisher creation).
4. Subscribes to state topics for **all** motors on both arms (enables fault monitoring on any replay arm).
5. Registers all services and action servers under `node_name`.
6. After 1 s: enables active reporting on both arms.

---

## Topics

All topics are prefixed with `node_name/` (e.g. `trajectory_tracker/`).

| Topic | Type | Direction | Description |
|---|---|---|---|
| `joint_command` | `custom_interfaces/JointCommand` | Published | One frame per `simulate_trajectory` tick |
| `step_trajectory` | `std_msgs/Bool` | Subscribed | `true` = advance one step in step-through replay; `false` = cancel |

---

## Services

All services are prefixed with `node_name/`.

### `pause_resume_replay`
Toggle pause/resume of an active `replay_trajectory` action. First call pauses, second resumes. While paused, motors hold the last commanded pose. Cancel and fault detection remain active during pause.

**Type:** `std_srvs/srv/Trigger`

```bash
ros2 service call /trajectory_tracker/pause_resume_replay std_srvs/srv/Trigger
```

---

### `capture_homing_pose`
Reads current motor positions from the specified physical arm and writes them to `[homing_pos]` in a homing `.toml` file.

**Type:** `custom_interfaces/srv/CaptureHomingPose`  
**Request:** `arm` (`"left_arm"` or `"right_arm"`), `config_file` (path or filename relative to `recorded_poses/`)  
**Response:** `success`, `message`, `motors[]`, `positions[]`

---

### `record_arm_pose`
Captures the current state of one physical arm and saves it as a CSV in `recorded_poses/`.

**Type:** `custom_interfaces/srv/RecordArmPose`  
**Request:** `arm` (`"left_arm"` or `"right_arm"`), `name` (filename; empty → timestamp)  
**Response:** `success`, `message`, `file_path`

**CSV columns:** `motor_name`, `position`, `velocity`, `torque`, `temperature`, `mode`, `fault`, `enabled`

---

### `set_arm_pose`
Loads a pose CSV and sends position commands to the specified arm, applying the appropriate transform if cross-arm.

**Type:** `custom_interfaces/srv/SetArmPose`  
**Request:** `name`, `arm` (`"left_arm"` or `"right_arm"`; empty → replay arm), `target_mode` (`"pp"` or `"csp"`), PP/CSP params (0.0 = config default)  
**Response:** `success`, `message`, `motors_set[]`

**Transform logic:**

| CSV contains | `arm` requested | Command sent to | Transform |
|---|---|---|---|
| Recording-arm motor | Replay arm | Mapped replay motor | `transform_map` |
| Recording-arm motor | Recording arm | Same motor | passthrough |
| Replay-arm motor | Recording arm | Mapped recording motor | `inverse_transform_map` |
| Replay-arm motor | Replay arm | Same motor | passthrough |

---

### `stop_trajectory_recording`
Stops an active `record_trajectory` action.

**Type:** `custom_interfaces/srv/StopTrajectoryRecording`  
**Response:** `success`, `message`, `file_path`, `samples_recorded`

---

### `trim_trajectory`
Removes rows from a trajectory CSV whose timestamp falls inside any of the given `(start_ts, end_ts)` ranges. Edits the file in-place.

**Type:** `custom_interfaces/srv/TrimTrajectory`  
**Request:** `trajectory_name`, `start_ts[]`, `end_ts[]`  
**Response:** `success`, `message`, `rows_before`, `rows_after`, `rows_removed`

---

## Actions

All actions are prefixed with `node_name/`.

### `homing`
Moves all motors listed in a homing config to their defined home positions using PP mode.

**Type:** `custom_interfaces/action/Homing`  
**Goal:** `config_path` (path or filename relative to `recorded_poses/`)  
**Feedback:** `motor_name`, `motors_done`, `motors_total`  
**Result:** `success`, `message`, `homed_motors[]`

The node resolves which physical arm the homing config belongs to from the motor names in `[homing_pos]` — no cross-arm transform is applied. Motors are left **enabled and holding** the homing position.

---

### `record_trajectory`
Records motor states to a CSV at `trajectory_record_hz`. Stops when cancelled or when `stop_trajectory_recording` is called.

**Type:** `custom_interfaces/action/RecordTrajectory`

**Goal fields:**

| Field | Description |
|---|---|
| `trajectory_name` | CSV filename without extension; empty → timestamp |
| `left_arm_source` | Record left arm motors |
| `right_arm_source` | Record right arm motors |

At least one of `left_arm_source` / `right_arm_source` must be `true`. Both can be `true` to record both arms simultaneously into one CSV.

**Feedback:** `samples_recorded`, `elapsed_time`  
**Result:** `success`, `message`, `file_path`, `samples_recorded`

**CSV metadata:**
```
# recorded_at: H_M_S_DD_MM_YY
# replay_hz: <trajectory_record_hz>
```

**CSV columns:** `timestamp` (seconds), then for each recorded motor: `{motor}_position`, `{motor}_velocity`, `{motor}_torque`, `{motor}_temperature`, `{motor}_mode`, `{motor}_fault`, `{motor}_enabled`

---

### `replay_trajectory`
Loads a trajectory CSV and replays it. Supports any recording→replay arm combination, simultaneous dual-arm replay, fault detection, and pause/resume.

**Type:** `custom_interfaces/action/ReplayTrajectory`

**Goal fields:**

| Field | Description |
|---|---|
| `trajectory_name` | CSV filename without extension |
| `replay_hz` | Playback rate; `0.0` → CSV metadata → config fallback |
| `target_mode` | `"pp"` or `"csp"`; `""` → config `replay_motor_mode` |
| `replay_left_arm` | Send commands to left arm motors |
| `replay_right_arm` | Send commands to right arm motors |
| `step_through` | Advance by `step_pct` per trigger instead of continuous |
| `step_pct` | Percentage of total frames per `step_trajectory` trigger |
| `pp_speed` / `pp_acceleration` / `pp_deceleration` / `pp_torque_limit` | PP overrides; `0.0` → config default |
| `csp_speed_limit` / `csp_current_limit` | CSP overrides; `0.0` → config default |

At least one of `replay_left_arm` / `replay_right_arm` must be `true`.

**Feedback:** `frames_published`, `frames_total`, `elapsed_time`, `progress_pct`  
**Result:** `success`, `message`, `frames_published`

#### Replay direction matrix

| `replay_left_arm` | `replay_right_arm` | CSV recorded from | What happens |
|---|---|---|---|
| true | false | left | left→left, passthrough |
| false | true | left | left→right, forward transform |
| true | false | right | right→left, inverse transform |
| false | true | right | right→right, passthrough |
| true | true | left | left→left (passthrough) + left→right (forward transform) simultaneously |
| true | true | right | right→right (passthrough) + right→left (inverse transform) simultaneously |
| true | false | both | left portion→left (passthrough), right portion discarded |
| false | true | both | right portion→right (passthrough), left portion discarded |
| true | true | both | left portion→left (passthrough) + right portion→right (passthrough) |

#### Fault detection

During replay, all replay motors are monitored for non-zero fault bitmasks. On fault:
- Replay stops immediately
- Action is cancelled with a descriptive message
- Motors are left **enabled and holding the last commanded pose**

#### Motor state after replay

Motors are always left **enabled and holding the last commanded pose** on all exit paths (normal completion, fault, cancel). Motors are only disabled on node shutdown.

#### Pause / resume

Use the `pause_resume_replay` service to toggle pause mid-replay. Motors hold the last pose while paused. Cancel and fault detection remain active.

---

### `simulate_trajectory`
Loads a trajectory CSV and publishes each frame as a `JointCommand` message without commanding motors.

**Type:** `custom_interfaces/action/SimulateTrajectory`  
**Goal:** `trajectory_name`, `replay_hz` (`0.0` → honour original inter-frame timestamps)  
**Feedback:** `frames_published`, `frames_total`, `elapsed_time`  
**Result:** `success`, `message`, `frames_published`

---

## Shutdown behaviour

On SIGINT, `shutdown_cleanup`:
1. Stops any active recording (CSV is closed cleanly).
2. Disables motors on both arms.
3. Disables active reporting on both arms.

---

## Launch

```bash
ros2 launch trajectory_tracker trajectory_tracker.launch.py

# Custom config
ros2 launch trajectory_tracker trajectory_tracker.launch.py \
  config_path:=/path/to/custom.toml
```

---

## Typical workflows

### Record left arm, replay on right arm

```bash
# 1. Start recording left arm
ros2 action send_goal /trajectory_tracker/record_trajectory \
  custom_interfaces/action/RecordTrajectory \
  "{trajectory_name: 'demo', left_arm_source: true, right_arm_source: false}"

# 2. Stop recording (or cancel the action)
ros2 service call /trajectory_tracker/stop_trajectory_recording \
  custom_interfaces/srv/StopTrajectoryRecording

# 3. Replay on right arm
ros2 action send_goal /trajectory_tracker/replay_trajectory \
  custom_interfaces/action/ReplayTrajectory \
  "{trajectory_name: 'demo', replay_hz: 0.0, target_mode: 'pp', \
    replay_left_arm: false, replay_right_arm: true, \
    step_through: false, step_pct: 0.0, \
    pp_speed: 0.0, pp_acceleration: 0.0, pp_deceleration: 0.0, pp_torque_limit: 0.0, \
    csp_speed_limit: 0.0, csp_current_limit: 0.0}"
```

### Record both arms, replay on both

```bash
ros2 action send_goal /trajectory_tracker/record_trajectory \
  custom_interfaces/action/RecordTrajectory \
  "{trajectory_name: 'both', left_arm_source: true, right_arm_source: true}"

ros2 action send_goal /trajectory_tracker/replay_trajectory \
  custom_interfaces/action/ReplayTrajectory \
  "{trajectory_name: 'both', replay_hz: 0.0, target_mode: 'pp', \
    replay_left_arm: true, replay_right_arm: true, \
    step_through: false, step_pct: 0.0, \
    pp_speed: 0.0, pp_acceleration: 0.0, pp_deceleration: 0.0, pp_torque_limit: 0.0, \
    csp_speed_limit: 0.0, csp_current_limit: 0.0}"
```

### Homing

```bash
ros2 action send_goal /trajectory_tracker/homing \
  custom_interfaces/action/Homing "{config_path: 'left_arm_homing.toml'}"
```

### Capture and restore a pose

```bash
# Capture current left arm pose
ros2 service call /trajectory_tracker/record_arm_pose \
  custom_interfaces/srv/RecordArmPose "{arm: 'left_arm', name: 'rest_pose'}"

# Send it to the right arm (applies forward transform)
ros2 service call /trajectory_tracker/set_arm_pose \
  custom_interfaces/srv/SetArmPose \
  "{name: 'rest_pose', arm: 'right_arm', target_mode: 'pp', \
    pp_speed: 0.0, pp_acceleration: 0.0, pp_deceleration: 0.0, pp_torque_limit: 0.0, \
    csp_speed_limit: 0.0, csp_current_limit: 0.0}"
```

### Step-through replay

```bash
# Start step-through (10% per step)
ros2 action send_goal /trajectory_tracker/replay_trajectory \
  custom_interfaces/action/ReplayTrajectory \
  "{trajectory_name: 'demo', replay_hz: 0.0, target_mode: 'pp', \
    replay_left_arm: false, replay_right_arm: true, \
    step_through: true, step_pct: 10.0, \
    pp_speed: 0.0, pp_acceleration: 0.0, pp_deceleration: 0.0, pp_torque_limit: 0.0, \
    csp_speed_limit: 0.0, csp_current_limit: 0.0}"

# Advance one step
ros2 topic pub --once /trajectory_tracker/step_trajectory std_msgs/msg/Bool "{data: true}"

# Cancel step-through
ros2 topic pub --once /trajectory_tracker/step_trajectory std_msgs/msg/Bool "{data: false}"
```
