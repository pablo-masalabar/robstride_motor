# trajectory_tracker

ROS 2 package for recording, replaying, and managing arm trajectories and poses for RobStride and Damiao motors. Supports two physical arms (`left_arm` / `right_arm`) with gripper motors recorded and replayed alongside arm motors, configurable per-joint transforms, flexible recording/replay direction, fault detection, and pause/resume control.

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
| `left_gripper_node_prefix` | ROS 2 node prefix for left gripper motors (e.g. `"/grippers"`) |
| `right_gripper_node_prefix` | ROS 2 node prefix for right gripper motors (e.g. `"/grippers"`) |
| `gripper_node_prefix` | Shared gripper node prefix (used when both grippers are on the same node) |
| `left_arm_motors` | Physical left arm motor names |
| `right_arm_motors` | Physical right arm motor names |
| `left_gripper_motors` | Left gripper motor names (e.g. `["AgL"]`) |
| `right_gripper_motors` | Right gripper motor names (e.g. `["AgR"]`) |
| `active_report_hz` | Active state reporting rate requested from all nodes at startup (Hz) |
| `trajectory_record_hz` | Sampling rate when recording a trajectory to CSV (Hz) |
| `replay_hz` | Fallback replay rate if not set in CSV metadata or action goal |
| `replay_robstride_motor_mode` | Default command mode for robstride replay: `"pp"` or `"csp"` |
| `replay_damiao_motor_mode` | Default command mode for damiao replay: `"position_velocity"`, `"mit"`, `"velocity"`, `"force_position_hybrid"` |
| `[motor_map]` | Recording motor → replay motor mapping. Keys determine which arm records; values determine which replays. |
| `[damiao_motor_map]` | Recording gripper → replay gripper mapping (e.g. `AgL = "AgR"`) |
| `[transform_map]` | Per-joint transform applied going **left → right**. Keys are **base names** (no L/R suffix). |
| `[inverse_transform_map]` | Per-joint transform applied going **right → left**. Keys are base names. |
| `[damiao_transform_map]` | Per-gripper transform going **left → right**. Keys are base names (e.g. `Ag`). |
| `[damiao_inverse_transform_map]` | Per-gripper transform going **right → left**. |
| `[robstride_pp_defaults]` | Default PP parameters: `speed`, `acceleration`, `deceleration`, `torque_limit` |
| `[csp_defaults]` | Default CSP parameters: `speed_limit`, `current_limit` |
| `[damiao_pv_defaults]` | Default PV parameters for gripper replay: `speed` |

### Direction detection

The node auto-detects recording and replay arms from `[motor_map]`:
- If motor_map keys ⊆ `left_arm_motors` → left arm records, right arm replays
- If motor_map keys ⊆ `right_arm_motors` → right arm records, left arm replays

### Transform keys

Transform map keys are **base motor names** (strip the trailing `L` or `R`). This makes them direction-independent — the same config works regardless of which arm records.

---

## Gripper support

Gripper (Damiao) motors are recorded and replayed automatically alongside their corresponding arm:

- **Trajectory recording**: gripper motors are appended to the CSV alongside arm motors
- **Trajectory replay**: gripper motors in the CSV are replayed using `damiao_motor_map` and `damiao_transform_map`; mode is set to `replay_damiao_motor_mode`
- **Pose recording** (`record_arm_pose`): arm motors + gripper motors captured together into the same CSV
- **Homing capture** (`capture_homing_pose`): arm motors + gripper motors captured into the homing TOML
- **Homing replay** (`homing`): gripper motors in `[homing_pos]` are replayed alongside arm motors

---

## Transforms

Defined in `transforms.py`. Each function: `(position: float) -> float`.

| Name | Formula |
|---|---|
| `passthrough` | `x` |
| `negate` | `-x` |
| `subtract_from_2pi` | `2π − x` |
| `subtract_2pi` | `x − 2π` |
| `add_half_pi` | `x + π/2` |

`[transform_map]` / `[damiao_transform_map]` is used going **left → right**.  
`[inverse_transform_map]` / `[damiao_inverse_transform_map]` is used going **right → left**.  
**Same-arm** replay always uses passthrough regardless of the maps.

---

## Node initialisation

On startup the node:

1. Loads `config.toml` from the `config_path` ROS 2 parameter.
2. Derives recording/replay arms from `[motor_map]`.
3. Creates PP, CSP, and PV command publishers for all arm and gripper motors on both sides.
4. Subscribes to state topics for all arm and gripper motors on both sides (fault monitoring).
5. Registers all services and action servers under `node_name`.
6. After 1 s: enables active reporting on arm and gripper nodes.

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
Reads current motor positions from the specified arm **and its grippers** and writes them to `[homing_pos]` in a homing `.toml` file.

**Type:** `custom_interfaces/srv/CaptureHomingPose`  
**Request:** `arm` (`"left_arm"`, `"right_arm"`, or `"both"`), `config_file` (path or filename relative to `recorded_poses/`)  
**Response:** `success`, `message`, `motors[]`, `positions[]`

---

### `record_arm_pose`
Captures the current state of one physical arm **and its grippers** and saves it as a CSV in `recorded_poses/`.

**Type:** `custom_interfaces/srv/RecordArmPose`  
**Request:** `arm` (`"left_arm"`, `"right_arm"`, or `"both"`), `name` (filename; empty → timestamp)  
**Response:** `success`, `message`, `file_path`

**CSV columns:** `motor_name`, `position`, `velocity`, `torque`, `temperature`, `mode`, `fault`, `enabled`  
Order: arm motors then grippers per arm. For `"both"`: left arm → left gripper → right arm → right gripper.

---

### `set_arm_pose`
Loads a pose CSV and sends position commands to the specified arm, applying the appropriate transform if cross-arm.

**Type:** `custom_interfaces/srv/SetArmPose`  
**Request:** `name`, `arm` (`"left_arm"`, `"right_arm"`, or `"both"`; empty → replay arm), `target_mode` (`"pp"` or `"csp"`), PP/CSP params (0.0 = config default)  
**Response:** `success`, `message`, `motors_set[]`

Gripper (damiao) motors in the CSV are commanded via `cmd_position_pv` using `[damiao_pv_defaults]` speed, regardless of `target_mode`.

**Transform logic:**

| CSV contains | `arm` requested | CSV has other side too? | Command sent to | Transform |
|---|---|---|---|---|
| Left arm motor | `left_arm` | any | Left arm (passthrough) | passthrough |
| Left arm motor | `right_arm` | no | Right arm | `transform_map` |
| Left arm motor | `right_arm` | yes | skipped | — |
| Right arm motor | `right_arm` | any | Right arm (passthrough) | passthrough |
| Right arm motor | `left_arm` | no | Left arm | `inverse_transform_map` |
| Right arm motor | `left_arm` | yes | skipped | — |
| Left arm motor | `both` | any | Left arm | passthrough |
| Right arm motor | `both` | any | Right arm | passthrough |
| Left gripper | `left_arm` | any | Left gripper | passthrough |
| Left gripper | `right_arm` | no right gripper in CSV | Right gripper | `damiao_transform_map` |
| Left gripper | `right_arm` | yes | skipped | — |
| Right gripper | `right_arm` | any | Right gripper | passthrough |
| Right gripper | `left_arm` | no left gripper in CSV | Left gripper | `damiao_inverse_transform_map` |
| Right gripper | `left_arm` | yes | skipped | — |
| Left/right gripper | `both` | any | Own side gripper | passthrough |

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
Moves all motors listed in a homing config (arm + grippers) to their defined home positions.

**Type:** `custom_interfaces/action/Homing`  
**Goal:** `config_path` (path or filename relative to `recorded_poses/`)  
**Feedback:** `motor_name`, `motors_done`, `motors_total`  
**Result:** `success`, `message`, `homed_motors[]`

- Arm motors are commanded in PP mode using `[robstride_pp_defaults]` from the homing config.
- Gripper motors (damiao) are commanded using `damiao_pv_defaults` speed.
- Motors are left **enabled and holding** the homing position.

---

### `record_trajectory`
Records motor states (arm + grippers) to a CSV at `trajectory_record_hz`. Stops when cancelled or when `stop_trajectory_recording` is called.

**Type:** `custom_interfaces/action/RecordTrajectory`

**Goal fields:**

| Field | Description |
|---|---|
| `trajectory_name` | CSV filename without extension; empty → timestamp |
| `left_arm_source` | Record left arm + left gripper motors |
| `right_arm_source` | Record right arm + right gripper motors |

At least one of `left_arm_source` / `right_arm_source` must be `true`. Both can be `true` to record both arms and grippers simultaneously into one CSV.

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
Loads a trajectory CSV and replays it (arm + grippers). Supports any recording→replay arm combination, simultaneous dual-arm replay, fault detection, and pause/resume.

**Type:** `custom_interfaces/action/ReplayTrajectory`

**Goal fields:**

| Field | Description |
|---|---|
| `trajectory_name` | CSV filename without extension |
| `replay_hz` | Playback rate; `0.0` → CSV metadata → config fallback |
| `target_mode` | `"pp"` or `"csp"` for arm motors; `""` → config `replay_robstride_motor_mode` |
| `replay_left_arm` | Send commands to left arm + left gripper motors |
| `replay_right_arm` | Send commands to right arm + right gripper motors |
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

Gripper motors follow the same arm direction logic using `damiao_motor_map` and `damiao_transform_map`.

#### Fault detection

During replay, all replay motors (arm + gripper) are monitored for non-zero fault bitmasks. On fault:
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
2. Disables motors on both arms and grippers.
3. Disables active reporting on all nodes.

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

### Record left arm + gripper, replay on right arm + gripper

```bash
# 1. Start recording left arm and gripper
ros2 action send_goal /trajectory_tracker/record_trajectory \
  custom_interfaces/action/RecordTrajectory \
  "{trajectory_name: 'demo', left_arm_source: true, right_arm_source: false}"

# 2. Stop recording
ros2 service call /trajectory_tracker/stop_trajectory_recording \
  custom_interfaces/srv/StopTrajectoryRecording

# 3. Replay on right arm and gripper
ros2 action send_goal /trajectory_tracker/replay_trajectory \
  custom_interfaces/action/ReplayTrajectory \
  "{trajectory_name: 'demo', replay_hz: 0.0, target_mode: 'pp', \
    replay_left_arm: false, replay_right_arm: true, \
    step_through: false, step_pct: 0.0, \
    pp_speed: 0.0, pp_acceleration: 0.0, pp_deceleration: 0.0, pp_torque_limit: 0.0, \
    csp_speed_limit: 0.0, csp_current_limit: 0.0}"
```

### Record both arms and grippers, replay on both

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

### Homing (arm + grippers)

```bash
# Capture current arm + gripper positions as homing reference
ros2 service call /trajectory_tracker/capture_homing_pose \
  custom_interfaces/srv/CaptureHomingPose "{arm: 'left_arm', config_file: 'left_arm_homing.toml'}"

# Move arm + grippers to homing positions
ros2 action send_goal /trajectory_tracker/homing \
  custom_interfaces/action/Homing "{config_path: 'left_arm_homing.toml'}"
```

### Capture and restore a pose

```bash
# Capture current left arm + gripper pose
ros2 service call /trajectory_tracker/record_arm_pose \
  custom_interfaces/srv/RecordArmPose "{arm: 'left_arm', name: 'rest_pose'}"

# Send it to the right arm + gripper (applies forward transform)
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
