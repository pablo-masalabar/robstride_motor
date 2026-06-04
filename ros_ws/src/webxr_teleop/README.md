# webxr_teleop

ROS 2 Jazzy package that bridges WebXR teleoperation with the robot's motor nodes. It subscribes to motor state feedback from all hardware nodes, forwards it to a WebXR client over Zenoh, and receives position commands back over Zenoh and publishes them as ROS motor commands.

## Architecture

```
WebXR Client
    │  feather_pb2.JointControlArray  (Zenoh → ROS)
    │  feather_pb2.JointStatesArray   (ROS → Zenoh)
    ▼
webxr_teleop_node
    ├── Subscribes: /{node}/motors/{motor}/state  (custom_interfaces/MotorState)
    ├── Publishes:  /{node}/cmd_position_pp        (PositionPPCommand)
    │              /{node}/cmd_position_csp        (PositionCSPCommand)
    │              /{node}/cmd_position_pv         (PositionPPCommand)
    └── Services:  ~/freeze, ~/enable_motors
```

## Dependencies

- `rclpy`, `custom_interfaces`
- `zenoh` Python library
- `feather` package with `feather_pb2` protobuf definitions

## Build & Run

```bash
cd ros_ws
colcon build --packages-select webxr_teleop
source install/setup.bash

ros2 launch webxr_teleop webxr_teleop.launch.py
# or with a custom config:
ros2 launch webxr_teleop webxr_teleop.launch.py config_path:=/path/to/config.toml
```

## Config (`config/config.toml`)

### Top-level fields

| Field | Required | Description |
|---|---|---|
| `node_name` | yes | ROS node name |
| `robstride_motor_mode` | yes | Control mode for robstride motors: `pp` or `csp` |
| `damiao_motor_mode` | yes | Control mode for damiao motors: `position_velocity`, `mit`, `velocity`, `force_position_hybrid` |
| `joints_state_publish_hz` | yes | Rate at which motor states are published to Zenoh (Hz) |
| `zenoh_config` | no | Path to a Zenoh JSON5 config file; uses default config if absent |
| `robstride_active_report_hz` | no | Global active report rate for all robstride nodes (Hz) |
| `damiao_active_report_hz` | no | Global active report rate for all damiao nodes (Hz) |

### Motor mode values

**Robstride** (`robstride_motor_mode`):
- `pp` — Position Profile (trapezoidal velocity profile, uses speed/accel/decel)
- `csp` — Cyclic Synchronous Position (direct position, uses speed/current limits)

**Damiao** (`damiao_motor_mode`):
- `position_velocity` — Position with velocity feedforward
- `mit` — MIT mode (raw torque/position/velocity gains)
- `velocity` — Velocity control
- `force_position_hybrid` — Force+position hybrid

### Default sections

```toml
[robstride_pp_defaults]
speed        = 15.0   # rad/s
acceleration = 10.0   # rad/s²
deceleration = 10.0   # rad/s²
torque_limit = 25.0   # N·m

[damiao_pv_defaults]
speed = 5.0    # rad/s
```

Both sections and all their fields are required. The node will log a fatal error and exit if any are missing.

### Zenoh keys

Feedback keys (ROS → Zenoh) and command keys (Zenoh → ROS) are specified as top-level fields. The naming convention is `{group_name}_zenoh_key` for feedback and `{group_name}_cmd_zenoh_key` for commands.

```toml
# Feedback (ROS → Zenoh)
left_arm_zenoh_key      = "robot/feedback/left_arm"
right_arm_zenoh_key     = "robot/feedback/right_arm"
left_gripper_zenoh_key  = "robot/feedback/left_gripper"
right_gripper_zenoh_key = "robot/feedback/right_gripper"
torso_zenoh_key         = "robot/feedback/torso"
chassis_zenoh_key       = "robot/feedback/chassis"

# Commands (Zenoh → ROS)
left_arm_cmd_zenoh_key      = "robot/cmd/left_arm"
right_arm_cmd_zenoh_key     = "robot/cmd/right_arm"
left_gripper_cmd_zenoh_key  = "robot/cmd/left_gripper"
right_gripper_cmd_zenoh_key = "robot/cmd/right_gripper"
torso_cmd_zenoh_key         = "robot/cmd/torso"
chassis_cmd_zenoh_key       = "robot/cmd/chassis"
```

### Zenoh motor groups

Maps each Zenoh group name to the motors whose states are published to / commands received from that key. Group names must match the `*_zenoh_key` prefixes above.

```toml
[zenoh_motor_groups]
left_arm      = ["SpL", "SrL", "SwL", "EpL", "WwL", "WpL", "WrL"]
right_arm     = ["SpR", "SrR", "SwR", "EpR", "WwR", "WpR", "WrR"]
left_gripper  = ["AgL"]
right_gripper = ["AgR"]
torso         = ["NpC", "NwC"]
chassis       = ["BwC", "BwR", "BwL", "BpC", "BpR", "BpL"]
```

### Hardware node sections

Each hardware node (arm, gripper, etc.) is declared as a TOML table `[node_name]` with optional fields and per-motor subtables `[node_name.MotorName]`.

**Node-level fields (all optional):**

| Field | Description |
|---|---|
| `active_report_hz` | Enable autonomous motor state push at this rate. Overrides the global `robstride/damiao_active_report_hz`. |
| `enable_disable_service_name` | Full service path for enabling/disabling motors. Defaults to `/{node_name}/enable_motor`. |

**Per-motor fields (all required):**

| Field | Description |
|---|---|
| `cmd_topic_name` | ROS topic to publish position commands to |
| `feedback_topic_name` | ROS topic to subscribe for motor state |
| `motor_type` | `robstride` or `damiao` |

**Example:**

```toml
[left_arm]
# active_report_hz            = 30.0
# enable_disable_service_name = "/left_arm/enable_motor"

[left_arm.SpL]
cmd_topic_name      = "/left_arm/cmd_position_pp"
feedback_topic_name = "/left_arm/motors/SpL/state"
motor_type          = "robstride"
```

### Currently configured motors

| Node section | Motors | Type | CAN bus |
|---|---|---|---|
| `left_arm` | SpL, SrL, SwL, EpL, WwL, WpL, WrL | robstride | can0 |
| `right_arm` | SpR, SrR, SwR, EpR, WwR, WpR, WrR | robstride | can2 |
| `base_and_neck` | BwC, BwR, BwL, BpC, BpR, BpL, NpC, NwC | robstride | can0 |
| `grippers` | AgL, AgR | damiao | can6 / can4 |

## Zenoh Bridge

### Feedback (ROS → Zenoh)

A timer fires at `joints_state_publish_hz`. For each Zenoh group, it reads the latest `MotorState` for each motor in the group and serialises them as a `feather_pb2.JointStatesArray` protobuf, then publishes to the group's feedback key.

Only motors that have received at least one state message are included in the payload — motors with no data are skipped silently.

### Commands (Zenoh → ROS)

One Zenoh subscriber is declared per group that has a `*_cmd_zenoh_key`. When a `feather_pb2.JointControlArray` payload arrives, the node:

1. Parses the protobuf.
2. Extracts entries with `cmd_type == POSITION`.
3. For each motor in the group that appears in the payload, checks the freeze state.
4. If not frozen, publishes the appropriate ROS command message to the motor's `cmd_topic_name`:
   - `cmd_position_pp` → `PositionPPCommand` (with pp defaults)
   - `cmd_position_csp` → `PositionCSPCommand` (with csp speed/current limits)
   - `cmd_position_pv` → `PositionPPCommand` (with damiao pv speed)

## Services

### `~/set_forwarding` (`std_srvs/srv/SetBool`)

Starts or stops forwarding Zenoh commands to ROS motor topics. **Forwarding is disabled by default** — Zenoh feedback (ROS → Zenoh) always runs, but incoming commands are dropped until this service is called with `data: true`.

```bash
# Start forwarding
ros2 service call /webxr_teleop_node/set_forwarding std_srvs/srv/SetBool "{data: true}"

# Stop forwarding
ros2 service call /webxr_teleop_node/set_forwarding std_srvs/srv/SetBool "{data: false}"
```

### `~/freeze` (`custom_interfaces/srv/Freeze`)

Prevents new commands from being published for a node or motor. The motor holds its last commanded position.

| Field | Type | Description |
|---|---|---|
| `name` | string | Node name or motor name. Empty string applies to all nodes. |
| `freeze` | bool | `true` to freeze, `false` to unfreeze |

**Response:** `success`, `message`, `frozen` (current state)

```bash
# Freeze a node
ros2 service call /webxr_teleop_node/freeze custom_interfaces/srv/Freeze "{name: 'left_arm', freeze: true}"

# Freeze a single motor
ros2 service call /webxr_teleop_node/freeze custom_interfaces/srv/Freeze "{name: 'SpL', freeze: true}"

# Unfreeze all
ros2 service call /webxr_teleop_node/freeze custom_interfaces/srv/Freeze "{name: '', freeze: false}"
```

### `~/enable_motors` (`custom_interfaces/srv/EnableMotors`)

Enables or disables motors by forwarding to the hardware node's `enable_motor` service.

| Field | Type | Description |
|---|---|---|
| `name` | string | Node name or motor name. Empty string applies to all motors. |
| `enable` | bool | `true` to enable, `false` to disable |
| `clear_fault` | bool | Clear latched faults when disabling |

**Response:** `success`, `message`

```bash
# Enable all motors
ros2 service call /webxr_teleop_node/enable_motors custom_interfaces/srv/EnableMotors "{name: '', enable: true, clear_fault: false}"

# Disable a single node
ros2 service call /webxr_teleop_node/enable_motors custom_interfaces/srv/EnableMotors "{name: 'left_arm', enable: false, clear_fault: false}"
```

## Startup sequence

1. Config is loaded and validated (fatal exit on missing required fields).
2. ROS subscriptions and publishers are created for all motors.
3. Zenoh session is opened.
4. Zenoh command subscribers are declared for each group with a cmd key.
5. Feedback timer starts at `joints_state_publish_hz`.
6. A one-shot timer fires after 1 second to call `_setup_once`:
   - Calls `set_active_report` on each node that has `active_report_hz` configured.
   - Calls `set_run_mode` per motor to put it in the configured control mode (`robstride_motor_mode` / `damiao_motor_mode`).

## Shutdown

On `Ctrl-C`, the node:
1. Disables active reporting on all hardware nodes that had it enabled.
2. Undeclares all Zenoh command subscribers.
3. Closes the Zenoh session.
