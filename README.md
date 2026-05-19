# RobStride Robot Workspace

ROS 2 workspace for a dual-arm robot (Featherbot) built on **RobStride RS01–RS05** quasi-direct-drive motors. Covers the full stack: raw CAN driver → ROS 2 motor node → arm mirroring → trajectory record/replay.

---

## Table of Contents

1. [Repository Layout](#1-repository-layout)
2. [Docker Quickstart](#2-docker-quickstart)
3. [Build (Native)](#3-build-native)
4. [CAN Interface Setup](#4-can-interface-setup)
5. [Package Overview](#5-package-overview)
6. [Hardware Overview](#6-hardware-overview)
7. [RS0x Private CAN Protocol](#7-rs0x-private-can-protocol)
   - [Frame Format](#71-frame-format)
   - [Communication Types](#72-communication-types)
   - [Operation Control Mode (type 1)](#73-operation-control-mode-type-1)
   - [Feedback Frame (type 2)](#74-feedback-frame-type-2)
   - [Fault Feedback Frame (type 21)](#75-fault-feedback-frame-type-21)
   - [Parameter Read / Write (types 17 & 18)](#76-parameter-read--write-types-17--18)
8. [robstride_p — Motor Node](#8-robstride_p--motor-node)
   - [config.toml](#81-configtoml)
   - [Joint Limits & Homing](#82-joint-limits--homing)
   - [Command Flow](#83-command-flow)
   - [Topics](#84-topics)
   - [Services](#85-services)
   - [Actions](#86-actions)
   - [Fault Detection & Recovery](#87-fault-detection--recovery)
9. [mimic — Arm Mirroring](#9-mimic--arm-mirroring)
10. [trajectory_tracker — Record & Replay](#10-trajectory_tracker--record--replay)
11. [custom_interfaces](#11-custom_interfaces)
12. [Software Architecture](#12-software-architecture)
13. [Standalone Usage (no ROS 2)](#13-standalone-usage-no-ros-2)

---

## 1. Repository Layout

```
claude/
├── docker/
│   ├── Dockerfile          # CUDA 12.6 + ROS 2 Jazzy + python-can + PyTorch
│   └── compose.yaml
├── README.md               # this file
└── ros_ws/
    └── src/
        ├── custom_interfaces/      # shared ROS 2 msgs / srvs / actions
        ├── robstride_p/            # CAN driver + ROS 2 motor node
        ├── mimic/                  # real-time arm mirroring
        ├── trajectory_tracker/     # trajectory record, replay, pose capture
        ├── teleop/                 # teleoperation
        ├── remote_joystick/        # remote joystick input
        ├── featherbot_bringup/     # robot bringup launch files
        ├── featherbot_description/ # URDF / robot description
        ├── featherbot_ros2_control/# ros2_control integration
        └── mimic_sim/              # simulation for mimic
```

---

## 2. Docker Quickstart

The Docker image bundles **CUDA 12.6**, **ROS 2 Jazzy**, **python-can**, **PyTorch (cu126)**, and all system tools (neovim, tmux, ranger, fzf).

```bash
# Build (run from the claude/ directory)
docker compose -f docker/compose.yaml build

# Start container
docker compose -f docker/compose.yaml up -d

# Shell into container
docker compose -f docker/compose.yaml exec anuj_exp bash

# Inside container — workspace is already sourced in .bashrc
cd /ros_ws && colcon build
```

**Key compose settings:**

| Setting | Value |
|---|---|
| `network_mode` | `host` — direct access to CAN interfaces and ROS 2 DDS |
| `privileged: true` | required for SocketCAN |
| `runtime: nvidia` | NVIDIA GPU passthrough |
| `/dev` volume | full device access (CAN adapters, etc.) |
| `ROS_DOMAIN_ID` | `0` |

**Build args** (override CUDA base):
```bash
docker build --build-arg CUDA_IMAGE=nvidia/cuda:12.4.1-cudnn-runtime-ubuntu24.04 -f docker/Dockerfile .
```

**User matching** — the Dockerfile accepts `USER_ID` / `GROUP_ID` build args (default 1000) so the in-container user owns workspace files without permission issues.

---

## 3. Build (Native)

```bash
# Prerequisites
sudo apt install ros-jazzy-ros-base ros-jazzy-sensor-msgs ros-jazzy-std-srvs \
                 ros-jazzy-action-msgs ros-jazzy-rosidl-default-generators \
                 python3-colcon-common-extensions can-utils iproute2
pip install python-can

# Build
cd claude/ros_ws
colcon build
source install/setup.bash

# Or build a single package
colcon build --packages-select robstride_p
```

---

## 4. CAN Interface Setup

```bash
# Bring up each CAN interface (repeat per interface)
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up

# Verify
ip link show can0
candump can0   # live frame monitor
```

The robot uses three CAN buses:

| Interface | Motors |
|---|---|
| `can0` | `base_and_neck` — torso and neck motors |
| `can1` | `right_arm` — SpR, SrR, SwR, EpR, WwR, WpR, WrR |
| `can2` | `left_arm` — SpL, SrL, SwL, EpL, WwL, WpL, WrL |

---

## 5. Package Overview

| Package | Description |
|---|---|
| `robstride_p` | CAN driver (comms.py, motor_base.py, rs01–rs05) + ROS 2 motor node. Runs one node per arm / body segment. |
| `mimic` | Mirrors one arm onto the other in real time with per-joint transforms and live direction switching. |
| `trajectory_tracker` | Records arm states to CSV and replays them with full cross-arm transform support. |
| `custom_interfaces` | Shared ROS 2 messages, services, and actions used by all packages above. |
| `teleop` | Keyboard / controller teleoperation. |
| `remote_joystick` | Remote joystick input forwarding. |
| `featherbot_bringup` | Top-level launch files. |
| `featherbot_description` | URDF and mesh assets. |
| `featherbot_ros2_control` | ros2_control hardware interface. |
| `mimic_sim` | Gazebo / simulation support for the mimic node. |

---

## 6. Hardware Overview

| Model | Peak torque | Output speed | Max current | Weight |
|---|---|---|---|---|
| RS01 | 17 N·m | 315 rpm (33 rad/s) | 23 Apk | 380 g |
| RS02 | 17 N·m | 410 rpm (44 rad/s) | 23 Apk | 380 g |
| RS03 | 60 N·m | 200 rpm (21 rad/s) | 43 Apk | 880 g |
| RS04 | 120 N·m | 200 rpm (21 rad/s) | 90 Apk | 1420 g |
| RS05 | 6 N·m | 480 rpm (50 rad/s) | 11 Apk | 191 g |

All motors share the same private CAN 2.0 protocol (1 Mbps, 29-bit extended frame) and connector pinout: **VBAT+**, **GND**, **CAN-H**, **CAN-L**.

**Wiring:**
- Power: 24–60 V DC (48 V nominal for RS02–RS05, 36 V for RS01).
- CAN: twisted-pair CAN-H / CAN-L with 120 Ω termination at both ends of the bus.
- Multiple motors daisy-chain on the same bus; each is addressed by its **motor CAN ID** (0–127).

**Robot motor assignment:**

| Joint | Left | Right | Model | CAN bus |
|---|---|---|---|---|
| Shoulder pitch | SpL (id 32) | SpR (id 16) | RS04 | can2 / can1 |
| Shoulder roll | SrL (id 33) | SrR (id 17) | RS04 | can2 / can1 |
| Shoulder wrist-yaw | SwL (id 34) | SwR (id 18) | RS03 | can2 / can1 |
| Elbow pitch | EpL (id 35) | EpR (id 19) | RS03 | can2 / can1 |
| Wrist wrist-yaw | WwL (id 36) | WwR (id 20) | RS02 | can2 / can1 |
| Wrist pitch | WpL (id 37) | WpR (id 21) | RS02 | can2 / can1 |
| Wrist roll | WrL (id 38) | WrR (id 22) | RS02 | can2 / can1 |

---

## 7. RS0x Private CAN Protocol

### 7.1 Frame Format

Every frame uses a **29-bit extended arbitration ID**:

```
 Bit 28       Bit 24   Bit 23        Bit 8   Bit 7       Bit 0
┌────────────────────┬───────────────────────┬──────────────────┐
│  Communication     │     Data Area 2       │  Destination     │
│  type  (5 bits)    │     (16 bits)         │  motor CAN ID    │
│                    │                       │  (8 bits)        │
└────────────────────┴───────────────────────┴──────────────────┘
```

```python
arb_id = ((comm_type & 0x1F) << 24) | ((data_area2 & 0xFFFF) << 8) | motor_id
```

### 7.2 Communication Types

| Type | Direction | Purpose |
|---|---|---|
| 0 | Host → Motor | Get device ID (64-bit MCU UID) |
| 1 | Host → Motor | **Operation control** (angle + velocity + Kp + Kd + torque ff) |
| 2 | Motor → Host | **Motor feedback** (position, velocity, torque, temperature, fault) |
| 3 | Host → Motor | Enable motor |
| 4 | Host → Motor | Stop motor |
| 6 | Host → Motor | Set mechanical zero |
| 7 | Host → Motor | Change motor CAN ID |
| 17 | Host ↔ Motor | Single parameter read |
| 18 | Host → Motor | Single parameter write (volatile) |
| 21 | Motor → Host | **Fault feedback** (extended fault + warning bitmasks) |
| 22 | Host → Motor | Save parameters to flash |
| 24 | Host → Motor | Enable / disable active reporting |

### 7.3 Operation Control Mode (type 1)

Control law: `t_ref = Kd × (v_set − v_actual) + Kp × (p_set − p_actual) + t_ff`

| Field | Physical range | 16-bit encoding |
|---|---|---|
| Position | −4π … +4π rad | 0 … 65535 |
| Velocity | −44 … +44 rad/s | 0 … 65535 |
| Kp | 0 … 500 | 0 … 65535 |
| Kd | 0 … 5 | 0 … 65535 |
| Torque | −17 … +17 N·m | 0 … 65535 |

### 7.4 Feedback Frame (type 2)

**29-bit ID layout:**

```
bits 28–24  0x02
bits 23–22  mode   (0=reset, 1=calibration, 2=run)
bits 21–16  fault bitmask (6-bit subset)
bits 15–8   source motor CAN ID
bits  7–0   host CAN ID
```

**8-byte payload:**

| Bytes | Content | Physical range |
|---|---|---|
| 0–1 | position | −4π … +4π rad |
| 2–3 | velocity | −V_MAX … +V_MAX rad/s |
| 4–5 | torque | −T_MAX … +T_MAX N·m |
| 6–7 | temperature × 10 | °C |

### 7.5 Fault Feedback Frame (type 21)

| Bytes | Content |
|---|---|
| 0–3 | Fault bitmask (uint32 LE) |
| 4–7 | Warning bitmask (uint32 LE) |

**Fault bits:**

| Bit | Name | Meaning |
|---|---|---|
| 0 | `OVER_TEMP` | Winding temp > 135 °C |
| 1 | `DRIVER_IC` | Driver chip fault |
| 2 | `UNDERVOLTAGE` | Bus voltage < 12 V |
| 3 | `OVERVOLTAGE` | Bus voltage > 60 V |
| 4 | `B_PHASE_OC` | B-phase overcurrent |
| 5 | `C_PHASE_OC` | C-phase overcurrent |
| 7 | `ENCODER_UNCAL` | Encoder not calibrated |
| 8 | `HW_ID_FAULT` | Hardware ID mismatch |
| 9 | `POS_INIT_FAULT` | Position init error |
| 14 | `STALL_OVERLOAD` | Stall I²t overload |
| 16 | `A_PHASE_OC` | A-phase overcurrent |

**Warning bits:**

| Bit | Meaning |
|---|---|
| 0 | Over-temperature warning (approaching 135 °C) |

### 7.6 Parameter Read / Write (types 17 & 18)

Common parameter indices (decimal):

| Index (hex) | Index (dec) | Name | Type | Description |
|---|---|---|---|---|
| 0x7005 | 28677 | run_mode | uint8 | 0=OPERATION 1=POS_PP 2=VELOCITY 3=CURRENT 5=POS_CSP |
| 0x7006 | 28678 | iq_ref | float | Current mode Iq command (A) |
| 0x700A | 28682 | spd_ref | float | Velocity command (rad/s) |
| 0x700B | 28683 | limit_torque | float | Torque limit (N·m) |
| 0x7016 | 28694 | loc_ref | float | Position command (rad) |
| 0x7017 | 28695 | limit_spd | float | CSP speed limit (rad/s) |
| 0x7018 | 28696 | limit_cur | float | Current limit (A) |
| 0x7019 | 28697 | mechPos | float | Read-only: mechanical angle (rad) |
| 0x701B | 28699 | mechVel | float | Read-only: load velocity (rad/s) |
| 0x701C | 28700 | VBUS | float | Read-only: bus voltage (V) |
| 0x200A | 8202 | CAN_ID | uint8 | Motor CAN ID (0-127) |
| 0x2009 | 8201 | motor_baud | uint8 | Baud rate flag (1=1M 2=500K 3=250K 4=125K) |

---

## 8. robstride_p — Motor Node

Located at `ros_ws/src/robstride_p/`. Runs one ROS 2 node per motor group (left arm, right arm, base/neck).

```bash
ros2 launch robstride_p motors.launch.py config:=left_arm.toml
ros2 launch robstride_p motors.launch.py config:=right_arm.toml
ros2 launch robstride_p motors.launch.py config:=base_and_neck.toml
```

### 8.1 config.toml

```toml
[defaults]
node_name                   = "left_arm"
use_node_name_as_topic_base = true
master_id                   = 253
channel                     = "can2"
bustype                     = "socketcan"
bitrate                     = 1000000
rx_timeout                  = 0.05
active_report_interval_ms   = 10
update_rate_hz              = 30.0

[SpL]
type              = "RS04"
motor_id          = 32
joint_limit_min   = -2.6114   # motor frame (rad)
joint_limit_max   =  2.6210   # motor frame (rad)
motor_homing_pos  =  0.0047   # motor frame (rad)
max_torque        = 120.0
max_current       =  90.0
```

Each motor section may override any `[defaults]` field. Motors sharing the same `(channel, bustype, bitrate)` reuse one `CANComms` instance.

`use_node_name_as_topic_base = true` → topics under `/{node_name}/` (e.g. `/left_arm/joint_states`).

### 8.2 Joint Limits & Homing

All positions in this codebase are in a single **motor frame** — the absolute mechanical angle reported by the motor encoder.

| Config field | Frame | Meaning |
|---|---|---|
| `motor_homing_pos` | motor frame | Absolute motor angle when the arm is at its home pose |
| `joint_limit_min` | motor frame | Lower position bound |
| `joint_limit_max` | motor frame | Upper position bound |
| `fb.position` / `MECH_POS` | motor frame | Motor's current absolute angle |

**Commands** arrive relative to the homing point (zero = home). The node converts them to motor frame before use:

```
motor_frame_position = command_relative_to_home + motor_homing_pos
```

**`calibrate_joint_limits`** runs at startup for every motor that has limits configured. It reads `MECH_POS` from the motor and checks whether it falls within `[joint_limit_min, joint_limit_max]`. If not, all three values (`min`, `max`, `motor_homing_pos`) are shifted by ±2π together to bring the current position inside the window:

```python
if mech_pos > joint_limit_max:
    joint_limit_max    += 2π
    joint_limit_min    += 2π
    motor_homing_pos   += 2π
elif mech_pos < joint_limit_min:
    joint_limit_max    -= 2π
    joint_limit_min    -= 2π
    motor_homing_pos   -= 2π
```

This handles the fact that the encoder wraps across power cycles and the stored limits may be offset by one full rotation.

### 8.3 Command Flow

For position commands (PP and CSP):

1. **Transform** relative command to motor frame: `motor_pos = cmd + motor_homing_pos`
2. **Check** `motor_pos` against motor-frame limits (`joint_limit_min` / `joint_limit_max`)
3. **Reject** (log error, drop command) if out of bounds
4. **Send** `motor_pos` to hardware

For velocity commands, `fb.position` (motor frame) is compared directly against the motor-frame limits. Velocity is zeroed if the motor is at a boundary and moving toward it.

### 8.4 Topics

| Topic | Type | Description |
|---|---|---|
| `~/joint_states` | `sensor_msgs/JointState` | All motors combined, at `update_rate_hz` |
| `~/motors/{name}/state` | `custom_interfaces/MotorState` | Per-motor full state |
| `~/motors/{name}/fault` | `custom_interfaces/MotorFault` | Per-motor fault — published **only on change** |
| `~/motors/{name}/cmd_position_pp` | `custom_interfaces/PositionPPCommand` | PP position command |
| `~/motors/{name}/cmd_position_csp` | `custom_interfaces/PositionCSPCommand` | CSP position command |
| `~/motors/{name}/cmd_velocity` | `custom_interfaces/VelocityCommand` | Velocity command |
| `~/motors/{name}/cmd_current` | `custom_interfaces/CurrentCommand` | Current command |

### 8.5 Services

Services that accept a `name` field support `"all"` to apply to every motor simultaneously (except `read_param` and `get_can_config`, which return a single value).

| Service | Type | `"all"` | Description |
|---|---|---|---|
| `~/enable_motor` | `EnableMotor` | ✅ | Enable or stop motors; `clear_fault=true` clears latched faults |
| `~/set_run_mode` | `SetRunMode` | ✅ | Switch control mode (motor must be stopped first) |
| `~/set_zero_position` | `SetZeroPosition` | ✅ | Set current position as mechanical zero |
| `~/read_param` | `ReadParam` | ❌ | Read a parameter by function code index |
| `~/write_param` | `WriteParam` | ✅ | Write a parameter; `persist=true` saves to flash |
| `~/help` | `Help` | — | List all parameter codes with names and types |
| `~/get_can_config` | `GetCanConfig` | ❌ | Read CAN ID and baud rate from motor firmware |
| `~/set_can_config` | `SetCanConfig` | ✅ | Change CAN ID (immediate) and/or baud rate (re-power) |
| `~/set_active_report` | `SetActiveReport` | ✅ | Enable or disable autonomous state reporting |
| `~/homing` | `std_srvs/Trigger` | — | Stop all motors then command position 0.0 via CSP |
| `~/stop_all` | `std_srvs/Trigger` | — | Immediately disable every motor |

**Common examples:**

```bash
# Enable / disable
ros2 service call /left_arm/enable_motor custom_interfaces/srv/EnableMotor \
  '{name: SpL, enable: true}'

# Enable all motors on an arm
ros2 service call /right_arm/enable_motor custom_interfaces/srv/EnableMotor \
  '{name: all, enable: true}'

# Set velocity mode on all motors
ros2 service call /left_arm/set_run_mode custom_interfaces/srv/SetRunMode \
  '{name: all, mode: 2}'

# Read current limit (0x7018 = 28696)
ros2 service call /right_arm/read_param custom_interfaces/srv/ReadParam \
  '{name: WwR, index: 28696}'

# Read mechanical position (0x7019 = 28697)
ros2 service call /right_arm/read_param custom_interfaces/srv/ReadParam \
  '{name: WwR, index: 28697}'

# Enable active reporting at 30 Hz
ros2 service call /left_arm/set_active_report custom_interfaces/srv/SetActiveReport \
  '{name: all, enable: true, hz: 30.0}'

# Emergency stop
ros2 service call /right_arm/stop_all std_srvs/srv/Trigger
```

### 8.6 Actions

| Action | Type | Description |
|---|---|---|
| `~/move_to_position` | `MoveToPosition` | Move to target position; succeeds when within `tolerance` rad |
| `~/set_velocity` | `SetVelocity` | Run at velocity for `duration` seconds (0 = until cancelled) |

```bash
ros2 action send_goal /left_arm/move_to_position \
  custom_interfaces/action/MoveToPosition \
  '{name: SpL, target_position: 0.5, speed_limit: 2.0, tolerance: 0.05, timeout: 10.0}'
```

Motor holds the target position (CSP, stays enabled) on success.

### 8.7 Fault Detection & Recovery

**Check:** `~/motors/{name}/fault` publishes a `MotorFault` message whenever the fault bitmask changes. `MotorState.fault` also carries the raw bitmask on every state message.

**Recover:**
```bash
# 1. Disable and clear fault
ros2 service call /right_arm/enable_motor custom_interfaces/srv/EnableMotor \
  "{name: WwR, enable: false, clear_fault: true}"

# 2. Re-enable
ros2 service call /right_arm/enable_motor custom_interfaces/srv/EnableMotor \
  "{name: WwR, enable: true, clear_fault: false}"
```

`clear_fault=true` sends a Type-4 CAN frame with `payload[0] = 0x01`, which clears latched fault flags in the motor firmware.

**Fault types and likely causes:**

| Fault | Likely cause |
|---|---|
| `OVER_TEMP` | Too much current/load, or poor cooling. Clears when temp drops. |
| `UNDERVOLTAGE` | Supply sag, high cable resistance, or too many motors on one supply. |
| `OVERVOLTAGE` | Regenerative braking energy with no sink. |
| `A/B/C_PHASE_OC` | Aggressive acceleration, short circuit. May need power cycle. |
| `STALL_OVERLOAD` | Mechanical blockage or sustained torque beyond I²t limit. |
| `ENCODER_UNCAL` | First power-on after factory reset — run calibration. |
| `DRIVER_IC` | Driver chip fault — may need power cycle. |

---

## 9. mimic — Arm Mirroring

See [`ros_ws/src/mimic/README.md`](ros_ws/src/mimic/README.md) for the full reference.

Mirrors one arm's positions onto the other in real time, applying per-joint transforms. Direction is runtime-switchable without restarting the node.

```bash
ros2 launch mimic mimic.launch.py
# Custom config
ros2 launch mimic mimic.launch.py config:=/path/to/mimic.toml
```

**Config summary (`mimic/config/mimic.toml`):**

```toml
mode             = "pp"
debug            = false
left_arm_node_prefix  = "/left_arm"
right_arm_node_prefix = "/right_arm"
target_node      = "right_arm"   # "right_arm" → left mirrors to right; "left_arm" → right mirrors to left
motors = ["Sp", "Sr", "Sw", "Ep", "Ww", "Wp", "Wr"]

[transform_map]          # applied when target_node = "right_arm" (left → right)
Sp = "negate"

[inverse_transform_map]  # applied when target_node = "left_arm" (right → left)
Sp = "negate"
```

**Services:**

| Service | Type | Description |
|---|---|---|
| `~/switch_target` | `SetMimicTarget` | Switch which arm is the target (`"left_arm"` or `"right_arm"`) without restart |
| `~/set_mode` | `SetMimicMode` | Change control mode (`"pp"` or `"csp"`) |
| `~/set_params` | `SetMimicParams` | Update PP/CSP motion parameters at runtime |
| `~/enable_motors` | `EnableMimicMotors` | Enable or disable target motors; supports `clear_fault` |

```bash
# Switch mimic direction to left arm as target
ros2 service call /mimic_node/switch_target \
  custom_interfaces/srv/SetMimicTarget "{target: 'left_arm'}"
```

**`debug = true`** — commands go to `~/mimic/debug/motors/{name}/cmd_*` instead of real motor topics. Motors are not enabled in debug mode.

---

## 10. trajectory_tracker — Record & Replay

See [`ros_ws/src/trajectory_tracker/README.md`](ros_ws/src/trajectory_tracker/README.md) for the full reference.

Records arm motor states to CSV and replays them, with full cross-arm direction matrix and per-joint transforms.

```bash
ros2 launch trajectory_tracker trajectory_tracker.launch.py
```

**Key config fields (`trajectory_tracker/config/config.toml`):**

```toml
left_arm_motors   = ["SpL", "SrL", "SwL", "EpL", "WwL", "WpL", "WrL"]
right_arm_motors  = ["SpR", "SrR", "SwR", "EpR", "WwR", "WpR", "WrR"]
replay_motor_mode = "pp"

[motor_map]      # determines recording arm (keys) and replay arm (values)
SpL = "SpR"      # left arm records → right arm replays

[transform_map]          # left → right; keys are base names (no L/R)
Sp = "negate"

[inverse_transform_map]  # right → left
Sp = "negate"
```

### Record

```bash
# Record left arm only
ros2 action send_goal /trajectory_tracker/record_trajectory \
  custom_interfaces/action/RecordTrajectory \
  "{trajectory_name: 'demo', left_arm_source: true, right_arm_source: false}"

# Stop recording (or cancel the action)
ros2 service call /trajectory_tracker/stop_trajectory_recording \
  custom_interfaces/srv/StopTrajectoryRecording
```

### Replay direction matrix

| `replay_left_arm` | `replay_right_arm` | CSV recorded from | Result |
|---|---|---|---|
| true | false | left | left→left, passthrough |
| false | true | left | left→right, forward transform |
| true | false | right | right→left, inverse transform |
| false | true | right | right→right, passthrough |
| true | true | left | left→left + left→right simultaneously |
| true | true | right | right→right + right→left simultaneously |
| true | true | both | left→left + right→right simultaneously |

```bash
# Replay left arm recording onto right arm
ros2 action send_goal /trajectory_tracker/replay_trajectory \
  custom_interfaces/action/ReplayTrajectory \
  "{trajectory_name: 'demo', replay_hz: 0.0, target_mode: 'pp', \
    replay_left_arm: false, replay_right_arm: true, \
    step_through: false, step_pct: 0.0, \
    pp_speed: 0.0, pp_acceleration: 0.0, pp_deceleration: 0.0, pp_torque_limit: 0.0, \
    csp_speed_limit: 0.0, csp_current_limit: 0.0}"
```

Motors are left **enabled and holding last pose** on all exit paths (normal completion, fault, cancel).

### Services

| Service | Type | Description |
|---|---|---|
| `~/pause_resume_replay` | `Trigger` | Toggle pause/resume of active replay |
| `~/stop_trajectory_recording` | `StopTrajectoryRecording` | Stop active recording |
| `~/record_arm_pose` | `RecordArmPose` | Snapshot current arm state to CSV |
| `~/set_arm_pose` | `SetArmPose` | Load pose CSV and send commands to an arm |
| `~/capture_homing_pose` | `CaptureHomingPose` | Read current positions into a homing TOML file |
| `~/trim_trajectory` | `TrimTrajectory` | Remove timestamp ranges from a trajectory CSV |

### Actions

| Action | Type | Description |
|---|---|---|
| `~/record_trajectory` | `RecordTrajectory` | Record to CSV until cancelled or stop service called |
| `~/replay_trajectory` | `ReplayTrajectory` | Replay CSV with direction matrix and transforms |
| `~/homing` | `Homing` | Move motors to positions defined in a homing TOML |
| `~/simulate_trajectory` | `SimulateTrajectory` | Publish CSV frames as `JointCommand` without motor commands |

---

## 11. custom_interfaces

Package: `ros_ws/src/custom_interfaces/`

**Messages:**

| Message | Key fields |
|---|---|
| `MotorState` | `name`, `position`, `velocity`, `torque`, `temperature`, `mode`, `fault`, `enabled` |
| `MotorFault` | `name`, `fault_code`, `warning_code`, decoded per-bit booleans (`over_temp`, `driver_ic`, `undervoltage`, `overvoltage`, `b_phase_oc`, `c_phase_oc`, `encoder_uncal`, `hw_id_fault`, `pos_init_fault`, `stall_overload`, `a_phase_oc`, `over_temp_warning`) |
| `PositionPPCommand` | `name`, `position`, `speed`, `acceleration`, `deceleration`, `torque_limit` |
| `PositionCSPCommand` | `name`, `position`, `speed_limit`, `current_limit` |
| `VelocityCommand` | `name`, `velocity`, `current_limit` |
| `JointCommand` | Joint-level command for simulation / visualisation |

**Services:**

| Service | Description |
|---|---|
| `EnableMotor` | Enable / disable with optional `clear_fault` |
| `SetRunMode` | Set control mode |
| `SetZeroPosition` | Set mechanical zero |
| `ReadParam` | Read parameter by index |
| `WriteParam` | Write parameter with optional flash persist |
| `Help` | List parameter codes |
| `GetCanConfig` / `SetCanConfig` | CAN ID and baud rate management |
| `SetActiveReport` | Enable / disable autonomous state reporting |
| `SetMimicTarget` | Switch mimic direction (`left_arm` / `right_arm`) |
| `SetMimicMode` | Change mimic control mode |
| `SetMimicParams` | Update mimic PP/CSP parameters |
| `EnableMimicMotors` | Enable / disable mimic target motors |
| `StopTrajectoryRecording` | Stop active recording |
| `RecordArmPose` / `SetArmPose` | Pose capture and playback |
| `CaptureHomingPose` | Write homing positions to TOML |
| `TrimTrajectory` | Edit trajectory CSV in-place |

**Actions:**

| Action | Description |
|---|---|
| `MoveToPosition` | Move to position with tolerance |
| `SetVelocity` | Run at velocity for duration |
| `RecordTrajectory` | Record arm states to CSV |
| `ReplayTrajectory` | Replay CSV with direction matrix |
| `Homing` | Move to stored home positions |
| `SimulateTrajectory` | Publish CSV frames without motor commands |

---

## 12. Software Architecture

```
┌───────────────────────────────────────────────────────────────────┐
│               mimic_node.py                                       │
│  Subscribes source arm state → applies transforms → publishes    │
│  position commands to target arm at op_hz or on every state msg  │
└───────────────────────────────────────────────────────────────────┘
         ▲ state topics                  │ cmd_position_pp/csp topics
         │                               ▼
┌─────────────────────────────────────────────────────────────────────────┐
│               trajectory_tracker_node.py                                │
│  record_trajectory: CSV ← motor states                                 │
│  replay_trajectory: CSV → position commands (with cross-arm transforms)│
└─────────────────────────────────────────────────────────────────────────┘
         ▲ state topics                  │ cmd_position_pp/csp topics
         │                               ▼
┌──────────────────────────────────────────────────────────────────┐
│               motor_node.py  (one per arm / body segment)        │
│  calibrate_joint_limits → check limits in motor frame            │
│  transform cmd (+ homing_pos) → check → send to hardware         │
└─────────────────────┬────────────────────────────────────────────┘
                      │ instantiates
┌─────────────────────▼────────────────────────────────────────────┐
│            Motor classes  rs01.py … rs05.py                      │
└─────────────────────┬────────────────────────────────────────────┘
                      │ inherits
┌─────────────────────▼────────────────────────────────────────────┐
│           RobStrideMotorBase  (motor_base.py)                    │
│  Full private protocol, all control modes, parameter read/write, │
│  active reporting, fault handling                                │
└─────────────────────┬────────────────────────────────────────────┘
                      │ uses
┌─────────────────────▼────────────────────────────────────────────┐
│                  CANComms  (comms.py)                            │
│  python-can wrapper, SocketCAN, hardware filters,                │
│  Notifier thread + per-motor callback dispatch                   │
└─────────────────────┬────────────────────────────────────────────┘
                      │
              SocketCAN  (can0 / can1 / can2)
                      │
    ┌─────────────────┴──────────────────────────────────┐
    │  can0: base/neck    can1: right arm    can2: left arm
    └────────────────────────────────────────────────────┘
```

---

## 13. Standalone Usage (no ROS 2)

All driver files are plain Python — no ROS 2 required.

```python
from comms import CANComms
from rs04 import RS04
from rs02 import RS02
from motor_base import RunMode

with CANComms("can1") as bus:
    bus.start_listener()

    hip   = RS04(motor_id=16, comms=bus)
    wrist = RS02(motor_id=20, comms=bus)

    hip.enable_active_report(enable=True, interval_ms=10)

    hip.set_run_mode(RunMode.POSITION_CSP)
    hip.enable()
    hip.set_position_csp(0.5, speed_limit_rad_s=3.0)

    import time
    for _ in range(100):
        print(hip.feedback)
        time.sleep(0.01)

    hip.disable()
```

**Reading parameters:**

```python
mech_pos = motor.read_param_float(ParamIndex.MECH_POS)   # absolute motor-frame angle
cur_limit = motor.read_param_float(ParamIndex.LIMIT_CUR)  # current limit (A)
```

**Saving parameters to flash:**

```python
motor.write_param_float(ParamIndex.LIMIT_CUR, 15.0)
motor.save_params()   # type 22 — survives power-off
```

**Check and clear a fault:**

```python
fb = motor.feedback
if fb.fault != 0:
    print(f"Fault bitmask: {fb.fault:#010x}")
    motor.disable(clear_fault=True)   # Type-4 frame with payload[0]=0x01
    time.sleep(0.1)
    motor.enable()
```
