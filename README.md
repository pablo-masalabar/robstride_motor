# RobStride Motor Driver

Python CAN driver and ROS 2 node for the **RobStride RS01–RS05** quasi-direct-drive
integrated motor modules.

---

## Table of Contents

1. [Hardware Overview](#1-hardware-overview)
2. [CAN Bus Primer](#2-can-bus-primer)
3. [RS0x Private CAN Protocol](#3-rs0x-private-can-protocol)
   - [Frame Format](#31-frame-format)
   - [Communication Types](#32-communication-types)
   - [Operation Control Mode (type 1)](#33-operation-control-mode-type-1)
   - [Feedback Frame (type 2)](#34-feedback-frame-type-2)
   - [Fault Feedback Frame (type 21)](#35-fault-feedback-frame-type-21)
   - [Parameter Read / Write (types 17 & 18)](#36-parameter-read--write-types-17--18)
4. [Software Architecture](#4-software-architecture)
5. [comms.py — CAN Transport Layer](#5-commspy--can-transport-layer)
6. [motor_base.py — Protocol Implementation](#6-motor_basepy--protocol-implementation)
7. [Motor Classes RS01–RS05](#7-motor-classes-rs01rs05)
8. [ROS 2 Package — robstride_p](#8-ros-2-package--robstride_p)
   - [config.toml](#81-configtoml)
   - [Topics](#82-topics)
   - [Services](#83-services)
   - [Actions](#84-actions)
   - [Custom Interfaces](#85-custom-interfaces)
9. [Build & Run](#9-build--run)
10. [Standalone Usage (no ROS 2)](#10-standalone-usage-no-ros-2)

---

## 1. Hardware Overview

| Model | Peak torque | Output no-load speed | Max current | Weight |
|-------|------------|---------------------|-------------|--------|
| RS01  | 17 N·m     | 315 rpm  (33 rad/s) | 23 Apk      | 380 g  |
| RS02  | 17 N·m     | 410 rpm  (44 rad/s) | 23 Apk      | 380 g  |
| RS03  | 60 N·m     | 200 rpm  (21 rad/s) | 43 Apk      | 880 g  |
| RS04  | 120 N·m    | 200 rpm  (21 rad/s) | 90 Apk      | 1420 g |
| RS05  | 6 N·m      | 480 rpm  (50 rad/s) | 11 Apk      | 191 g  |

All motors share the same private CAN 2.0 protocol (1 Mbps, 29-bit extended
frame) and the same driver board interface: **VBAT+**, **GND**, **CAN-H**, **CAN-L**.

**Wiring:**
- Power: 24–60 V DC on VBAT+ / GND (48 V nominal for RS02–RS05, 36 V for RS01).
- CAN: twisted-pair CAN-H / CAN-L with 120 Ω termination resistors at both ends of the bus.
- Multiple motors daisy-chain on the same CAN bus; each is addressed by its **motor CAN ID** (0–127).

---

## 2. CAN Bus Primer

CAN (Controller Area Network) is a differential serial bus designed for noisy
environments.  Key properties relevant to this driver:

| Property | Value |
|----------|-------|
| Baud rate | 1 Mbps |
| Frame format | CAN 2.0B **extended** (29-bit arbitration ID) |
| Data payload | 8 bytes per frame |
| Linux interface | SocketCAN (`can0`, `can1`, …) |

**Bring up a SocketCAN interface:**

```bash
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
```

**python-can** is used as the CAN library abstraction:

```bash
pip install python-can
```

---

## 3. RS0x Private CAN Protocol

The RS0x motors support three protocols: **Private** (default), CANopen, and MIT.
This driver uses the **Private protocol exclusively**.

### 3.1 Frame Format

Every frame uses a **29-bit extended arbitration ID** structured as:

```
 Bit 28       Bit 24   Bit 23        Bit 8   Bit 7       Bit 0
┌────────────────────┬───────────────────────┬──────────────────┐
│  Communication     │     Data Area 2       │  Destination     │
│  type  (5 bits)    │     (16 bits)         │  motor CAN ID    │
│                    │                       │  (8 bits)        │
└────────────────────┴───────────────────────┴──────────────────┘
```

The 8-byte data payload (Data Area 1) carries command arguments.

**ID construction in Python:**

```python
arb_id = ((comm_type & 0x1F) << 24) | ((data_area2 & 0xFFFF) << 8) | motor_id
```

### 3.2 Communication Types

| Type | Hex | Direction | Purpose |
|------|-----|-----------|---------|
| 0  | 0x00 | Host → Motor | Get device ID (64-bit MCU UID) |
| 1  | 0x01 | Host → Motor | **Operation control** (angle + velocity + Kp + Kd + torque ff) |
| 2  | 0x02 | Motor → Host | **Motor feedback** (position, velocity, torque, temperature, fault) |
| 3  | 0x03 | Host → Motor | Enable motor (enter run state) |
| 4  | 0x04 | Host → Motor | Stop motor |
| 6  | 0x06 | Host → Motor | Set mechanical zero |
| 7  | 0x07 | Host → Motor | Change motor CAN ID |
| 17 | 0x11 | Host ↔ Motor | Single parameter read (request + reply) |
| 18 | 0x12 | Host → Motor | Single parameter write (volatile) |
| 21 | 0x15 | Motor → Host | **Fault feedback** (extended fault + warning bitmasks) |
| 22 | 0x16 | Host → Motor | Save parameters to flash |
| 23 | 0x17 | Host → Motor | Change baud rate (re-power-on effect) |
| 24 | 0x18 | Host → Motor | Enable / disable active reporting |
| 25 | 0x19 | Host → Motor | Switch protocol (re-power-on effect) |

### 3.3 Operation Control Mode (type 1)

Implements the control law:

```
t_ref = Kd × (v_set − v_actual) + Kp × (p_set − p_actual) + t_ff
```

The torque feedforward (`t_ff`) is packed into **Data Area 2** of the 29-bit ID.
The 8-byte payload carries position, velocity, Kp, Kd — each encoded as a
16-bit unsigned integer over a physical range:

| Field    | Physical range     | 16-bit range |
|----------|--------------------|--------------|
| Position | −4π … +4π rad      | 0 … 65535 |
| Velocity | −44 … +44 rad/s    | 0 … 65535 |
| Kp       | 0 … 500            | 0 … 65535 |
| Kd       | 0 … 5              | 0 … 65535 |
| Torque   | −17 … +17 N·m      | 0 … 65535 |

Encoding formula:

```python
raw = int((x - x_min) / (x_max - x_min) * 65535)
```

### 3.4 Feedback Frame (type 2)

The motor responds to every command with a type-2 frame.  It also pushes
type-2 frames autonomously when **active reporting** is enabled (type 24).

**29-bit ID layout of the response:**

```
bits 28–24  0x02
bits 23–22  mode   (0=reset, 1=calibration, 2=run)
bits 21–16  fault bitmask (6-bit subset — see type-21 for full bitmask)
bits 15–8   source motor CAN ID
bits  7–0   host CAN ID
```

**8-byte payload:**

| Bytes | Content | Physical range |
|-------|---------|----------------|
| 0–1 | position | −4π … +4π rad |
| 2–3 | velocity | −V_MAX … +V_MAX rad/s |
| 4–5 | torque   | −T_MAX … +T_MAX N·m |
| 6–7 | temperature × 10 | °C |

### 3.5 Fault Feedback Frame (type 21)

The motor pushes type-21 frames autonomously when a fault or warning condition
changes.  This frame carries the **full** fault bitmask (32 bits) and a separate
warning bitmask, providing more detail than the 6-bit subset in the type-2 ID.

**8-byte payload:**

| Bytes | Content |
|-------|---------|
| 0–3 | Fault bitmask (uint32 LE) — see `FaultBit` enum |
| 4–7 | Warning bitmask (uint32 LE) — see `WarnBit` enum |

**Fault bits (bytes 0–3):**

| Bit | Meaning |
|-----|---------|
| 0 | Motor over-temperature (> 135 °C) |
| 1 | Driver IC fault |
| 2 | Under-voltage (< 12 V) |
| 3 | Over-voltage (> 60 V) |
| 4 | B-phase over-current |
| 5 | C-phase over-current |
| 7 | Encoder not calibrated |
| 8 | Hardware ID fault |
| 9 | Position init fault |
| 14 | Stall / I²t overload |
| 16 | A-phase over-current |

**Warning bits (bytes 4–7):**

| Bit | Meaning |
|-----|---------|
| 0 | Motor over-temperature warning (approaching 135 °C threshold) |

### 3.6 Parameter Read / Write (types 17 & 18)

The motor exposes an object dictionary indexed by 16-bit function codes.
See `ParamIndex` in `motor_base.py` for the full list.  Common ones:

| Index  | Name          | Type   | Description |
|--------|---------------|--------|-------------|
| 0x2009 | motor_baud    | uint8  | Baud rate flag: 1=1Mbps 2=500Kbps 3=250Kbps 4=125Kbps (re-power required) |
| 0x200A | CAN_ID        | uint8  | Motor CAN ID (0-127), effective immediately |
| 0x200B | CAN_MASTER    | uint8  | Host CAN ID |
| 0x7005 | run_mode      | uint8  | 0=OPERATION 1=POS_PP 2=VELOCITY 3=CURRENT 5=POS_CSP |
| 0x7006 | iq_ref        | float  | Current mode Iq command (A) |
| 0x700A | spd_ref       | float  | Velocity command (rad/s) |
| 0x700B | limit_torque  | float  | Torque limit (N·m) |
| 0x7016 | loc_ref       | float  | Position command (rad) |
| 0x7017 | limit_spd     | float  | CSP speed limit (rad/s) |
| 0x7019 | mechPos       | float  | Read-only: mechanical angle (rad) |
| 0x701B | mechVel       | float  | Read-only: load velocity (rad/s) |
| 0x701C | VBUS          | float  | Read-only: bus voltage (V) |

Writes are volatile (lost on power-off) unless followed by a **type 22** (save) frame.

---

## 4. Software Architecture

```
┌──────────────────────────────────────────────────────────────┐
│                     ROS 2 Node (motor_node.py)               │
│  Topics / Services / Actions — robstride_p package           │
└─────────────────────┬────────────────────────────────────────┘
                      │ instantiates
┌─────────────────────▼────────────────────────────────────────┐
│            Motor classes  rs01.py … rs05.py                  │
│          (RS01 … RS05 — motor-specific limits only)          │
└─────────────────────┬────────────────────────────────────────┘
                      │ inherits
┌─────────────────────▼────────────────────────────────────────┐
│           RobStrideMotorBase  (motor_base.py)                │
│  Full private protocol: frame building, encode/decode,       │
│  all control modes, parameter read/write, active reporting.  │
│  _on_frame_received handles all incoming comm types and      │
│  keeps _feedback current via threading.Event synchronisation │
└─────────────────────┬────────────────────────────────────────┘
                      │ uses
┌─────────────────────▼────────────────────────────────────────┐
│                  CANComms  (comms.py)                        │
│  python-can wrapper: SocketCAN, hardware filters,            │
│  Notifier thread + per-motor callback dispatch               │
└─────────────────────┬────────────────────────────────────────┘
                      │
              SocketCAN (can0)
                      │
              CAN bus  ──────── RS01 ── RS02 ── RS03 …
```

---

## 5. comms.py — CAN Transport Layer

`CANComms` wraps `python-can` and provides the transport for one or more
motors sharing a physical CAN bus.

### Hardware filters

Each motor registered via `add_motor_filter(motor_id, callback)` installs a
kernel-level receive filter:

```
accept frame if (arbitration_id >> 8) & 0xFF == motor_id
```

Frames for unregistered motors are dropped by the driver, not Python,
keeping CPU load low.

### Background listener and callback dispatch

`start_listener()` must be called once before using any motor.  It starts a
`can.Notifier` background thread that feeds `_MotorDispatcher`.  The dispatcher
extracts the motor ID from bits 15–8 of the extended arbitration ID and calls
the registered per-motor callback directly — no queues, no intermediate buffering.

```
Notifier thread
    │
    ▼
_MotorDispatcher.on_message_received(msg)
    └── motor._on_frame_received(msg)   ← handles all comm types, updates state
```

All frame parsing happens inside `_on_frame_received` in `motor_base.py`.
For request-response exchanges (param read, device ID) the callback sets a
`threading.Event` that the calling thread waits on.

---

## 6. motor_base.py — Protocol Implementation

`RobStrideMotorBase` contains the complete private-protocol implementation.
Motor subclasses override only four class attributes:

```python
class RS03(RobStrideMotorBase):
    V_MIN: float = -21.0   # rad/s
    V_MAX: float =  21.0
    T_MIN: float = -60.0   # N·m
    T_MAX: float =  60.0
    MAX_CURRENT_A: float = 43.0
```

### Frame handler

`_on_frame_received` is the single entry-point for all incoming frames.
It is called by `_MotorDispatcher` in the Notifier background thread:

| Comm type | Action |
|-----------|--------|
| 2 (MOTOR_FEEDBACK) | Decode and replace `_feedback` |
| 21 (FAULT_FEEDBACK) | Update `_feedback.fault` and `_feedback.warning` |
| 17 (PARAM_READ reply) | Store raw bytes, set `_param_event` |
| 0 (GET_DEVICE_ID reply) | Store raw bytes, set `_device_id_event` |

Because the callback fires in the Notifier thread, `motor.feedback` is always
current regardless of what the calling thread is doing.

### Control modes

| RunMode | Value | How to use |
|---------|-------|------------|
| OPERATION | 0 | `set_operation_control(position, velocity, torque_ff, kp, kd)` |
| POSITION_PP | 1 | `set_position_pp(position_rad, speed_rad_s, acceleration_rad_s2)` |
| VELOCITY | 2 | `set_velocity(velocity_rad_s)` |
| CURRENT | 3 | `set_current(iq_ref_a)` |
| POSITION_CSP | 5 | `set_position_csp(position_rad, speed_limit_rad_s)` |

Mode must be set **while the motor is disabled**.

### Typical command sequence

```python
motor.set_run_mode(RunMode.VELOCITY)   # 1. set mode (motor stopped)
motor.enable()                          # 2. enable (type 3)
motor.set_velocity(2.0)                 # 3. command
motor.disable()                         # 4. stop (type 4)
```

### Active reporting

Active reporting is **off by default**.  Enable it via the `~/set_active_report`
ROS 2 service (or by calling `motor.enable_active_report()` directly).  When
active, the motor pushes type-2 frames at the configured rate without waiting
for a command, and `motor.feedback` is updated automatically by the callback.

---

## 7. Motor Classes RS01–RS05

Each file (`rs01.py` … `rs05.py`) is a thin subclass that sets the physical
limits and re-exports the shared enumerations for convenience.

| Class | T_MAX | V_MAX  | MAX_CURRENT_A | Decel ratio |
|-------|-------|--------|---------------|-------------|
| RS01  | 17 N·m | 33 rad/s | 23 A | 7.75:1 |
| RS02  | 17 N·m | 44 rad/s | 23 A | 7.75:1 |
| RS03  | 60 N·m | 21 rad/s | 43 A | 9:1 |
| RS04  | 120 N·m | 21 rad/s | 90 A | 9:1 |
| RS05  | 6 N·m  | 50 rad/s | 11 A | 7.75:1 |

All motors share: P_MIN = −4π rad, P_MAX = +4π rad, KP_MAX = 500, KD_MAX = 5.

---

## 8. ROS 2 Package — robstride_p

Located at `ros_ws/src/robstride_p/`.

### 8.1 config.toml

All motor configuration lives in one TOML file.  The `[defaults]` section is
**required**; the node exits with a fatal error if it is absent.

```toml
[defaults]
node_name                   = "motor_node"  # ROS 2 node name
use_node_name_as_topic_base = true          # false → topics at root namespace
master_id                   = 253           # host CAN ID (0xFD)
channel                     = "can0"        # SocketCAN interface
bustype                     = "socketcan"
bitrate                     = 1000000
rx_timeout                  = 0.05          # seconds — used for param read timeout
active_report_interval_ms   = 10            # fallback interval for set_active_report (hz=0)
update_rate_hz              = 100.0         # feedback publish rate

[left_hip]
type     = "RS04"
motor_id = 1

[right_hip]
type     = "RS04"
motor_id = 2

[left_knee]
type     = "RS03"
motor_id = 3
# override a single default for this motor only:
channel  = "can1"
```

Each motor section may override any `[defaults]` field.  Motors that share
the same `(channel, bustype, bitrate)` tuple reuse one `CANComms` instance.

`node_name` is read **before** the ROS 2 node is constructed (via a short-lived
temporary node) so it can be passed to `Node.__init__()`.

**Topic namespace control:**
- `use_node_name_as_topic_base = true` (default): topics live under `/{node_name}/` — e.g. `/motor_node/joint_states`
- `use_node_name_as_topic_base = false`: topics are at the root namespace — e.g. `/joint_states`

All examples below use `~` notation (`~/topic` = `/{node_name}/topic`).

**Active reporting** is disabled by default.  Use the `~/set_active_report`
service to enable it per-motor or for all motors.  `active_report_interval_ms`
in `[defaults]` is the fallback interval used when the service is called with
`hz = 0`.

### 8.2 Topics

| Topic | Type | Description |
|-------|------|-------------|
| `~/joint_states` | `sensor_msgs/JointState` | All motors combined, at `update_rate_hz` |
| `~/motors/{name}/state` | `custom_interfaces/MotorState` | Per-motor state (position, velocity, torque, temperature, mode, fault, enabled) |
| `~/motors/{name}/fault` | `custom_interfaces/MotorFault` | Per-motor decoded fault and warning bits — published **only on change** |
| `~/motors/{name}/command` | `custom_interfaces/MotorCommand` | Subscribe to send commands |

**Sending a command:**

```bash
# velocity command
ros2 topic pub /motor_node/motors/left_hip/command \
  custom_interfaces/msg/MotorCommand \
  '{name: left_hip, command_type: velocity, value: 2.0}'

# operation control (MIT-style)
ros2 topic pub /motor_node/motors/left_hip/command \
  custom_interfaces/msg/MotorCommand \
  '{name: left_hip, command_type: operation, position: 1.57, velocity: 0.0, kp: 30.0, kd: 1.0, torque_ff: 0.0}'
```

`command_type` values: `position` | `velocity` | `current` | `torque` | `operation`

For `position`, `velocity`, `current`, `torque` — the `value` field carries the scalar.
For `operation` — use the full `position`, `velocity`, `kp`, `kd`, `torque_ff` fields.

**Note:** The node tracks each motor's current run mode.  A command whose
`command_type` does not match the motor's active mode is rejected with a
logged error (e.g. sending `velocity` while the motor is in `POSITION_CSP`
mode).  Set the correct mode first with `~/set_run_mode`.

### 8.3 Services

Services that accept a `name` field support `"all"` to apply to every motor
simultaneously — except `read_param` and `get_can_config`, which return a
single value.

| Service | Type | `"all"` | Description |
|---------|------|---------|-------------|
| `~/enable_motor` | `EnableMotor` | ✅ | Enable or stop motors; optionally clear faults |
| `~/set_run_mode` | `SetRunMode` | ✅ | Switch control mode (motor must be stopped first) |
| `~/set_zero_position` | `SetZeroPosition` | ✅ | Set current position as mechanical zero |
| `~/read_param` | `ReadParam` | ❌ | Read a parameter by function code index |
| `~/write_param` | `WriteParam` | ✅ | Write a parameter; `persist=true` saves to flash |
| `~/help` | `Help` | — | List all parameter codes, names, types, access, descriptions |
| `~/get_can_config` | `GetCanConfig` | ❌ | Read CAN ID and baud rate from motor firmware |
| `~/set_can_config` | `SetCanConfig` | ✅ | Change CAN ID (immediate) and/or baud rate (re-power) |
| `~/set_active_report` | `SetActiveReport` | ✅ | Enable or disable autonomous status reporting |
| `~/homing` | `std_srvs/Trigger` | — | Stop all motors then command position 0.0 via CSP |
| `~/stop_all` | `std_srvs/Trigger` | — | Immediately disable every motor |

**Examples:**

```bash
# Enable a single motor
ros2 service call /motor_node/enable_motor custom_interfaces/srv/EnableMotor \
  '{name: left_hip, enable: true}'

# Enable all motors at once
ros2 service call /motor_node/enable_motor custom_interfaces/srv/EnableMotor \
  '{name: all, enable: true}'

# Set run mode to velocity on all motors
ros2 service call /motor_node/set_run_mode custom_interfaces/srv/SetRunMode \
  '{name: all, mode: 2}'

# Read mechanical position (0x7019 = 28697 decimal)
ros2 service call /motor_node/read_param custom_interfaces/srv/ReadParam \
  '{name: left_hip, index: 28697}'

# Write CSP speed limit and persist to flash (0x7017 = 28695 decimal)
ros2 service call /motor_node/write_param custom_interfaces/srv/WriteParam \
  '{name: left_hip, index: 28695, value: 5.0, persist: true}'

# List all parameters containing "velocity"
ros2 service call /motor_node/help custom_interfaces/srv/Help \
  '{filter: velocity}'

# Read CAN ID and baud rate from a motor
ros2 service call /motor_node/get_can_config custom_interfaces/srv/GetCanConfig \
  '{name: left_hip}'

# Change CAN ID only (immediate effect)
ros2 service call /motor_node/set_can_config custom_interfaces/srv/SetCanConfig \
  '{name: left_hip, can_id: 5, baud_flag: 0}'

# Change baud rate to 500 Kbps (re-power required)
ros2 service call /motor_node/set_can_config custom_interfaces/srv/SetCanConfig \
  '{name: left_hip, can_id: 0, baud_flag: 2}'

# Enable active reporting at 100 Hz on all motors
ros2 service call /motor_node/set_active_report custom_interfaces/srv/SetActiveReport \
  '{name: all, enable: true, hz: 100.0}'

# Disable active reporting on one motor
ros2 service call /motor_node/set_active_report custom_interfaces/srv/SetActiveReport \
  '{name: left_hip, enable: false, hz: 0.0}'

# Emergency stop
ros2 service call /motor_node/stop_all std_srvs/srv/Trigger

# Home all motors to position 0.0
ros2 service call /motor_node/homing std_srvs/srv/Trigger
```

**Baud flag values for `get_can_config` / `set_can_config`:**

| `baud_flag` | Rate | Notes |
|-------------|------|-------|
| 1 | 1 Mbps | Default |
| 2 | 500 Kbps | |
| 3 | 250 Kbps | |
| 4 | 125 Kbps | |

CAN ID changes take effect immediately.
Baud rate changes are saved to flash and take effect after re-powering the motor.

**`set_active_report` fields:**
- `name` — motor name or `"all"`
- `enable` — `true` to start reporting, `false` to stop
- `hz` — reporting rate in Hz (minimum ~100 Hz = 10 ms interval); `0.0` uses `active_report_interval_ms` from config.toml

### 8.4 Actions

| Action | Type | Description |
|--------|------|-------------|
| `~/move_to_position` | `MoveToPosition` | Move to a target position; succeeds when within `tolerance` rad |
| `~/set_velocity` | `SetVelocity` | Run at a velocity for `duration` seconds (0 = until cancelled) |

**Move to position:**

```bash
ros2 action send_goal /motor_node/move_to_position \
  custom_interfaces/action/MoveToPosition \
  '{name: left_knee, target_position: 1.57, speed_limit: 2.0, tolerance: 0.05, timeout: 10.0}'
```

Feedback publishes `current_position`, `position_error`, `elapsed_time` at 100 Hz.
The motor holds the target position (CSP mode, motor stays enabled) on success.

**Set velocity:**

```bash
ros2 action send_goal /motor_node/set_velocity \
  custom_interfaces/action/SetVelocity \
  '{name: left_hip, target_velocity: 3.0, duration: 5.0, current_limit: 20.0, deceleration: 10.0}'
```

Velocity is ramped to 0 (using `deceleration` if provided, else `acceleration`) when
duration expires or the goal is cancelled.

### 8.5 Custom Interfaces

Package: `custom_interfaces` (`ros_ws/src/custom_interfaces/`)

**Messages** (`msg/`):

| Message | Key fields |
|---------|------------|
| `MotorState` | `name`, `position`, `velocity`, `torque`, `temperature`, `mode`, `fault`, `enabled` |
| `MotorCommand` | `name`, `command_type`, `value`, `position`, `velocity`, `torque_ff`, `kp`, `kd` |
| `MotorFault` | `name`, `fault_code`, `warning_code`, decoded per-bit booleans for faults and `over_temp_warning` |

**Services** (`srv/`): `EnableMotor`, `SetRunMode`, `SetZeroPosition`, `ReadParam`, `WriteParam`, `Help`, `GetCanConfig`, `SetCanConfig`, `SetActiveReport`

**Actions** (`action/`): `MoveToPosition`, `SetVelocity`

---

## 9. Build & Run

### Prerequisites

```bash
# ROS 2 (Jazzy or compatible)
sudo apt install ros-${ROS_DISTRO}-ros-base

# python-can
pip install python-can

# SocketCAN utilities
sudo apt install can-utils
```

### Build

```bash
cd claude/ros_ws
colcon build
source install/setup.bash
```

### Launch

```bash
# Default config
ros2 launch robstride_p motors.launch.py

# Custom config file
ros2 launch robstride_p motors.launch.py config:=/path/to/my_config.toml
```

### CAN interface setup (one-time per boot)

```bash
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
# Verify
ip link show can0
candump can0   # live frame monitor
```

---

## 10. Standalone Usage (no ROS 2)

All driver files are plain Python — no ROS 2 required.

```python
from comms import CANComms
from rs04 import RS04
from rs02 import RS02
from motor_base import RunMode

with CANComms("can0") as bus:
    bus.start_listener()            # start Notifier background thread

    hip  = RS04(motor_id=1, comms=bus)
    knee = RS02(motor_id=3, comms=bus)

    # Enable active reporting at 100 Hz — _feedback updated automatically
    hip.enable_active_report(enable=True, interval_ms=10)
    knee.enable_active_report(enable=True, interval_ms=10)

    # Velocity mode on hip
    hip.set_run_mode(RunMode.VELOCITY)
    hip.enable()
    hip.set_velocity(2.0)           # 2 rad/s

    # CSP position on knee
    knee.set_run_mode(RunMode.POSITION_CSP)
    knee.enable()
    knee.set_position_csp(1.57, speed_limit_rad_s=3.0)

    import time
    for _ in range(100):
        print(hip.feedback, knee.feedback)   # always current via callback
        time.sleep(0.01)

    hip.disable()
    knee.disable()
```

**Multiple motors on separate buses:**

```python
with CANComms("can0") as bus0, CANComms("can1") as bus1:
    bus0.start_listener()
    bus1.start_listener()
    m1 = RS04(motor_id=1, comms=bus0)
    m2 = RS03(motor_id=1, comms=bus1)   # same CAN ID, different bus — fine
```

**Reading a parameter:**

```python
# Blocking call — waits up to rx_timeout for the reply frame via threading.Event
pos = motor.read_mech_pos()   # rad
vel = motor.read_mech_vel()   # rad/s
vbus = motor.read_vbus()      # V
```

**Saving parameters to flash:**

```python
motor.write_param_float(ParamIndex.LOC_KP, 50.0)
motor.save_params()   # type 22 — survives power-off
```

---

## File Structure

```
claude/
└── ros_ws/
    └── src/
        ├── custom_interfaces/          ROS 2 msgs, srvs, actions
        │   ├── msg/
        │   │   MotorState.msg
        │   │   MotorCommand.msg
        │   │   MotorFault.msg
        │   ├── srv/
        │   │   EnableMotor.srv         SetRunMode.srv  SetZeroPosition.srv
        │   │   ReadParam.srv           WriteParam.srv  Help.srv
        │   │   GetCanConfig.srv        SetCanConfig.srv
        │   │   SetActiveReport.srv
        │   └── action/
        │       MoveToPosition.action   SetVelocity.action
        └── robstride_p/                ROS 2 Python package
            ├── config/config.toml
            ├── launch/motors.launch.py
            ├── docs/                   RS01–RS05 user manuals (PDF)
            └── robstride_p/
                ├── comms.py            CAN transport (CANComms, _MotorDispatcher)
                ├── motor_base.py       Protocol base class, enums, MotorFeedback
                ├── rs01.py … rs05.py   Motor subclasses (limits only)
                └── motor_node.py       ROS 2 node
```
