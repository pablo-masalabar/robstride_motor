# robstride_p

ROS 2 Jazzy driver for RobStride RS01–RS05 quasi-direct-drive motors over SocketCAN.
Supports all five motor models on one or more CAN buses, with position, velocity, current,
and MIT operation-control modes.

## Motor models

| Model | Peak torque | Peak speed (output) | Max current | Weight | Gear ratio |
|---|---|---|---|---|---|
| RS01 | 17 N·m | 33 rad/s (315 rpm) | 23 A | 380 g | 7.75:1 |
| RS02 | 17 N·m | 44 rad/s (410 rpm) | 23 A | 380 g | 7.75:1 |
| RS03 | 60 N·m | 21 rad/s (200 rpm) | 43 A | 880 g | 9:1 |
| RS04 | 120 N·m | 21 rad/s (200 rpm) | 90 A | 1420 g | 9:1 |
| RS05 | 5.5 N·m | 50 rad/s (480 rpm) | 11 A | 191 g | 7.75:1 |

All models run at 1 Mbps on a 29-bit extended CAN frame. See `docs/RS0x.pdf` for datasheets.

## Build & Run

```bash
cd ros_ws
colcon build --packages-select robstride_p
source install/setup.bash

# left arm (default config)
ros2 launch robstride_p left_arm.launch.py

# right arm
ros2 launch robstride_p right_arm.launch.py

# base + neck
ros2 launch robstride_p base_and_neck.launch.py

# custom config
ros2 launch robstride_p left_arm.launch.py config:=/path/to/config.toml
```

## Config (`config/*.toml`)

Each config file drives one node with one or more motors on a shared CAN bus.

### `[defaults]` section — required

| Field | Type | Description |
|---|---|---|
| `node_name` | string | ROS 2 node name |
| `use_node_name_as_topic_base` | bool | `true` → topics at `/{node_name}/…`; `false` → root namespace |
| `master_id` | int | Host CAN ID sent in outgoing frames (default `0xFD` = 253) |
| `channel` | string | SocketCAN interface name, e.g. `can0` |
| `bustype` | string | python-can bus type — always `socketcan` |
| `bitrate` | int | CAN baud rate in bps — always `1000000` |
| `rx_timeout` | float | Reply wait time in seconds (default `0.05`) |
| `active_report_interval_ms` | int | Motor autonomous push interval in ms (minimum `10`) |
| `update_rate_hz` | float | `joint_states` publish rate (Hz) |
| `operation_mode` | string | Default run mode for all motors (see modes below) |

### Per-motor sections — `[MotorName]`

| Field | Required | Description |
|---|---|---|
| `type` | yes | Motor model: `RS01`, `RS02`, `RS03`, `RS04`, or `RS05` |
| `motor_id` | yes | CAN ID of the motor (0–127) |
| `channel` | no | Override `defaults.channel` for this motor only |
| `master_id` | no | Override `defaults.master_id` |
| `rx_timeout` | no | Override `defaults.rx_timeout` |
| `operation_mode` | no | Override `defaults.operation_mode` for this motor |
| `joint_limit_min` | no | Minimum allowed position in user frame (rad) |
| `joint_limit_max` | no | Maximum allowed position in user frame (rad) |
| `motor_homing_pos` | no | Motor-frame position corresponding to user-frame zero (rad) |
| `max_torque` | no | Torque limit written to motor at startup and used as command clamp (N·m) |
| `max_current` | no | Current limit written to motor at startup and used as velocity-mode clamp (A) |
| `max_vel` | no | Software velocity clamp (rad/s) |
| `max_accel` | no | Software acceleration clamp (rad/s²) |
| `max_decel` | no | Software deceleration clamp (rad/s²) |
| `loc_kp` | no | Position loop Kp written to motor at startup |
| `spd_kp` | no | Speed loop Kp written to motor at startup |
| `spd_ki` | no | Speed loop Ki written to motor at startup |
| `cur_kp` | no | Current loop Kp written to motor at startup |
| `cur_ki` | no | Current loop Ki written to motor at startup |
| `kp` | no | MIT operation-mode position proportional gain |
| `kd` | no | MIT operation-mode velocity derivative gain |

### Position frame (`motor_homing_pos`)

All position commands and feedback are in the **user frame**:

```
user_pos  = motor_pos − motor_homing_pos   (published on ~/motors/{name}/state)
motor_pos = user_pos  + motor_homing_pos   (applied to commands before sending)
```

Joint limits are also in motor frame and are compared against the raw motor position.
At startup, if `MECH_POS` is outside `[joint_limit_min, joint_limit_max]`, all three
values are shifted by ±2π to bring them into the same range as the current motor position.

### Run modes

| Mode string | Integer | Description |
|---|---|---|
| `OPERATION` | 0 | MIT-style: `τ = Kd·(v_set − v) + Kp·(p_set − p) + τ_ff` |
| `POSITION_PP` | 1 | Profile Position — trapezoidal velocity profile, speed/accel/decel configurable |
| `VELOCITY` | 2 | Velocity control with current limit and acceleration ramp |
| `CURRENT` | 3 | Direct Iq current reference |
| `POSITION_CSP` | 5 | Cyclic Synchronous Position — direct position with speed and current limit |

## Topics

All topics are namespaced under `/{node_name}/` when `use_node_name_as_topic_base = true`.

### Published

| Topic | Message type | Description |
|---|---|---|
| `~/joint_states` | `sensor_msgs/JointState` | All motors combined, published at `update_rate_hz` |
| `~/motors/{name}/state` | `custom_interfaces/MotorState` | Per-motor state (position in user frame, velocity, torque, temperature, mode, fault, enabled) |
| `~/motors/{name}/fault` | `custom_interfaces/MotorFault` | Published only on fault/warning transitions |

### Subscribed (per motor, per mode)

| Topic | Message type | Mode required |
|---|---|---|
| `~/motors/{name}/cmd_operation` | `custom_interfaces/OperationCommand` | `OPERATION` |
| `~/motors/{name}/cmd_position_pp` | `custom_interfaces/PositionPPCommand` | `POSITION_PP` |
| `~/motors/{name}/cmd_velocity` | `custom_interfaces/VelocityCommand` | `VELOCITY` |
| `~/motors/{name}/cmd_current` | `custom_interfaces/CurrentCommand` | `CURRENT` |
| `~/motors/{name}/cmd_position_csp` | `custom_interfaces/PositionCSPCommand` | `POSITION_CSP` |

Commands published in the wrong mode are logged and dropped.

## Services

All services accept `name = "all"` to apply to every motor, except where noted.

### `~/enable_motor` (`custom_interfaces/EnableMotor`)

Enable or disable a motor. Returns the motor's position/velocity/torque at the moment
of the call (single-motor only; `"all"` returns zeros).

| Field | Description |
|---|---|
| `name` | Motor name or `"all"` |
| `enable` | `true` to enable, `false` to disable |
| `clear_fault` | Clear latched fault flags when disabling |

```bash
ros2 service call /left_arm/enable_motor custom_interfaces/srv/EnableMotor "{name: 'SpL', enable: true, clear_fault: false}"
```

### `~/set_run_mode` (`custom_interfaces/SetRunMode`)

Change control mode. Pass `automatic_enable_disable: true` to auto-disable before and
re-enable after the mode change (required if the motor is currently enabled).

| Field | Description |
|---|---|
| `name` | Motor name or `"all"` |
| `mode` | Integer mode: `0`=OPERATION `1`=POS_PP `2`=VELOCITY `3`=CURRENT `5`=POS_CSP |
| `automatic_enable_disable` | Disable → change mode → enable automatically |

```bash
ros2 service call /left_arm/set_run_mode custom_interfaces/srv/SetRunMode "{name: 'all', mode: 1, automatic_enable_disable: true}"
```

### `~/set_zero_position` (`custom_interfaces/SetZeroPosition`)

Set the motor's current mechanical position as its zero point (written to firmware).

```bash
ros2 service call /left_arm/set_zero_position custom_interfaces/srv/SetZeroPosition "{name: 'SpL'}"
```

### `~/homing` (`std_srvs/Trigger`)

Stop all motors, switch to `POSITION_CSP`, and command position `0.0` (user frame)
for every motor in this node.

```bash
ros2 service call /left_arm/homing std_srvs/srv/Trigger
```

### `~/stop_all` (`std_srvs/Trigger`)

Immediately disable every motor on the bus (comm type 4).

```bash
ros2 service call /left_arm/stop_all std_srvs/srv/Trigger
```

### `~/set_active_report` (`custom_interfaces/SetActiveReport`)

Enable or disable the motor's autonomous push mode. When enabled the motor sends
feedback at `hz` without waiting for a command; the node's publish timer is also
adjusted to match.

| Field | Description |
|---|---|
| `name` | Motor name or `"all"` |
| `enable` | `true` to enable, `false` to disable |
| `hz` | Desired report rate in Hz (min 100 Hz = 10 ms) |

```bash
ros2 service call /left_arm/set_active_report custom_interfaces/srv/SetActiveReport "{name: 'all', enable: true, hz: 50.0}"
```

### `~/read_param` / `~/write_param` (`custom_interfaces/ReadParam` / `WriteParam`)

Low-level access to firmware parameter registers by index. See `~/help` for the full
list of register indices, types, and descriptions. `"all"` is not supported for
`read_param` (single float64 return value).

```bash
# Read bus voltage from SpL
ros2 service call /left_arm/read_param custom_interfaces/srv/ReadParam "{name: 'SpL', index: 28700}"  # 0x701C

# Write speed loop Kp to all motors
ros2 service call /left_arm/write_param custom_interfaces/srv/WriteParam "{name: 'all', index: 28703, value: 8.0, persist: false}"
```

### `~/motor_param` (`custom_interfaces/MotorParam`)

Get or set a named software or firmware parameter at runtime without knowing register indices.
Firmware parameters (`loc_kp`, `spd_kp`, `spd_ki`, `cur_kp`, `cur_ki`, `max_torque`, `max_current`)
are also written to the motor immediately when set.

Software-only parameters (`kp`, `kd`, `joint_limit_min`, `joint_limit_max`, `max_vel`, `max_accel`,
`max_decel`, `motor_homing_pos`) affect command clamping and position frame calculations only.

| Field | Description |
|---|---|
| `name` | Motor name (`"all"` not supported) |
| `param` | Parameter name string |
| `set` | `true` to set, `false` to read |
| `value` | New value (only used when `set: true`) |

```bash
# Read joint limit
ros2 service call /left_arm/motor_param custom_interfaces/srv/MotorParam "{name: 'SpL', param: 'joint_limit_max', set: false, value: 0.0}"

# Update torque limit at runtime
ros2 service call /left_arm/motor_param custom_interfaces/srv/MotorParam "{name: 'SpL', param: 'max_torque', set: true, value: 80.0}"
```

### `~/get_can_config` / `~/set_can_config` (`custom_interfaces/GetCanConfig` / `SetCanConfig`)

Read or change the motor's CAN ID and baud rate. CAN ID takes effect immediately;
baud rate is saved to flash and requires re-power.

```bash
# Read current CAN ID and baud rate
ros2 service call /left_arm/get_can_config custom_interfaces/srv/GetCanConfig "{name: 'SpL'}"

# Change CAN ID to 20 (pass 0 to leave unchanged)
ros2 service call /left_arm/set_can_config custom_interfaces/srv/SetCanConfig "{name: 'SpL', can_id: 20, baud_flag: 0}"
```

Baud flag: `0`=keep `1`=1 Mbps `2`=500 Kbps `3`=250 Kbps `4`=125 Kbps.

### `~/scan_motors` (`std_srvs/Trigger`)

Query every configured motor for its MCU unique ID and log the results. Useful for
verifying all motors are alive after startup.

```bash
ros2 service call /left_arm/scan_motors std_srvs/srv/Trigger
```

### `~/help` (`custom_interfaces/Help`)

List all known parameter registers with index, type, access, and description.
Pass `filter` to narrow results.

```bash
ros2 service call /left_arm/help custom_interfaces/srv/Help "{filter: 'kp'}"
```

## Actions

### `~/move_to_position` (`custom_interfaces/MoveToPosition`)

Switch to `POSITION_CSP`, enable the motor, command a target, and wait until within
`tolerance` rad or until `timeout` seconds elapse. Publishes position error as feedback.

```bash
ros2 action send_goal /left_arm/move_to_position custom_interfaces/action/MoveToPosition \
  "{name: 'SpL', target_position: 1.0, speed_limit: 5.0, tolerance: 0.01, timeout: 10.0}"
```

### `~/set_velocity` (`custom_interfaces/SetVelocity`)

Switch to `VELOCITY`, enable the motor, and run for `duration` seconds
(or indefinitely if `duration: 0`). Cancellation applies a deceleration ramp to zero.

```bash
ros2 action send_goal /left_arm/set_velocity custom_interfaces/action/SetVelocity \
  "{name: 'SpL', target_velocity: 2.0, current_limit: 10.0, acceleration: 5.0, duration: 3.0}"
```

## Fault and warning bits

Faults are reported on `~/motors/{name}/fault` and logged on every transition.

| Bit | Name | Description |
|---|---|---|
| 0 | `OVER_TEMP` | Motor over-temperature |
| 1 | `DRIVER_IC` | Driver IC fault |
| 2 | `UNDERVOLTAGE` | Bus undervoltage |
| 3 | `OVERVOLTAGE` | Bus overvoltage |
| 4 | `B_PHASE_OC` | B-phase overcurrent |
| 5 | `C_PHASE_OC` | C-phase overcurrent |
| 7 | `ENCODER_UNCAL` | Encoder not calibrated |
| 8 | `HW_ID_FAULT` | Hardware ID fault |
| 9 | `POS_INIT_FAULT` | Position init fault |
| 14 | `STALL_OVERLOAD` | Stall overload |
| 16 | `A_PHASE_OC` | A-phase overcurrent |

Warning bit 0 (`OVER_TEMP_WARN`): winding temperature approaching the 135 °C threshold.

## Utilities

```bash
# Scan a bus for Robstride motors (standalone, no ROS required)
python3 ros_ws/find_robstride_motors.py can0
python3 ros_ws/find_robstride_motors.py can0 --min 0 --max 127 --timeout 0.05

# Scan all buses and update channel fields in config TOMLs
python3 ros_ws/update_motor_can.py
python3 ros_ws/update_motor_can.py --dry-run
```

## Protocol notes

All frames use the RobStride private CAN 2.0 protocol (29-bit extended ID, 1 Mbps).

**29-bit arbitration ID layout:**
```
bits 28–24   communication type (5 bits)
bits 23–8    data area 2 / payload (16 bits)
bits  7–0    destination motor CAN ID (8 bits)
```

Key communication types:

| Type | Name | Direction |
|---|---|---|
| 0 | GET_DEVICE_ID | host → motor, motor → host |
| 1 | OPERATION_CTRL | host → motor |
| 2 | MOTOR_FEEDBACK | motor → host |
| 3 | MOTOR_ENABLE | host → motor |
| 4 | MOTOR_STOP | host → motor |
| 6 | SET_MECH_ZERO | host → motor |
| 7 | SET_CAN_ID | host → motor |
| 17 | PARAM_READ | host → motor, motor → host |
| 18 | PARAM_WRITE | host → motor |
| 21 | FAULT_FEEDBACK | motor → host |
| 22 | DATA_SAVE | host → motor |
| 24 | ACTIVE_REPORT | host → motor (enable/disable push) |
