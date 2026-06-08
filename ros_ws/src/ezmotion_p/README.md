# ezmotion_p

ROS 2 Jazzy driver package for **EZmotion PCN and SCN C2 series** motor driver modules. Communication is over **CANopen** (CiA DS301 + DS402) using standard 11-bit CAN frames.

## Applicable products

| Part number | Type | Voltage |
|---|---|---|
| MMS760400-48-C2-1 | SCN (smart motor) | 48 V |
| MMS760200-48-C2-1 | SCN | 48 V |
| MMP760400-75-C2-1 | PCN (driver module) | 75 V |
| MMP760200-75-C2-1 | PCN | 75 V |
| MMP740100-55-C2-1 | PCN | 55 V |
| MMS740100-24-C2-1 | SCN | 24 V |

## Package layout

```
ezmotion_p/
├── ezmotion_p/
│   ├── comms.py              — CAN transport (EZMotionCANComms)
│   ├── motor_base.py         — CANopen DS301/DS402 protocol implementation
│   ├── motor_node.py         — ROS 2 node
│   ├── mms760400_48_c2_1.py  — MMS760400-48-C2-1 motor subclass
│   └── __init__.py
├── config/
│   └── config.toml
├── launch/
│   └── ezmotion.launch.py
└── docs/
    ├── PCN_SCN_C2_Series_User_Guide_r1.1.pdf
    └── MMS760400-48-C2-1.eds
```

## Dependencies

- `python-can` — CAN bus interface
- `rclpy`, `custom_interfaces`

## Build & run

```bash
cd ros_ws
colcon build --packages-select ezmotion_p custom_interfaces
source install/setup.bash

ros2 launch ezmotion_p ezmotion.launch.py
# or with a custom config:
ros2 launch ezmotion_p ezmotion.launch.py config_path:=/path/to/config.toml
```

---

## ROS 2 node

All topics and services are published under `/{node_name}/` where `node_name` comes from `[defaults].node_name` in `config.toml` (e.g. `/ezmotion/joint_states`). Falls back to ROS node-relative namespace (`~`) if `node_name` is not set.

### Topics

| Direction | Topic | Type | Description |
|---|---|---|---|
| Published | `/{node_name}/joint_states` | `sensor_msgs/JointState` | All motors combined |
| Published | `/{node_name}/motors/{name}/state` | `custom_interfaces/MotorState` | Raw state: position (rad), velocity (rad/s) |
| Published | `/{node_name}/motors/{name}/processed_state` | `custom_interfaces/MotorState` | Converted state: position (mm), velocity (mm/s) |
| Published | `/{node_name}/motors/{name}/fault` | `custom_interfaces/MotorFault` | Published on fault change only |
| Subscribed | `/{node_name}/motors/{name}/cmd_position_pp` | `custom_interfaces/PositionPPCommand` | PP mode position command (rad) |
| Subscribed | `/{node_name}/motors/{name}/cmd_position_csp` | `custom_interfaces/PositionCSPCommand` | CSP mode position command (rad) |
| Subscribed | `/{node_name}/motors/{name}/cmd_velocity` | `custom_interfaces/VelocityCommand` | PV/CSV mode velocity command (rad/s) |
| Subscribed | `/{node_name}/motors/{name}/go_to` | `std_msgs/Float64` | PP mode: move to absolute height (mm) |
| Subscribed | `/{node_name}/motors/{name}/safe_vel` | `std_msgs/Float64` | PV mode: set velocity (mm/s) with soft limit protection |

#### `processed_state` topic

Same `MotorState` message type as `/state`, but with units converted for linear actuator use:

| Field | Unit | Description |
|---|---|---|
| `position` | mm | Height from bottom (converted from rad via mechanical coupling) |
| `velocity` | mm/s | Linear velocity (converted from rad/s, positive = up) |
| `torque` | N·m | Unchanged |
| `mode`, `fault`, `enabled` | — | Unchanged |

#### `go_to` topic

Publishes an absolute height in mm. Rejects values outside `[bottom_height_mm, top_height_mm]`. Requires PP mode.

```bash
ros2 topic pub --once /ezmotion/motors/MotorA/go_to std_msgs/msg/Float64 "{data: 300.0}"
```

#### `safe_vel` topic

Sets continuous velocity in mm/s. **Silently ignored** if:
- `current_height >= top_height_mm` and velocity > 0 (at max, trying to go up)
- `current_height <= bottom_height_mm` and velocity < 0 (at min, trying to go down)

Requires PV mode.

```bash
ros2 topic pub /ezmotion/motors/MotorA/safe_vel std_msgs/msg/Float64 "{data: 50.0}"
```

### Services

| Service | Type | Description |
|---|---|---|
| `/{node_name}/enable_motor` | `custom_interfaces/EnableMotor` | Enable or disable by motor name or `"all"` |
| `/{node_name}/set_run_mode` | `custom_interfaces/SetRunMode` | Switch DS402 operation mode (integer = DS402 value) |
| `/{node_name}/set_active_report` | `custom_interfaces/SetActiveReport` | Configure TPDO event timer rate |
| `/{node_name}/read_param` | `custom_interfaces/ReadParam` | SDO read by 16-bit OD index (sub=0) |
| `/{node_name}/write_param` | `custom_interfaces/WriteParam` | SDO write by 16-bit OD index (sub=0) |
| `/{node_name}/stop_all` | `std_srvs/Trigger` | Disable all motors immediately |
| `/{node_name}/save_params` | `std_srvs/Trigger` | Persist parameters to NVM (1010h) |

### Actions

| Action | Type | Description |
|---|---|---|
| `/{node_name}/homing` | `custom_interfaces/EZMotionHoming` | DS402 homing mode — drive to hard stop, then restore configured mode |
| `/{node_name}/move_to_position` | `custom_interfaces/MoveToPosition` | Move to target position (rad) and wait |
| `/{node_name}/set_velocity` | `custom_interfaces/SetVelocity` | Run at velocity (rad/s) for a duration |

---

## Config (`config/config.toml`)

### `[defaults]` section (required)

| Field | Description |
|---|---|
| `node_name` | ROS 2 node name and topic namespace prefix |
| `channel` | SocketCAN interface (e.g. `"can0"`) |
| `bustype` | python-can bus type (default `"socketcan"`) |
| `bitrate` | Baud rate in bps (default `1000000`) |
| `rx_timeout` | SDO reply timeout in seconds (default `0.1`) |
| `update_rate_hz` | `joint_states` publish rate (Hz) |
| `operation_mode` | Default DS402 mode: `PP` `PV` `PT` `HM` `CSP` `CSV` `CST` |

### Per-motor `[MotorName]` section

| Field | Required | Description |
|---|---|---|
| `type` | yes | Motor class name, e.g. `"MMS760400_48_C2_1"` |
| `node_id` | yes | CANopen node ID (1–127) |
| `operation_mode` | no | Per-motor DS402 mode override |
| `joint_limit_min` | no | Minimum position (rad) — commands rejected outside range |
| `joint_limit_max` | no | Maximum position (rad) |
| `max_vel` | no | Velocity clamp for all rad/s commands (rad/s) |
| `profile_velocity` | no | Written to 6081h at startup (rad/s) |
| `profile_acceleration` | no | Written to 6083h at startup (rad/s²) |
| `profile_deceleration` | no | Written to 6084h at startup (rad/s²) |
| `max_torque` | no | Written to 6072h at startup (N·m) |
| `top_height_mm` | no | Maximum height for `go_to`/`safe_vel` (mm, default `600.0`) |
| `bottom_height_mm` | no | Minimum height for `go_to`/`safe_vel` (mm, default `50.0`) |
| `homing_height_mm` | no | Physical height at motor position 0 after homing (mm, default = `top_height_mm`) |
| `homing_method` | no | DS402 6098h, default `-3` (torque hard-stop upward) |
| `homing_max_torque_permil` | no | Written to 2070h sub1, default `1000` (100% rated) |
| `homing_speed_rps` | no | Written to 6099h sub1+sub2 (rad/s), default `1.0` |
| `homing_acceleration_rps2` | no | Written to 609Ah (rad/s²), default `50.0` |
| `homing_offset_rotations` | no | Zero offset from hard stop in rotations — written to 607Ch, default `1.0` |
| `homing_timeout` | no | Seconds to wait for statusword bit 10, default `30.0` |

---

## Homing

Homing uses DS402 **Homing Mode** (mode 6) to find a physical reference position by driving into a hard stop (torque limit) and treating that point as the encoder zero. After homing completes the motor is automatically restored to the `operation_mode` from config.

### DS402 homing sequence

```
1. Controlword = 0x000F  (Enable)
2. Controlword = 0x0006  (Shutdown → Ready to Switch On)
3. 6060h = 6             (Set Homing mode)
4. Controlword = 0x000F  (Enable in Homing mode)
5. 6098h = homing_method (-3 = HOME_USING_MAX_TORQUE_UP)
6. 2070h sub1 = homing_max_torque_permil
7. 6099h sub1 = homing_speed   (search speed)
8. 6099h sub2 = homing_speed   (zero speed)
9. 609Ah      = homing_accel
10. 607Ch     = homing_offset  (rotations from hard stop)
11. Controlword = 0x001F (Trigger homing)
── poll statusword bit 10 (target reached) ──
12. Controlword = 0x0006  (Shutdown)
13. 6060h = configured_mode  (restore operation mode from config)
14. Controlword = 0x000F  (Enable)
```

Completion is detected by polling **statusword bit 10** (target reached). Bit 13 (homing error) and bit 3 (generic fault) abort with an error message.

### Homing action

```bash
# Home all motors
ros2 action send_goal /ezmotion/homing custom_interfaces/action/EZMotionHoming \
  "{motor_name: ''}"

# Home a single motor
ros2 action send_goal /ezmotion/homing custom_interfaces/action/EZMotionHoming \
  "{motor_name: 'MotorA'}"
```

**Feedback** per motor: `motor_name`, `phase` (`starting` | `complete` | `error`), `elapsed_time`, `statusword`.

**Result**: `success`, `message`, `homed_motors[]`.

---

## Linear actuator unit conversion

`go_to`, `safe_vel`, and `processed_state` use mm and mm/s. The mechanical coupling constant (from ref.py) is:

```
2.54 motor rotations per 1 cm of height
```

Conversion factor: `_RAD_PER_MM = 2.54 × 2π / 10 ≈ 1.596 rad/mm`

Direction: increasing height (up) = **decreasing** motor position (screw mechanism inversion).

| Conversion | Formula |
|---|---|
| mm → rad | `rad = -(mm_delta × 1.596)` |
| rad → mm | `mm = homing_height - (pos_rad / 1.596)` |
| mm/s → rad/s | `rad_s = -(mm_s × 1.596)` |
| rad/s → mm/s | `mm_s = -(rad_s / 1.596)` |

`homing_height_mm` in config defines what physical height the motor position 0 corresponds to (set after homing to the hard stop at the top).

---

## CANopen communication overview

EZmotion C2 motors use **standard 11-bit CAN frames** at configurable baud rates (10 kbps – 1 Mbps).

### COB-IDs per node

| Direction | Object | COB-ID |
|---|---|---|
| Host → Motor | NMT | `0x000` |
| Host → Motor | SDO download (write) | `0x600 + node_id` |
| Host → Motor | RPDO1 | `0x200 + node_id` |
| Host → Motor | RPDO2 | `0x300 + node_id` |
| Host → Motor | RPDO3 | `0x400 + node_id` |
| Host → Motor | RPDO4 | `0x500 + node_id` |
| Motor → Host | SDO upload (read reply) | `0x580 + node_id` |
| Motor → Host | TPDO1 | `0x180 + node_id` |
| Motor → Host | TPDO2 | `0x280 + node_id` |
| Motor → Host | TPDO3 | `0x380 + node_id` |
| Motor → Host | TPDO4 | `0x480 + node_id` |
| Motor → Host | NMT heartbeat | `0x700 + node_id` |

### Default PDO mappings (from EDS)

| PDO | Mapped objects |
|---|---|
| RPDO3 (`0x400+n`) | Controlword (6040h, 16-bit) + Target position (607Ah, 32-bit) |
| RPDO4 (`0x500+n`) | Controlword (6040h, 16-bit) + Target velocity (60FFh, 32-bit) |
| TPDO3 (`0x380+n`) | Statusword (6041h, 16-bit) + Position actual (6064h, 32-bit) |
| TPDO4 (`0x480+n`) | Statusword (6041h, 16-bit) + Velocity actual (606Ch, 32-bit) |

### Unit conventions

| Quantity | Unit | Encoder unit | Conversion |
|---|---|---|---|
| Position | rad | counts (int32) | 65536 counts/rev ÷ 2π |
| Velocity | rad/s | counts/s (int32) | same factor |
| Torque | N·m | 0.1% of rated torque | ÷ 1000 × rated_torque_Nm |

---

## DS402 state machine

```
Power on
   │
Not Ready to Switch On  (auto)
   │
Switch On Disabled      (auto)
   │
Ready to Switch On  ←── controlword 0x0006  (Shutdown)
   │
Operation Enabled   ←── controlword 0x000F  (Switch on + Enable operation)
```

`motor.enable()` performs the full sequence automatically, including fault reset if in fault state.

## Operation modes (DS402 6060h)

| Mode | Value | Description |
|---|---|---|
| PP | 1 | Profile Position — trapezoidal trajectory |
| PV | 3 | Profile Velocity |
| PT | 4 | Profile Torque |
| HM | 6 | Homing |
| CSP | 8 | Cyclic Synchronous Position |
| CSV | 9 | Cyclic Synchronous Velocity |
| CST | 10 | Cyclic Synchronous Torque |

---

## Usage example (Python API)

```python
from ezmotion_p import EZMotionCANComms, MMS760400_48_C2_1, OperationMode

with EZMotionCANComms(channel='can0', bitrate=1_000_000) as bus:
    bus.start_listener()

    motor = MMS760400_48_C2_1(node_id=1, comms=bus)

    motor.nmt_start()
    motor.enable()

    # Profile Position
    motor.set_operation_mode(OperationMode.PP)
    motor.set_profile_velocity(1.0)
    motor.set_profile_acceleration(5.0)
    motor.set_profile_deceleration(5.0)
    motor.trigger_move_pp(1.5708)   # 90° — ENABLED → TRIGGER → ENABLED

    # Profile Velocity
    motor.set_operation_mode(OperationMode.PV)
    motor.set_target_velocity_pv(2.0)   # rad/s

    # Cyclic Synchronous Position (real-time loop)
    motor.set_operation_mode(OperationMode.CSP)
    motor.set_cyclic_position(0.0)

    motor.disable()
```

---

## Coexistence with RobStride motors on the same CAN bus

RobStride uses **extended 29-bit frames**; EZmotion uses **standard 11-bit frames**. Both work on the same physical wire. Each comms instance opens a separate SocketCAN socket with independent hardware filters — no conflicts:

```python
from robstride_p import CANComms as RSComms, RS04
from ezmotion_p import EZMotionCANComms, MMS760400_48_C2_1

rs_bus = RSComms(channel='can0')
ez_bus = EZMotionCANComms(channel='can0')   # separate socket

rs_bus.start_listener()
ez_bus.start_listener()

rs_motor = RS04(motor_id=1, comms=rs_bus)
ez_motor = MMS760400_48_C2_1(node_id=5, comms=ez_bus)
```

---

## Key objects

### `EZMotionCANComms`
CAN transport. One SocketCAN socket with `extended=False` hardware filter. Dispatches frames to motor callbacks by COB-ID.

### `EZMotionMotorBase`
- `nmt_start/stop/pre_operational/reset()` — NMT state
- `enable() / disable() / quick_stop() / fault_reset()` — DS402 state machine
- `set_operation_mode(OperationMode.PP)` — switch mode (SDO 6060h)
- `write_sdo_u32/s32/u16/s16/u8`, `read_sdo_u32/s32/u16/s16/u8` — SDO access
- `set_profile_velocity/acceleration/deceleration()` — motion profile
- `trigger_move_pp(rad)` — PP position command via RPDO3 (ENABLED → TRIGGER → ENABLED)
- `set_target_velocity_pv(rad_s)` — PV velocity via RPDO4
- `set_cyclic_position/velocity()` — CSP/CSV real-time commands
- `set_target_torque/set_max_torque()` — torque (SDO)
- `save_params()` — persist to NVM (1010h)
- `feedback` property — latest `MotorFeedback` from TPDOs

### `MMS760400_48_C2_1`
Subclass: `RATED_TORQUE_NM=3.5`, `MAX_SPEED_RPM=3000`, `MAX_TORQUE_PERMIL=3000`.

### `MotorFeedback` (dataclass)

| Field | Type | Description |
|---|---|---|
| `position` | float | rad — TPDO3 |
| `velocity` | float | rad/s — TPDO4 |
| `statusword` | int | raw DS402 statusword |
| `drive_state` | DriveState | decoded state |
| `op_mode` | int | active mode — TPDO2 |
| `fault` | bool | statusword bit 3 |
| `enabled` | bool | Operation Enabled state |

### `OD` (IntEnum) — key entries

| Entry | Index | Description |
|---|---|---|
| `CTRL_WORD` | 0x6040 | Controlword |
| `STATUS_WORD` | 0x6041 | Statusword |
| `MODES_OF_OP` | 0x6060 | Modes of operation |
| `POS_ACTUAL` | 0x6064 | Position actual |
| `VEL_ACTUAL` | 0x606C | Velocity actual |
| `TARGET_POSITION` | 0x607A | Target position |
| `PROFILE_VEL` | 0x6081 | Profile velocity |
| `PROFILE_ACCEL` | 0x6083 | Profile acceleration |
| `PROFILE_DECEL` | 0x6084 | Profile deceleration |
| `TARGET_VELOCITY` | 0x60FF | Target velocity |
| `HOMING_METHOD` | 0x6098 | Homing method |
| `HOMING_SPEED` | 0x6099 | Homing speeds (sub1=search, sub2=zero) |
| `HOMING_ACCEL` | 0x609A | Homing acceleration |
| `HOME_OFFSET` | 0x607C | Home offset |
| `HOMING_TORQUE` | 0x2070 | Max homing torque (sub1) |
| `TEMPERATURE` | 0x2040 | Motor temperature |
| `ERROR_STATUS` | 0x200B | Error status (sub 0x0B) |
| `STORE_PARAMS` | 0x1010 | Save to NVM |
