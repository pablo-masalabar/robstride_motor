# damiao_p

ROS2 Jazzy driver package for Damiao DM-J43xx geared motors over CAN.

Currently supported motor: **DM-J4310-2EC** (10:1 gear ratio, 12.5 N·m peak, 20 A peak, CAN @ 1 Mbps).

---

## Package layout

```
damiao_p/
├── damiao_p/
│   ├── __init__.py        # public exports
│   ├── comms.py           # CAN transport layer (DamiaoCANComms)
│   ├── motor_base.py      # protocol implementation (DamiaoMotorBase)
│   ├── j4310_2ec.py       # motor-specific subclass (J4310_2EC)
│   └── motor_node.py      # ROS2 node
├── config/
│   └── grippers.toml      # example gripper configuration
├── launch/
│   └── grippers.launch.py # launch file
└── docs/
    └── DM-J4310-2EC_V1.2.pdf
```

---

## CAN protocol overview

Damiao motors use **standard 11-bit CAN frames** (CAN 2.0B STD) at 1 Mbps — different from the extended 29-bit frames used by RobStride. Both can coexist on the same physical bus; the kernel's per-socket filters isolate them.

### Frame IDs

| Direction | Frame type | Arbitration ID |
|---|---|---|
| Host → Motor | MIT control | `motor_id` |
| Host → Motor | Position-Velocity control | `0x100 + motor_id` |
| Host → Motor | Velocity control | `0x200 + motor_id` |
| Host → Motor | Force-Position Hybrid control | `0x300 + motor_id` |
| Host → Motor | Parameter read/write/save | `0x7FF` |
| Motor → Host | All replies (feedback + param) | `master_id` (default 0) |

All motor replies arrive on the same `master_id` arbitration ID. The motor is identified within the reply by `D[0]`:
- Feedback frames: `D[0] = (ERR << 4) | (motor_id & 0x0F)`
- Parameter replies: `D[0] = motor_id & 0xFF`, distinguished by `D[2]` (0x33 = read, 0x55 = write, 0xAA = save)

Motor IDs must be **0–15** (4-bit limit of the feedback frame encoding).

### Feedback frame (8 bytes)

| Byte | Field | Description |
|---|---|---|
| D[0] | `(ERR<<4) \| motor_id` | ERR upper nibble, motor ID lower nibble |
| D[1:2] | POS[15:0] | 16-bit signed position, range ±PMAX |
| D[3] upper / D[4] upper | VEL[11:0] | 12-bit signed velocity, range ±VMAX |
| D[4] lower / D[5] | T[11:0] | 12-bit signed torque, range ±TMAX |
| D[6] | T_MOS | MOSFET temperature °C |
| D[7] | T_Rotor | Motor winding temperature °C |

Position, velocity, and torque are signed fixed-point values linearly mapped to ±PMAX / ±VMAX / ±TMAX. These ranges must match the motor's `PMAX` (0x15), `VMAX` (0x16), `TMAX` (0x17) registers.

### Control modes

| Mode | `RunMode` enum | Register value |
|---|---|---|
| MIT (torque + position + velocity gains) | `RunMode.MIT` | 1 |
| Position-Velocity (trapezoidal profile) | `RunMode.POSITION_VELOCITY` | 2 |
| Velocity | `RunMode.VELOCITY` | 3 |
| Force-Position Hybrid (pos + current cap) | `RunMode.FORCE_POSITION_HYBRID` | 4 |

### Special command bytes (last byte of control frame)

| Value | Command |
|---|---|
| `0xFC` | Enable motor |
| `0xFD` | Disable motor |
| `0xFE` | Set current position as zero |
| `0xFB` | Clear faults |

### Save parameters constraint

The motor only writes parameters to flash while **disabled**. Any operation that calls `save_params()` — the `save_params` service and `write_param` with `persist=True` — automatically disables the motor, saves, waits 50 ms for the flash write, then re-enables. Flash endurance is ~10,000 cycles.

---

## Python library (`damiao_p` module)

### `DamiaoCANComms`

CAN transport. One instance per physical bus (shared across all motors on that bus).

```python
from damiao_p import DamiaoCANComms, J4310_2EC

bus = DamiaoCANComms(channel='can0', master_id=0)
bus.start_listener()           # starts background receive thread

m1 = J4310_2EC(motor_id=1, master_id=0, comms=bus)
m2 = J4310_2EC(motor_id=2, master_id=0, comms=bus)

# context manager is also supported:
with DamiaoCANComms('can0') as bus:
    bus.start_listener()
    ...
```

### `DamiaoMotorBase` / `J4310_2EC`

Protocol implementation. `J4310_2EC` is the concrete class — it sets the motor-specific limits:

```
T_MAX = 12.5 N·m    MIT_P_MAX = 12.5 rad
V_MAX = 20.94 rad/s  MIT_V_MAX = 45.0 rad/s
MAX_CURRENT_A = 20 A MIT_T_MAX = 12.0 N·m
```

#### Core commands

```python
motor.enable()
motor.disable()
motor.clear_faults()
motor.set_zero_position()        # set current position as zero reference
motor.set_run_mode(RunMode.MIT)  # switch control mode (volatile unless saved)
```

#### Control commands

```python
# MIT mode
motor.set_operation_control(position=1.0, velocity=0.0, kp=30.0, kd=0.5, torque_ff=0.0)

# Position-Velocity mode (trapezoidal profile)
motor.set_position_velocity(position_rad=1.57, velocity_rad_s=2.0)

# Velocity mode
motor.set_velocity(velocity_rad_s=5.0)

# Force-Position Hybrid mode
motor.set_force_position(position_rad=1.57, velocity_rad_s=2.0, current_per_unit=0.8)
```

#### Parameter read/write

```python
from damiao_p import RegAddr

motor.write_param_float(RegAddr.ACC, 10.0)     # volatile
motor.write_param_uint(RegAddr.CTRL_MODE, 2)

val = motor.read_param_float(RegAddr.VBUS)     # blocks up to rx_timeout
val = motor.read_param_uint(RegAddr.SW_VER)

motor.disable()
motor.save_params()                            # persist to flash (must be disabled)
motor.enable()
```

Convenience helpers:

```python
motor.set_acceleration(10.0)        # ACC register
motor.set_deceleration(10.0)        # DEC register (written as negative internally)
motor.set_max_speed(15.0)           # MAX_SPD register

motor.read_output_position()        # XOUT (0x51) — absolute output shaft rad
motor.read_motor_position()         # P_M  (0x50) — motor-side rotor rad
motor.read_vbus()                   # bus voltage V
motor.read_pcb_temp()               # PCB temperature °C
motor.read_motor_temp()             # winding temperature °C
motor.read_firmware_version()       # SW_VER
motor.read_gear_ratio()             # GR (10.0 for J4310-2EC)
motor.read_imax()                   # driver current limit A
```

#### Getting feedback

**Callback (event-driven)** — called in the Notifier thread on every CAN reply:

```python
from damiao_p import MotorFeedback

def on_feedback(fb: MotorFeedback):
    print(fb.position, fb.velocity, fb.torque, fb.t_mos, fb.err, fb.enabled)

motor.set_feedback_callback(on_feedback)
```

**Property** — last received state, updated after every command:

```python
fb = motor.feedback
print(fb.position, fb.velocity, fb.torque)
```

`MotorFeedback` fields:

| Field | Type | Description |
|---|---|---|
| `position` | float | Output shaft position (rad), relative to zero |
| `velocity` | float | Output shaft velocity (rad/s) |
| `torque` | float | Estimated torque (N·m) |
| `t_mos` | float | MOSFET/driver temperature (°C) |
| `t_rotor` | float | Motor winding temperature (°C) |
| `err` | int | `FaultCode` value |
| `enabled` | bool | True when `err == FaultCode.ENABLED` |

#### `FaultCode` values

| Code | Meaning |
|---|---|
| 0 | Disabled (default after power-on) |
| 1 | Enabled (normal run state) |
| 8 | Over-voltage |
| 9 | Under-voltage |
| 10 | Over-current |
| 11 | MOSFET over-temperature |
| 12 | Coil over-temperature |
| 13 | Communication lost |
| 14 | Stall / overload |

---

## ROS2 node (`motor_node`)

### Topics

| Topic | Type | Description |
|---|---|---|
| `~/joint_states` | `sensor_msgs/JointState` | All joints at `update_rate_hz` (position, velocity, effort) |
| `~/motors/{name}/state` | `custom_interfaces/MotorState` | Per-motor state on every CAN reply |
| `~/motors/{name}/fault` | `custom_interfaces/MotorFault` | Per-motor fault flags, published only on change |
| `~/motors/{name}/cmd_mit` | `custom_interfaces/OperationCommand` | MIT mode command |
| `~/motors/{name}/cmd_position_pv` | `custom_interfaces/PositionPPCommand` | Position-Velocity command |
| `~/motors/{name}/cmd_velocity` | `custom_interfaces/VelocityCommand` | Velocity command |
| `~/motors/{name}/cmd_force_position` | `custom_interfaces/PositionCSPCommand` | Force-Position Hybrid command |

`~/joint_states` publishes at a fixed timer rate. `~/motors/{name}/state` publishes at the command rate (every time a command is sent and the motor replies).

### Services

| Service | Type | Description |
|---|---|---|
| `~/enable_motor` | `custom_interfaces/EnableMotor` | Enable or disable a motor (or `"all"`) |
| `~/set_run_mode` | `custom_interfaces/SetRunMode` | Switch control mode |
| `~/set_zero_position` | `custom_interfaces/SetZeroPosition` | Set zero reference |
| `~/read_param` | `custom_interfaces/ReadParam` | Read a register by address |
| `~/write_param` | `custom_interfaces/WriteParam` | Write a register (`persist=true` saves to flash) |
| `~/motor_param` | `custom_interfaces/MotorParam` | Get/set software motor params (limits, gains) |
| `~/help` | `custom_interfaces/Help` | List registers with types and access |
| `~/stop_all` | `std_srvs/Trigger` | Disable all motors immediately |
| `~/clear_faults` | `std_srvs/Trigger` | Clear latched fault state on all motors |
| `~/save_params` | `std_srvs/Trigger` | Disable → save flash → re-enable all motors |
| `~/homing` | `std_srvs/Trigger` | Command all motors to position 0.0 in PV mode |

### Actions

| Action | Type | Description |
|---|---|---|
| `~/move_to_position` | `custom_interfaces/MoveToPosition` | Move to a target position with tolerance and timeout |
| `~/set_velocity` | `custom_interfaces/SetVelocity` | Run at a target velocity for a given duration |

### Node parameter

| Parameter | Default | Description |
|---|---|---|
| `config_path` | _(required)_ | Absolute path to the TOML config file |

---

## Configuration (`config/grippers.toml`)

### `[defaults]` section

| Key | Required | Description |
|---|---|---|
| `master_id` | yes | Host CAN ID (must match `MST_ID` register on motors) |
| `channel` | yes | SocketCAN interface, e.g. `"can0"` |
| `bustype` | yes | python-can bus type, typically `"socketcan"` |
| `bitrate` | yes | CAN baud rate in bps (`1000000` for DM-J43xx) |
| `rx_timeout` | yes | Seconds to wait for a parameter read reply |
| `update_rate_hz` | yes | `joint_states` publish rate (Hz) |
| `node_name` | no | ROS2 node name and topic namespace prefix |
| `use_node_name_as_topic_base` | no | `true` → topics at `~/<name>`; `false` → root namespace |
| `operation_mode` | no | Default control mode for all motors |

### Per-motor section (`[joint_name]`)

| Key | Required | Description |
|---|---|---|
| `type` | yes | Motor class, e.g. `"J4310_2EC"` |
| `motor_id` | yes | Motor ESC_ID register value (0–15) |
| `channel` | no | Overrides `defaults.channel` for this motor |
| `bustype` | no | Overrides `defaults.bustype` |
| `bitrate` | no | Overrides `defaults.bitrate` |
| `master_id` | no | Overrides `defaults.master_id` |
| `rx_timeout` | no | Overrides `defaults.rx_timeout` |
| `operation_mode` | no | Overrides `defaults.operation_mode` |
| `joint_limit_min` | no | Minimum allowed position (rad); commands outside range rejected |
| `joint_limit_max` | no | Maximum allowed position (rad) |
| `motor_homing_pos` | no | Subtracted from raw motor position to give user-frame position (rad) |
| `max_vel` | no | Velocity command clamp (rad/s) |
| `max_accel` | no | Acceleration clamp for action server goals (rad/s²) |
| `max_decel` | no | Deceleration clamp (rad/s²); stored but not yet applied in commands |
| `acc` | no | ACC register (0x04) — acceleration ramp rad/s²; written at startup if present |
| `dec` | no | DEC register (0x05) — deceleration magnitude rad/s²; written as negative internally |
| `max_spd` | no | MAX_SPD register (0x06) — velocity cap rad/s; written at startup if present |
| `kp_apr` | no | KP_APR register (0x1B) — position loop Kp; written at startup if present |
| `ki_apr` | no | KI_APR register (0x1C) — position loop Ki; written at startup if present |
| `kp_asr` | no | KP_ASR register (0x19) — velocity loop Kp; written at startup if present |
| `ki_asr` | no | KI_ASR register (0x1A) — velocity loop Ki; written at startup if present |
| `kp` | no | MIT mode position gain [0, 500]; defaults to 0.0 if absent |
| `kd` | no | MIT mode velocity gain [0, 5]; defaults to 0.0 if absent |

---

## Running

### Build

```bash
cd ros_ws
colcon build --packages-select damiao_p
source install/setup.bash
```

### Launch

```bash
# default config
ros2 launch damiao_p grippers.launch.py

# custom config
ros2 launch damiao_p grippers.launch.py config:=/path/to/my_config.toml
```

### Quick service calls

```bash
# enable motor
ros2 service call /damiao/enable_motor custom_interfaces/srv/EnableMotor "{name: 'joint_1', enable: true}"

# move to position (action)
ros2 action send_goal /damiao/move_to_position custom_interfaces/action/MoveToPosition \
  "{name: 'joint_1', target_position: 1.57, speed_limit: 2.0, tolerance: 0.01, timeout: 10.0}"

# read bus voltage
ros2 service call /damiao/read_param custom_interfaces/srv/ReadParam "{name: 'joint_1', index: 60}"
# 0x3C = 60 = VBUS

# save parameters to flash
ros2 service call /damiao/save_params std_srvs/srv/Trigger {}

# stop all
ros2 service call /damiao/stop_all std_srvs/srv/Trigger {}
```

### Monitor feedback

```bash
ros2 topic echo /damiao/joint_states
ros2 topic echo /damiao/motors/joint_1/state
ros2 topic echo /damiao/motors/joint_1/fault
```

---

## Multi-motor / multi-bus

Motors on the same interface share one `DamiaoCANComms` instance and one background receive thread. Motors on different interfaces each get their own instance, determined automatically from config:

```toml
[defaults]
channel = "can0"
master_id = 0

[joint_1]
motor_id = 1          # uses can0, master_id=0

[joint_2]
motor_id = 2          # uses can0, master_id=0 — shares the same bus instance

[joint_3]
motor_id = 1
channel = "can1"      # different interface — gets its own bus instance
```

Bus instances are keyed on `(channel, bustype, bitrate, master_id)`. Two motors that differ only in `master_id` also get separate instances.

## Coexistence with RobStride on the same bus

RobStride motors use extended 29-bit frames; Damiao uses standard 11-bit frames. The CAN physical layer carries both simultaneously. Each driver installs a kernel-level frame-type filter on its own socket, so neither sees the other's traffic.
