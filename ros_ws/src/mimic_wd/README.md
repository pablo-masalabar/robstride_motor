# mimic_wd

Whole-body dynamics mimic node. Mirrors one arm onto the other with gravity and Coriolis compensation computed via Pinocchio, optionally using a full FK→mirror→IK pipeline instead of direct joint angle forwarding.

Extends `mimic` by enabling active reporting on **both** source and target arms and using a fixed-rate IK timer as the sole command publisher.

---

## Overview

| Mode | Robstride command | Damiao command |
|---|---|---|
| `use_ik = false` | Direct joint mirroring + NLE torque_ff | Direct joint mirroring |
| `use_ik = true` | FK → mirror → IK + NLE torque_ff | Direct joint mirroring |

**Motor control law (OPERATION mode):**
```
τ = Kp·(p_set − p_actual) + Kd·(v_set − v_actual) + τ_ff
```
`Kp`/`Kd` are set per-motor in the drive config at startup. `τ_ff` is filled from `pin.nonLinearEffects` evaluated at the target arm's current (or IK-solved) joint state.

---

## Launch

```bash
ros2 launch mimic_wd mimic_wd.launch.py
```

Custom config:
```bash
ros2 launch mimic_wd mimic_wd.launch.py config:=/path/to/mimic.toml
```

---

## Config (`config/mimic.toml`)

### URDF / Kinematics

| Field | Description |
|---|---|
| `urdf_prefix` | ROS package that contains the URDF files |
| `left_arm_urdf_path` | Path to left arm URDF relative to the package share directory |
| `right_arm_urdf_path` | Path to right arm URDF relative to the package share directory |
| `left_ee_frame` | End-effector frame name in the left arm URDF |
| `right_ee_frame` | End-effector frame name in the right arm URDF |
| `base_frame` | Base frame used for RViz marker publishing |

### Behaviour flags

| Field | Default | Description |
|---|---|---|
| `use_ik` | `false` | `true` → FK→mirror→IK pipeline; `false` → direct joint angle mirroring |
| `debug` | `false` | `true` → commands routed to `~/mimic_wd/debug/…` topics only, motors untouched |
| `visualize_rviz` | `false` | `true` → publish EE axes as `MarkerArray` on `~/ee_markers` |

### Rates

| Field | Default | Description |
|---|---|---|
| `active_report_hz` | `50.0` | Motor state reporting rate enabled on both source and target arms |
| `ik_hz` | `30.0` | Rate of the IK timer (command publish rate) |

### Direction

| Field | Description |
|---|---|
| `target_node` | `"right_arm"` → left arm is source, right arm receives commands; `"left_arm"` → reversed |
| `left_arm_node_prefix` | ROS topic prefix for left arm node |
| `right_arm_node_prefix` | ROS topic prefix for right arm node |
| `left_gripper_node_prefix` | ROS topic prefix for left gripper node |
| `right_gripper_node_prefix` | ROS topic prefix for right gripper node |
| `robstride_motors` | Base motor names (L/R suffix appended automatically) |
| `damiao_motors` | Base motor names for Damiao gripper motors |

### Motor defaults

```toml
[robstride_operation_defaults]
velocity  = 0.0   # rad/s — velocity feedforward (overridden per-motor by NLE when model loaded)
torque_ff = 0.0   # N·m  — fallback torque_ff if NLE unavailable

[damiao_pv_defaults]
speed = 25.0      # rad/s — max speed for position_velocity mode

[damiao_mit_defaults]
velocity  = 0.0   # rad/s — velocity feedforward
torque_ff = 0.0   # N·m  — torque feedforward
```

### Transform maps

Applied to joint positions and velocities when mirroring source → target. Keys are base motor names (no L/R suffix). Available functions: `passthrough`, `negate`, `add_half_pi` (from `mimic/transforms.py`).

```toml
[robstride_transform_map]        # used when target_node = "right_arm"
[robstride_inverse_transform_map] # used when target_node = "left_arm"
[damiao_transform_map]
[damiao_inverse_transform_map]
```

---

## Topics

### Subscribed

| Topic | Type | Description |
|---|---|---|
| `{src_prefix}/motors/{motor}/state` | `MotorState` | Source arm joint states |
| `{tgt_prefix}/motors/{motor}/state` | `MotorState` | Target arm joint states (used for IK warm-start and NLE) |
| `{src_gripper_prefix}/motors/{motor}/state` | `MotorState` | Source gripper states |
| `{tgt_gripper_prefix}/motors/{motor}/state` | `MotorState` | Target gripper states |

### Published

| Topic | Type | Description |
|---|---|---|
| `{tgt_prefix}/motors/{motor}/cmd_operation` | `OperationCommand` | Robstride MIT impedance commands |
| `{tgt_gripper_prefix}/motors/{motor}/cmd_position_pv` | `PositionPPCommand` | Damiao PV commands |
| `{tgt_gripper_prefix}/motors/{motor}/cmd_mit` | `OperationCommand` | Damiao MIT commands |
| `~/mimic_wd/debug/motors/{motor}/cmd_*` | — | Debug topic mirrors (active when `debug=true`) |
| `~/ee_markers` | `MarkerArray` | EE axis markers for source and target arm (active when `visualize_rviz=true`) |

---

## Services

| Service | Type | Description |
|---|---|---|
| `~/switch_target` | `SetMimicTarget` | Swap source and target arm at runtime |
| `~/set_mode` | `SetMimicMode` | Change robstride run mode (currently only `operation`) |
| `~/set_params` | `SetMimicParams` | Update runtime defaults |
| `~/enable_motors` | `EnableMimicMotors` | Enable/disable specific target motors |
| `~/set_debug` | `SetBool` | Toggle debug mode at runtime |

### `set_params` modes

```bash
# Robstride operation defaults
ros2 service call ~/set_params custom_interfaces/srv/SetMimicParams \
  "{mode: 'operation', speed: 1.0, torque_limit: 0.5}"

# Damiao position_velocity speed
ros2 service call ~/set_params custom_interfaces/srv/SetMimicParams \
  "{mode: 'position_velocity', speed: 10.0}"

# Damiao MIT defaults
ros2 service call ~/set_params custom_interfaces/srv/SetMimicParams \
  "{mode: 'mit', speed: 0.5, torque_limit: 0.2}"
```

### Switch target direction

```bash
ros2 service call ~/switch_target custom_interfaces/srv/SetMimicTarget "{target: 'left_arm'}"
```

---

## IK pipeline (`use_ik = true`)

1. Build source arm joint config `q_src` from current source motor states
2. Run FK → source EE pose (SE3) in base frame
3. Mirror pose: `t_mirror = [tx, -ty, tz]`, `R_mirror = M @ R @ M` where `M = diag(1, -1, 1)`
4. Warm-start IK from current target arm joint state
5. Damped least-squares IK on target arm model until EE reaches mirrored pose
6. Compute `tau_nle = pin.nonLinearEffects(model, data, q_ik, 0)` — gravity compensation only
7. Publish `OperationCommand` per motor with IK joint angles and NLE torques

---

## Joint mapping

`hw_joints_to_sim_joint_mapping.py` maps hardware motor names (`SpL`, `EpR`, …) to URDF joint names. Pinocchio joint indices are resolved from this mapping at startup via `_build_joint_map`. Motors not found in the URDF produce a startup warning and fall back to config torque_ff defaults.

---

## Dependencies

- `rclpy`, `custom_interfaces`, `mimic`
- `pinocchio` — rigid-body dynamics and IK
- `visualization_msgs`, `geometry_msgs` — RViz marker publishing
