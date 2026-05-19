# mimic

ROS 2 package that mirrors one arm's position onto the other in real time, with configurable per-joint transforms and live direction switching.

---

## Package structure

```
mimic/
├── config/
│   └── mimic.toml               # Single unified config
├── launch/
│   └── mimic.launch.py
├── mimic/
│   ├── mimic_node.py
│   └── transforms.py            # Per-joint position transform functions
```

---

## Config (`mimic.toml`)

| Field | Description |
|---|---|
| `left_arm_node_prefix` | ROS 2 node prefix for the physical left arm (e.g. `"/left_arm"`) |
| `right_arm_node_prefix` | ROS 2 node prefix for the physical right arm (e.g. `"/right_arm"`) |
| `target_node` | Which arm receives commands: `"left_arm"` or `"right_arm"`. The other arm becomes the source. |
| `motors` | Base motor names without L/R suffix (e.g. `["Sp", "Sr", "Sw", "Ep", "Ww", "Wp", "Wr"]`). L/R suffix is appended automatically. |
| `mode` | Control mode applied to target motors at startup: `"csp"` or `"pp"` |
| `debug` | If `true`, commands go to `~/mimic/debug/motors/{name}/cmd_*` instead of real topics |
| `active_report_hz` | Active reporting rate requested from both arms at startup (Hz) |
| `op_hz` | If set, publish at this fixed rate using latest state; if absent or `0`, forward on every incoming state message |
| `[transform_map]` | Transforms applied when `target_node = "right_arm"` (source = left arm). Keys are base names (e.g. `Sp`). |
| `[inverse_transform_map]` | Transforms applied when `target_node = "left_arm"` (source = right arm). Keys are base names. |
| `[pp_defaults]` | Default PP parameters: `speed`, `acceleration`, `deceleration`, `torque_limit` |
| `[csp_defaults]` | Default CSP parameters: `speed_limit`, `current_limit` |

### Direction logic

| `target_node` | Source | Target | Transform used |
|---|---|---|---|
| `"right_arm"` | left arm | right arm | `transform_map` |
| `"left_arm"` | right arm | left arm | `inverse_transform_map` |

Switch direction at runtime without restarting the node via the `switch_target` service.

---

## Transforms

Defined in `transforms.py`. Each function: `(position: float) -> float`.

| Name | Formula |
|---|---|
| `passthrough` | `x` |
| `negate` | `-x` |
| `subtract_from_2pi` | `2π − x` |
| `subtract_2pi` | `x − 2π` |

---

## Node initialisation

On startup the node:

1. Loads `config.toml` from the `config_path` ROS 2 parameter.
2. Reads `target_node` to determine initial source/target direction.
3. Wires up state subscriptions (source arm) and command publishers (target arm).
4. Creates service clients for `set_active_report`, `set_run_mode`, and `enable_motor` on both arms.
5. After 1 s: enables active reporting on both arms, sets target motors to the configured run mode.

Direction changes via `switch_target` repeat steps 2–5 atomically.

---

## Services

### `~/set_mode`
Change the control mode (`"pp"` or `"csp"`) applied to target motors.

**Type:** `custom_interfaces/srv/SetMimicMode`  
**Request:** `mode` (`"pp"` or `"csp"`)  
**Response:** `success`, `message`

---

### `~/switch_target`
Switch which arm is the target (and therefore which is the source) without restarting the node. Tears down existing subscriptions/clients and rebuilds them for the new direction, then re-runs the deferred setup.

**Type:** `custom_interfaces/srv/SetMimicTarget`  
**Request:** `target` (`"left_arm"` or `"right_arm"`)  
**Response:** `success`, `message`

```bash
ros2 service call /mimic_node/switch_target \
  custom_interfaces/srv/SetMimicTarget "{target: 'left_arm'}"
```

---

### `~/set_params`
Update default PP or CSP motion parameters at runtime.

**Type:** `custom_interfaces/srv/SetMimicParams`  
**Request:** `mode` (`"pp"` or `"csp"`), plus the relevant parameter fields (0.0 = no change)  
**Response:** `success`, `message`

---

### `~/enable_motors`
Enable or disable target motors.

**Type:** `custom_interfaces/srv/EnableMimicMotors`  
**Request:** `names[]` (`["all"]` for broadcast), `enable`, `clear_fault`  
**Response:** `success`, `message`

---

## Shutdown behaviour

On SIGINT, `shutdown_cleanup` disables active reporting on both arms and disables all target motors before the ROS 2 context shuts down.

---

## Launch

```bash
ros2 launch mimic mimic.launch.py

# Custom config
ros2 launch mimic mimic.launch.py config:=/path/to/custom.toml
```
