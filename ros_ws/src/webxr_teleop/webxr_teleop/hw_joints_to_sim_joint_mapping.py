"""Hardware → sim joint name mapping for all Feather services.

Keys are the joint identifiers published by the hardware actuator stack.
Values are the corresponding joint ids used by the feather services and the
sim (arm.py, torso.py, neck.py, chassis.py).

Usage::

    from feather.hw_joints_to_sim_joint_mapping import HW_TO_SIM, SIM_TO_HW

    sim_name = HW_TO_SIM[hw_name]
    hw_name  = SIM_TO_HW[sim_name]
"""

HW_TO_SIM: dict[str, str] = {
    # ── Left arm ────────────────────────────────────────────────────────────
    "SpL":  "left_shoulder_pitch_joint",
    "SrL":   "left_shoulder_roll_joint",
    "SwL":    "left_shoulder_yaw_joint",
    "EpL":     "left_elbow_pitch_joint",
    "WwL":   "left_forearm_twist_joint",
    "WpL":     "left_wrist_pitch_joint",
    "WrL":                  "WpL",
    "AgL":    "gripperL_to_tip_left",
    "arm_to_gripperL": "arm_to_gripperL",

    # ── Right arm ───────────────────────────────────────────────────────────
    "SpR": "right_shoulder_pitch_joint",
    "SrR":  "right_shoulder_roll_joint",
    "SwR":   "right_shoulder_yaw_joint",
    "EpR":    "right_elbow_pitch_joint",
    "WwR":  "right_forearm_twist_joint",
    "WpR":    "right_wrist_pitch_joint",
    "WrR":                  "WpR",
    "AgR":   "gripperR_to_tip_left",
    "arm_to_gripperR": "arm_to_gripperR",

    # ── Torso ────────────────────────────────────────────────────────────────
    "torso": "torso_lift_joint",

    # ── Neck ─────────────────────────────────────────────────────────────────
    "NwC":   "neck_yaw_joint",
    "NpC": "neck_pitch_joint",

    # ── Chassis — steer (POSITION) ───────────────────────────────────────────
    "BwL":  "front_left_steer_joint",
    "BwR": "front_right_steer_joint",
    "BwC":        "rear_steer_joint",

    # ── Chassis — wheel (VELOCITY) ───────────────────────────────────────────
    "BpL":  "front_left_wheel_joint",
    "BpR": "front_right_wheel_joint",
    "BpC":        "rear_wheel_joint",

}

# Reverse mapping — built automatically, no need to maintain separately.
SIM_TO_HW: dict[str, str] = {v: k for k, v in HW_TO_SIM.items()}
