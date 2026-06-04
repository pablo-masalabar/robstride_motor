"""Bridge ROS 2 R1 Lite topics (cameras + joint feedback) onto zenoh.

Subscribes to 3 ROS 2 camera topics (head + wrist L/R, all mono) and 6
`sensor_msgs/JointState` feedback topics (arms, grippers, torso, chassis),
republishes them as `r1lite_pb2` payloads on `r1_lite/...` zenoh keys
that the gateway crate (and other r1lite services) already consume.

The joint `id` strings on the wire are derived from the *parent ROS topic*
rather than from `sensor_msgs/JointState.name` — the real-robot publishers
don't reliably populate `name`, and downstream code (`arm.py._on_state`,
`gateway/src/r1lite_bridge.rs:forward_feedback`) keys off URDF joint names
to drive IK seeding and the ghost-overlay state map. Each topic maps to a
fixed canonical id list (see `TOPIC_JOINT_IDS`) matching the URDF and the
ids that `r1lite/base.py`, `r1lite/torso.py`, and `gateway/src/sim_pub.rs`
write back on the command side.

Run:
    cd galaxea_r1_webxr/r1lite
    .venv/bin/python -m utilities.r1lite_ros2_bridge

Requires `rclpy`, `cv_bridge`-free `sensor_msgs`, `cv2`, and a sourced
ROS 2 Humble environment alongside r1lite's zenoh/protobuf deps.
"""

from __future__ import annotations

import argparse
import os

import cv2
import numpy as np
import rclpy
import zenoh
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import CompressedImage, JointState as RosJointState
from hdas_msg.msg import Bms as RosBms

from r1lite import r1lite_pb2


# URDF prismatic limit for each gripper finger joint. Matches `arm.py:61`.
GRIPPER_LIMIT_M = 0.05

# QoS used by /motion_target/* publishers in the mobiman repo (see
# `understandings/motion_target_topics_r1_lite.md` §0). The jointTracker
# and gripper controller binaries are expected on this profile.
MOTION_TARGET_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)


# Canonical joint id list per feedback group. Matches the URDFs in
# `r1lite/config/` and the ids written on the command side by
# `r1lite/base.py`, `r1lite/torso.py`, and `gateway/src/sim_pub.rs`.
# Order matches the index order the real-robot ROS publishers use, so
# `position[i]` lines up with `TOPIC_JOINT_IDS[group][i]`.
TOPIC_JOINT_IDS: dict[str, list[str]] = {
    "arm_left":      [f"left_arm_joint{i}"  for i in range(1, 7)],
    "arm_right":     [f"right_arm_joint{i}" for i in range(1, 7)],
    "gripper_left":  ["left_gripper_finger_joint1",  "left_gripper_finger_joint2"],
    "gripper_right": ["right_gripper_finger_joint1", "right_gripper_finger_joint2"],
    "torso":         [f"torso_joint{i}" for i in range(1, 4)],
    "chassis": [
        "steer_motor_joint1", "steer_motor_joint2", "steer_motor_joint3",
        "wheel_motor_joint1", "wheel_motor_joint2", "wheel_motor_joint3",
    ],
}

def open_zenoh_session(config_path: str | None):
    if config_path and os.path.isfile(config_path):
        return zenoh.open(zenoh.Config.from_file(config_path)), config_path
    return zenoh.open(zenoh.Config()), None


class R1LiteRos2Bridge(Node):
    def __init__(self, args):
        super().__init__("r1lite_ros2_bridge")

        self.zenoh_session, used_config = open_zenoh_session(args.zenoh_config)
        if used_config:
            self.get_logger().info(f"Zenoh session opened (config: {used_config}).")
        else:
            self.get_logger().info("Zenoh session opened (default config).")

        group = ReentrantCallbackGroup()

        joint_bindings = [
            ("arm_left",      args.left_arm_topic,       args.left_arm_zenoh_key),
            ("arm_right",     args.right_arm_topic,      args.right_arm_zenoh_key),
            ("gripper_left",  args.left_gripper_topic,   args.left_gripper_zenoh_key),
            ("gripper_right", args.right_gripper_topic,  args.right_gripper_zenoh_key),
            ("torso",         args.torso_topic,          args.torso_zenoh_key),
            ("chassis",       args.chassis_topic,        args.chassis_zenoh_key),
        ]
        for feedback_group, ros_topic, zenoh_key in joint_bindings:
            self._add_joint_state(feedback_group, ros_topic, zenoh_key, group)

        # Reverse direction: zenoh joints_cmd -> ROS motion_target/*.
        # Each `JointControlArray` on r1_lite/joints_cmd/arm_{side}` carries
        # 6 arm joints + 2 mirrored gripper fingers. We split it into a
        # 6-DOF arm `JointState` and a 1-DOF gripper `JointState` (in %).
        self._zenoh_arm_cmd_subs: dict[str, zenoh.Subscriber] = {}
        self._ros_arm_cmd_pubs: dict[str, "rclpy.publisher.Publisher"] = {}
        self._ros_gripper_cmd_pubs: dict[str, "rclpy.publisher.Publisher"] = {}

        self._add_arm_cmd_bridge(
            side="left",
            zenoh_key="r1_lite/joints_cmd/arm_left",
            arm_ros_topic="/motion_target/target_joint_state_arm_left",
            gripper_ros_topic="/motion_target/target_position_gripper_left",
            group=group,
        )
        self._add_arm_cmd_bridge(
            side="right",
            zenoh_key="r1_lite/joints_cmd/arm_right",
            arm_ros_topic="/motion_target/target_joint_state_arm_right",
            gripper_ros_topic="/motion_target/target_position_gripper_right",
            group=group,
        )

    def _add_joint_state(self, feedback_group, ros_topic, zenoh_key, group):
        if feedback_group not in TOPIC_JOINT_IDS:
            raise ValueError(f"unknown feedback group {feedback_group!r}")
        joint_ids = TOPIC_JOINT_IDS[feedback_group]
        self.create_subscription(
            RosJointState,
            ros_topic,
            lambda msg, k=zenoh_key, ids=joint_ids, label=feedback_group:
                self._on_joint_state(msg, k, ids, label),
            10,
            callback_group=group,
        )
        self.get_logger().info(
            f"Joint state [{feedback_group}]: {ros_topic} -> {zenoh_key} "
            f"({len(joint_ids)} joints)"
        )

    def _add_bms(self, ros_topic: str, zenoh_key: str, group):
        # /hdas/bms publishes `hdas_msg/Bms` with `capital` carrying the SoC
        # percentage (0..100), alongside `voltage` (V) and `current` (A).
        # Mirrored onto `r1_lite/hdas/bms` as `r1lite.BmsState`, renaming
        # `capital -> soc` so consumers see an obvious name.
        self.create_subscription(
            RosBms,
            ros_topic,
            lambda msg, k=zenoh_key: self._on_bms(msg, k),
            10,
            callback_group=group,
        )
        self.get_logger().info(f"BMS: {ros_topic} -> {zenoh_key}")

    def _on_bms(self, msg: RosBms, zenoh_key: str):
        state = r1lite_pb2.BmsState()
        state.stamp.seconds = int(msg.header.stamp.sec)
        state.stamp.nanoseconds = int(msg.header.stamp.nanosec)
        state.voltage = float(msg.voltage)
        state.current = float(msg.current)
        state.soc = float(msg.capital)
        self.zenoh_session.put(zenoh_key, state.SerializeToString())

    def _on_joint_state(
        self,
        msg: RosJointState,
        zenoh_key: str,
        joint_ids: list[str],
        feedback_group: str,
    ):
        # ROS publishers on the real robot don't reliably populate
        # `msg.name`; we drive id assignment off the topic's canonical id
        # list instead. Whichever of position/velocity/effort the publisher
        # provides gets index-mapped onto `joint_ids`; missing values fall
        # back to 0.0 (proto3 defaults). Trailing samples beyond the
        # canonical count are dropped silently — the real robot publishes
        # extra padding on some topics (e.g. `/hdas/feedback_torso` carries
        # 4 values where only `torso_joint1..3` are URDF-modeled).
        positions = msg.position
        velocities = msg.velocity
        efforts = msg.effort

        if feedback_group in ("gripper_left", "gripper_right"):
            self._publish_gripper_feedback(zenoh_key, joint_ids, positions, velocities, efforts)
            return

        incoming_count = max(len(positions), len(velocities), len(efforts))

        array = r1lite_pb2.JointStatesArray()
        for index, joint_id in enumerate(joint_ids):
            if index >= incoming_count:
                break
            joint = array.joints.add()
            joint.id = joint_id
            joint.position = float(positions[index]) if index < len(positions) else 0.0
            joint.velocity = float(velocities[index]) if index < len(velocities) else 0.0
            joint.torque = float(efforts[index]) if index < len(efforts) else 0.0

        self.zenoh_session.put(zenoh_key, array.SerializeToString())

    def _publish_gripper_feedback(
        self,
        zenoh_key: str,
        joint_ids: list[str],
        positions,
        velocities,
        efforts,
    ):
        # The real-robot gripper controller publishes a SINGLE value per
        # `/hdas/feedback_gripper_*`: the inter-finger gap as 0..100 (mm,
        # also the unit the command path sends in `_on_arm_joints_cmd`).
        # The URDF instead has two mirrored prismatic finger joints
        # (joint1 ∈ [0, +GRIPPER_LIMIT_M], joint2 ∈ [-GRIPPER_LIMIT_M, 0]).
        # Without this conversion, downstream consumers receive a raw
        # 0..100 directly into joint1.position — URDFLoader silently
        # clamps to 0.05 and the gripper visualization sticks at max
        # open. Mirror the per-finger position onto joint2 so a single
        # source-of-truth update animates both fingers symmetrically.
        if not positions:
            return
        gap_input = float(positions[0])
        per_finger_m = max(
            0.0,
            min(GRIPPER_LIMIT_M, (gap_input / 100.0) * GRIPPER_LIMIT_M),
        )

        array = r1lite_pb2.JointStatesArray()
        primary = array.joints.add()
        primary.id = joint_ids[0]
        primary.position = per_finger_m
        primary.velocity = float(velocities[0]) if velocities else 0.0
        primary.torque = float(efforts[0]) if efforts else 0.0

        mirror = array.joints.add()
        mirror.id = joint_ids[1]
        mirror.position = -per_finger_m
        # Velocity / torque on the mirror finger aren't separately
        # measured — the gap is one DOF on the real hardware. Leaving
        # them at proto3 zero keeps the message honest.

        self.zenoh_session.put(zenoh_key, array.SerializeToString())

    def _add_arm_cmd_bridge(
        self,
        side: str,
        zenoh_key: str,
        arm_ros_topic: str,
        gripper_ros_topic: str,
        group,
    ):
        arm_pub = self.create_publisher(
            RosJointState, arm_ros_topic, MOTION_TARGET_QOS, callback_group=group
        )
        gripper_pub = self.create_publisher(
            RosJointState, gripper_ros_topic, MOTION_TARGET_QOS, callback_group=group
        )
        self._ros_arm_cmd_pubs[side] = arm_pub
        self._ros_gripper_cmd_pubs[side] = gripper_pub

        sub = self.zenoh_session.declare_subscriber(
            zenoh_key,
            lambda sample, s=side: self._on_arm_joints_cmd(sample, s),
        )
        self._zenoh_arm_cmd_subs[side] = sub

        self.get_logger().info(
            f"Arm cmd bridge [{side}]: zenoh {zenoh_key} -> "
            f"ROS {arm_ros_topic} + {gripper_ros_topic}"
        )

    def _on_arm_joints_cmd(self, sample, side: str):
        array = r1lite_pb2.JointControlArray()
        try:
            array.ParseFromString(sample.payload.to_bytes())
        except Exception as e:
            self.get_logger().error(f"[{side}] bad JointControlArray: {e}")
            return

        # Build {joint_id: command} from the POSITION entries only — base.py
        # and torso.py mix POSITION / VELOCITY, but arm.py emits POSITION for
        # all 8 entries (6 arm joints + 2 gripper fingers).
        positions_by_id = {
            j.id: float(j.command)
            for j in array.joints
            if j.cmd_type == r1lite_pb2.JointControl.POSITION
        }

        arm_joint_names = [f"{side}_arm_joint{i}" for i in range(1, 7)]
        finger_pos_name = f"{side}_gripper_finger_joint1"  # +g, [0, 0.05]

        missing_arm = [n for n in arm_joint_names if n not in positions_by_id]
        if not missing_arm:
            arm_msg = RosJointState()
            arm_msg.header.stamp = self.get_clock().now().to_msg()
            arm_msg.position = [positions_by_id[n] for n in arm_joint_names]
            self._ros_arm_cmd_pubs[side].publish(arm_msg)
        else:
            self.get_logger().warn(
                f"[{side}] arm cmd dropped: missing joints {missing_arm}"
            )

        if finger_pos_name in positions_by_id:
            g_pos = positions_by_id[finger_pos_name]  # URDF prismatic, 0..0.05
            pct = max(0.0, min(100.0, (g_pos / GRIPPER_LIMIT_M) * 100.0))
            gripper_msg = RosJointState()
            gripper_msg.header.stamp = self.get_clock().now().to_msg()
            gripper_msg.position = [pct]
            self._ros_gripper_cmd_pubs[side].publish(gripper_msg)

    def close(self):
        for sub in self._zenoh_arm_cmd_subs.values():
            try:
                sub.undeclare()
            except Exception as e:
                self.get_logger().warning(f"undeclare arm-cmd sub: {e}")
        self._zenoh_arm_cmd_subs.clear()
        self.zenoh_session.close()


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Bridge R1 Lite ROS 2 topics (cameras + joint feedback) to zenoh."
    )

    parser.add_argument(
        "--zenoh-config",
        default="zenoh_config.json5",
        help="Path to a Zenoh JSON5 config file. If the file doesn't exist, "
             "the default Zenoh config is used.",
    )

    parser.add_argument("--left-arm-topic",       default="/hdas/feedback_arm_left")
    parser.add_argument("--right-arm-topic",      default="/hdas/feedback_arm_right")
    parser.add_argument("--left-gripper-topic",   default="/hdas/feedback_gripper_left")
    parser.add_argument("--right-gripper-topic",  default="/hdas/feedback_gripper_right")
    parser.add_argument("--torso-topic",          default="/hdas/feedback_torso")
    parser.add_argument("--chassis-topic",        default="/hdas/feedback_chassis")

    parser.add_argument("--left-arm-zenoh-key",       default="r1_lite/hdas/feedback_arm_left")
    parser.add_argument("--right-arm-zenoh-key",      default="r1_lite/hdas/feedback_arm_right")
    parser.add_argument("--left-gripper-zenoh-key",   default="r1_lite/hdas/feedback_gripper_left")
    parser.add_argument("--right-gripper-zenoh-key",  default="r1_lite/hdas/feedback_gripper_right")
    parser.add_argument("--torso-zenoh-key",          default="r1_lite/hdas/feedback_torso")
    parser.add_argument("--chassis-zenoh-key",        default="r1_lite/hdas/feedback_chassis")

    parser.add_argument("--bms-topic",     default="/hdas/bms")
    parser.add_argument("--bms-zenoh-key", default="r1_lite/hdas/bms")

    return parser
