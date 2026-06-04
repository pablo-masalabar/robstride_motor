"""
mms760400_48_c2_1.py – EZmotion MMS760400-48-C2-1 smart motor.

Physical specifications (from datasheet):
  Rated torque:      ~3.5 N·m
  Peak torque:       ~10.5 N·m (estimated 300% rated)
  Rated speed:       ~3000 RPM
  Supply voltage:    48 V
  Encoder:           65536 counts/rev (magnetic, integrated)
  Communication:     CANopen DS301 + DS402, 10kbps–1Mbps

Default DS402 profile parameters (from EDS):
  Profile velocity:   655360 counts/s (~62.8 rad/s, ~10 rev/s)
  Profile accel:    3276800 counts/s²
  Profile decel:    3276800 counts/s²
  Max profile vel:  3276800 counts/s
  Max motor speed:     3000 RPM
  Max torque:          3000  (300% of rated)
"""

from .motor_base import EZMotionMotorBase


class MMS760400_48_C2_1(EZMotionMotorBase):
    """EZmotion MMS760400-48-C2-1 all-in-one smart motor (48 V, CANopen)."""

    RATED_TORQUE_NM:   float = 3.5
    MAX_SPEED_RPM:     float = 3000.0
    MAX_TORQUE_PERMIL: int   = 3000   # 300% of rated torque
