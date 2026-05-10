"""
RS01 – RobStride 17 N·m Quasi-Direct Drive Motor
CAN 2.0 private protocol driver (extended 29-bit frame, 1 Mbps)

Specs (from RS01 user manual):
  Rated voltage:      36 VDC   (operating range 24–50 V)
  Peak torque:        17 N·m
  No-load speed:      315 rpm  (output shaft → 33 rad/s)
  Max phase current:  23 Apk
  Deceleration ratio: 7.75:1
  Poles:              28
  Weight:             380 g
  Encoder resolution: 14-bit absolute
"""

from .motor_base import (
    FaultBit,
    MotorFeedback,
    CommType,
    ParamIndex,
    RobStrideMotorBase,
    RunMode,
)


class RS01(RobStrideMotorBase):
    """RobStride RS01 17 N·m quasi-direct-drive motor."""

    V_MIN: float = -33.0   # rad/s  (315 rpm output no-load)
    V_MAX: float =  33.0
    T_MIN: float = -17.0   # N·m
    T_MAX: float =  17.0
    MAX_CURRENT_A: float = 23.0

    def __repr__(self) -> str:
        return (
            f"RS01(motor_id={self.motor_id}, master_id=0x{self.master_id:02X}, "
            f"pos={self._feedback.position:.3f} rad, "
            f"vel={self._feedback.velocity:.3f} rad/s, "
            f"torque={self._feedback.torque:.3f} N·m)"
        )
