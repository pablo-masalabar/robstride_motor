"""
RS03 – RobStride 60 N·m Quasi-Direct Drive Motor
CAN 2.0 private protocol driver (extended 29-bit frame, 1 Mbps)

Specs (from RS03 user manual):
  Rated voltage:      48 VDC   (operating range 24–60 V)
  Rated torque:       20 N·m   (at 100 rpm, 215 mm × 220 mm heat sink)
  Peak torque:        60 N·m
  No-load speed:      200 rpm  (output shaft → 21 rad/s)
  Max phase current:  43 Apk
  Deceleration ratio: 9:1
  Poles:              42
  Weight:             880 g
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


class RS03(RobStrideMotorBase):
    """RobStride RS03 60 N·m quasi-direct-drive motor."""

    V_MIN: float = -21.0   # rad/s  (200 rpm output no-load)
    V_MAX: float =  21.0
    T_MIN: float = -60.0   # N·m
    T_MAX: float =  60.0
    MAX_CURRENT_A: float = 43.0

    def __repr__(self) -> str:
        return (
            f"RS03(motor_id={self.motor_id}, master_id=0x{self.master_id:02X}, "
            f"pos={self._feedback.position:.3f} rad, "
            f"vel={self._feedback.velocity:.3f} rad/s, "
            f"torque={self._feedback.torque:.3f} N·m)"
        )
