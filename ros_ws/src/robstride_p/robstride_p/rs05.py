"""
RS05 – RobStride 6 N·m Quasi-Direct Drive Motor
CAN 2.0 private protocol driver (extended 29-bit frame, 1 Mbps)

Specs (from RS05 user manual):
  Rated voltage:      48 VDC   (operating range 15–60 V)
  Rated torque:       1.6 N·m  (at 100 rpm, 70 mm × 70 mm heat sink)
  Peak torque:        5.5 N·m  (model rated at 6 N·m)
  No-load speed:      480 rpm  (output shaft → 50 rad/s)
  Max phase current:  11 Apk
  Deceleration ratio: 7.75:1
  Poles:              20
  Weight:             191 g
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


class RS05(RobStrideMotorBase):
    """RobStride RS05 6 N·m quasi-direct-drive motor."""

    V_MIN: float = -50.0   # rad/s  (480 rpm output no-load)
    V_MAX: float =  50.0
    T_MIN: float =  -6.0   # N·m   (using model-name rating; spec peak is 5.5 N·m)
    T_MAX: float =   6.0
    MAX_CURRENT_A: float = 11.0

    def __repr__(self) -> str:
        return (
            f"RS05(motor_id={self.motor_id}, master_id=0x{self.master_id:02X}, "
            f"pos={self._feedback.position:.3f} rad, "
            f"vel={self._feedback.velocity:.3f} rad/s, "
            f"torque={self._feedback.torque:.3f} N·m)"
        )
