"""
j4310_2ec.py – Damiao DM-J4310-2EC geared motor driver.

Physical specifications (from datasheet V1.2):
  Rated torque:      3.5 N·m
  Peak torque:       12.5 N·m
  Rated speed:       120 rpm output shaft
  Max no-load speed: 200 rpm @ 24 V  /  450 rpm @ 48 V
  Peak phase current: 20 A
  Gear ratio:        10:1
  Encoder:           16-bit magnetic, dual (motor-side + output shaft)

Speed conversions (output shaft):
  120 rpm → 12.57 rad/s (rated)
  200 rpm → 20.94 rad/s (24 V no-load)
  450 rpm → 47.12 rad/s (48 V no-load)

MIT_P_MAX / MIT_V_MAX / MIT_T_MAX MUST match the motor's PMAX (0x15),
VMAX (0x16), and TMAX (0x17) register values for correct encoding.
The defaults here match the factory firmware values; if you have modified
those registers, update these class attributes or pass them at construction.
"""

from .motor_base import DamiaoMotorBase


class J4310_2EC(DamiaoMotorBase):
    """DM-J4310-2EC geared motor (10:1, 12.5 N·m peak, CAN@1Mbps)."""

    # Physical limits
    T_MAX:         float = 12.5    # N·m peak torque
    V_MAX:         float = 20.94   # rad/s output shaft (24 V, 200 rpm no-load)
    MAX_CURRENT_A: float = 20.0    # A peak phase current

    # MIT-mode encoding ranges — must equal PMAX/VMAX/TMAX firmware registers
    MIT_P_MAX: float = 12.5    # rad  (~±2 output shaft turns)
    MIT_V_MAX: float = 45.0    # rad/s (generous bound covering 48V operation)
    MIT_T_MAX: float = 12.0    # N·m
