"""
motor_base.py – Shared base class for all RobStride quasi-direct-drive motors.

All RS0x motors (RS01–RS05) use the same private CAN 2.0 protocol
(extended 29-bit frame, 1 Mbps, section 4 of each motor's user manual).
This module contains the complete protocol implementation; subclasses
override only motor-specific physical limits.

Frame handling
--------------
All incoming frames are dispatched through _on_frame_received, which is
registered as the per-motor callback in CANComms and called by the Notifier
background thread for every arriving frame:

  type 2  (MOTOR_FEEDBACK)  → decode and replace _feedback
  type 21 (FAULT_FEEDBACK)  → update _feedback.fault and _feedback.warning
  type 17 (PARAM_READ reply) → store result, set _param_event
  type 0  (GET_DEVICE_ID)   → store result, set _device_id_event

motor.feedback is therefore always current without any explicit polling.
Parameter reads block the caller via threading.Event.wait(rx_timeout).

Shared protocol constants (same for all RS0x motors):
  P_MIN / P_MAX   ±4π rad   position encoding range for operation-control mode
  KP_MIN / KP_MAX 0 – 500   Kp gain encoding range
  KD_MIN / KD_MAX 0 – 5     Kd gain encoding range

Motor-specific limits (set as class attributes in each subclass):
  T_MIN / T_MAX   peak torque  (N·m)
  V_MIN / V_MAX   peak output shaft velocity (rad/s)
  MAX_CURRENT_A   maximum phase current (A) – used as default for set_velocity()
"""

import struct
import threading
from dataclasses import dataclass
from enum import IntEnum
from typing import TYPE_CHECKING, Optional

if TYPE_CHECKING:
    import can

from .comms import CANComms


# ── Protocol constants (identical across all RS0x motors) ──────────────────
P_MIN  = -12.57   # rad  (-4π)
P_MAX  =  12.57   # rad  (+4π)
KP_MIN =   0.0
KP_MAX = 500.0
KD_MIN =   0.0
KD_MAX =   5.0


# ── Enumerations ───────────────────────────────────────────────────────────

class RunMode(IntEnum):
    OPERATION    = 0  # MIT-style: angle + velocity + Kp_Kd + torque ff  (default at power-on)
    POSITION_PP  = 1  # Profile Position with S-curve planning
    VELOCITY     = 2  # Velocity control
    CURRENT      = 3  # Iq current control
    POSITION_CSP = 5  # Cyclic Synchronous Position


class CommType(IntEnum):
    """29-bit extended frame: bits 28~24 = communication type."""
    GET_DEVICE_ID  = 0x00
    OPERATION_CTRL = 0x01
    MOTOR_FEEDBACK = 0x02
    MOTOR_ENABLE   = 0x03
    MOTOR_STOP     = 0x04
    SET_MECH_ZERO  = 0x06
    SET_CAN_ID     = 0x07
    PARAM_READ     = 0x11  # type 17 – single parameter read
    PARAM_WRITE    = 0x12  # type 18 – single parameter write (volatile)
    FAULT_FEEDBACK = 0x15  # type 21
    DATA_SAVE      = 0x16  # type 22 – persist parameters to flash
    BAUD_RATE_MOD  = 0x17  # type 23 – re-power-on effect
    ACTIVE_REPORT  = 0x18  # type 24 – enable autonomous reporting
    PROTOCOL_MOD   = 0x19  # type 25 – switch protocol (re-power-on effect)


class ParamIndex(IntEnum):
    """Parameter indices for PARAM_READ_PARAM_WRITE (section 4.1.13)."""
    MOTOR_BAUD     = 0x2009  # uint8, Settings  – baud rate flag (1=1M 2=500K 3=250K 4=125K)
    CAN_ID         = 0x200a  # uint8, Settings  – motor CAN ID (0-127)
    CAN_MASTER     = 0x200b  # uint8, Settings  – host CAN ID
    RUN_MODE       = 0x7005
    IQ_REF         = 0x7006  # current mode Iq command (A)
    SPD_REF        = 0x700A  # velocity command (rad/s)
    LIMIT_TORQUE   = 0x700B  # torque limit (N·m)
    CUR_KP         = 0x7010
    CUR_KI         = 0x7011
    CUR_FILT_GAIN  = 0x7014
    LOC_REF        = 0x7016  # position command (rad)
    LIMIT_SPD      = 0x7017  # CSP_velocity speed limit (rad/s)
    LIMIT_CUR      = 0x7018  # velocity_position current limit (A)
    MECH_POS       = 0x7019  # read-only: mechanical angle (rad)
    IQF            = 0x701A  # read-only: iq filter value (A)
    MECH_VEL       = 0x701B  # read-only: speed of load (rad/s)
    VBUS           = 0x701C  # read-only: bus voltage (V)
    LOC_KP         = 0x701E
    SPD_KP         = 0x701F
    SPD_KI         = 0x7020
    SPD_FILT_GAIN  = 0x7021
    ACC_RAD        = 0x7022  # velocity mode acceleration (rad/s²)
    VEL_MAX        = 0x7024  # PP position mode speed (rad/s)
    ACC_SET        = 0x7025  # PP position mode acceleration (rad/s²)
    EPS_SCAN_TIME  = 0x7026  # report interval (1 = 10 ms, +1 = +5 ms)
    CAN_TIMEOUT    = 0x7028  # 20000 = 1 s; 0 = disabled
    ZERO_STA       = 0x7029  # 0 → 0–2π range; 1 → −π to π
    DAMPER         = 0x702A  # damping switch
    ADD_OFFSET     = 0x702B  # zero position offset (rad)
    ALVEOLOUS_OPEN = 0x702C  # cogging compensation switch
    IQ_TEST        = 0x702D  # motor init calibration switch
    DCC_SET        = 0x702E  # PP deceleration (rad/s²)


class FaultBit(IntEnum):
    """Fault bitmask (faultSta, 0x3022; bytes 0-3 of type-21 frame)."""
    OVER_TEMP      = (1 << 0)
    DRIVER_IC      = (1 << 1)
    UNDERVOLTAGE   = (1 << 2)
    OVERVOLTAGE    = (1 << 3)
    B_PHASE_OC     = (1 << 4)
    C_PHASE_OC     = (1 << 5)
    ENCODER_UNCAL  = (1 << 7)
    HW_ID_FAULT    = (1 << 8)
    POS_INIT_FAULT = (1 << 9)
    STALL_OVERLOAD = (1 << 14)
    A_PHASE_OC     = (1 << 16)


class WarnBit(IntEnum):
    """Warning bitmask (bytes 4-7 of type-21 frame)."""
    OVER_TEMP_WARN = (1 << 0)   # winding temp approaching 135 °C threshold


# ── Data structures ────────────────────────────────────────────────────────

@dataclass
class MotorFeedback:
    """Decoded motor state, updated by type-2 and type-21 frames."""
    motor_id:    int   = 0
    position:    float = 0.0   # rad
    velocity:    float = 0.0   # rad/s
    torque:      float = 0.0   # N·m
    temperature: float = 0.0   # °C
    mode:        int   = 0     # 0=reset, 1=calibration, 2=run
    fault:       int   = 0     # bitmask – see FaultBit (type-2 bits 21-16; type-21 bytes 0-3)
    warning:     int   = 0     # bitmask – see WarnBit  (type-21 bytes 4-7)


# ── Base motor class ───────────────────────────────────────────────────────

class RobStrideMotorBase:
    """
    Shared protocol implementation for all RobStride RS0x motors.

    Subclasses must define the following class attributes:
        V_MIN, V_MAX    peak output shaft velocity range (rad/s)
        T_MIN, T_MAX    peak torque range (N·m)
        MAX_CURRENT_A   maximum phase current (A)
    """

    # Motor-specific limits – subclasses override these
    V_MIN: float = -44.0
    V_MAX: float =  44.0
    T_MIN: float = -17.0
    T_MAX: float =  17.0
    MAX_CURRENT_A: float = 23.0

    def __init__(
        self,
        motor_id:   int      = 1,
        master_id:  int      = 0xFD,
        comms:      CANComms = None,
        rx_timeout: float    = 0.05,
    ):
        """
        Args:
            motor_id:   CAN ID of the motor (0–127).
            master_id:  Host CAN ID embedded in outgoing frames (default 0xFD).
            comms:      CANComms instance to use for transport.
            rx_timeout: Seconds to wait for a reply frame (param read, device ID).
        """
        if comms is None:
            raise ValueError("comms must be a CANComms instance; create one with CANComms('can0')")
        self.motor_id   = motor_id
        self.master_id  = master_id
        self.rx_timeout = rx_timeout
        self._comms     = comms
        self._feedback  = MotorFeedback(motor_id=motor_id)

        # Per-instance events for request-response comm types (param read, device ID).
        # Clear the event before sending the request, then wait on it.
        self._param_result:     Optional[bytes] = None
        self._param_event                       = threading.Event()
        self._device_id_result: Optional[bytes] = None
        self._device_id_event                   = threading.Event()
        self._feedback_callback                 = None

        self._comms.add_motor_filter(motor_id, callback=self._on_frame_received)

    # ── Scalar helpers ─────────────────────────────────────────────────────

    def _float_to_uint(self, x: float, x_min: float, x_max: float, bits: int) -> int:
        x = max(x_min, min(x_max, x))
        return int((x - x_min) / (x_max - x_min) * ((1 << bits) - 1))

    def _uint_to_float(self, raw: int, x_min: float, x_max: float, bits: int) -> float:
        return x_min + raw * (x_max - x_min) / ((1 << bits) - 1)

    # ── Frame construction ─────────────────────────────────────────────────

    def _ext_id(self, comm_type: int, data_area2: int) -> int:
        """
        Build the 29-bit extended CAN arbitration ID (section 4, private protocol).

        Layout:
          bits 28–24  communication type (5 bits)
          bits 23–8   data area 2       (16 bits)
          bits  7–0   destination motor CAN ID (8 bits)
        """
        return (
            ((comm_type  & 0x1F)   << 24) |
            ((data_area2 & 0xFFFF) <<  8) |
            (self.motor_id & 0xFF)
        )

    def _send(self, comm_type: int, data_area2: int, payload: bytes) -> None:
        self._comms.send_extended(self._ext_id(comm_type, data_area2), payload)

    # ── Feedback frame parser ──────────────────────────────────────────────

    def _parse_type2(self, msg: "can.Message") -> MotorFeedback:
        """
        Decode a type-2 motor feedback frame (section 4.1.2).

        29-bit ID layout:
          bits 28–24  0x02
          bits 23–22  mode  (0=reset, 1=cali, 2=run)
          bits 21–16  fault bitmask
          bits 15–8   motor CAN ID
          bits  7–0   host CAN ID
        """
        eid      = msg.arbitration_id
        motor_id = (eid >>  8) & 0xFF
        fault    = (eid >> 16) & 0x3F
        mode     = (eid >> 22) & 0x03

        d = msg.data
        raw_pos  = (d[0] << 8) | d[1]
        raw_vel  = (d[2] << 8) | d[3]
        raw_tor  = (d[4] << 8) | d[5]
        raw_temp = (d[6] << 8) | d[7]

        return MotorFeedback(
            motor_id    = motor_id,
            position    = self._uint_to_float(raw_pos, P_MIN,       P_MAX,       16),
            velocity    = self._uint_to_float(raw_vel, self.V_MIN,  self.V_MAX,  16),
            torque      = self._uint_to_float(raw_tor, self.T_MIN,  self.T_MAX,  16),
            temperature = raw_temp / 10.0,
            mode        = mode,
            fault       = fault,
        )

    def _on_frame_received(self, msg: "can.Message") -> None:
        """
        Unified frame handler — called by _MotorDispatcher in the Notifier thread
        for every incoming frame addressed to this motor.  Dispatches on comm type
        and updates state immediately; no queues involved.
        """
        if not msg.is_extended_id:
            return
        comm_type = (msg.arbitration_id >> 24) & 0x1F
        if comm_type == CommType.MOTOR_FEEDBACK:
            self._feedback = self._parse_type2(msg)
            if self._feedback_callback:
                self._feedback_callback(self._feedback)
        elif comm_type == CommType.ACTIVE_REPORT:
            # Same ID layout as type-2 (mode bits 23-22, fault bits 21-16).
            # Data bytes 0-3: position and velocity (same encoding as type-2).
            # Data bytes 4-7: Kp/Kd — ignored.
            eid = msg.arbitration_id
            d   = msg.data
            raw_pos = (d[0] << 8) | d[1]
            raw_vel = (d[2] << 8) | d[3]
            self._feedback.position = self._uint_to_float(raw_pos, P_MIN,      P_MAX,      16)
            self._feedback.velocity = self._uint_to_float(raw_vel, self.V_MIN, self.V_MAX, 16)
            self._feedback.fault    = (eid >> 16) & 0x3F
            self._feedback.mode     = (eid >> 22) & 0x03
            if self._feedback_callback:
                self._feedback_callback(self._feedback)
        elif comm_type == CommType.FAULT_FEEDBACK:
            if len(msg.data) >= 8:
                self._feedback.fault   = struct.unpack_from('<I', msg.data, 0)[0]
                self._feedback.warning = struct.unpack_from('<I', msg.data, 4)[0]
        elif comm_type == CommType.ACTIVE_REPORT:
            # Same ID layout as type-2 (mode bits 23-22, fault bits 21-16).
            # Data bytes 0-3: position and velocity (same encoding as type-2).
            # Data bytes 4-7: Kp/Kd — not needed, ignored.
            eid = msg.arbitration_id
            d   = msg.data
            raw_pos = (d[0] << 8) | d[1]
            raw_vel = (d[2] << 8) | d[3]
            self._feedback.position = self._uint_to_float(raw_pos, P_MIN,      P_MAX,      16)
            self._feedback.velocity = self._uint_to_float(raw_vel, self.V_MIN, self.V_MAX, 16)
            self._feedback.fault    = (eid >> 16) & 0x3F
            self._feedback.mode     = (eid >> 22) & 0x03
        elif comm_type == CommType.PARAM_READ:
            if len(msg.data) >= 8:
                self._param_result = bytes(msg.data[4:8])
                self._param_event.set()
        elif comm_type == CommType.GET_DEVICE_ID:
            if len(msg.data) >= 8:
                self._device_id_result = bytes(msg.data)
                self._device_id_event.set()

    # ── Core motor commands ────────────────────────────────────────────────

    def enable(self) -> MotorFeedback:
        """Type 3 – enable motor (enter run state)."""
        self._send(CommType.MOTOR_ENABLE, self.master_id, bytes(8))
        return self._feedback

    def disable(self, clear_fault: bool = False) -> MotorFeedback:
        """
        Type 4 – stop motor.

        Args:
            clear_fault: Set True to also clear latched fault flags.
        """
        payload = bytearray(8)
        if clear_fault:
            payload[0] = 0x01
        self._send(CommType.MOTOR_STOP, self.master_id, bytes(payload))
        return self._feedback

    def set_operation_control(
        self,
        position:  float = 0.0,
        velocity:  float = 0.0,
        torque_ff: float = 0.0,
        kp:        float = 0.0,
        kd:        float = 0.0,
    ) -> MotorFeedback:
        """
        Type 1 – operation-control mode command.

        Implements:  t_ref = Kd*(v_set - v_actual) + Kp*(p_set - p_actual) + t_ff

        Frame layout:
          torque_ff  → data_area2 (bits 23–8 of the 29-bit ID)
          position   → Byte 0–1
          velocity   → Byte 2–3
          Kp         → Byte 4–5
          Kd         → Byte 6–7
        """
        raw_tor = self._float_to_uint(torque_ff, self.T_MIN, self.T_MAX, 16)
        raw_pos = self._float_to_uint(position,  P_MIN,      P_MAX,      16)
        raw_vel = self._float_to_uint(velocity,  self.V_MIN, self.V_MAX, 16)
        raw_kp  = self._float_to_uint(kp,        KP_MIN,     KP_MAX,     16)
        raw_kd  = self._float_to_uint(kd,        KD_MIN,     KD_MAX,     16)

        payload = bytes([
            (raw_pos >> 8) & 0xFF,  raw_pos & 0xFF,
            (raw_vel >> 8) & 0xFF,  raw_vel & 0xFF,
            (raw_kp  >> 8) & 0xFF,  raw_kp  & 0xFF,
            (raw_kd  >> 8) & 0xFF,  raw_kd  & 0xFF,
        ])
        self._send(CommType.OPERATION_CTRL, raw_tor, payload)
        return self._feedback

    def set_zero_position(self) -> MotorFeedback:
        """Type 6 – set current mechanical position as zero."""
        payload = bytearray(8)
        payload[0] = 0x01
        self._send(CommType.SET_MECH_ZERO, self.master_id, bytes(payload))
        return self._feedback

    def set_can_id(self, new_id: int) -> None:
        """Type 7 – change motor CAN ID (effective immediately, no reply)."""
        data_area2 = ((new_id & 0xFF) << 8) | (self.master_id & 0xFF)
        self._send(CommType.SET_CAN_ID, data_area2, bytes(8))
        self.motor_id = new_id

    def get_device_id(self) -> Optional[bytes]:
        """Type 0 – request 64-bit MCU unique identifier. Returns 8 raw bytes."""
        self._device_id_event.clear()
        self._send(CommType.GET_DEVICE_ID, self.master_id, bytes(8))
        if self._device_id_event.wait(timeout=self.rx_timeout):
            return self._device_id_result
        return None

    # ── Parameter read_write (Types 17 & 18) ──────────────────────────────

    def write_param_uint8(self, index: int, value: int) -> None:
        """Type 18 – write uint8 parameter (volatile; save with save_params())."""
        payload = bytearray(8)
        struct.pack_into('<H', payload, 0, index & 0xFFFF)
        payload[4] = value & 0xFF
        self._send(CommType.PARAM_WRITE, self.master_id, bytes(payload))

    def write_param_float(self, index: int, value: float) -> None:
        """Type 18 – write float parameter (volatile; save with save_params())."""
        payload = bytearray(8)
        struct.pack_into('<H', payload, 0, index & 0xFFFF)
        struct.pack_into('<f', payload, 4, value)
        self._send(CommType.PARAM_WRITE, self.master_id, bytes(payload))

    def read_param_raw(self, index: int) -> Optional[bytes]:
        """Type 17 – read a parameter; returns the raw 4-byte data field."""
        payload = bytearray(8)
        struct.pack_into('<H', payload, 0, index & 0xFFFF)
        self._param_event.clear()
        self._send(CommType.PARAM_READ, self.master_id, bytes(payload))
        if self._param_event.wait(timeout=self.rx_timeout):
            return self._param_result
        return None

    def read_param_float(self, index: int) -> Optional[float]:
        """Type 17 – read a float parameter."""
        raw = self.read_param_raw(index)
        if raw and len(raw) >= 4:
            return struct.unpack('<f', raw[:4])[0]
        return None

    def read_param_uint(self, index: int) -> Optional[int]:
        """Type 17 – read an integer parameter (little-endian)."""
        raw = self.read_param_raw(index)
        if raw and len(raw) >= 4:
            return struct.unpack('<I', raw[:4])[0]
        return None

    def save_params(self) -> None:
        """Type 22 – persist parameters to flash so they survive power-off."""
        self._send(CommType.DATA_SAVE, self.master_id,
                   bytes([0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08]))

    # ── High-level mode control ────────────────────────────────────────────

    def set_run_mode(self, mode: RunMode) -> None:
        """
        Write the run_mode parameter (0x7005).
        Must be called while the motor is disabled (stopped).
        The motor defaults to RunMode.OPERATION on power-on.
        """
        self.write_param_uint8(ParamIndex.RUN_MODE, int(mode))

    def set_velocity(
        self,
        velocity_rad_s:      float,
        current_limit_a:     Optional[float] = None,
        acceleration_rad_s2: float = 20.0,
    ) -> None:
        """
        Configure velocity-mode parameters (RunMode.VELOCITY must be active).
        Motor must be enabled separately with enable().
        """
        if current_limit_a is None:
            current_limit_a = self.MAX_CURRENT_A
        self.write_param_float(ParamIndex.LIMIT_CUR, current_limit_a)
        self.write_param_float(ParamIndex.ACC_RAD,   acceleration_rad_s2)
        self.write_param_float(ParamIndex.SPD_REF,   velocity_rad_s)

    def set_position_csp(
        self,
        position_rad:      float,
        speed_limit_rad_s: float = 2.0,
        current_limit_a:   Optional[float] = None,
    ) -> None:
        """
        Send a CSP position target (RunMode.POSITION_CSP must be active).
        Motor must be enabled separately with enable().
        """
        if current_limit_a is None:
            current_limit_a = self.MAX_CURRENT_A
        self.write_param_float(ParamIndex.LIMIT_CUR, current_limit_a)
        self.write_param_float(ParamIndex.LIMIT_SPD, speed_limit_rad_s)
        self.write_param_float(ParamIndex.LOC_REF,   position_rad)

    def set_position_pp(
        self,
        position_rad:         float,
        speed_rad_s:          float = 2.0,
        acceleration_rad_s2:  float = 10.0,
        deceleration_rad_s2:  Optional[float] = None,
        torque_limit_nm:      Optional[float] = None,
    ) -> None:
        """
        Send a PP position target with speed_acceleration planning.
        (RunMode.POSITION_PP must be active, motor must be enabled.)
        Note: speed and acceleration cannot be changed during motion in PP mode.

        deceleration_rad_s2: if provided, written to DCC_SET (0x702E) so the
            motor uses a different deceleration ramp than acceleration.
            None (default) leaves DCC_SET at its last written value.
        """
        if torque_limit_nm is None:
            torque_limit_nm = self.T_MAX
        self.write_param_float(ParamIndex.LIMIT_TORQUE, torque_limit_nm)
        self.write_param_float(ParamIndex.VEL_MAX,      speed_rad_s)
        self.write_param_float(ParamIndex.ACC_SET,      acceleration_rad_s2)
        if deceleration_rad_s2 is not None:
            self.write_param_float(ParamIndex.DCC_SET,  deceleration_rad_s2)
        self.write_param_float(ParamIndex.LOC_REF,      position_rad)

    def set_current(self, iq_ref_a: float) -> None:
        """Set Iq reference in current mode (RunMode.CURRENT must be active)."""
        self.write_param_float(ParamIndex.IQ_REF, iq_ref_a)

    def set_torque_limit(self, limit_nm: float) -> None:
        self.write_param_float(ParamIndex.LIMIT_TORQUE, limit_nm)

    def set_can_timeout(self, timeout_ms: int) -> None:
        """
        Configure the CAN watchdog. Motor resets if no frame received within timeout.
        timeout_ms=0 disables the watchdog. Unit: 20000 counts = 1 second.
        """
        self.write_param_uint8(ParamIndex.CAN_TIMEOUT, int(timeout_ms * 20))

    # ── Active reporting ───────────────────────────────────────────────────

    def enable_active_report(self, enable: bool = True, interval_ms: int = 10) -> None:
        """
        Type 24 – toggle the motor's autonomous status reporting.

        When enabled the motor pushes type-2 frames at the configured interval
        without waiting for a command (minimum 10 ms, +5 ms per step).
        """
        if enable:
            steps = max(1, (interval_ms - 10) // 5 + 1)
            self.write_param_uint8(ParamIndex.EPS_SCAN_TIME, steps)
        payload = bytearray([0x01, 0x02, 0x03, 0x04, 0x05, 0x06,
                              0x01 if enable else 0x00, 0x00])
        self._send(CommType.ACTIVE_REPORT, self.master_id, bytes(payload))

    # ── Telemetry ──────────────────────────────────────────────────────────

    def read_mech_pos(self) -> Optional[float]:
        """Read mechanical position via parameter read (rad)."""
        return self.read_param_float(ParamIndex.MECH_POS)

    def read_mech_vel(self) -> Optional[float]:
        """Read mechanical velocity via parameter read (rad/s)."""
        return self.read_param_float(ParamIndex.MECH_VEL)

    def read_vbus(self) -> Optional[float]:
        """Read bus voltage (V)."""
        return self.read_param_float(ParamIndex.VBUS)

    def set_feedback_callback(self, callback) -> None:
        """Register a callable invoked with the latest MotorFeedback on every feedback frame."""
        self._feedback_callback = callback

    @property
    def feedback(self) -> MotorFeedback:
        """Current motor state — kept up-to-date by the Notifier callback."""
        return self._feedback
