"""
motor_base.py – Protocol implementation for Damiao DM-J43xx geared motors.

CAN protocol (section "CAN Communication" of the user manual):
  - Standard 11-bit frames at 1 Mbps (CAN 2.0B STD)
  - Four control modes; frame ID depends on the active mode:
      MIT mode:                  ID = motor_id
      Position-Velocity mode:    ID = 0x100 + motor_id
      Velocity mode:             ID = 0x200 + motor_id
      Force-Position Hybrid:     ID = 0x300 + motor_id
  - Parameter read/write/save:  ID = 0x7FF  (4 or 8 data bytes)
  - All motor replies arrive on ID = master_id (default 0)

Feedback frame (8 bytes, ID = master_id):
  D[0]: (ERR << 4) | (motor_id & 0x0F)
  D[1]: POS[15:8]   – 16-bit signed position
  D[2]: POS[7:0]
  D[3]: VEL[11:4]   – 12-bit signed velocity
  D[4]: VEL[3:0] | T[11:8]
  D[5]: T[7:0]      – 12-bit signed torque
  D[6]: T_MOS       – MOSFET temperature (°C, raw byte)
  D[7]: T_Rotor     – Motor coil temperature (°C, raw byte)

Position, velocity, and torque are signed fixed-point values linearly mapped
to the motor's PMAX / VMAX / TMAX register ranges.  MIT_P_MAX, MIT_V_MAX,
and MIT_T_MAX in each subclass MUST match the motor's configured registers
(0x15, 0x16, 0x17) for correct scaling.

Parameter read/write frame (ID = 0x7FF):
  Read  (4 bytes): D[0]=CANID_L, D[1]=CANID_H, D[2]=0x33, D[3]=RID
  Write (8 bytes): D[0]=CANID_L, D[1]=CANID_H, D[2]=0x55, D[3]=RID, D[4-7]=value
  Save  (4 bytes): D[0]=CANID_L, D[1]=CANID_H, D[2]=0xAA, D[3]=0x01
    – Save only takes effect while the motor is disabled.

Motor IDs 0–15 are supported (motor identity fits in 4 bits in the feedback frame).
"""

import struct
import threading
from dataclasses import dataclass
from enum import IntEnum
from typing import TYPE_CHECKING, Optional

if TYPE_CHECKING:
    import can

from .comms import DamiaoCANComms


# ── Gain encoding ranges (MIT mode, same as manual section "Control Frame in MIT Mode") ──
KP_MAX = 500.0
KD_MAX =   5.0


# ── Enumerations ──────────────────────────────────────────────────────────────

class RunMode(IntEnum):
    MIT                   = 1
    POSITION_VELOCITY     = 2
    VELOCITY              = 3
    FORCE_POSITION_HYBRID = 4


class RegAddr(IntEnum):
    """Register addresses (section "Register Map")."""
    UV_VALUE  = 0x00  # Undervoltage threshold (float)
    KT_VALUE  = 0x01  # Torque constant (float, RO after calibration)
    OT_VALUE  = 0x02  # Over-temperature threshold (float) [80.0, 200]
    OC_VALUE  = 0x03  # Overcurrent threshold (float, per-unit) (0.0, 1.0)
    ACC       = 0x04  # Acceleration (float)
    DEC       = 0x05  # Deceleration (float, negative value)
    MAX_SPD   = 0x06  # Max velocity (float)
    MST_ID    = 0x07  # Master CAN ID (uint32) [0, 0x7FF]
    ESC_ID    = 0x08  # Motor command CAN ID (uint32) [0, 0x7FF]
    TIMEOUT   = 0x09  # Communication timeout threshold (uint32)
    CTRL_MODE = 0x0A  # Control mode (uint32) [1=MIT, 2=PV, 3=VEL, 4=FPH]
    DAMP      = 0x0B  # Viscous damping coefficient (float, RO)
    INERTIA   = 0x0C  # Rotor inertia (float, RO)
    HW_VER    = 0x0D  # Hardware version (uint32, RO)
    SW_VER    = 0x0E  # Firmware version (uint32, RO)
    SN        = 0x0F  # Serial number (uint32, RO)
    NPP       = 0x10  # Number of pole pairs (uint32, RO)
    RS        = 0x11  # Phase resistance (float, RO)
    LS        = 0x12  # Phase inductance (float, RO)
    FLUX      = 0x13  # Flux linkage (float, RO)
    GR        = 0x14  # Gear reduction ratio (float, RO)
    PMAX      = 0x15  # MIT position encoding range ±PMAX (float)
    VMAX      = 0x16  # MIT velocity encoding range ±VMAX (float)
    TMAX      = 0x17  # MIT torque encoding range ±TMAX (float)
    I_BW      = 0x18  # Current-loop bandwidth (float) [100.0, 1e4]
    KP_ASR    = 0x19  # Velocity loop proportional gain (float)
    KI_ASR    = 0x1A  # Velocity loop integral gain (float)
    KP_APR    = 0x1B  # Position loop proportional gain (float)
    KI_APR    = 0x1C  # Position loop integral gain (float)
    OV_VALUE  = 0x1D  # Overvoltage threshold (float)
    GREF      = 0x1E  # Gear torque efficiency (float) [0.0, 1.0]
    DETA      = 0x1F  # Velocity loop damping coefficient (float) [1.0, 30.0]
    V_BW      = 0x20  # Velocity loop filter bandwidth (float) [0.0, 500.0]
    IQ_C1     = 0x21  # Iq gain (float) [100.0, 1e4]
    VL_C1     = 0x22  # Velocity loop gain factor (float) (0.0, 1e4)
    CAN_BR    = 0x23  # CAN baud rate code (uint32) [0=125K … 9=5M]
    SUB_VER   = 0x24  # Sub-version (uint32, RO)
    BOOT_VER  = 0x25  # Bootloader version (uint32, RO)
    DIR       = 0x37  # Rotation direction (float, RO)
    M_OFF     = 0x38  # Motor-side angle offset (float, RO)
    IMAX      = 0x3B  # Driver maximum current (float, RO)
    VBUS      = 0x3C  # Bus voltage (float, RO)
    TPCB      = 0x3D  # PCB temperature (float, RO)
    TMTR      = 0x3E  # Motor coil temperature (float, RO)
    I_U_OFF   = 0x3F  # Phase U current offset (float, RO)
    I_V_OFF   = 0x40  # Phase V current offset (float, RO)
    I_W_OFF   = 0x41  # Phase W current offset (float, RO)
    P_M       = 0x50  # Motor-side position (float, RO, rad)
    XOUT      = 0x51  # Output shaft position (float, RO, rad)


class FaultCode(IntEnum):
    """ERR field values in the feedback frame D[0] upper nibble."""
    DISABLED       = 0x0  # Default state after power-on
    ENABLED        = 0x1  # Normal run state
    OVER_VOLTAGE   = 0x8
    UNDER_VOLTAGE  = 0x9
    OVER_CURRENT   = 0xA
    MOS_OVER_TEMP  = 0xB  # MOSFET over-temperature
    COIL_OVER_TEMP = 0xC  # Motor winding over-temperature
    COMM_LOST      = 0xD
    OVERLOAD       = 0xE


# ── Data structures ────────────────────────────────────────────────────────────

@dataclass
class MotorFeedback:
    """Decoded motor state, updated on every reply frame."""
    motor_id: int   = 0
    position: float = 0.0   # rad (output shaft, relative to zero position)
    velocity: float = 0.0   # rad/s (output shaft)
    torque:   float = 0.0   # N·m
    t_mos:    float = 0.0   # °C – MOSFET/driver temperature
    t_rotor:  float = 0.0   # °C – motor coil temperature
    err:      int   = 0     # FaultCode
    enabled:  bool  = False


# ── Base motor class ───────────────────────────────────────────────────────────

class DamiaoMotorBase:
    """
    Shared CAN protocol implementation for Damiao DM-J43xx geared motors.

    Subclasses must define:
        T_MAX         peak output torque (N·m)
        V_MAX         peak output shaft velocity (rad/s)
        MAX_CURRENT_A maximum phase current (A)
        MIT_P_MAX     position encoding range for MIT mode (rad) — must equal PMAX register
        MIT_V_MAX     velocity encoding range for MIT mode (rad/s) — must equal VMAX register
        MIT_T_MAX     torque encoding range for MIT mode (N·m) — must equal TMAX register
    """

    T_MAX:         float = 12.5
    V_MAX:         float = 20.94
    MAX_CURRENT_A: float = 20.0

    MIT_P_MAX: float = 12.5
    MIT_V_MAX: float = 45.0
    MIT_T_MAX: float = 12.0

    def __init__(
        self,
        motor_id:   int          = 1,
        master_id:  int          = 0,
        comms:      DamiaoCANComms = None,
        rx_timeout: float        = 0.05,
    ):
        """
        Args:
            motor_id:   CAN ID of the motor (ESC_ID register, 0–15).
            master_id:  Host CAN ID (MST_ID register on motor, default 0).
            comms:      DamiaoCANComms instance.
            rx_timeout: Seconds to wait for a parameter read reply.
        """
        if comms is None:
            raise ValueError(
                "comms must be a DamiaoCANComms instance; "
                "create one with DamiaoCANComms('can0')"
            )
        self.motor_id   = motor_id
        self.master_id  = master_id
        self.rx_timeout = rx_timeout
        self._comms     = comms
        self._feedback  = MotorFeedback(motor_id=motor_id)
        self._mode      = RunMode.MIT

        self._param_result: Optional[bytes] = None
        self._param_event                   = threading.Event()
        self._feedback_callback             = None

        comms.add_motor_callback(motor_id, self._on_frame_received)

    # ── Fixed-point encoding / decoding ───────────────────────────────────────

    @staticmethod
    def _float_to_s16(x: float, x_max: float) -> int:
        """Float → 16-bit two's-complement unsigned (for big-endian packing)."""
        raw = round(x / x_max * 32767.0)
        raw = max(-32768, min(32767, raw))
        return raw & 0xFFFF

    @staticmethod
    def _float_to_s12(x: float, x_max: float) -> int:
        """Float → 12-bit two's-complement unsigned."""
        raw = round(x / x_max * 2047.0)
        raw = max(-2048, min(2047, raw))
        return raw & 0xFFF

    @staticmethod
    def _float_to_u12(x: float, x_max: float) -> int:
        """Float → 12-bit unsigned (for Kp, Kd which are non-negative)."""
        raw = round(x / x_max * 4095.0)
        return max(0, min(4095, raw))

    @staticmethod
    def _s16_to_float(raw: int, x_max: float) -> float:
        """16-bit two's-complement unsigned → float."""
        if raw >= 32768:
            raw -= 65536
        return raw / 32767.0 * x_max

    @staticmethod
    def _s12_to_float(raw: int, x_max: float) -> float:
        """12-bit two's-complement unsigned → float."""
        if raw >= 2048:
            raw -= 4096
        return raw / 2047.0 * x_max

    # ── Control frame ID ──────────────────────────────────────────────────────

    def _ctrl_id(self) -> int:
        _offsets = {
            RunMode.MIT:                   0x000,
            RunMode.POSITION_VELOCITY:     0x100,
            RunMode.VELOCITY:              0x200,
            RunMode.FORCE_POSITION_HYBRID: 0x300,
        }
        return (_offsets.get(self._mode, 0x000) + self.motor_id) & 0x7FF

    def _send_ctrl(self, data: bytes) -> None:
        self._comms.send(self._ctrl_id(), data)

    def _send_param(self, data: bytes) -> None:
        self._comms.send(0x7FF, data)

    # ── Frame handler ──────────────────────────────────────────────────────────

    def _on_frame_received(self, msg: "can.Message") -> None:
        """
        Unified frame handler called by DamiaoDispatcher in the Notifier thread.

        Frame type is distinguished by D[2]:
          0x33 → parameter read reply
          0x55 → parameter write reply
          0xAA → save parameters acknowledgement
          other → motor feedback
        """
        if msg.is_extended_id:
            return
        d = msg.data
        if len(d) < 4:
            return

        d2 = d[2]

        if d2 in (0x33, 0x55):
            # Parameter read/write reply: D[0] = CANID_L of addressed motor
            if d[0] != (self.motor_id & 0xFF):
                return
            if len(d) >= 8:
                self._param_result = bytes(d[4:8])
                self._param_event.set()
            return

        if d2 == 0xAA:
            # Save-parameters acknowledgement (4 bytes, no data to extract)
            return

        # Motor feedback frame (8 bytes)
        if len(d) < 8:
            return
        # D[0] lower nibble = motor_id & 0x0F
        if (d[0] & 0x0F) != (self.motor_id & 0x0F):
            return

        err = (d[0] >> 4) & 0x0F

        raw_pos = (d[1] << 8) | d[2]
        raw_vel = (d[3] << 4) | (d[4] >> 4)
        raw_tor = ((d[4] & 0x0F) << 8) | d[5]

        self._feedback = MotorFeedback(
            motor_id = self.motor_id,
            position = self._s16_to_float(raw_pos, self.MIT_P_MAX),
            velocity = self._s12_to_float(raw_vel, self.MIT_V_MAX),
            torque   = self._s12_to_float(raw_tor, self.MIT_T_MAX),
            t_mos    = float(d[6]),
            t_rotor  = float(d[7]),
            err      = err,
            enabled  = (err == FaultCode.ENABLED),
        )
        if self._feedback_callback:
            self._feedback_callback(self._feedback)

    # ── Core motor commands ────────────────────────────────────────────────────

    def enable(self) -> MotorFeedback:
        """Send enable command; motor enters run state and replies with feedback."""
        self._send_ctrl(b'\xff\xff\xff\xff\xff\xff\xff\xfc')
        return self._feedback

    def disable(self) -> MotorFeedback:
        """Send disable command; motor exits run state (default power-on state)."""
        self._send_ctrl(b'\xff\xff\xff\xff\xff\xff\xff\xfd')
        return self._feedback

    def set_zero_position(self) -> MotorFeedback:
        """Set current output shaft position as the zero reference."""
        self._send_ctrl(b'\xff\xff\xff\xff\xff\xff\xff\xfe')
        return self._feedback

    def clear_faults(self) -> MotorFeedback:
        """Clear latched fault state (over-temperature, etc.)."""
        self._send_ctrl(b'\xff\xff\xff\xff\xff\xff\xff\xfb')
        return self._feedback

    # ── Mode control ──────────────────────────────────────────────────────────

    def set_run_mode(self, mode: RunMode) -> None:
        """
        Write CTRL_MODE register (0x0A) to switch control modes.

        On switch the motor resets position, velocity, torque, Kp, and Kd
        command values to zero.  Mode change is volatile — issue save_params()
        (while disabled) to persist across power cycles.
        """
        self._mode = mode
        self.write_param_uint(RegAddr.CTRL_MODE, int(mode))

    # ── MIT mode ──────────────────────────────────────────────────────────────

    def set_operation_control(
        self,
        position:  float = 0.0,
        velocity:  float = 0.0,
        kp:        float = 0.0,
        kd:        float = 0.0,
        torque_ff: float = 0.0,
    ) -> MotorFeedback:
        """
        MIT mode torque command (RunMode.MIT must be active).

        Implements:  T_ref = Kp*(p_des - p_act) + Kd*(v_des - v_act) + t_ff

        Args:
            position:  Desired position (rad), range ±MIT_P_MAX.
            velocity:  Desired velocity (rad/s), range ±MIT_V_MAX.
            kp:        Position proportional gain [0, 500].
            kd:        Velocity derivative gain [0, 5].
            torque_ff: Feedforward torque (N·m), range ±MIT_T_MAX.

        Note: kd must be non-zero when position control is used (kp > 0),
        otherwise the motor may oscillate or lose control.
        """
        p   = self._float_to_s16(position,  self.MIT_P_MAX)
        v   = self._float_to_s12(velocity,  self.MIT_V_MAX)
        kp_ = self._float_to_u12(kp,        KP_MAX)
        kd_ = self._float_to_u12(kd,        KD_MAX)
        t   = self._float_to_s12(torque_ff, self.MIT_T_MAX)

        payload = bytes([
            (p   >> 8) & 0xFF,
             p         & 0xFF,
            (v   >> 4) & 0xFF,
            ((v  & 0xF) << 4) | ((kp_ >> 8) & 0xF),
             kp_        & 0xFF,
            (kd_ >> 4)  & 0xFF,
            ((kd_ & 0xF) << 4) | ((t >> 8) & 0xF),
             t          & 0xFF,
        ])
        self._send_ctrl(payload)
        return self._feedback

    # ── Position-Velocity mode ─────────────────────────────────────────────────

    def set_position_velocity(
        self,
        position_rad:   float,
        velocity_rad_s: float = 2.0,
    ) -> MotorFeedback:
        """
        Position-Velocity mode: trapezoidal acceleration profile
        (RunMode.POSITION_VELOCITY must be active).

        The acceleration and deceleration ramps are set via the ACC (0x04)
        and DEC (0x05) registers.

        Args:
            position_rad:   Target output shaft position (rad).
            velocity_rad_s: Maximum velocity during the move (rad/s).
        """
        payload = struct.pack('<ff', position_rad, velocity_rad_s)
        self._send_ctrl(payload)
        return self._feedback

    # ── Velocity mode ──────────────────────────────────────────────────────────

    def set_velocity(self, velocity_rad_s: float) -> MotorFeedback:
        """
        Velocity mode command (RunMode.VELOCITY must be active).

        The damping factor (DETA register 0x1F) must be set to a non-zero
        positive value (recommended 4.0) for stable operation.
        """
        payload = struct.pack('<f', velocity_rad_s)
        self._send_ctrl(payload)
        return self._feedback

    # ── Force-Position Hybrid mode ─────────────────────────────────────────────

    def set_force_position(
        self,
        position_rad:      float,
        velocity_rad_s:    float = 2.0,
        current_per_unit:  float = 1.0,
    ) -> MotorFeedback:
        """
        Force-Position Hybrid mode: position control with torque saturation
        (RunMode.FORCE_POSITION_HYBRID must be active).

        Args:
            position_rad:     Target output shaft position (rad).
            velocity_rad_s:   Velocity limit (0–100 rad/s; capped at 100).
            current_per_unit: Current saturation limit as fraction of Imax
                              (0.0–1.0; capped at 1.0).
        """
        v_raw = int(min(max(velocity_rad_s,   0.0), 100.0) * 100)
        i_raw = int(min(max(current_per_unit, 0.0),   1.0) * 10000)
        payload = struct.pack('<fHH', position_rad, v_raw, i_raw)
        self._send_ctrl(payload)
        return self._feedback

    # ── Parameter read / write ─────────────────────────────────────────────────

    def write_param_float(self, addr: int, value: float) -> None:
        """
        Write a float register (volatile; call save_params() while disabled to persist).

        Args:
            addr:  Register address (RegAddr or raw int).
            value: Value to write.
        """
        payload = bytearray(8)
        payload[0] = self.motor_id & 0xFF
        payload[1] = (self.motor_id >> 8) & 0xFF
        payload[2] = 0x55
        payload[3] = int(addr) & 0xFF
        struct.pack_into('<f', payload, 4, value)
        self._send_param(bytes(payload))

    def write_param_uint(self, addr: int, value: int) -> None:
        """Write a uint32 register (volatile; call save_params() to persist)."""
        payload = bytearray(8)
        payload[0] = self.motor_id & 0xFF
        payload[1] = (self.motor_id >> 8) & 0xFF
        payload[2] = 0x55
        payload[3] = int(addr) & 0xFF
        struct.pack_into('<I', payload, 4, value & 0xFFFFFFFF)
        self._send_param(bytes(payload))

    def read_param_raw(self, addr: int) -> Optional[bytes]:
        """
        Read a register; returns the raw 4-byte data field (D[4:8] of reply),
        or None on timeout.
        """
        request = bytes([
            self.motor_id & 0xFF,
            (self.motor_id >> 8) & 0xFF,
            0x33,
            int(addr) & 0xFF,
        ])
        self._param_event.clear()
        self._send_param(request)
        if self._param_event.wait(timeout=self.rx_timeout):
            return self._param_result
        return None

    def read_param_float(self, addr: int) -> Optional[float]:
        """Read a float register."""
        raw = self.read_param_raw(addr)
        if raw and len(raw) >= 4:
            return struct.unpack('<f', raw[:4])[0]
        return None

    def read_param_uint(self, addr: int) -> Optional[int]:
        """Read a uint32 register."""
        raw = self.read_param_raw(addr)
        if raw and len(raw) >= 4:
            return struct.unpack('<I', raw[:4])[0]
        return None

    def save_params(self) -> None:
        """
        Persist all current parameter values to on-chip flash.

        Must be called while the motor is disabled.  Flash supports
        approximately 10,000 erase/write cycles — avoid calling frequently.
        """
        request = bytes([
            self.motor_id & 0xFF,
            (self.motor_id >> 8) & 0xFF,
            0xAA,
            0x01,
        ])
        self._send_param(request)

    # ── Motion profile configuration ───────────────────────────────────────────

    def set_acceleration(self, acc_rad_s2: float) -> None:
        """Set acceleration ramp for Position-Velocity mode (ACC register 0x04)."""
        self.write_param_float(RegAddr.ACC, acc_rad_s2)

    def set_deceleration(self, dec_rad_s2: float) -> None:
        """
        Set deceleration ramp for Position-Velocity mode (DEC register 0x05).
        Value must be negative (e.g., -10.0 for 10 rad/s²).
        """
        self.write_param_float(RegAddr.DEC, -abs(dec_rad_s2))

    def set_max_speed(self, max_rad_s: float) -> None:
        """Set maximum velocity limit (MAX_SPD register 0x06)."""
        self.write_param_float(RegAddr.MAX_SPD, max_rad_s)

    def set_can_timeout(self, timeout_counts: int) -> None:
        """
        Configure the CAN watchdog (TIMEOUT register 0x09).
        Motor exits Enable Mode if no command is received within the timeout.
        Set 0 to disable the watchdog.
        """
        self.write_param_uint(RegAddr.TIMEOUT, timeout_counts)

    # ── Telemetry ──────────────────────────────────────────────────────────────

    def read_output_position(self) -> Optional[float]:
        """Read output shaft absolute position via parameter read (rad)."""
        return self.read_param_float(RegAddr.XOUT)

    def read_motor_position(self) -> Optional[float]:
        """Read motor-side rotor position via parameter read (rad)."""
        return self.read_param_float(RegAddr.P_M)

    def read_vbus(self) -> Optional[float]:
        """Read bus voltage (V)."""
        return self.read_param_float(RegAddr.VBUS)

    def read_pcb_temp(self) -> Optional[float]:
        """Read PCB (MOSFET) temperature (°C)."""
        return self.read_param_float(RegAddr.TPCB)

    def read_motor_temp(self) -> Optional[float]:
        """Read motor winding temperature (°C)."""
        return self.read_param_float(RegAddr.TMTR)

    def read_firmware_version(self) -> Optional[int]:
        """Read firmware version number."""
        return self.read_param_uint(RegAddr.SW_VER)

    def read_gear_ratio(self) -> Optional[float]:
        """Read gear reduction ratio (RO)."""
        return self.read_param_float(RegAddr.GR)

    def read_imax(self) -> Optional[float]:
        """Read driver maximum current limit (A, RO)."""
        return self.read_param_float(RegAddr.IMAX)

    # ── Feedback callback / property ───────────────────────────────────────────

    def set_feedback_callback(self, callback) -> None:
        """Register a callable invoked with MotorFeedback on every feedback frame."""
        self._feedback_callback = callback

    @property
    def feedback(self) -> MotorFeedback:
        """Current motor state, updated on every feedback frame."""
        return self._feedback
