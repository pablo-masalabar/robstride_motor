"""
motor_base.py – CANopen DS301/DS402 protocol implementation for EZmotion C2 motors.

Communication overview
----------------------
EZmotion PCN/SCN C2 motors use CANopen over standard 11-bit CAN frames.

NMT (Network Management)
  COB-ID 0x000.  Controls node state: Operational, Pre-Operational, Stopped.
  D[0] = command specifier (CS), D[1] = node-id (0 = broadcast).

SDO (Service Data Object)
  Used for parameter configuration.  Client/server model — host sends a
  request to 0x600+node_id, motor replies on 0x580+node_id.
  Expedited transfer only (≤4 bytes per message).

PDO (Process Data Object)
  Real-time data.  No reply frame — fire-and-forget.
  RPDO (host → motor): 0x200+, 0x300+, 0x400+, 0x500+ node_id
  TPDO (motor → host): 0x180+, 0x280+, 0x380+, 0x480+ node_id

Default PDO mappings (from EDS file MMS760400-48-C2-1.eds):
  RPDO3 (0x400+n): Controlword (6040h, 16-bit) | Target position (607Ah, 32-bit)
  RPDO4 (0x500+n): Controlword (6040h, 16-bit) | Target velocity (60FFh, 32-bit)
  TPDO3 (0x380+n): Statusword (6041h, 16-bit)  | Position actual (6064h, 32-bit)
  TPDO4 (0x480+n): Statusword (6041h, 16-bit)  | Velocity actual (606Ch, 32-bit)

DS402 state machine
-------------------
Sequence to reach Operation Enabled:
  1. NMT start (CS=1)
  2. Write controlword 0x0006 (Shutdown)   → Ready to Switch On
  3. Write controlword 0x000F (Switch on + Enable operation) → Operation Enabled

Unit conventions
----------------
Position  : radians (converted from encoder counts internally)
Velocity  : rad/s   (converted from counts/s internally)
Torque    : N·m     (converted from DS402 0.1% rated torque units internally)
"""

import math
import struct
import threading
from dataclasses import dataclass, field
from enum import IntEnum
from typing import TYPE_CHECKING, Optional

if TYPE_CHECKING:
    import can

from .comms import EZMotionCANComms


# ── Unit conversion ────────────────────────────────────────────────────────────

COUNTS_PER_REV: int = 65536  # encoder increments per revolution (from EDS 608Fh)
RAD_PER_COUNT:  float = (2.0 * math.pi) / COUNTS_PER_REV
COUNT_PER_RAD:  float = COUNTS_PER_REV / (2.0 * math.pi)


def _rad_to_counts(rad: float) -> int:
    return round(rad * COUNT_PER_RAD)

def _counts_to_rad(counts: int) -> float:
    return counts * RAD_PER_COUNT

def _rad_s_to_counts_s(rad_s: float) -> int:
    return round(rad_s * COUNT_PER_RAD)

def _counts_s_to_rad_s(counts_s: int) -> float:
    return counts_s * RAD_PER_COUNT


# ── Enumerations ───────────────────────────────────────────────────────────────

class NMTCommand(IntEnum):
    START        = 1    # → Operational
    STOP         = 2    # → Stopped
    PRE_OP       = 128  # → Pre-Operational
    RESET_NODE   = 129  # Full node reset
    RESET_COMMS  = 130  # Reset communication objects only


class OperationMode(IntEnum):
    """DS402 6060h Modes of Operation."""
    PP  = 1   # Profile Position
    PV  = 3   # Profile Velocity
    PT  = 4   # Profile Torque
    HM  = 6   # Homing
    CSP = 8   # Cyclic Synchronous Position
    CSV = 9   # Cyclic Synchronous Velocity
    CST = 10  # Cyclic Synchronous Torque


class DriveState(IntEnum):
    """DS402 state machine states decoded from statusword."""
    NOT_READY_TO_SWITCH_ON = 0
    SWITCH_ON_DISABLED     = 1
    READY_TO_SWITCH_ON     = 2
    SWITCHED_ON            = 3
    OPERATION_ENABLED      = 4
    QUICK_STOP_ACTIVE      = 5
    FAULT_REACTION_ACTIVE  = 6
    FAULT                  = 7
    UNKNOWN                = -1


# DS402 controlword commands
_CTRL_SHUTDOWN        = 0x0006
_CTRL_SWITCH_ON       = 0x0007
_CTRL_ENABLE_OP       = 0x000F
_CTRL_DISABLE_OP      = 0x0007
_CTRL_DISABLE_VOLTAGE = 0x0000
_CTRL_QUICK_STOP      = 0x0002
_CTRL_FAULT_RESET     = 0x0080
_CTRL_NEW_SETPOINT    = 0x001F  # PP: enable op + new setpoint trigger (bit4)


# ── SDO constants ──────────────────────────────────────────────────────────────

# SDO download (write) command bytes
_SDO_WRITE_4 = 0x23  # 4 bytes
_SDO_WRITE_2 = 0x2B  # 2 bytes
_SDO_WRITE_1 = 0x2F  # 1 byte

# SDO upload (read) command bytes
_SDO_READ_REQ = 0x40  # read request
_SDO_READ_4   = 0x43  # response: 4 bytes
_SDO_READ_2   = 0x4B  # response: 2 bytes
_SDO_READ_1   = 0x4F  # response: 1 byte
_SDO_ABORT    = 0x80
_SDO_WRITE_ACK = 0x60  # server response to expedited download


# ── DS402 object indices (16-bit) ──────────────────────────────────────────────

class OD(IntEnum):
    """Frequently used object dictionary indices."""
    CTRL_WORD         = 0x6040
    STATUS_WORD       = 0x6041
    MODES_OF_OP       = 0x6060
    MODES_OF_OP_DISP  = 0x6061
    POS_DEMAND        = 0x6062
    POS_ACTUAL        = 0x6064
    VEL_DEMAND        = 0x606B
    VEL_ACTUAL        = 0x606C
    TARGET_TORQUE     = 0x6071
    MAX_TORQUE        = 0x6072
    MAX_CURRENT       = 0x6073
    TORQUE_ACTUAL     = 0x6077
    TARGET_POSITION   = 0x607A
    MAX_PROFILE_VEL   = 0x607F
    PROFILE_VEL       = 0x6081
    PROFILE_ACCEL     = 0x6083
    PROFILE_DECEL     = 0x6084
    TARGET_VELOCITY   = 0x60FF
    STORE_PARAMS      = 0x1010
    CAN_NODE_ID       = 0x2101
    CAN_BIT_RATE      = 0x2102
    TEMPERATURE       = 0x2040   # manufacturer — motor temperature (°C)
    ERROR_STATUS      = 0x200B   # manufacturer — error status (sub 0x0B = error_status)
    HOMING_TORQUE     = 0x2070   # manufacturer — max homing torque % (sub 0x01)
    HOME_OFFSET       = 0x607C   # home offset (counts)
    HOMING_METHOD     = 0x6098   # homing method
    HOMING_SPEED      = 0x6099   # homing speed (sub1=search, sub2=zero)
    HOMING_ACCEL      = 0x609A   # homing acceleration


def _decode_drive_state(sw: int) -> DriveState:
    """Decode DS402 statusword to DriveState."""
    if   (sw & 0x4F) == 0x00: return DriveState.NOT_READY_TO_SWITCH_ON
    elif (sw & 0x4F) == 0x40: return DriveState.SWITCH_ON_DISABLED
    elif (sw & 0x6F) == 0x21: return DriveState.READY_TO_SWITCH_ON
    elif (sw & 0x6F) == 0x23: return DriveState.SWITCHED_ON
    elif (sw & 0x6F) == 0x27: return DriveState.OPERATION_ENABLED
    elif (sw & 0x6F) == 0x07: return DriveState.QUICK_STOP_ACTIVE
    elif (sw & 0x4F) == 0x0F: return DriveState.FAULT_REACTION_ACTIVE
    elif (sw & 0x4F) == 0x08: return DriveState.FAULT
    return DriveState.UNKNOWN


# ── Data structures ────────────────────────────────────────────────────────────

@dataclass
class MotorFeedback:
    """Decoded motor state, updated from TPDO3 and TPDO4 frames."""
    node_id:     int        = 0
    position:    float      = 0.0   # rad  (from TPDO3)
    velocity:    float      = 0.0   # rad/s (from TPDO4)
    torque:      float      = 0.0   # N·m  (from SDO read, not mapped by default TPDO)
    statusword:  int        = 0
    drive_state: DriveState = DriveState.UNKNOWN
    op_mode:     int        = 0     # from TPDO2 if enabled, else updated on SDO read
    fault:       bool       = False
    enabled:     bool       = False


# ── Base motor class ───────────────────────────────────────────────────────────

class EZMotionMotorBase:
    """
    CANopen DS401/DS402 implementation for EZmotion PCN/SCN C2 series motors.

    Subclasses define motor-specific physical limits:
        RATED_TORQUE_NM   rated torque (N·m) — used for torque unit conversion
        MAX_SPEED_RPM     maximum speed (RPM)
        MAX_TORQUE_PERMIL max torque in 0.1% units (default 3000 = 300% rated)
    """

    RATED_TORQUE_NM:   float = 1.27
    MAX_SPEED_RPM:     float = 3000.0
    MAX_TORQUE_PERMIL: int   = 3000   # 300% of rated

    def __init__(
        self,
        node_id:    int,
        comms:      EZMotionCANComms,
        rx_timeout: float = 0.1,
    ):
        if comms is None:
            raise ValueError('comms must be an EZMotionCANComms instance')
        self.node_id    = node_id
        self.rx_timeout = rx_timeout
        self._comms     = comms
        self._feedback  = MotorFeedback(node_id=node_id)

        self._sdo_result: Optional[bytes]  = None
        self._sdo_event                    = threading.Event()
        self._sdo_lock                     = threading.Lock()
        self._sdo_pending: tuple           = (None, None)  # (index, sub) of in-flight request
        self._feedback_callback            = None

        self._sdo_rx_id  = 0x580 + node_id
        self._sdo_tx_id  = 0x600 + node_id
        self._rpdo1_id   = 0x200 + node_id
        self._rpdo2_id   = 0x300 + node_id
        self._rpdo3_id   = 0x400 + node_id
        self._rpdo4_id   = 0x500 + node_id

        comms.add_motor_callback(node_id, self._on_frame_received)

    # ── Frame dispatch ─────────────────────────────────────────────────────────

    def _on_frame_received(self, msg: 'can.Message') -> None:
        arb = msg.arbitration_id
        d   = msg.data

        if arb == self._sdo_rx_id:
            self._handle_sdo_response(d)
        elif arb == 0x180 + self.node_id:
            self._handle_tpdo1(d)
        elif arb == 0x280 + self.node_id:
            self._handle_tpdo2(d)
        elif arb == 0x380 + self.node_id:
            self._handle_tpdo3(d)
        elif arb == 0x480 + self.node_id:
            self._handle_tpdo4(d)
        elif arb == 0x700 + self.node_id:
            self._handle_heartbeat(d)

    def _handle_sdo_response(self, d: bytes) -> None:
        if len(d) < 8:
            return
        cmd        = d[0]
        resp_index = d[1] | (d[2] << 8)
        resp_sub   = d[3]

        # Drop frames that don't match the in-flight request
        pending_index, pending_sub = self._sdo_pending
        if pending_index is not None:
            if resp_index != pending_index or resp_sub != pending_sub:
                return

        if cmd == _SDO_ABORT:
            self._sdo_result = None
            self._sdo_event.set()
        elif cmd in (_SDO_READ_4, _SDO_READ_2, _SDO_READ_1):
            self._sdo_result = bytes(d[4:8])
            self._sdo_event.set()
        elif cmd == _SDO_WRITE_ACK:
            self._sdo_result = bytes(4)
            self._sdo_event.set()

    def _handle_tpdo1(self, d: bytes) -> None:
        """TPDO1 default: Statusword (16-bit)."""
        if len(d) < 2:
            return
        sw = struct.unpack_from('<H', d, 0)[0]
        self._update_statusword(sw)

    def _handle_tpdo2(self, d: bytes) -> None:
        """TPDO2 default: Statusword (16-bit) + Modes of operation display (8-bit)."""
        if len(d) < 3:
            return
        sw      = struct.unpack_from('<H', d, 0)[0]
        op_mode = d[2]
        self._update_statusword(sw)
        self._feedback.op_mode = op_mode

    def _handle_tpdo3(self, d: bytes) -> None:
        """TPDO3 default: Statusword (16-bit) + Position actual value (32-bit)."""
        if len(d) < 6:
            return
        sw     = struct.unpack_from('<H', d, 0)[0]
        counts = struct.unpack_from('<i', d, 2)[0]
        self._update_statusword(sw)
        self._feedback.position = _counts_to_rad(counts)
        if self._feedback_callback:
            self._feedback_callback(self._feedback)

    def _handle_tpdo4(self, d: bytes) -> None:
        """TPDO4 default: Statusword (16-bit) + Velocity actual value (32-bit)."""
        if len(d) < 6:
            return
        sw      = struct.unpack_from('<H', d, 0)[0]
        counts_s = struct.unpack_from('<i', d, 2)[0]
        self._update_statusword(sw)
        self._feedback.velocity = _counts_s_to_rad_s(counts_s)

    def _handle_heartbeat(self, d: bytes) -> None:
        pass  # heartbeat/boot-up — no action needed by default

    def _update_statusword(self, sw: int) -> None:
        self._feedback.statusword  = sw
        self._feedback.drive_state = _decode_drive_state(sw)
        self._feedback.fault       = bool(sw & (1 << 3))
        self._feedback.enabled     = (self._feedback.drive_state == DriveState.OPERATION_ENABLED)

    # ── NMT ───────────────────────────────────────────────────────────────────

    def _nmt(self, command: NMTCommand) -> None:
        self._comms.send(0x000, bytes([int(command), self.node_id]))

    def nmt_start(self) -> None:
        """Put node into Operational state (PDOs active)."""
        self._nmt(NMTCommand.START)

    def nmt_stop(self) -> None:
        """Put node into Stopped state."""
        self._nmt(NMTCommand.STOP)

    def nmt_pre_operational(self) -> None:
        """Put node into Pre-Operational state (SDO active, PDOs inactive)."""
        self._nmt(NMTCommand.PRE_OP)

    def nmt_reset(self) -> None:
        """Reset the node (full restart)."""
        self._nmt(NMTCommand.RESET_NODE)

    # ── SDO ───────────────────────────────────────────────────────────────────

    def _sdo_request(self, data: bytes) -> Optional[bytes]:
        """Send SDO request and block until reply or timeout. Returns reply data bytes."""
        with self._sdo_lock:
            self._sdo_pending = (data[1] | (data[2] << 8), data[3])
            self._sdo_event.clear()
            self._sdo_result = None
            self._comms.send(self._sdo_tx_id, data)
            if self._sdo_event.wait(timeout=self.rx_timeout):
                return self._sdo_result
            self._sdo_pending = (None, None)
            return None

    def write_sdo_u16(self, index: int, sub: int, value: int) -> bool:
        req = bytes([_SDO_WRITE_2, index & 0xFF, (index >> 8) & 0xFF, sub]) + \
              struct.pack('<H', value & 0xFFFF) + b'\x00\x00'
        return self._sdo_request(req) is not None

    def write_sdo_s16(self, index: int, sub: int, value: int) -> bool:
        return self.write_sdo_u16(index, sub, value & 0xFFFF)

    def write_sdo_u32(self, index: int, sub: int, value: int) -> bool:
        req = bytes([_SDO_WRITE_4, index & 0xFF, (index >> 8) & 0xFF, sub]) + \
              struct.pack('<I', value & 0xFFFFFFFF)
        return self._sdo_request(req) is not None

    def write_sdo_s32(self, index: int, sub: int, value: int) -> bool:
        req = bytes([_SDO_WRITE_4, index & 0xFF, (index >> 8) & 0xFF, sub]) + \
              struct.pack('<i', value)
        return self._sdo_request(req) is not None

    def write_sdo_u8(self, index: int, sub: int, value: int) -> bool:
        req = bytes([_SDO_WRITE_1, index & 0xFF, (index >> 8) & 0xFF, sub,
                     value & 0xFF, 0, 0, 0])
        return self._sdo_request(req) is not None

    def read_sdo_raw(self, index: int, sub: int) -> Optional[bytes]:
        req = bytes([_SDO_READ_REQ, index & 0xFF, (index >> 8) & 0xFF, sub, 0, 0, 0, 0])
        return self._sdo_request(req)

    def read_sdo_u32(self, index: int, sub: int) -> Optional[int]:
        raw = self.read_sdo_raw(index, sub)
        return struct.unpack('<I', raw[:4])[0] if raw and len(raw) >= 4 else None

    def read_sdo_s32(self, index: int, sub: int) -> Optional[int]:
        raw = self.read_sdo_raw(index, sub)
        return struct.unpack('<i', raw[:4])[0] if raw and len(raw) >= 4 else None

    def read_sdo_u16(self, index: int, sub: int) -> Optional[int]:
        raw = self.read_sdo_raw(index, sub)
        return struct.unpack('<H', raw[:2])[0] if raw and len(raw) >= 2 else None

    def read_sdo_s16(self, index: int, sub: int) -> Optional[int]:
        raw = self.read_sdo_raw(index, sub)
        return struct.unpack('<h', raw[:2])[0] if raw and len(raw) >= 2 else None

    def read_sdo_u8(self, index: int, sub: int) -> Optional[int]:
        raw = self.read_sdo_raw(index, sub)
        return raw[0] if raw else None

    # ── Controlword helpers ───────────────────────────────────────────────────

    def _write_controlword(self, value: int) -> bool:
        return self.write_sdo_u16(OD.CTRL_WORD, 0x00, value)

    # ── DS402 state machine ───────────────────────────────────────────────────

    def fault_reset(self) -> None:
        """Clear fault state (rising edge on bit 7 of controlword)."""
        self._write_controlword(_CTRL_FAULT_RESET)
        self._write_controlword(0x0000)

    def shutdown(self) -> None:
        """Transition to Ready to Switch On."""
        self._write_controlword(_CTRL_SHUTDOWN)

    def switch_on(self) -> None:
        """Transition to Switched On (power stage on, drive not running)."""
        self._write_controlword(_CTRL_SWITCH_ON)

    def enable(self) -> MotorFeedback:
        """Transition through the full DS402 sequence to Operation Enabled.

        Sequence: Shutdown → Ready to Switch On → Operation Enabled.
        If in fault state, performs fault reset first.
        """
        state = self._feedback.drive_state
        if state == DriveState.FAULT:
            self.fault_reset()
        if state != DriveState.OPERATION_ENABLED:
            self._write_controlword(_CTRL_SHUTDOWN)   # → Ready to Switch On
            self._write_controlword(_CTRL_ENABLE_OP)  # → Operation Enabled
        return self._feedback

    def disable(self) -> MotorFeedback:
        """Transition to Switch On Disabled (power stage off)."""
        self._write_controlword(_CTRL_DISABLE_VOLTAGE)
        return self._feedback

    def quick_stop(self) -> MotorFeedback:
        """Apply quick stop (deceleration ramp, motor remains powered)."""
        self._write_controlword(_CTRL_QUICK_STOP)
        return self._feedback

    # ── Operation mode ────────────────────────────────────────────────────────

    def set_operation_mode(self, mode: OperationMode) -> bool:
        """Set DS402 operation mode (6060h). Motor must be in Operation Enabled."""
        return self.write_sdo_u8(OD.MODES_OF_OP, 0x00, int(mode) & 0xFF)

    def read_operation_mode(self) -> Optional[int]:
        """Read active operation mode from 6061h (Modes of Operation Display)."""
        return self.read_sdo_u8(OD.MODES_OF_OP_DISP, 0x00)

    # ── Profile Position (PP) mode commands ──────────────────────────────────

    def set_target_position_pp(self, position_rad: float) -> None:
        """
        PP mode: send target position via RPDO3.

        RPDO3 mapping: Controlword (16-bit) | Target position (32-bit).
        Sends with controlword=0x000F (Operation Enabled).
        The move starts on the next new-setpoint trigger — call trigger_move_pp()
        after this to actually start motion.
        """
        ctrl   = _CTRL_ENABLE_OP
        counts = _rad_to_counts(position_rad)
        data   = struct.pack('<Hi', ctrl, counts)
        self._comms.send(self._rpdo3_id, data)

    def trigger_move_pp(self, position_rad: float) -> None:
        """
        PP mode: set target position and trigger the move.

        Sequence (matching ref.py):
          1. Controlword = ENABLED  (0x000F) + position — confirm enabled state
          2. Controlword = TRIGGER  (0x001F) + position — new setpoint bit4 set
          3. Controlword = ENABLED  (0x000F) + position — clear bit4
        """
        counts = _rad_to_counts(position_rad)
        # Step 1: confirm enabled
        self._comms.send(self._rpdo3_id, struct.pack('<Hi', _CTRL_ENABLE_OP, counts))
        # Step 2: trigger new setpoint (bit 4)
        self._comms.send(self._rpdo3_id, struct.pack('<Hi', _CTRL_NEW_SETPOINT, counts))
        # Step 3: clear bit 4
        self._comms.send(self._rpdo3_id, struct.pack('<Hi', _CTRL_ENABLE_OP, counts))

    # ── Profile Velocity (PV) mode commands ───────────────────────────────────

    def set_target_velocity_pv(self, velocity_rad_s: float) -> None:
        """
        PV mode: send target velocity via RPDO4.

        RPDO4 mapping: Controlword (16-bit) | Target velocity (32-bit, counts/s).
        """
        counts_s = _rad_s_to_counts_s(velocity_rad_s)
        data     = struct.pack('<Hi', _CTRL_ENABLE_OP, counts_s)
        self._comms.send(self._rpdo4_id, data)

    # ── CSP / CSV mode commands ───────────────────────────────────────────────

    def set_cyclic_position(self, position_rad: float) -> None:
        """CSP mode: send target position via RPDO3."""
        counts = _rad_to_counts(position_rad)
        data   = struct.pack('<Hi', _CTRL_ENABLE_OP, counts)
        self._comms.send(self._rpdo3_id, data)

    def set_cyclic_velocity(self, velocity_rad_s: float) -> None:
        """CSV mode: send target velocity via RPDO4."""
        counts_s = _rad_s_to_counts_s(velocity_rad_s)
        data     = struct.pack('<Hi', _CTRL_ENABLE_OP, counts_s)
        self._comms.send(self._rpdo4_id, data)

    # ── Profile parameters (SDO) ──────────────────────────────────────────────

    def set_profile_velocity(self, velocity_rad_s: float) -> bool:
        """Set PP/PV profile velocity (6081h) via SDO."""
        return self.write_sdo_u32(OD.PROFILE_VEL, 0x00, _rad_s_to_counts_s(abs(velocity_rad_s)))

    def set_profile_acceleration(self, accel_rad_s2: float) -> bool:
        """Set profile acceleration (6083h) via SDO (counts/s²)."""
        return self.write_sdo_u32(OD.PROFILE_ACCEL, 0x00, _rad_s_to_counts_s(abs(accel_rad_s2)))

    def set_profile_deceleration(self, decel_rad_s2: float) -> bool:
        """Set profile deceleration (6084h) via SDO (counts/s²)."""
        return self.write_sdo_u32(OD.PROFILE_DECEL, 0x00, _rad_s_to_counts_s(abs(decel_rad_s2)))

    def set_max_torque(self, torque_Nm: float) -> bool:
        """Set max torque (6072h) via SDO. Clamped to MAX_TORQUE_PERMIL."""
        permil = min(
            round(torque_Nm / self.RATED_TORQUE_NM * 1000.0),
            self.MAX_TORQUE_PERMIL,
        )
        return self.write_sdo_u16(OD.MAX_TORQUE, 0x00, permil)

    def set_target_torque(self, torque_Nm: float) -> bool:
        """Set target torque (6071h) via SDO in PT/CST mode."""
        permil = round(torque_Nm / self.RATED_TORQUE_NM * 1000.0)
        return self.write_sdo_s16(OD.TARGET_TORQUE, 0x00, permil)

    # ── Feedback reads (SDO) ──────────────────────────────────────────────────

    def read_position(self) -> Optional[float]:
        """Read position actual value (6064h) via SDO (rad)."""
        raw = self.read_sdo_s32(OD.POS_ACTUAL, 0x00)
        return _counts_to_rad(raw) if raw is not None else None

    def read_velocity(self) -> Optional[float]:
        """Read velocity actual value (606Ch) via SDO (rad/s)."""
        raw = self.read_sdo_s32(OD.VEL_ACTUAL, 0x00)
        return _counts_s_to_rad_s(raw) if raw is not None else None

    def read_statusword(self) -> Optional[int]:
        """Read statusword (6041h) via SDO and update feedback."""
        sw = self.read_sdo_u16(OD.STATUS_WORD, 0x00)
        if sw is not None:
            self._update_statusword(sw)
        return sw

    def read_drive_state(self) -> DriveState:
        """Read and decode current DS402 drive state."""
        self.read_statusword()
        return self._feedback.drive_state

    # ── Persistence ───────────────────────────────────────────────────────────

    def save_params(self) -> bool:
        """
        Save all parameters to NVM (object 1010h sub1).

        Write the signature 0x65766173 ('evas' in little-endian ASCII) to
        trigger the store.  Motor must be in disabled state.
        """
        return self.write_sdo_u32(OD.STORE_PARAMS, 0x01, 0x65766173)

    # ── CAN configuration (SDO) ───────────────────────────────────────────────

    def set_can_node_id(self, new_id: int) -> bool:
        """Write new CAN Node ID (2101h, UINT8). Requires save + reset to take effect."""
        return self.write_sdo_u8(OD.CAN_NODE_ID, 0x00, new_id & 0xFF)

    def set_can_bitrate(self, bitrate_kbps: int) -> bool:
        """Write CAN bit rate (2102h, UINT16) in kbps. Requires save + reset to take effect.
        Common values: 125, 250, 500, 1000."""
        return self.write_sdo_u16(OD.CAN_BIT_RATE, 0x00, bitrate_kbps & 0xFFFF)

    # ── Feedback callback / property ──────────────────────────────────────────

    def set_feedback_callback(self, callback) -> None:
        """Register a callable invoked with MotorFeedback on every TPDO3 frame."""
        self._feedback_callback = callback

    @property
    def feedback(self) -> MotorFeedback:
        """Latest motor state decoded from TPDOs."""
        return self._feedback
