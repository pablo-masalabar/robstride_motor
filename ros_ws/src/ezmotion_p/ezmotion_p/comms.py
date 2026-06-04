"""
comms.py – CAN transport layer for EZmotion PCN/SCN C2 series motors (CANopen).

Architecture
------------
EZmotion motors communicate over CANopen (CiA DS301 + DS402) using standard
11-bit CAN frames at configurable baud rates (10 kbps – 1 Mbps).

Each motor has a unique CANopen Node-ID. The COB-IDs used per motor are:

  TX (host → motor):
    NMT:              0x000  (broadcast)
    SDO download:     0x600 + node_id
    RPDO1:            0x200 + node_id
    RPDO2:            0x300 + node_id
    RPDO3:            0x400 + node_id
    RPDO4:            0x500 + node_id

  RX (motor → host):
    SDO upload reply: 0x580 + node_id
    TPDO1:            0x180 + node_id
    TPDO2:            0x280 + node_id
    TPDO3:            0x380 + node_id
    TPDO4:            0x480 + node_id
    NMT heartbeat:    0x700 + node_id

The dispatcher routes incoming frames to the registered motor callback based
on arbitration ID. Multiple motors on the same bus use separate node IDs and
each register their own COB-ID set.

Usage::

    with EZMotionCANComms("can0") as bus:
        bus.start_listener()
        m = MMS760400_48_C2_1(node_id=1, comms=bus)
        m.nmt_start()
        m.enable()

Requires: python-can
"""

from typing import Callable, Dict, List, Optional

import can
from can import BusState


def _rx_cob_ids(node_id: int) -> List[int]:
    """Return all COB-IDs the motor transmits for a given node_id."""
    return [
        0x580 + node_id,  # SDO upload response
        0x180 + node_id,  # TPDO1
        0x280 + node_id,  # TPDO2
        0x380 + node_id,  # TPDO3
        0x480 + node_id,  # TPDO4
        0x700 + node_id,  # NMT heartbeat / boot-up
    ]


class _SafeNotifier(can.Notifier):
    def __init__(self, bus, listeners, error_handler=None, **kwargs):
        self._error_handler = error_handler
        super().__init__(bus, listeners, **kwargs)

    def _rx_thread(self, bus):
        try:
            super()._rx_thread(bus)
        except Exception as exc:
            if self._error_handler:
                self._error_handler(exc)
            else:
                print(f'[EZMotionCANComms] Bus error: {exc}', flush=True)


class _EZMotionDispatcher(can.Listener):
    """Routes incoming frames to the motor callback registered for that COB-ID."""

    def __init__(self) -> None:
        self._callbacks: Dict[int, Callable] = {}

    def register(
        self,
        cob_ids: List[int],
        callback: Callable[['can.Message'], None],
    ) -> None:
        for cob_id in cob_ids:
            self._callbacks[cob_id] = callback

    def on_message_received(self, msg: 'can.Message') -> None:
        if msg.is_extended_id or msg.is_error_frame:
            return
        cb = self._callbacks.get(msg.arbitration_id)
        if cb:
            cb(msg)


class EZMotionCANComms:
    """CANopen CAN bus wrapper for EZmotion PCN/SCN C2 series motors."""

    def __init__(
        self,
        channel:   str   = 'can0',
        bustype:   str   = 'socketcan',
        bitrate:   int   = 1_000_000,
        on_error:  Optional[Callable[[Exception], None]] = None,
    ):
        self._error_callback = on_error
        self._bus = can.interface.Bus(
            channel=channel,
            bustype=bustype,
            bitrate=bitrate,
        )
        # Accept only standard (11-bit) frames at the kernel level.
        # Keeps extended-frame traffic (e.g. RobStride) off this socket
        # when both motor types share the same physical CAN interface.
        self._bus.set_filters([{'can_id': 0, 'can_mask': 0, 'extended': False}])
        self._dispatcher = _EZMotionDispatcher()
        self._notifier:  Optional[can.Notifier] = None

    # ── Context manager ────────────────────────────────────────────────────

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()

    def close(self) -> None:
        self.stop_listener()
        self._bus.shutdown()

    # ── Motor registration ─────────────────────────────────────────────────

    def add_motor_callback(
        self,
        node_id:  int,
        callback: Callable[['can.Message'], None],
    ) -> None:
        """Register a motor callback for all incoming COB-IDs of the given node_id."""
        cob_ids = _rx_cob_ids(node_id)
        self._dispatcher.register(cob_ids, callback)

    # ── Background listener ────────────────────────────────────────────────

    def _on_notifier_error(self, exc: Exception) -> None:
        if self._error_callback:
            self._error_callback(exc)
        else:
            print(f'[EZMotionCANComms] Bus error on {self._bus.channel_info}: {exc}', flush=True)

    def start_listener(self, extra_listeners: Optional[List[can.Listener]] = None) -> None:
        if self._notifier is not None:
            return
        listeners = [self._dispatcher] + (extra_listeners or [])
        self._notifier = _SafeNotifier(
            self._bus, listeners, error_handler=self._on_notifier_error
        )

    def stop_listener(self) -> None:
        if self._notifier is not None:
            self._notifier.stop()
            self._notifier = None

    # ── Bus state ──────────────────────────────────────────────────────────

    @property
    def state(self) -> BusState:
        return self._bus.state

    # ── Transmit ───────────────────────────────────────────────────────────

    def send(self, arb_id: int, data: bytes) -> None:
        """Send a standard 11-bit CAN frame."""
        self._bus.send(can.Message(
            arbitration_id=arb_id,
            data=bytes(data),
            is_extended_id=False,
        ))
