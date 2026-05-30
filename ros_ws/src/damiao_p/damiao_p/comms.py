"""
comms.py – CAN transport layer for Damiao DM-J43xx geared motors.

Architecture
------------
Damiao motors use standard 11-bit CAN frames (CAN 2.0B STD), unlike the
RobStride extended-ID protocol.

All motor replies (feedback, parameter read/write, save acknowledgement)
are transmitted by the motor with arbitration ID = master_id (default 0).
Motor identity within a reply is encoded in D[0]:
  - Feedback frames:     D[0] = (ERR << 4) | (motor_id & 0x0F)
  - Parameter replies:   D[0] = CANID_L = motor_id & 0xFF

For motor IDs 0–15, both encodings give the same lower nibble, so a
single filter on D[0] & 0x0F unambiguously identifies the source motor.
(Motors with IDs 16+ are not supported by this dispatcher.)

_DamiaoDispatcher broadcasts every master_id frame to all registered motor
callbacks; each motor callback verifies the frame belongs to it before acting.

Usage (multi-motor)::

    with DamiaoCANComms("can0") as bus:
        bus.start_listener()
        m1 = J4310_2EC(motor_id=1, comms=bus)
        m2 = J4310_2EC(motor_id=2, comms=bus)

Requires: python-can  (pip install python-can)
"""

from typing import Callable, List, Optional

import can
from can import BusState


class _SafeNotifier(can.Notifier):
    """can.Notifier subclass that routes receive-thread exceptions to a handler."""

    def __init__(self, bus, listeners, error_handler=None, **kwargs):
        self._error_handler = error_handler
        super().__init__(bus, listeners, **kwargs)

    def _rx_thread(self, bus):
        try:
            super()._rx_thread(bus)
        except Exception as exc:
            if self._error_handler is not None:
                self._error_handler(exc)
            else:
                print(f'[DamiaoCANComms] Bus error on {bus.channel_info}: {exc}', flush=True)


class _DamiaoDispatcher(can.Listener):
    """
    Broadcasts every reply frame (ID = master_id) to all registered motor callbacks.

    Motor callbacks are responsible for filtering frames that belong to them
    by inspecting D[0].
    """

    def __init__(self, master_id: int) -> None:
        self._master_id = master_id
        self._callbacks: List[Callable[[can.Message], None]] = []

    def register(self, callback: Callable[[can.Message], None]) -> None:
        self._callbacks.append(callback)

    def on_message_received(self, msg: can.Message) -> None:
        if msg.is_error_frame or msg.is_extended_id:
            return
        if msg.arbitration_id != self._master_id:
            return
        for cb in self._callbacks:
            cb(msg)


class DamiaoCANComms:
    """CAN bus wrapper for Damiao DM-J43xx motors (standard frames, 1 Mbps)."""

    def __init__(
        self,
        channel:    str   = "can0",
        bustype:    str   = "socketcan",
        bitrate:    int   = 1_000_000,
        master_id:  int   = 0,
        rx_timeout: float = 0.05,
        on_error:   Optional[Callable[[Exception], None]] = None,
    ):
        """
        Args:
            channel:    SocketCAN interface name, e.g. "can0".
            bustype:    python-can bus type (default "socketcan").
            bitrate:    CAN baud rate in bps (default 1 Mbps).
            master_id:  Host CAN ID; must match MST_ID register on each motor (default 0).
            rx_timeout: Seconds to wait for a parameter read reply.
            on_error:   Optional callback invoked when the Notifier thread catches
                        a bus exception.
        """
        self.master_id       = master_id
        self.rx_timeout      = rx_timeout
        self._error_callback = on_error
        self._bus = can.interface.Bus(
            channel=channel,
            bustype=bustype,
            bitrate=bitrate,
        )
        self._dispatcher = _DamiaoDispatcher(master_id)
        self._notifier:  Optional[can.Notifier] = None
        self._filter_set = False

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
        motor_id: int,
        callback: Callable[[can.Message], None],
    ) -> None:
        """
        Register a motor callback and install the receive filter for master_id.

        The filter is set once for the shared master_id; subsequent calls are
        idempotent.  All motor callbacks receive all frames arriving on master_id;
        each callback is responsible for verifying the frame is addressed to it.

        Args:
            motor_id: CAN ID of the motor (0–15 supported).
            callback: Invoked in the Notifier thread for every reply frame.
                      Motor instances pass ``_on_frame_received`` here.
        """
        self._dispatcher.register(callback)
        if not self._filter_set:
            filt = {"can_id": self.master_id, "can_mask": 0x7FF, "extended": False}
            self._bus.set_filters([filt])
            self._filter_set = True

    def clear_filters(self) -> None:
        """Remove all receive filters — accept every frame on the bus."""
        self._bus.set_filters(None)
        self._filter_set = False

    # ── Background listener ────────────────────────────────────────────────

    def _on_notifier_error(self, exc: Exception) -> None:
        if self._error_callback is not None:
            self._error_callback(exc)
        else:
            print(
                f'[DamiaoCANComms] Bus error on {self._bus.channel_info}: {exc}',
                flush=True,
            )

    def start_listener(self, extra_listeners: Optional[List[can.Listener]] = None) -> None:
        """
        Start a background Notifier thread that feeds _DamiaoDispatcher.
        Must be called before using any motor.
        """
        if self._notifier is not None:
            return
        all_listeners  = [self._dispatcher] + (extra_listeners or [])
        self._notifier = _SafeNotifier(
            self._bus, all_listeners, error_handler=self._on_notifier_error
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
        """Transmit a standard (11-bit) CAN frame with the given data bytes."""
        self._bus.send(can.Message(
            arbitration_id=arb_id,
            data=bytes(data),
            is_extended_id=False,
        ))
