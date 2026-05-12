"""
comms.py – CAN transport layer for RobStride RS01–RS05 motors.

Architecture
------------
Each motor instance calls add_motor_filter(), which installs a hardware
receive filter and registers the motor's _on_frame_received callback with
_MotorDispatcher.

_MotorDispatcher (a can.Listener) is the sole entry-point for incoming
frames.  It extracts the motor ID from bits 15–8 of the 29-bit extended
arbitration ID and invokes the registered per-motor callback directly.
All frame parsing and state updates happen inside that callback — no
queues, no intermediate buffering.

start_listener() must be called once to start the can.Notifier background
thread that feeds _MotorDispatcher.

Usage (active-reporting, multi-motor)::

    with CANComms("can0") as bus:
        bus.start_listener()
        m1 = RS04(motor_id=1, comms=bus)
        m2 = RS04(motor_id=2, comms=bus)
        # _feedback on each motor is kept current automatically via callbacks

Requires: python-can  (pip install python-can)
"""

from typing import Callable, Dict, List, Optional

import can
from can import BusState


class _MotorDispatcher(can.Listener):
    """
    Routes every valid incoming CAN frame to the registered per-motor callback.

    Motor ID is read from bits 15–8 of a 29-bit extended arbitration ID,
    which is where the RS0x private protocol places the source motor ID
    in all reply frames (type-2 feedback, type-17 param read, etc.).

    For 11-bit standard frames (MIT protocol), the motor ID sits in
    bits 7–0 instead.
    """

    def __init__(self) -> None:
        self._callbacks: Dict[int, Callable[[can.Message], None]] = {}

    def register(
        self,
        motor_id: int,
        callback: Optional[Callable[[can.Message], None]] = None,
    ) -> None:
        """
        Register a callback for motor_id, invoked on every frame arrival.

        Args:
            motor_id: CAN ID of the motor.
            callback: Called with the raw ``can.Message`` each time a frame
                      for this motor is received.  Motor instances pass their
                      own ``_on_frame_received`` here, which handles all frame
                      types and updates internal state.
        """
        if callback is not None:
            self._callbacks[motor_id] = callback

    # can.Listener interface ──────────────────────────────────────────────────

    def on_message_received(self, msg: can.Message) -> None:
        if msg.is_error_frame:
            return

        motor_id = (
            (msg.arbitration_id >> 8) & 0xFF   # extended: bits 15–8
            if msg.is_extended_id else
            msg.arbitration_id & 0xFF           # standard: bits  7–0
        )

        callback = self._callbacks.get(motor_id)
        if callback is not None:
            callback(msg)


class CANComms:
    """Generic CAN bus wrapper used by RobStride motor drivers."""

    def __init__(
        self,
        channel:    str   = "can0",
        bustype:    str   = "socketcan",
        bitrate:    int   = 1_000_000,
        rx_timeout: float = 0.05,
    ):
        """
        Args:
            channel:    SocketCAN interface name, e.g. "can0".
            bustype:    python-can bus type (default "socketcan").
            bitrate:    Baud rate in bps. RS0x motors default to 1 Mbps.
            rx_timeout: Timeout (seconds) used by motor instances when waiting
                        for a response frame via threading.Event.
        """
        self.rx_timeout = rx_timeout
        self._bus       = can.interface.Bus(
            channel=channel,
            bustype=bustype,
            bitrate=bitrate,
        )
        self._dispatcher = _MotorDispatcher()
        self._notifier:  Optional[can.Notifier] = None
        self._filters:   List[dict]             = []

    # ── Context manager ────────────────────────────────────────────────────────

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()

    def close(self) -> None:
        """Stop the listener (if running) and release the CAN bus."""
        self.stop_listener()
        self._bus.shutdown()

    # ── Hardware filters ───────────────────────────────────────────────────────

    def add_motor_filter(
        self,
        motor_id: int,
        callback: Optional[Callable[[can.Message], None]] = None,
    ) -> None:
        """
        Register motor_id and add a hardware receive filter.

        The hardware filter admits any extended frame whose bits 15–8 equal
        motor_id, covering all RS0x reply types (feedback, param-read, fault,
        etc.) without locking to a single comm-type.

        Args:
            motor_id: CAN ID of the motor to register.
            callback: Invoked in the Notifier thread for every frame addressed
                      to this motor.  Motor instances pass ``_on_frame_received``
                      here — it handles all comm types and updates state directly.
        """
        can_id   = (motor_id & 0xFF) << 8
        can_mask = 0xFF << 8
        filt = {"can_id": can_id, "can_mask": can_mask, "extended": True}
        if filt not in self._filters:
            self._filters.append(filt)
            self._bus.set_filters(self._filters)

        self._dispatcher.register(motor_id, callback=callback)

    def clear_filters(self) -> None:
        """Remove all hardware receive filters — accept every frame on the bus."""
        self._filters.clear()
        self._bus.set_filters(None)

    # ── Background listener ────────────────────────────────────────────────────

    def start_listener(self, extra_listeners: Optional[List[can.Listener]] = None) -> None:
        """
        Start a ``can.Notifier`` background thread that feeds ``_MotorDispatcher``.

        Must be called before any motor is used.  All frame handling happens
        inside the per-motor callbacks invoked by the Notifier thread.

        Args:
            extra_listeners: Optional additional ``can.Listener`` instances,
                             e.g. ``can.Logger("session.asc")`` for recording.
        """
        if self._notifier is not None:
            return
        all_listeners  = [self._dispatcher] + (extra_listeners or [])
        self._notifier = can.Notifier(self._bus, all_listeners)

    def stop_listener(self) -> None:
        """Stop the Notifier thread; no further callbacks will fire."""
        if self._notifier is not None:
            self._notifier.stop()
            self._notifier = None

    # ── Bus state ──────────────────────────────────────────────────────────────

    @property
    def state(self) -> BusState:
        """Current bus state: ``BusState.ACTIVE``, ``PASSIVE``, or ``ERROR``."""
        return self._bus.state

    # ── Transmit ───────────────────────────────────────────────────────────────

    def send_extended(self, arb_id: int, data: bytes) -> None:
        """Transmit an 8-byte frame using a 29-bit extended arbitration ID."""
        payload = bytes(data).ljust(8, b"\x00")[:8]
        self._bus.send(can.Message(
            arbitration_id=arb_id,
            data=payload,
            is_extended_id=True,
        ))

    def send_standard(self, arb_id: int, data: bytes) -> None:
        """Transmit an 8-byte frame using an 11-bit standard arbitration ID."""
        payload = bytes(data).ljust(8, b"\x00")[:8]
        self._bus.send(can.Message(
            arbitration_id=arb_id,
            data=payload,
            is_extended_id=False,
        ))

