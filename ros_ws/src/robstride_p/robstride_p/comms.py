"""
comms.py – CAN transport layer for RobStride motors.

Architecture
------------
Each RS02 motor that is instantiated calls add_motor_filter(), which:
  1. Installs a hardware receive filter so only frames whose bits 15–8
     match that motor's CAN ID reach the host application.
  2. Creates a dedicated can.BufferedReader queue inside _MotorDispatcher.

_MotorDispatcher (a can.Listener) is always the sole entry-point for
incoming frames.  It extracts the motor ID from the arbitration ID and
places the frame in the matching per-motor queue.  Frames for unknown
motor IDs are silently dropped.

When start_listener() is active, a can.Notifier feeds _MotorDispatcher
from a background thread.  recv_for_motor() then drains the per-motor
queue — completely non-blocking-friendly and thread-safe.

Without start_listener(), recv_for_motor() drives the bus synchronously:
it calls bus.recv() in a loop and routes each frame through the dispatcher
until the requested motor's queue is populated or the timeout expires.
Frames that arrive out of order are buffered for other motors rather than
discarded.

Usage (active-reporting, multi-motor)::

    with CANComms("can0") as bus:
        bus.start_listener()
        m1 = RS02(motor_id=1, comms=bus)
        m2 = RS02(motor_id=2, comms=bus)
        while True:
            f1 = m1.spin_once()   # only processes motor-1 frames
            f2 = m2.spin_once()   # only processes motor-2 frames

Requires: python-can  (pip install python-can)
"""

import time
from typing import Callable, Dict, List, Optional

import can
from can import BusState


class _MotorDispatcher(can.Listener):
    """
    Routes every valid incoming CAN frame to the correct per-motor queue.

    Motor ID is read from bits 15–8 of a 29-bit extended arbitration ID,
    which is where the RS02 private protocol places the source motor ID
    in all reply frames (type-2 feedback, type-17 param read, etc.).

    For 11-bit standard frames (MIT protocol), the motor ID sits in
    bits 7–0 instead.
    """

    def __init__(self, on_rx: Callable[[can.Message], None]) -> None:
        """
        Args:
            on_rx: Called once for every valid (non-error) frame received.
                   Used by CANComms to update bandwidth counters.
        """
        self._readers:   Dict[int, can.BufferedReader]       = {}
        self._callbacks: Dict[int, Callable[[can.Message], None]] = {}
        self._on_rx      = on_rx

    def register(
        self,
        motor_id: int,
        callback: Optional[Callable[[can.Message], None]] = None,
    ) -> None:
        """
        Create a dedicated BufferedReader queue for motor_id and optionally
        register a callback that is invoked immediately on every frame arrival.

        Args:
            motor_id: CAN ID of the motor.
            callback: Called with the raw ``can.Message`` each time a frame
                      for this motor is received, before it enters the queue.
                      Used by RS02 to keep its internal state current.
        """
        if motor_id not in self._readers:
            self._readers[motor_id] = can.BufferedReader()
        if callback is not None:
            self._callbacks[motor_id] = callback

    def get_reader(self, motor_id: int) -> Optional[can.BufferedReader]:
        return self._readers.get(motor_id)

    # can.Listener interface ──────────────────────────────────────────────────

    def on_message_received(self, msg: can.Message) -> None:
        if msg.is_error_frame:
            return

        self._on_rx(msg)

        motor_id = (
            (msg.arbitration_id >> 8) & 0xFF   # extended: bits 15–8
            if msg.is_extended_id else
            msg.arbitration_id & 0xFF           # standard: bits  7–0
        )

        callback = self._callbacks.get(motor_id)
        if callback is not None:
            callback(msg)

        reader = self._readers.get(motor_id)
        if reader is not None:
            reader.on_message_received(msg)


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
            bitrate:    Baud rate in bps. RS02 runs at 1 Mbps.
            rx_timeout: Default receive timeout passed to recv_for_motor()
                        when no explicit timeout is given.
        """
        self.rx_timeout = rx_timeout
        self._bitrate   = bitrate
        self._bus       = can.interface.Bus(
            channel=channel,
            bustype=bustype,
            bitrate=bitrate,
        )
        self._dispatcher = _MotorDispatcher(on_rx=self._track_rx)
        self._notifier:  Optional[can.Notifier] = None
        self._filters:   List[dict]             = []
        self._bw_reset()

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
        Register motor_id for per-motor routing and add a hardware filter.

        The hardware filter admits any extended frame whose bits 15–8 equal
        motor_id, covering all RS02 reply types (feedback, param-read, fault,
        etc.) without locking to a single comm-type.  A per-motor
        BufferedReader queue is created in the dispatcher.

        Args:
            motor_id: CAN ID of the motor to register.
            callback: Optional callable invoked immediately on every frame
                      arrival for this motor.  RS02 passes its own
                      ``_on_frame_received`` here to keep ``_feedback`` current.
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

        Once running, recv_for_motor() drains the per-motor BufferedReader queue
        rather than blocking on the bus directly.  This is the correct pattern
        for active-reporting mode where the motor pushes frames autonomously.

        Args:
            extra_listeners: Optional additional ``can.Listener`` instances,
                             e.g. ``can.Logger("session.asc")`` for recording.
        """
        if self._notifier is not None:
            return
        all_listeners  = [self._dispatcher] + (extra_listeners or [])
        self._notifier = can.Notifier(self._bus, all_listeners)

    def stop_listener(self) -> None:
        """Stop the Notifier thread.  recv_for_motor() reverts to sync mode."""
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
        self._tx_frames += 1
        self._tx_bits   += self._frame_bits(extended=True, data_len=len(payload))

    def send_standard(self, arb_id: int, data: bytes) -> None:
        """Transmit an 8-byte frame using an 11-bit standard arbitration ID."""
        payload = bytes(data).ljust(8, b"\x00")[:8]
        self._bus.send(can.Message(
            arbitration_id=arb_id,
            data=payload,
            is_extended_id=False,
        ))
        self._tx_frames += 1
        self._tx_bits   += self._frame_bits(extended=False, data_len=len(payload))

    # ── Receive ────────────────────────────────────────────────────────────────

    def recv_for_motor(
        self,
        motor_id: int,
        timeout:  Optional[float] = None,
    ) -> Optional[can.Message]:
        """
        Return the next frame destined for ``motor_id``, or ``None`` on timeout.

        **Async mode** (after ``start_listener()``):
            Drains the motor's dedicated BufferedReader queue.  The Notifier
            thread populates all queues concurrently; only this motor's frames
            are ever returned here, regardless of how many motors share the bus.

        **Sync mode** (no listener):
            Calls ``bus.recv()`` in a loop, routes each frame through the
            dispatcher (buffering frames for other motors in their own queues),
            and returns as soon as the requested motor's queue is non-empty.

        Args:
            motor_id: CAN ID of the motor whose data is requested.
            timeout:  Seconds to wait.  ``None`` uses ``self.rx_timeout``.
                      Pass ``0`` for a non-blocking poll.
        """
        t      = self.rx_timeout if timeout is None else timeout
        reader = self._dispatcher.get_reader(motor_id)
        if reader is None:
            return None

        if self._notifier is not None:
            return reader.get_message(timeout=t)

        # Sync path: drive the bus manually until the right motor responds.
        deadline = time.monotonic() + t
        while True:
            remaining = max(0.0, deadline - time.monotonic())
            raw = self._bus.recv(timeout=remaining)
            if raw is None:
                return None
            if not raw.is_error_frame:
                self._dispatcher.on_message_received(raw)
                msg = reader.get_message(timeout=0)
                if msg is not None:
                    return msg
            if time.monotonic() >= deadline:
                return None

    # ── Bandwidth ──────────────────────────────────────────────────────────────

    def bandwidth(self) -> None:
        """
        Print TX and RX bandwidth stats since the last call (or since init),
        then reset the counters.

        Bit counts include full CAN 2.0 frame overhead (ID, DLC, CRC, EOF,
        IFS, etc.) so the bus-load figure reflects actual wire utilisation.
        """
        elapsed = time.monotonic() - self._bw_t0
        if elapsed <= 0:
            print("CANComms bandwidth: no elapsed time yet")
            return

        tx_kbps  = self._tx_bits / elapsed / 1_000
        rx_kbps  = self._rx_bits / elapsed / 1_000
        bus_load = (self._tx_bits + self._rx_bits) / elapsed / self._bitrate * 100

        print(
            f"CANComms bandwidth ({elapsed:.2f} s window):\n"
            f"  TX  {self._tx_frames:>6} frames  {tx_kbps:>8.2f} kbps\n"
            f"  RX  {self._rx_frames:>6} frames  {rx_kbps:>8.2f} kbps\n"
            f"  Bus load: {bus_load:.1f}% of {self._bitrate // 1_000} kbps"
        )
        self._bw_reset()

    # ── Internal helpers ───────────────────────────────────────────────────────

    def _track_rx(self, msg: can.Message) -> None:
        """Bandwidth counter update called by the dispatcher for each valid frame."""
        self._rx_frames += 1
        self._rx_bits   += self._frame_bits(
            extended=msg.is_extended_id,
            data_len=len(msg.data),
        )

    def _bw_reset(self) -> None:
        self._bw_t0     = time.monotonic()
        self._tx_frames = 0
        self._rx_frames = 0
        self._tx_bits   = 0
        self._rx_bits   = 0

    @staticmethod
    def _frame_bits(extended: bool, data_len: int) -> int:
        """
        CAN 2.0 on-wire bit count (worst-case, excluding bit-stuffing).

        Standard (11-bit):  SOF(1) + ID(11) + RTR(1) + IDE(1) + r0(1)
                            + DLC(4) + data(n×8) + CRC(15) + CRCDEL(1)
                            + ACK(2) + EOF(7) + IFS(3)  →  47 + n×8
        Extended (29-bit):  SOF(1) + BaseID(11) + SRR(1) + IDE(1)
                            + ExtID(18) + RTR(1) + r1(1) + r0(1) + DLC(4)
                            + data(n×8) + CRC(15) + CRCDEL(1) + ACK(2)
                            + EOF(7) + IFS(3)            →  67 + n×8
        """
        overhead = 67 if extended else 47
        return overhead + data_len * 8
