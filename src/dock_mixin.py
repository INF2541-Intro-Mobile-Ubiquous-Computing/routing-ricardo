"""
CoreDockMixin — shared dock handshake logic for hub and rim nodes.

Responsibilities
----------------
- Answer ``dock_hello`` from a satellite if this node is currently aligned
  with that face's dock point.
- On receiving ``dock_data`` from a satellite, store the message in the
  core buffer and call ``route_in_core(msg)``.
- Provide ``_try_deliver(face)`` — deliver all buffered messages destined for
  ``face`` to the satellite currently docked there.
- Increment the hop counter on every radio transmission.
- Ignore ``dock_*`` messages addressed to other nodes (unicast filter).

This mixin must be placed *before* the base protocol in the MRO:

    class _Hub(CoreDockMixin, FloodingMixin, CoreHubProtocol): ...

It requires the host class to have:
    self._core_buffer       — Dict[str, dict]
    self._current_position  — (x, y, z)
    self._aligned_with(dock_2d) — bool
    self.provider
"""
import json
import logging
import math
from typing import Dict, Optional, Set

from gradysim.protocol.messages.communication import (
    BroadcastMessageCommand,
    SendMessageCommand,
)

from utils.common.config import (
    ALIGN_EPS,
    FACES,
    dock_point_of,
    opposite_face,
    EXCHANGE_TIMEOUT,
)

# Message type labels
MSG_DOCK_HELLO   = "dock_hello"
MSG_DOCK_ACK     = "dock_ack"
MSG_DOCK_DATA    = "dock_data"
MSG_DOCK_DELIVER = "dock_deliver"
BEACON_MSG_TYPE  = "core_beacon"

# ── Global registry of currently docked satellites ──────────────────────────
# face_name → satellite_id  (set when dock_data is received, cleared after delivery)
_DOCKED_SATS: Dict[str, Optional[int]] = {}


def get_docked_sat(face: str) -> Optional[int]:
    return _DOCKED_SATS.get(face)

def set_docked_sat(face: str, sat_id: Optional[int]) -> None:
    _DOCKED_SATS[face] = sat_id


class CoreDockMixin:
    """
    Mixin providing the dock handshake for all core nodes (hub and rim).
    Placed before the base protocol so handle_packet / handle_timer chain
    through correctly.
    """

    # ── Required from host ────────────────────────────────────────────────
    _core_buffer: Dict[str, dict]
    _current_position: tuple

    # _aligned_with() is provided by CoreHubProtocol / CoreRimProtocol via MRO

    # ── Packet handler ────────────────────────────────────────────────────

    def handle_packet(self, message: str) -> None:
        try:
            raw = json.loads(message)
        except json.JSONDecodeError:
            return super().handle_packet(message)

        msg_type = raw.get("msg_type")

        # ── dock_hello — satellite asking for alignment ────────────────
        if msg_type == MSG_DOCK_HELLO:
            face = raw.get("face")
            sat_id = raw.get("sender_id")
            if face is None or sat_id is None:
                return

            dp = dock_point_of(face)
            if self._aligned_with(dp):
                # Reply with unicast dock_ack
                ack = json.dumps({
                    "msg_type": MSG_DOCK_ACK,
                    "face": face,
                    "sender_id": self.provider.get_id(),
                })
                self.provider.send_communication_command(
                    SendMessageCommand(ack, sat_id)
                )
                logging.debug(
                    f"[CORE {self.provider.get_id()}] aligned with {face} dock, sent ack to sat {sat_id}"
                )
            # Others silently ignore
            return

        # ── dock_data — satellite uploading its payload ────────────────
        if msg_type == MSG_DOCK_DATA:
            target = raw.get("target_id")
            if target != self.provider.get_id():
                return  # not for us (unicast went to wrong node — shouldn't happen)

            origin_face = raw.get("origin_face")
            dest_face   = raw.get("dest_face")
            payload     = raw.get("payload", {})
            msg_id      = raw.get("id")
            sat_id      = raw.get("sender_id")
            created_at  = raw.get("created_at", 0.0)

            if not msg_id or not payload:
                # Nothing to forward; but still record dock and deliver buffered
                self._try_deliver(origin_face, sat_id)
                return

            # Record upload in metrics
            from utils.metrics.stats import record_upload
            record_upload(msg_id, origin_face, dest_face, created_at, hops=1)

            # Store in core buffer
            entry = dict(raw)
            entry["hops"] = 1
            self._core_buffer[msg_id] = entry

            logging.info(
                f"[CORE {self.provider.get_id()}] received dock_data from {origin_face} sat "
                f"(msg_id={msg_id}, dest={dest_face}, {len(payload)} sensors)"
            )

            # Try immediate delivery if we're already aligned with dest face
            dest_dp = dock_point_of(dest_face)
            if self._aligned_with(dest_dp):
                dest_sat = get_docked_sat(dest_face)
                if dest_sat is not None:
                    self._deliver_to_sat(dest_face, dest_sat, [entry])

            # Route through the core (strategy-specific)
            self.route_in_core(entry)

            # Deliver any buffered messages for the origin face (reverse channel)
            self._try_deliver(origin_face, sat_id)
            return

        # ── Unrecognised dock messages ─────────────────────────────────
        if msg_type in (MSG_DOCK_ACK, MSG_DOCK_DELIVER):
            # Core nodes shouldn't receive these (they send them)
            return

        super().handle_packet(message)

    # ── Telemetry tick — retry pending deliveries ─────────────────────────

    def handle_telemetry(self, telemetry) -> None:
        super().handle_telemetry(telemetry)
        # On each position update, try to deliver any buffered message whose
        # destination dock is now within ALIGN_EPS
        for face_entry in FACES:
            face_name = face_entry[0]
            dp = face_entry[3]  # 2-D dock point
            if self._aligned_with(dp):
                sat_id = get_docked_sat(face_name)
                if sat_id is not None:
                    # Find messages destined for this face
                    to_deliver = [
                        msg for msg in self._core_buffer.values()
                        if msg.get("dest_face") == face_name
                    ]
                    if to_deliver:
                        self._deliver_to_sat(face_name, sat_id, to_deliver)

    # ── Delivery helpers ──────────────────────────────────────────────────

    def _try_deliver(self, face: str, sat_id: int) -> None:
        """Deliver all buffered messages destined for *face* to sat_id."""
        to_deliver = [
            msg for msg in self._core_buffer.values()
            if msg.get("dest_face") == face
        ]
        if to_deliver:
            self._deliver_to_sat(face, sat_id, to_deliver)

        # Register this satellite as docked at its face
        set_docked_sat(face, sat_id)

    def _deliver_to_sat(self, face: str, sat_id: int, messages: list) -> None:
        """Send dock_deliver to sat_id for each message in *messages*."""
        for msg in messages:
            msg_id = msg.get("id")
            if not msg_id:
                continue
            hops = msg.get("hops", 1) + 1
            deliver_payload = json.dumps({
                "msg_type": MSG_DOCK_DELIVER,
                "id": msg_id,
                "origin_face": msg.get("origin_face"),
                "dest_face": face,
                "payload": msg.get("payload", {}),
                "hops": hops,
                "sender_id": self.provider.get_id(),
            })
            self.provider.send_communication_command(
                SendMessageCommand(deliver_payload, sat_id)
            )
            logging.info(
                f"[CORE {self.provider.get_id()}] delivered msg {msg_id} to {face} sat {sat_id} "
                f"(hops={hops})"
            )
            # Remove from buffer
            self._core_buffer.pop(msg_id, None)

    # ── route_in_core hook (overridden by strategy mixins) ────────────────

    def route_in_core(self, msg: dict) -> None:
        """Called when a dock_data message arrives.  Override in subclass."""
        pass

    # ── finish ────────────────────────────────────────────────────────────

    def finish(self) -> None:
        super().finish()
