"""
Flooding mixin for core nodes — TTL-limited broadcast routing.

Adapted from the original FloodingMixin to:
- Work with the dock-based message model (dest_face instead of ENDPOINT_GROUND).
- Override route_in_core() to flood dock_data messages within the core.
- Ignore raw sensor packets (satellites handle collection).
- Maintain neighbor table from core beacons for geocasting compatibility.
"""
import json
from typing import Dict, Set

from gradysim.protocol.messages.communication import BroadcastMessageCommand

from utils.common.agent_types import AgentType
from src.dock_mixin import MSG_DOCK_DATA


class FloodingMixin:
    """Broadcast each routing message once, subject to a strict TTL."""

    routing_msg_type = "routing_flooding"
    default_ttl = 8

    def initialize(self) -> None:
        if hasattr(super(), "initialize"):
            super().initialize()
        self._flood_seen: Set[str] = set()

    def _flood_message(self, msg: dict, ttl: int = None) -> None:
        """Flood a dock_data-style message through the core."""
        message_id = msg.get("id", "")
        if message_id in self._flood_seen:
            return
        flood_msg = {
            "msg_type": self.routing_msg_type,
            "message_id": message_id,
            "sender_id": self.provider.get_id(),
            "origin_face": msg.get("origin_face"),
            "dest_face": msg.get("dest_face"),
            "payload": msg.get("payload", {}),
            "id": message_id,
            "created_at": msg.get("created_at", 0.0),
            "hops": msg.get("hops", 1),
            "ttl": self.default_ttl if ttl is None else ttl,
        }
        self._flood_seen.add(message_id)
        self._broadcast_routing(flood_msg)

    def _broadcast_routing(self, message: dict) -> None:
        self.provider.send_communication_command(
            BroadcastMessageCommand(json.dumps(message))
        )

    def route_in_core(self, msg: dict) -> None:
        """Flood the message through the core on receipt of dock_data."""
        self._flood_message(msg)

    def handle_packet(self, message: str) -> None:
        try:
            raw = json.loads(message)
        except json.JSONDecodeError:
            return super().handle_packet(message)

        # Ignore raw sensor packets — satellites collect, core nodes don't
        if raw.get("sender_type") == AgentType.SENSOR.value:
            return

        if raw.get("msg_type") != self.routing_msg_type:
            return super().handle_packet(message)

        message_id = raw.get("message_id")
        ttl = int(raw.get("ttl", 0))
        if not message_id or message_id in self._flood_seen or ttl <= 0:
            return

        self._flood_seen.add(message_id)

        # Store in core buffer (so dock_deliver can be sent when aligned)
        if not hasattr(self, "_core_buffer"):
            self._core_buffer = {}
        if message_id not in self._core_buffer:
            entry = {
                "id": message_id,
                "origin_face": raw.get("origin_face"),
                "dest_face": raw.get("dest_face"),
                "payload": raw.get("payload", {}),
                "created_at": raw.get("created_at", 0.0),
                "hops": raw.get("hops", 1) + 1,
            }
            self._core_buffer[message_id] = entry

        # Re-flood with decremented TTL
        if ttl > 1:
            forwarded = dict(raw)
            forwarded["sender_id"] = self.provider.get_id()
            forwarded["ttl"] = ttl - 1
            forwarded["hops"] = raw.get("hops", 1) + 1
            self._broadcast_routing(forwarded)

    def handle_telemetry(self, telemetry) -> None:
        self._current_position = tuple(telemetry.current_position)
        super().handle_telemetry(telemetry)

    def finish(self) -> None:
        super().finish()
