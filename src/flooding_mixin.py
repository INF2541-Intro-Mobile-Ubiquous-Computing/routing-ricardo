import json
from typing import Dict, Set

from gradysim.protocol.messages.communication import BroadcastMessageCommand

from utils.common.agent_types import AgentType


class FloodingMixin:
    """Broadcast each routing message once, subject to a strict TTL."""

    routing_msg_type = "routing_flooding"
    default_ttl = 8

    def initialize(self) -> None:
        super().initialize()
        self._flood_seen: Set[str] = set()
        self._routing_sequence = 0

    def _flood_message(self, payload: Dict[str, int], ttl: int = None) -> None:
        self._routing_sequence += 1
        message_id = f"{self.provider.get_id()}:{self._routing_sequence}"
        message = {
            "msg_type": self.routing_msg_type,
            "message_id": message_id,
            "sender_id": self.provider.get_id(),
            "origin_id": self.provider.get_id(),
            "payload": dict(payload),
            "ttl": self.default_ttl if ttl is None else ttl,
        }
        self._flood_seen.add(message_id)
        self._broadcast_routing(message)

    def _broadcast_routing(self, message: dict) -> None:
        self.provider.send_communication_command(
            BroadcastMessageCommand(json.dumps(message))
        )

    def handle_packet(self, message: str) -> None:
        try:
            raw = json.loads(message)
        except json.JSONDecodeError:
            return super().handle_packet(message)

        if raw.get("sender_type") == AgentType.SENSOR.value:
            super().handle_packet(message)
            payload = raw.get("packets", {})
            if payload:
                self._flood_message(payload)
            return

        if raw.get("msg_type") != self.routing_msg_type:
            return super().handle_packet(message)

        message_id = raw.get("message_id")
        ttl = int(raw.get("ttl", 0))
        if not message_id or message_id in self._flood_seen or ttl <= 0:
            return
        self._flood_seen.add(message_id)
        forwarded = dict(raw)
        forwarded["sender_id"] = self.provider.get_id()
        forwarded["ttl"] = ttl - 1
        if forwarded["ttl"] > 0:
            self._broadcast_routing(forwarded)

    def handle_telemetry(self, telemetry) -> None:
        self._current_position = tuple(telemetry.current_position)
        super().handle_telemetry(telemetry)

    def finish(self) -> None:
        super().finish()
