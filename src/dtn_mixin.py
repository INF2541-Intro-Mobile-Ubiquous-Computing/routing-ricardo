import json
from typing import Dict

from gradysim.protocol.messages.communication import BroadcastMessageCommand

from utils.common.agent_types import AgentType
from utils.common.config import MAX_BUFFER_SIZE


class DTNMixin:
    """Bounded epidemic routing that retains messages between contacts."""

    routing_msg_type = "routing_dtn"

    def initialize(self) -> None:
        super().initialize()
        self._dtn_buffer: Dict[str, dict] = {}
        self._dtn_contacts = set()
        self._routing_sequence = 0

    def _make_dtn_message(self, payload: Dict[str, int]) -> dict:
        self._routing_sequence += 1
        return {
            "msg_type": self.routing_msg_type,
            "message_id": f"{self.provider.get_id()}:{self._routing_sequence}",
            "sender_id": self.provider.get_id(),
            "origin_id": self.provider.get_id(),
            "payload": dict(payload),
            "copies": 1,
        }

    def _store_dtn(self, message: dict) -> bool:
        message_id = message.get("message_id")
        if not message_id or message_id in self._dtn_buffer:
            return False
        if len(self._dtn_buffer) >= MAX_BUFFER_SIZE:
            self._dtn_buffer.pop(next(iter(self._dtn_buffer)))
        self._dtn_buffer[message_id] = dict(message)
        return True

    def _send_dtn(self, message: dict) -> None:
        self.provider.send_communication_command(
            BroadcastMessageCommand(json.dumps(message))
        )

    def _replicate_pending(self) -> None:
        for message in self._dtn_buffer.values():
            outgoing = dict(message)
            outgoing["sender_id"] = self.provider.get_id()
            outgoing["copies"] = int(outgoing.get("copies", 1)) + 1
            self._send_dtn(outgoing)

    def handle_packet(self, message: str) -> None:
        try:
            raw = json.loads(message)
        except json.JSONDecodeError:
            return super().handle_packet(message)

        if raw.get("sender_type") == AgentType.SENSOR.value:
            super().handle_packet(message)
            payload = raw.get("packets", {})
            if payload:
                self._store_dtn(self._make_dtn_message(payload))
            return

        if raw.get("msg_type") == self.routing_msg_type:
            if self._store_dtn(raw):
                self._send_dtn(raw)
            return

        if raw.get("sender_type") == AgentType.UAV.value:
            self._remember_contact(raw)
        return super().handle_packet(message)

    def _remember_contact(self, raw: dict) -> None:
        peer_id = raw.get("sender_id")
        if peer_id is not None and peer_id not in self._dtn_contacts:
            self._dtn_contacts.add(peer_id)
            self._replicate_pending()

    def handle_telemetry(self, telemetry) -> None:
        self._current_position = tuple(telemetry.current_position)
        super().handle_telemetry(telemetry)

    def finish(self) -> None:
        super().finish()
