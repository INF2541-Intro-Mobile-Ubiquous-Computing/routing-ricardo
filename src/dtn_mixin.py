"""
DTN mixin for core nodes — bounded epidemic routing.

Adapted from the original DTNMixin to:
- Work with the dock-based message model (dest_face).
- Override route_in_core() to store and rebroadcast dock_data messages.
- Ignore raw sensor packets (satellites handle collection).
"""
import json
from typing import Dict

from gradysim.protocol.messages.communication import BroadcastMessageCommand

from utils.common.agent_types import AgentType
from utils.common.config import MAX_BUFFER_SIZE
from src.dock_mixin import MSG_DOCK_DATA


class DTNMixin:
    """Bounded epidemic routing that retains messages between contacts."""

    routing_msg_type = "routing_dtn"

    def initialize(self) -> None:
        super().initialize()
        self._dtn_buffer: Dict[str, dict] = {}
        self._dtn_contacts = set()
        self._routing_sequence = 0

    def _store_dtn(self, message: dict) -> bool:
        message_id = message.get("id") or message.get("message_id")
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
        for message in list(self._dtn_buffer.values()):
            outgoing = dict(message)
            outgoing["msg_type"] = self.routing_msg_type
            outgoing["sender_id"] = self.provider.get_id()
            outgoing["copies"] = int(outgoing.get("copies", 1)) + 1
            outgoing["hops"] = int(outgoing.get("hops", 1)) + 1
            self._send_dtn(outgoing)

    def route_in_core(self, msg: dict) -> None:
        """Store and broadcast dock_data on receipt."""
        wrapped = dict(msg)
        wrapped["msg_type"] = self.routing_msg_type
        wrapped["message_id"] = msg.get("id", "")
        if self._store_dtn(wrapped):
            self._send_dtn(wrapped)

    def handle_packet(self, message: str) -> None:
        try:
            raw = json.loads(message)
        except json.JSONDecodeError:
            return super().handle_packet(message)

        # Ignore raw sensor packets
        if raw.get("sender_type") == AgentType.SENSOR.value:
            return

        if raw.get("msg_type") == self.routing_msg_type:
            message_id = raw.get("id") or raw.get("message_id")
            if not message_id:
                return super().handle_packet(message)

            if self._store_dtn(raw):
                # Store in _core_buffer too for dock delivery
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
                # Epidemic rebroadcast
                outgoing = dict(raw)
                outgoing["sender_id"] = self.provider.get_id()
                outgoing["copies"] = int(raw.get("copies", 1)) + 1
                outgoing["hops"] = int(raw.get("hops", 1)) + 1
                self._send_dtn(outgoing)
            return

        return super().handle_packet(message)

    def handle_telemetry(self, telemetry) -> None:
        self._current_position = tuple(telemetry.current_position)
        super().handle_telemetry(telemetry)

    def finish(self) -> None:
        super().finish()
