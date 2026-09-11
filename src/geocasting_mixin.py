import json
import math
from typing import Dict

from gradysim.protocol.messages.communication import SendMessageCommand

from utils.common.agent_types import AgentType
from utils.common.config import ENDPOINT_GROUND, MAX_BUFFER_SIZE


class GeocastingMixin:
    """Greedy geographic forwarding with bounded Store-and-Wait buffering."""

    routing_msg_type = "routing_geocasting"
    destination = tuple(ENDPOINT_GROUND)
    destination_radius = 5.0

    def initialize(self) -> None:
        super().initialize()
        self._geo_buffer: Dict[str, dict] = {}
        self._geo_seen = set()
        self._geo_neighbors: Dict[int, tuple] = {}
        self._routing_sequence = 0
        self._current_position = tuple(self._waypoints[0])

    @staticmethod
    def _distance(first, second) -> float:
        return math.sqrt(sum((a - b) ** 2 for a, b in zip(first[:2], second[:2])))

    def _make_geo_message(self, payload: Dict[str, int]) -> dict:
        self._routing_sequence += 1
        message = {
            "msg_type": self.routing_msg_type,
            "message_id": f"{self.provider.get_id()}:{self._routing_sequence}",
            "sender_id": self.provider.get_id(),
            "origin_id": self.provider.get_id(),
            "payload": dict(payload),
            "destination": self.destination,
            "radius": self.destination_radius,
            "waiting": False,
        }
        self._geo_seen.add(message["message_id"])
        return message

    def _store_geo(self, message: dict) -> None:
        message_id = message.get("message_id")
        if not message_id or message_id in self._geo_buffer:
            return
        if len(self._geo_buffer) >= MAX_BUFFER_SIZE:
            self._geo_buffer.pop(next(iter(self._geo_buffer)))
        waiting = dict(message)
        waiting["waiting"] = True
        self._geo_buffer[message_id] = waiting

    def _best_neighbor(self):
        current_distance = self._distance(self._current_position, self.destination)
        candidates = [
            (peer_id, position)
            for peer_id, position in self._geo_neighbors.items()
            if self._distance(position, self.destination) < current_distance
        ]
        if not candidates:
            return None
        return min(candidates, key=lambda item: self._distance(item[1], self.destination))[0]

    def _forward_geo(self, message: dict) -> bool:
        if self._distance(self._current_position, self.destination) <= float(message.get("radius", self.destination_radius)):
            self._geo_buffer.pop(message.get("message_id"), None)
            return True
        peer_id = self._best_neighbor()
        if peer_id is None:
            self._store_geo(message)
            return False
        outgoing = dict(message)
        outgoing["sender_id"] = self.provider.get_id()
        outgoing["waiting"] = False
        self.provider.send_communication_command(
            SendMessageCommand(json.dumps(outgoing), peer_id)
        )
        self._geo_buffer.pop(message.get("message_id"), None)
        return True

    def _retry_geo(self) -> None:
        for message in list(self._geo_buffer.values()):
            self._forward_geo(message)

    def handle_packet(self, message: str) -> None:
        try:
            raw = json.loads(message)
        except json.JSONDecodeError:
            return super().handle_packet(message)

        if raw.get("sender_type") == AgentType.SENSOR.value:
            super().handle_packet(message)
            payload = raw.get("packets", {})
            if payload:
                self._forward_geo(self._make_geo_message(payload))
            return

        if raw.get("msg_type") == self.routing_msg_type:
            message_id = raw.get("message_id")
            if not message_id or message_id in self._geo_seen:
                return
            self._geo_seen.add(message_id)
            self._forward_geo(raw)
            return

        if raw.get("sender_type") == AgentType.UAV.value:
            peer_id = raw.get("sender_id")
            position = raw.get("position")
            if peer_id is not None and position is not None:
                self._geo_neighbors[peer_id] = tuple(position)
                self._retry_geo()
        return super().handle_packet(message)

    def handle_telemetry(self, telemetry) -> None:
        self._current_position = tuple(telemetry.current_position)
        super().handle_telemetry(telemetry)
        self._retry_geo()

    def finish(self) -> None:
        super().finish()
