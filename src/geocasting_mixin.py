"""
Geocasting mixin for core nodes — greedy geographic forwarding.

Adapted from the original GeocastingMixin to:
- Work with the dock-based message model (dest_face → dock point coordinates).
- Override route_in_core() to greedily forward toward the destination dock point.
- Ignore raw sensor packets (satellites handle collection).
- Use core beacons to maintain neighbor positions.
"""
import json
import math
from typing import Dict

from gradysim.protocol.messages.communication import SendMessageCommand, BroadcastMessageCommand

from utils.common.agent_types import AgentType
from utils.common.config import MAX_BUFFER_SIZE, ALIGN_EPS, dock_point_of
from src.dock_mixin import MSG_DOCK_DATA, BEACON_MSG_TYPE


class GeocastingMixin:
    """Greedy geographic forwarding toward the destination dock point."""

    routing_msg_type = "routing_geocasting"

    def initialize(self) -> None:
        super().initialize()
        self._geo_buffer: Dict[str, dict] = {}
        self._geo_seen = set()
        self._geo_neighbors: Dict[int, tuple] = {}
        self._routing_sequence = 0
        self._current_position = (0.0, 0.0, 0.0)

    @staticmethod
    def _distance_2d(first, second) -> float:
        return math.sqrt((first[0] - second[0]) ** 2 + (first[1] - second[1]) ** 2)

    def _best_neighbor_for(self, dest_2d: tuple):
        """Return the neighbor peer_id closest to dest_2d, if closer than self."""
        current_dist = self._distance_2d(self._current_position, dest_2d)
        candidates = [
            (peer_id, pos)
            for peer_id, pos in self._geo_neighbors.items()
            if self._distance_2d(pos, dest_2d) < current_dist
        ]
        if not candidates:
            return None
        return min(candidates, key=lambda item: self._distance_2d(item[1], dest_2d))[0]

    def _forward_geo(self, msg: dict) -> bool:
        """Try to forward msg toward its dest_face dock point. Return True if forwarded."""
        dest_face = msg.get("dest_face")
        if not dest_face:
            return False
        dest_2d = dock_point_of(dest_face)

        # If we are within ALIGN_EPS of the destination dock, we are the deliverer
        if self._distance_2d(self._current_position, dest_2d) <= ALIGN_EPS:
            # Mark deliverable — CoreDockMixin._try_deliver will handle actual delivery
            self._geo_buffer.pop(msg.get("id"), None)
            return True

        peer_id = self._best_neighbor_for(dest_2d)
        if peer_id is None:
            self._store_geo(msg)
            return False

        outgoing = dict(msg)
        outgoing["msg_type"] = self.routing_msg_type
        outgoing["sender_id"] = self.provider.get_id()
        outgoing["hops"] = msg.get("hops", 1) + 1
        self.provider.send_communication_command(
            SendMessageCommand(json.dumps(outgoing), peer_id)
        )
        self._geo_buffer.pop(msg.get("id"), None)
        return True

    def _store_geo(self, msg: dict) -> None:
        msg_id = msg.get("id")
        if not msg_id or msg_id in self._geo_buffer:
            return
        if len(self._geo_buffer) >= MAX_BUFFER_SIZE:
            self._geo_buffer.pop(next(iter(self._geo_buffer)))
        self._geo_buffer[msg_id] = dict(msg)

    def _retry_geo(self) -> None:
        for msg in list(self._geo_buffer.values()):
            self._forward_geo(msg)

    def route_in_core(self, msg: dict) -> None:
        """Forward greedily toward dest_face dock point on receipt of dock_data."""
        self._forward_geo(msg)

    def handle_packet(self, message: str) -> None:
        try:
            raw = json.loads(message)
        except json.JSONDecodeError:
            return super().handle_packet(message)

        # Ignore raw sensor packets
        if raw.get("sender_type") == AgentType.SENSOR.value:
            return

        msg_type = raw.get("msg_type")

        if msg_type == self.routing_msg_type:
            msg_id = raw.get("id") or raw.get("message_id")
            if not msg_id or msg_id in self._geo_seen:
                return
            self._geo_seen.add(msg_id)

            # Store in _core_buffer too
            if not hasattr(self, "_core_buffer"):
                self._core_buffer = {}
            if msg_id not in self._core_buffer:
                entry = {
                    "id": msg_id,
                    "origin_face": raw.get("origin_face"),
                    "dest_face": raw.get("dest_face"),
                    "payload": raw.get("payload", {}),
                    "created_at": raw.get("created_at", 0.0),
                    "hops": raw.get("hops", 1),
                }
                self._core_buffer[msg_id] = entry

            self._forward_geo(raw)
            return

        if msg_type == BEACON_MSG_TYPE:
            peer_id = raw.get("sender_id")
            position = raw.get("position")
            if peer_id is not None and position is not None:
                self._geo_neighbors[peer_id] = tuple(position)
                self._retry_geo()
            return super().handle_packet(message)

        return super().handle_packet(message)

    def handle_telemetry(self, telemetry) -> None:
        self._current_position = tuple(telemetry.current_position)
        super().handle_telemetry(telemetry)
        self._retry_geo()

    def finish(self) -> None:
        super().finish()
