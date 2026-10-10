"""
NeighborMixin — shared neighbor tracking for core UAVs.

Listens to `core_beacon` messages broadcast by core nodes and tracks
active neighbors within `NEIGHBOR_TIMEOUT`.
"""
import json
import logging
from typing import Dict

from utils.common.config import NEIGHBOR_TIMEOUT
from src.dock_mixin import BEACON_MSG_TYPE

class NeighborMixin:
    _neighbors: Dict[int, dict]

    def initialize(self) -> None:
        self._neighbors = {}
        if hasattr(super(), "initialize"):
            super().initialize()

    def handle_packet(self, message: str) -> None:
        try:
            raw = json.loads(message)
        except json.JSONDecodeError:
            if hasattr(super(), "handle_packet"):
                super().handle_packet(message)
            return

        msg_type = raw.get("msg_type")
        if msg_type == BEACON_MSG_TYPE:
            sender_id = raw.get("sender_id")
            if sender_id is not None and sender_id != self.provider.get_id():
                self._neighbors[sender_id] = {
                    "position": raw.get("position"),
                    "role": raw.get("role"),
                    "last_seen": self.provider.current_time()
                }
            return # Consumed

        if hasattr(super(), "handle_packet"):
            super().handle_packet(message)

    def fresh_neighbors(self) -> Dict[int, dict]:
        """Returns neighbors that have been seen within NEIGHBOR_TIMEOUT."""
        current_time = self.provider.current_time()
        fresh = {}
        for nid, data in self._neighbors.items():
            if current_time - data["last_seen"] <= NEIGHBOR_TIMEOUT:
                fresh[nid] = data
        return fresh
