"""
Geocasting mixin for core nodes — GPSR + Region Flooding.

- Uses NeighborMixin for fresh neighbor positions.
- In-region: nodes inside GEOCAST_REGION_RADIUS flood the message.
- Out-of-region: GPSR greedy forwarding, falling back to perimeter routing
  on a planarised graph (RNG) using the right-hand rule.
"""
import json
import math
from typing import Dict, List, Tuple

from gradysim.protocol.messages.communication import SendMessageCommand, BroadcastMessageCommand
from utils.common.config import GEOCAST_REGION_RADIUS, dock_point_of
from src.neighbor_mixin import NeighborMixin

class GeocastingMixin:
    _geo_seen: set
    routing_msg_type = "routing_geocasting"

    def initialize(self) -> None:
        if hasattr(super(), "initialize"):
            super().initialize()
        self._geo_seen = set()
        self._current_position = (0.0, 0.0, 0.0)

    @staticmethod
    def _distance_2d(p1: tuple, p2: tuple) -> float:
        return math.hypot(p1[0] - p2[0], p1[1] - p2[1])

    def _in_region(self, dest_2d: tuple) -> bool:
        return self._distance_2d(self._current_position, dest_2d) <= GEOCAST_REGION_RADIUS

    def _rng_neighbors(self) -> Dict[int, tuple]:
        if not hasattr(self, "fresh_neighbors"):
            return {}
        neighbors = {nid: data["position"] for nid, data in self.fresh_neighbors().items()}
        my_pos = self._current_position
        
        rng = {}
        for v_id, v_pos in neighbors.items():
            d_uv = self._distance_2d(my_pos, v_pos)
            defeated = False
            for w_id, w_pos in neighbors.items():
                if w_id == v_id: continue
                d_uw = self._distance_2d(my_pos, w_pos)
                d_vw = self._distance_2d(v_pos, w_pos)
                if d_uw < d_uv and d_vw < d_uv:
                    defeated = True
                    break
            if not defeated:
                rng[v_id] = v_pos
        return rng

    def _ccw_angle_between(self, ref_vec: tuple, tgt_vec: tuple) -> float:
        a1 = math.atan2(ref_vec[1], ref_vec[0])
        a2 = math.atan2(tgt_vec[1], tgt_vec[0])
        diff = a2 - a1
        if diff < 0:
            diff += 2 * math.pi
        return diff

    def _right_hand_next_hop(self, back_vec: tuple, rng_neighbors: Dict[int, tuple]) -> int:
        best_nid = None
        min_angle = float('inf')
        my_pos = self._current_position
        for nid, pos in rng_neighbors.items():
            vec = (pos[0] - my_pos[0], pos[1] - my_pos[1])
            angle = self._ccw_angle_between(back_vec, vec)
            if angle == 0 and len(rng_neighbors) > 1:
                angle = 2 * math.pi
            if angle < min_angle:
                min_angle = angle
                best_nid = nid
        return best_nid

    def _send_geo_unicast(self, msg: dict, target_id: int):
        outgoing = dict(msg)
        outgoing["msg_type"] = self.routing_msg_type
        outgoing["sender_id"] = self.provider.get_id()
        outgoing["next_hop"] = target_id
        outgoing["hops"] = outgoing.get("hops", 1) + 1
        self.provider.send_communication_command(
            SendMessageCommand(json.dumps(outgoing), target_id)
        )

    def _send_geo_broadcast(self, msg: dict):
        outgoing = dict(msg)
        outgoing["msg_type"] = self.routing_msg_type
        outgoing["sender_id"] = self.provider.get_id()
        outgoing.pop("next_hop", None)
        outgoing["hops"] = outgoing.get("hops", 1) + 1
        self.provider.send_communication_command(
            BroadcastMessageCommand(json.dumps(outgoing))
        )

    def _forward_gpsr(self, msg: dict, dest_2d: tuple) -> None:
        my_pos = self._current_position
        mode = msg.get("gpsr_mode", "greedy")
        
        if not hasattr(self, "fresh_neighbors"):
            return
        neighbors = {nid: data["position"] for nid, data in self.fresh_neighbors().items()}
        
        if not neighbors:
            return 
            
        d_self = self._distance_2d(my_pos, dest_2d)
        
        if mode == "greedy":
            best_nid = None
            best_d = d_self
            for nid, pos in neighbors.items():
                d = self._distance_2d(pos, dest_2d)
                if d < best_d:
                    best_d = d
                    best_nid = nid
            
            if best_nid is not None:
                self._send_geo_unicast(msg, best_nid)
            else:
                msg["gpsr_mode"] = "perimeter"
                msg["perimeter_start"] = my_pos
                
                rng = self._rng_neighbors()
                if not rng: return
                
                back_vec = (dest_2d[0] - my_pos[0], dest_2d[1] - my_pos[1])
                next_hop = self._right_hand_next_hop(back_vec, rng)
                
                if next_hop is not None:
                    msg["prev_pos"] = my_pos
                    self._send_geo_unicast(msg, next_hop)
        else: # perimeter
            p_start = msg.get("perimeter_start")
            if p_start and d_self < self._distance_2d(p_start, dest_2d):
                msg["gpsr_mode"] = "greedy"
                msg.pop("perimeter_start", None)
                msg.pop("prev_pos", None)
                self._forward_gpsr(msg, dest_2d)
                return
                
            rng = self._rng_neighbors()
            if not rng: return
            
            prev_pos = msg.get("prev_pos", my_pos)
            back_vec = (prev_pos[0] - my_pos[0], prev_pos[1] - my_pos[1])
            if back_vec == (0, 0):
                back_vec = (dest_2d[0] - my_pos[0], dest_2d[1] - my_pos[1])
                
            next_hop = self._right_hand_next_hop(back_vec, rng)
            if next_hop is not None:
                msg["prev_pos"] = my_pos
                self._send_geo_unicast(msg, next_hop)

    def route_in_core(self, msg: dict) -> None:
        msg_id = msg.get("id")
        if not msg_id or msg_id in self._geo_seen:
            return
        self._geo_seen.add(msg_id)
        
        dest_face = msg.get("dest_face")
        if not dest_face: return
        dest_2d = dock_point_of(dest_face)
        
        if self._in_region(dest_2d):
            self._send_geo_broadcast(msg)
        else:
            self._forward_gpsr(msg, dest_2d)

    def handle_packet(self, message: str) -> None:
        try:
            raw = json.loads(message)
        except json.JSONDecodeError:
            if hasattr(super(), "handle_packet"):
                super().handle_packet(message)
            return

        msg_type = raw.get("msg_type")
        if msg_type != self.routing_msg_type:
            if hasattr(super(), "handle_packet"):
                super().handle_packet(message)
            return

        msg_id = raw.get("id")
        if not msg_id or msg_id in self._geo_seen:
            return
            
        self._geo_seen.add(msg_id)

        # Store in _core_buffer for dock delivery
        if not hasattr(self, "_core_buffer"):
            self._core_buffer = {}
        if msg_id not in self._core_buffer:
            self._core_buffer[msg_id] = {
                "id": msg_id,
                "origin_face": raw.get("origin_face"),
                "dest_face": raw.get("dest_face"),
                "payload": raw.get("payload", {}),
                "created_at": raw.get("created_at", 0.0),
                "hops": raw.get("hops", 1) + 1,
            }

        dest_face = raw.get("dest_face")
        if not dest_face: return
        dest_2d = dock_point_of(dest_face)

        if self._in_region(dest_2d):
            self._send_geo_broadcast(raw)
            return

        next_hop = raw.get("next_hop")
        if next_hop == self.provider.get_id():
            self._forward_gpsr(raw, dest_2d)

    def handle_telemetry(self, telemetry) -> None:
        self._current_position = tuple(telemetry.current_position)
        if hasattr(super(), "handle_telemetry"):
            super().handle_telemetry(telemetry)

    def finish(self) -> None:
        if hasattr(super(), "finish"):
            super().finish()
