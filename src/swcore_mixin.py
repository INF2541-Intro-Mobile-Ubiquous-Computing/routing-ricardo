"""
SWCore mixin for core nodes — event-driven routing table (paper, Algorithm 1).

- RT[Sx] is kept per *satellite*: ``direct`` (Sx is docked at this node), a
  next-hop core node, or ``unreachable`` (no entry counts as unreachable).
- ``connect(Sx)`` / ``disconnect(Sx)`` are triggered by the satellite's
  ``dock_connect`` / ``dock_disconnect`` and produce an RT-update that is
  flooded over the routing neighbours (the hub-and-spoke tree in the star
  topology, the radio neighbours otherwise).
- Every RT-update carries a version (origin time, origin id, counter); a node
  ignores updates that are not newer, which keeps the flood loop-free and
  processes updates in order.
- Messages for a satellite are forwarded along the table, delivered when the
  satellite is ``direct``, and otherwise kept in ``_core_buffer`` until a
  ``reachable`` update shows where to send them.
- The node where the sender is ``direct`` acknowledges each upload.

This mixin disables CoreDockMixin's delivery-on-alignment so that delivery only
happens through ``RT[Sy] == direct``.
"""
import json
import logging
from typing import Dict, List, Optional

from gradysim.protocol.messages.communication import SendMessageCommand
from utils.common.config import SWCORE_MAX_HOPS
from src.dock_mixin import (
    MSG_DOCK_CONNECT,
    MSG_DOCK_DISCONNECT,
    MSG_DOCK_MSG_ACK,
    get_sat,
)

DIRECT = "direct"
HOP = "hop"
UNREACHABLE = "unreachable"


class SWCoreMixin:
    routing_msg_type = "routing_swcore"
    rt_msg_type = "swcore_rt"
    deliver_on_alignment = False   # read by CoreDockMixin

    _rt: Dict[int, dict]

    def initialize(self) -> None:
        if hasattr(super(), "initialize"):
            super().initialize()
        self._rt = {}
        self._ver_counter = 0

    # ── Helpers ───────────────────────────────────────────────────────────

    def _new_ver(self) -> list:
        self._ver_counter += 1
        return [self.provider.current_time(), self.provider.get_id(), self._ver_counter]

    def _routing_neighbours(self) -> List[int]:
        """Hub-and-spoke tree in star (rim nodes only talk to the hub); all
        radio neighbours otherwise (hub, random topology)."""
        neighbours = self.fresh_neighbors()
        hubs = [nid for nid, data in neighbours.items() if data.get("role") == "hub"]
        return hubs if hubs else list(neighbours)

    def _send(self, msg: dict, target: int) -> None:
        msg["sender_id"] = self.provider.get_id()
        self.provider.send_communication_command(
            SendMessageCommand(json.dumps(msg), target)
        )

    def _send_rt_update(self, sat: int, state: str, ver: list, exclude: Optional[int] = None) -> None:
        for nid in self._routing_neighbours():
            if nid != exclude:
                self._send(
                    {"msg_type": self.rt_msg_type, "sat": sat, "state": state, "ver": ver},
                    nid,
                )

    def _buffered_for(self, sat: int) -> List[dict]:
        return [m for m in self._core_buffer.values() if m.get("dest_sat") == sat]

    def _forward(self, entry: dict, next_hop: int) -> bool:
        hops = entry.get("hops", 1) + 1
        if hops > SWCORE_MAX_HOPS:
            return False
        self._send({
            "msg_type": self.routing_msg_type,
            "id": entry["id"],
            "origin_face": entry.get("origin_face"),
            "dest_face": entry.get("dest_face"),
            "origin_sat": entry.get("origin_sat"),
            "dest_sat": entry.get("dest_sat"),
            "payload": entry.get("payload", {}),
            "created_at": entry.get("created_at", 0.0),
            "hops": hops,
            "next_hop": next_hop,
        }, next_hop)
        self._core_buffer.pop(entry["id"], None)  # custody moves to the next hop
        return True

    # ── Algorithm 1 ───────────────────────────────────────────────────────

    def _on_connect(self, sat: int, face: str) -> None:
        self._rt[sat] = {"state": DIRECT, "next_hop": None, "ver": self._new_ver()}
        logging.info(f"[SWCORE {self.provider.get_id()}] connect({sat}) RT[{sat}]=direct")
        self._send_rt_update(sat, "reachable", self._rt[sat]["ver"])
        pending = self._buffered_for(sat)
        if pending:
            self._deliver_to_sat(face, sat, pending)

    def _on_disconnect(self, sat: int) -> None:
        self._rt[sat] = {"state": UNREACHABLE, "next_hop": None, "ver": self._new_ver()}
        logging.info(f"[SWCORE {self.provider.get_id()}] disconnect({sat}) RT[{sat}]=unreachable")
        self._send_rt_update(sat, "unreachable", self._rt[sat]["ver"])

    def _on_rt_update(self, raw: dict) -> None:
        sat, sender, state = raw["sat"], raw["sender_id"], raw["state"]
        ver = list(raw["ver"])
        cur = self._rt.get(sat)
        if cur is not None and ver <= cur["ver"]:
            return  # stale or duplicate
        if state == "reachable":
            self._rt[sat] = {"state": HOP, "next_hop": sender, "ver": ver}
        else:
            self._rt[sat] = {"state": UNREACHABLE, "next_hop": None, "ver": ver}
        logging.info(
            f"[SWCORE {self.provider.get_id()}] RT[{sat}]={state} via {sender}"
        )
        self._send_rt_update(sat, state, ver, exclude=sender)
        if state == "reachable":
            for entry in self._buffered_for(sat):
                self._forward(entry, sender)

    def _route(self, entry: dict) -> None:
        """recv(message, Sx, Sy): deliver, forward or keep in the buffer."""
        sat = entry.get("dest_sat")
        row = self._rt.get(sat)
        if row is None or row["state"] == UNREACHABLE:
            return  # stays in _core_buffer
        if row["state"] == DIRECT:
            self._deliver_to_sat(entry["dest_face"], sat, [entry])
        elif row["next_hop"] in self.fresh_neighbors():
            self._forward(entry, row["next_hop"])

    def _ack_if_direct(self, entry: dict) -> None:
        origin = entry.get("origin_sat")
        row = self._rt.get(origin)
        if row is not None and row["state"] == DIRECT:
            self._send({"msg_type": MSG_DOCK_MSG_ACK, "id": entry["id"]}, origin)

    # ── Hooks ─────────────────────────────────────────────────────────────

    def on_dock_upload(self, msg: dict) -> None:
        """Called by CoreDockMixin when a satellite uploads dock_data.

        (Not ``route_in_core``: CoreDockMixin precedes the strategy mixins in the
        MRO, so its no-op ``route_in_core`` shadows any override.)"""
        sat = msg.get("sender_id")
        row = self._rt.get(sat)
        if row is None or row["state"] != DIRECT:
            self._on_connect(sat, msg["origin_face"])  # connect not seen yet
        msg["origin_sat"] = sat
        msg["dest_sat"] = get_sat(msg["dest_face"])
        self._route(msg)
        self._ack_if_direct(msg)

    def handle_packet(self, message: str) -> None:
        try:
            raw = json.loads(message)
        except json.JSONDecodeError:
            if hasattr(super(), "handle_packet"):
                super().handle_packet(message)
            return

        msg_type = raw.get("msg_type")

        if msg_type == MSG_DOCK_CONNECT:
            self._on_connect(raw["sender_id"], raw["face"])
            return
        if msg_type == MSG_DOCK_DISCONNECT:
            self._on_disconnect(raw["sender_id"])
            return
        if msg_type == self.rt_msg_type:
            self._on_rt_update(raw)
            return
        if msg_type == self.routing_msg_type:
            if raw.get("next_hop") != self.provider.get_id() or not raw.get("id"):
                return
            entry = {k: raw.get(k) for k in (
                "id", "origin_face", "dest_face", "origin_sat", "dest_sat",
                "payload", "created_at", "hops",
            )}
            self._core_buffer[entry["id"]] = entry
            self._route(entry)
            self._ack_if_direct(entry)
            return

        if hasattr(super(), "handle_packet"):
            super().handle_packet(message)

    def finish(self) -> None:
        if hasattr(super(), "finish"):
            super().finish()
