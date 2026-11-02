"""
SWCore mixin for core nodes — dynamic distance-vector routing table (DSDV-like).

- Each core node keeps a routing table with one row per dock face:
      face -> {next_hop, hops, seq, updated}
- A node aligned with a face's dock point originates a 0-hop route to it.
- Rows are advertised periodically (``swcore_adv``) and relayed hop by hop.
  The sequence number is the originator's simulation time, so fresher
  information always wins and equal-freshness ties go to the fewer hops.
- Rows expire if not refreshed, or when their next hop leaves the neighbor table.
- ``route_in_core`` unicasts the message along the table; without a route the
  message waits in ``_core_buffer`` and is retried on every advertisement tick
  (physical docking delivery by CoreDockMixin keeps working as the fallback).
"""
import json
import logging
from typing import Dict, Optional

from gradysim.protocol.messages.communication import SendMessageCommand, BroadcastMessageCommand
from utils.common.config import (
    FACES,
    SWCORE_ADV_INTERVAL,
    SWCORE_MAX_HOPS,
    SWCORE_ROUTE_TIMEOUT,
)

ADV_TIMER = "swcore_adv"


class SWCoreMixin:
    _rt: Dict[str, dict]
    routing_msg_type = "routing_swcore"
    adv_msg_type = "swcore_adv"

    def initialize(self) -> None:
        if hasattr(super(), "initialize"):
            super().initialize()
        self._rt = {}
        self.provider.schedule_timer(
            ADV_TIMER, self.provider.current_time() + SWCORE_ADV_INTERVAL
        )

    # ── Routing table ─────────────────────────────────────────────────────

    def _valid_route(self, face: str) -> Optional[dict]:
        """Row for *face* if it is fresh and its next hop is still a neighbor."""
        row = self._rt.get(face)
        if row is None:
            return None
        if row["next_hop"] == self.provider.get_id():
            return row
        if row["next_hop"] not in self.fresh_neighbors():
            return None
        return row

    def _refresh_table(self) -> None:
        """Expire stale rows and (re)originate routes for faces we are docked at."""
        now = self.provider.current_time()
        me = self.provider.get_id()

        for face, row in list(self._rt.items()):
            if now - row["updated"] > SWCORE_ROUTE_TIMEOUT:
                del self._rt[face]
                logging.info(f"[SWCORE {me}] route to {face} expired")

        for face_entry in FACES:
            face, dock_pt = face_entry[0], face_entry[3]
            row = self._rt.get(face)
            if self._aligned_with(dock_pt):
                if row is None or row["next_hop"] != me:
                    logging.info(f"[SWCORE {me}] originating route to {face}")
                self._rt[face] = {"next_hop": me, "hops": 0, "seq": now, "updated": now}
            elif row is not None and row["next_hop"] == me:
                del self._rt[face]  # no longer aligned: stop claiming the dock

    def _advertise(self) -> None:
        vector = {
            face: {"hops": row["hops"], "seq": row["seq"], "next_hop": row["next_hop"]}
            for face, row in self._rt.items()
        }
        if not vector:
            return
        msg = json.dumps({
            "msg_type": self.adv_msg_type,
            "sender_id": self.provider.get_id(),
            "vector": vector,
        })
        self.provider.send_communication_command(BroadcastMessageCommand(msg))

    def _handle_adv(self, raw: dict) -> None:
        sender = raw.get("sender_id")
        me = self.provider.get_id()
        if sender is None or sender == me:
            return
        now = self.provider.current_time()
        for face, adv in raw.get("vector", {}).items():
            if adv.get("next_hop") == me:
                continue  # poison reverse: sender routes through us
            hops = adv["hops"] + 1
            seq = adv["seq"]
            cur = self._rt.get(face)
            if cur is None or seq > cur["seq"] or (seq == cur["seq"] and hops < cur["hops"]):
                if cur is None or cur["next_hop"] != sender:
                    logging.info(
                        f"[SWCORE {me}] route to {face} via {sender} ({hops} hops)"
                    )
                self._rt[face] = {"next_hop": sender, "hops": hops, "seq": seq, "updated": now}

    # ── Forwarding ────────────────────────────────────────────────────────

    def _forward(self, msg: dict) -> bool:
        """Unicast *msg* to the next hop toward its dest face. True if sent."""
        dest_face = msg.get("dest_face")
        row = self._valid_route(dest_face) if dest_face else None
        if row is None or row["next_hop"] == self.provider.get_id():
            return False
        hops = msg.get("hops", 1) + 1
        if hops > SWCORE_MAX_HOPS:
            return False
        outgoing = {
            "msg_type": self.routing_msg_type,
            "id": msg["id"],
            "origin_face": msg.get("origin_face"),
            "dest_face": dest_face,
            "payload": msg.get("payload", {}),
            "created_at": msg.get("created_at", 0.0),
            "hops": hops,
            "sender_id": self.provider.get_id(),
            "next_hop": row["next_hop"],
        }
        self.provider.send_communication_command(
            SendMessageCommand(json.dumps(outgoing), row["next_hop"])
        )
        self._core_buffer.pop(msg["id"], None)  # custody moves to the next hop
        return True

    def _retry_buffered(self) -> None:
        for msg in list(self._core_buffer.values()):
            if msg.get("id") and msg.get("dest_face"):
                self._forward(msg)

    def route_in_core(self, msg: dict) -> None:
        # Already delivered by an immediate dock delivery, nothing left to route
        if not msg.get("id") or msg["id"] not in self._core_buffer:
            return
        self._refresh_table()
        self._forward(msg)

    # ── Hooks ─────────────────────────────────────────────────────────────

    def handle_timer(self, timer: str) -> None:
        if timer == ADV_TIMER:
            self._refresh_table()
            self._advertise()
            self._retry_buffered()
            self.provider.schedule_timer(
                ADV_TIMER, self.provider.current_time() + SWCORE_ADV_INTERVAL
            )
            return
        if hasattr(super(), "handle_timer"):
            super().handle_timer(timer)

    def handle_packet(self, message: str) -> None:
        try:
            raw = json.loads(message)
        except json.JSONDecodeError:
            if hasattr(super(), "handle_packet"):
                super().handle_packet(message)
            return

        msg_type = raw.get("msg_type")

        if msg_type == self.adv_msg_type:
            self._handle_adv(raw)
            return

        if msg_type == self.routing_msg_type:
            if raw.get("next_hop") != self.provider.get_id():
                return
            msg_id = raw.get("id")
            if not msg_id:
                return
            entry = {
                "id": msg_id,
                "origin_face": raw.get("origin_face"),
                "dest_face": raw.get("dest_face"),
                "payload": raw.get("payload", {}),
                "created_at": raw.get("created_at", 0.0),
                "hops": raw.get("hops", 1),
            }
            self._core_buffer[msg_id] = entry
            self._refresh_table()
            self._forward(entry)
            return

        if hasattr(super(), "handle_packet"):
            super().handle_packet(message)

    def finish(self) -> None:
        if hasattr(super(), "finish"):
            super().finish()
