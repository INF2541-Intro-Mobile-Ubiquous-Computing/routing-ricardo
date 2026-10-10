"""
DTN mixin for core nodes — Epidemic Routing (Vahdat & Becker, 2000).

- Nodes maintain a bounded buffer (_core_buffer).
- When encountering a new peer (via beacon), the lower-ID node initiates
  an anti-entropy session by sending its summary vector.
- Peers exchange missing messages.
- Messages that reach DTN_MAX_HOPS are excluded from routing.
"""
import json
import logging
from typing import Dict, List

from gradysim.protocol.messages.communication import SendMessageCommand
from utils.common.config import MAX_BUFFER_SIZE, DTN_SESSION_COOLDOWN, DTN_MAX_HOPS

MSG_DTN_SUMMARY = "dtn_summary"
MSG_DTN_REQUEST = "dtn_request"
MSG_DTN_DATA_REQ = "dtn_data_and_request"
MSG_DTN_DATA = "dtn_data"

class DTNMixin:
    _dtn_last_session: Dict[int, float]

    def initialize(self) -> None:
        if hasattr(super(), "initialize"):
            super().initialize()
        self._dtn_last_session = {}

    def _enforce_buffer_limit(self):
        if not hasattr(self, "_core_buffer"):
            self._core_buffer = {}
        while len(self._core_buffer) > MAX_BUFFER_SIZE:
            # Pop the oldest (first) item
            self._core_buffer.pop(next(iter(self._core_buffer)))

    def route_in_core(self, msg: dict) -> None:
        # CoreDockMixin already put this in _core_buffer. Just enforce limit.
        self._enforce_buffer_limit()

    def _get_summary_vector(self) -> List[str]:
        if not hasattr(self, "_core_buffer"):
            return []
        sv = []
        for msg_id, msg in self._core_buffer.items():
            if msg.get("hops", 1) < DTN_MAX_HOPS:
                sv.append(msg_id)
        return sv

    def _get_missing_messages(self, peer_summary: List[str]) -> List[str]:
        my_sv = set(self._get_summary_vector())
        return [msg_id for msg_id in peer_summary if msg_id not in my_sv]

    def _get_messages_for_ids(self, requested_ids: List[str]) -> List[dict]:
        if not hasattr(self, "_core_buffer"):
            return []
        msgs = []
        for msg_id in requested_ids:
            if msg_id in self._core_buffer:
                msgs.append(self._core_buffer[msg_id])
        return msgs

    def _store_dtn_messages(self, messages: List[dict]):
        if not hasattr(self, "_core_buffer"):
            self._core_buffer = {}
        for msg in messages:
            msg_id = msg.get("id")
            if msg_id and msg_id not in self._core_buffer:
                # Increment hops upon receiving
                new_msg = dict(msg)
                new_msg["hops"] = new_msg.get("hops", 1) + 1
                self._core_buffer[msg_id] = new_msg
        self._enforce_buffer_limit()

    def handle_packet(self, message: str) -> None:
        try:
            raw = json.loads(message)
        except json.JSONDecodeError:
            if hasattr(super(), "handle_packet"):
                super().handle_packet(message)
            return

        msg_type = raw.get("msg_type")
        if msg_type not in (MSG_DTN_SUMMARY, MSG_DTN_REQUEST, MSG_DTN_DATA_REQ, MSG_DTN_DATA):
            if hasattr(super(), "handle_packet"):
                super().handle_packet(message)
            return

        sender_id = raw.get("sender_id")
        if sender_id is None:
            return

        my_id = self.provider.get_id()
        
        # 1. Receive dtn_summary (from initiator)
        if msg_type == MSG_DTN_SUMMARY:
            peer_summary = raw.get("summary", [])
            want_ids = self._get_missing_messages(peer_summary)
            my_sv = self._get_summary_vector()
            
            # Send dtn_request
            reply = {
                "msg_type": MSG_DTN_REQUEST,
                "sender_id": my_id,
                "request": want_ids,
                "summary": my_sv
            }
            self.provider.send_communication_command(SendMessageCommand(json.dumps(reply), sender_id))
            self._dtn_last_session[sender_id] = self.provider.current_time()
            return

        # 2. Receive dtn_request (from peer)
        if msg_type == MSG_DTN_REQUEST:
            peer_request = raw.get("request", [])
            peer_summary = raw.get("summary", [])
            
            data_to_send = self._get_messages_for_ids(peer_request)
            want_ids = self._get_missing_messages(peer_summary)
            
            reply = {
                "msg_type": MSG_DTN_DATA_REQ,
                "sender_id": my_id,
                "data": data_to_send,
                "request": want_ids
            }
            self.provider.send_communication_command(SendMessageCommand(json.dumps(reply), sender_id))
            return

        # 3. Receive dtn_data_and_request (from initiator)
        if msg_type == MSG_DTN_DATA_REQ:
            data_received = raw.get("data", [])
            self._store_dtn_messages(data_received)
            
            peer_request = raw.get("request", [])
            if peer_request:
                data_to_send = self._get_messages_for_ids(peer_request)
                reply = {
                    "msg_type": MSG_DTN_DATA,
                    "sender_id": my_id,
                    "data": data_to_send
                }
                self.provider.send_communication_command(SendMessageCommand(json.dumps(reply), sender_id))
            return

        # 4. Receive dtn_data (from peer)
        if msg_type == MSG_DTN_DATA:
            data_received = raw.get("data", [])
            self._store_dtn_messages(data_received)
            return

    def handle_telemetry(self, telemetry) -> None:
        if hasattr(super(), "handle_telemetry"):
            super().handle_telemetry(telemetry)
        
        my_id = self.provider.get_id()
        current_time = self.provider.current_time()
        
        # Initiate sessions with fresh neighbors
        if hasattr(self, "fresh_neighbors"):
            neighbors = self.fresh_neighbors()
            for nid in neighbors:
                # Lower ID initiates
                if my_id < nid:
                    last_sess = self._dtn_last_session.get(nid, 0.0)
                    if current_time - last_sess >= DTN_SESSION_COOLDOWN:
                        self._dtn_last_session[nid] = current_time
                        msg = {
                            "msg_type": MSG_DTN_SUMMARY,
                            "sender_id": my_id,
                            "summary": self._get_summary_vector()
                        }
                        self.provider.send_communication_command(
                            SendMessageCommand(json.dumps(msg), nid)
                        )
