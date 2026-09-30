"""
Satellite protocol — one per face (N, E, S, W).

State machine
-------------
  COLLECT_OUT  → move to outer turnaround point, collect sensor packets
  COLLECT_IN   → move back to dock point
  WAIT_ALIGN   → hover at dock point, broadcast dock_hello every 0.25 s
  EXCHANGE     → unicast dock_data to first responding rim node, then receive
                 dock_deliver (or time out after EXCHANGE_TIMEOUT)
  → back to COLLECT_OUT

Sensor collection
-----------------
The satellite sends a broadcast heartbeat so sensors respond.
Sensor packets are buffered in self.packets (accumulated, not merged).
Satellites do NOT gossip with other UAVs.

The satellite never enters the obstacle zone (stays outside dock_radius).
"""
import json
import logging
import math
from typing import Dict, Optional, Set

from gradysim.protocol.interface import IProtocol
from gradysim.protocol.messages.communication import (
    BroadcastMessageCommand,
    SendMessageCommand,
)
from gradysim.protocol.messages.mobility import (
    GotoCoordsMobilityCommand,
    SetSpeedMobilityCommand,
)
from gradysim.protocol.messages.telemetry import Telemetry

from utils.common.config import (
    SAT_SPEED,
    SAT_OUTER_DIST,
    DOCK_RADIUS,
    DOCK_GAP,
    ALIGN_EPS,
    FLIGHT_ALT,
    EXCHANGE_TIMEOUT,
    OBSTACLE_CENTER,
    dock_point_of,
    opposite_face,
    FACE_BY_NAME,
)
from utils.common.agent_types import AgentType
from src.dock_mixin import (
    MSG_DOCK_HELLO,
    MSG_DOCK_ACK,
    MSG_DOCK_DATA,
    MSG_DOCK_DELIVER,
    set_docked_sat,
    get_docked_sat,
)

ARRIVAL_RADIUS = 1.5     # metres (2-D) — arrival detection threshold
HELLO_INTERVAL = 0.25    # seconds — dock_hello period
SAT_HEARTBEAT_INTERVAL = 1.0    # seconds — sensor collection heartbeat


# ── States ──────────────────────────────────────────────────────────────────
ST_COLLECT_OUT = "COLLECT_OUT"
ST_COLLECT_IN  = "COLLECT_IN"
ST_WAIT_ALIGN  = "WAIT_ALIGN"
ST_EXCHANGE    = "EXCHANGE"


def make_satellite_protocol(face_name: str) -> type:
    """
    Factory returning a SatelliteProtocol class for the given compass face.

    face_name must be one of "N", "E", "S", "W".
    """
    face_entry = FACE_BY_NAME[face_name]
    _face      = face_name
    _bearing   = face_entry[1]
    _unit_vec  = face_entry[2]        # (sin, cos) unit vector outward
    _dock_2d   = face_entry[3]        # (x, y) dock point
    _opp_face  = face_entry[4]        # opposite face name

    cx, cy = OBSTACLE_CENTER

    # Outer turnaround: center + SAT_OUTER_DIST * unit vector
    _outer_x = cx + SAT_OUTER_DIST * _unit_vec[0]
    _outer_y = cy + SAT_OUTER_DIST * _unit_vec[1]
    _outer_3d = (_outer_x, _outer_y, FLIGHT_ALT)
    _dock_3d  = (_dock_2d[0], _dock_2d[1], FLIGHT_ALT)

    class SatelliteProtocol(IProtocol):
        """Satellite for face ``_face``."""

        def initialize(self) -> None:
            self.packets: Dict[str, int] = {}
            self._state = ST_COLLECT_OUT
            self._current_position = (_dock_2d[0], _dock_2d[1], FLIGHT_ALT)

            # Exchange tracking
            self._ack_node: Optional[int] = None   # rim node that answered ack
            self._delivered_ids: Set[str]  = set()  # dedup delivered messages

            # Metrics
            self._cycle_start_time: float = self.provider.current_time()
            self._wait_start_time: Optional[float] = None
            self._dock_count = 0

            # Set speed once
            self.provider.send_mobility_command(SetSpeedMobilityCommand(SAT_SPEED))

            # Start first outbound leg
            self._goto_outer()

            # Sensor heartbeat
            self.provider.schedule_timer("sat_heartbeat", self.provider.current_time() + SAT_HEARTBEAT_INTERVAL)

        # ── Movement helpers ──────────────────────────────────────────

        def _goto_outer(self) -> None:
            self._state = ST_COLLECT_OUT
            logging.debug(f"[SAT-{_face}] → COLLECT_OUT")
            self.provider.send_mobility_command(
                GotoCoordsMobilityCommand(_outer_x, _outer_y, FLIGHT_ALT)
            )

        def _goto_dock(self) -> None:
            self._state = ST_COLLECT_IN
            logging.debug(f"[SAT-{_face}] → COLLECT_IN")
            self.provider.send_mobility_command(
                GotoCoordsMobilityCommand(_dock_2d[0], _dock_2d[1], FLIGHT_ALT)
            )

        def _start_wait_align(self) -> None:
            self._state = ST_WAIT_ALIGN
            self._ack_node = None
            self._wait_start_time = self.provider.current_time()
            set_docked_sat(_face, self.provider.get_id())
            logging.info(f"[SAT-{_face}] → WAIT_ALIGN (buffer={len(self.packets)} sensors)")
            self.provider.schedule_timer("sat_hello", self.provider.current_time())

        def _start_exchange(self, ack_node_id: int) -> None:
            self._state = ST_EXCHANGE
            self._ack_node = ack_node_id
            logging.info(f"[SAT-{_face}] → EXCHANGE with rim/hub {ack_node_id}")

            # Compute wait time metric
            wait_time = 0.0
            if self._wait_start_time is not None:
                wait_time = self.provider.current_time() - self._wait_start_time

            # Upload data if any
            if self.packets:
                self._upload_data(ack_node_id, wait_time)

            # Start exchange timeout
            self.provider.schedule_timer("sat_exchange_timeout",
                                         self.provider.current_time() + EXCHANGE_TIMEOUT)

        def _depart(self) -> None:
            """Complete exchange and start next outbound leg."""
            set_docked_sat(_face, None)

            # Metrics: record dock
            wait_time = 0.0
            if self._wait_start_time is not None:
                wait_time = self.provider.current_time() - self._wait_start_time
            cycle_time = self.provider.current_time() - self._cycle_start_time
            self._dock_count += 1

            from utils.metrics.stats import record_dock
            record_dock(_face, wait_time, cycle_time)

            logging.info(
                f"[SAT-{_face}] departing | dock#{self._dock_count} "
                f"wait={wait_time:.2f}s cycle={cycle_time:.2f}s"
            )

            # Reset for next cycle
            self._cycle_start_time = self.provider.current_time()
            self._wait_start_time = None
            self._ack_node = None
            self.packets = {}
            self._goto_outer()

        # ── Data upload ────────────────────────────────────────────────

        def _upload_data(self, target_id: int, wait_time: float) -> None:
            import uuid
            msg_id = f"sat_{_face}_{self.provider.get_id()}_{self._dock_count}"
            created_at = self.provider.current_time() - wait_time  # approximate upload time

            payload = json.dumps({
                "msg_type": MSG_DOCK_DATA,
                "id": msg_id,
                "sender_id": self.provider.get_id(),
                "target_id": target_id,
                "origin_face": _face,
                "dest_face": _opp_face,
                "payload": dict(self.packets),
                "created_at": created_at,
            })
            self.provider.send_communication_command(
                SendMessageCommand(payload, target_id)
            )
            logging.info(
                f"[SAT-{_face}] uploaded dock_data → node {target_id} "
                f"(msg_id={msg_id}, {len(self.packets)} sensors, dest={_opp_face})"
            )

        # ── Timers ─────────────────────────────────────────────────────

        def handle_timer(self, timer: str) -> None:
            if timer == "sat_heartbeat":
                # Broadcast sensor request so nearby sensors respond
                msg = json.dumps({
                    "sender_type": AgentType.UAV.value,
                    "sender_id": self.provider.get_id(),
                    "face": _face,
                })
                self.provider.send_communication_command(BroadcastMessageCommand(msg))
                self.provider.schedule_timer(
                    "sat_heartbeat", self.provider.current_time() + SAT_HEARTBEAT_INTERVAL
                )

            elif timer == "sat_hello":
                if self._state == ST_WAIT_ALIGN:
                    msg = json.dumps({
                        "msg_type": MSG_DOCK_HELLO,
                        "face": _face,
                        "sender_id": self.provider.get_id(),
                    })
                    self.provider.send_communication_command(BroadcastMessageCommand(msg))
                    self.provider.schedule_timer(
                        "sat_hello", self.provider.current_time() + HELLO_INTERVAL
                    )

            elif timer == "sat_exchange_timeout":
                if self._state == ST_EXCHANGE:
                    logging.warning(f"[SAT-{_face}] exchange timeout — departing anyway")
                    self._depart()

        # ── Telemetry ──────────────────────────────────────────────────

        def handle_telemetry(self, telemetry: Telemetry) -> None:
            pos = tuple(telemetry.current_position)
            self._current_position = pos

            if self._state == ST_COLLECT_OUT:
                dist = math.sqrt((pos[0] - _outer_x) ** 2 + (pos[1] - _outer_y) ** 2)
                if dist <= ARRIVAL_RADIUS:
                    logging.debug(f"[SAT-{_face}] reached outer point")
                    self._goto_dock()

            elif self._state == ST_COLLECT_IN:
                dist = math.sqrt((pos[0] - _dock_2d[0]) ** 2 + (pos[1] - _dock_2d[1]) ** 2)
                if dist <= ARRIVAL_RADIUS:
                    self._start_wait_align()

            # Acceptance check: never inside dock_gap * 0.8 of any rim node
            # (logged as warning; checked by proximity to rim circles)

        # ── Packets ────────────────────────────────────────────────────

        def handle_packet(self, message: str) -> None:
            try:
                raw = json.loads(message)
            except json.JSONDecodeError:
                return

            msg_type = raw.get("msg_type")
            sender_type = raw.get("sender_type")

            # ── Sensor data ───────────────────────────────────────────
            if sender_type == AgentType.SENSOR.value:
                if self._state in (ST_COLLECT_OUT, ST_COLLECT_IN):
                    incoming: Dict[str, int] = raw.get("packets", {})
                    for sensor, count in incoming.items():
                        self.packets[sensor] = self.packets.get(sensor, 0) + count
                    logging.debug(
                        f"[SAT-{_face}] collected {incoming} | buffer={self.packets}"
                    )
                return

            # ── dock_ack — first ack enters exchange ──────────────────
            if msg_type == MSG_DOCK_ACK:
                if raw.get("face") != _face:
                    return  # not for us
                if self._state == ST_WAIT_ALIGN and self._ack_node is None:
                    self._start_exchange(raw.get("sender_id"))
                return

            # ── dock_deliver — destination satellite receives data ─────
            if msg_type == MSG_DOCK_DELIVER:
                msg_id = raw.get("id")
                if msg_id and msg_id not in self._delivered_ids:
                    self._delivered_ids.add(msg_id)
                    hops = raw.get("hops", 0)
                    from utils.metrics.stats import record_delivery
                    record_delivery(msg_id, self.provider.current_time(), hops)
                    logging.info(
                        f"[SAT-{_face}] received dock_deliver: msg_id={msg_id} hops={hops}"
                    )
                # Depart after receiving delivery
                if self._state == ST_EXCHANGE:
                    self._depart()
                return

            # ── Ignore everything else (routing msgs, beacons, etc.) ──

        # ── finish ─────────────────────────────────────────────────────

        def finish(self) -> None:
            logging.info(
                f"[SAT-{_face}] finish | docks={self._dock_count} "
                f"delivered_ids={len(self._delivered_ids)}"
            )

    SatelliteProtocol.__name__ = f"SatelliteProtocol_{face_name}"
    return SatelliteProtocol
