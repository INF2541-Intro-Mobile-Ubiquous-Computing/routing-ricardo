"""
Core rim node protocol — follower in the rotating wheel formation.

Each rim node k:
  - Uses MobilityFollowerPlugin to track the hub's position.
  - Every CORE_SPIN_DT seconds, recomputes its relative offset from the hub
    based on the current HUB_SPIN_STATE.current_angle, then calls
    follower.set_relative_position() so the follow-mobility plugin issues
    a fresh GotoCoordsMobilityCommand.
  - Participates in dock handshake with satellites and routing.
  - Ignores raw sensor packets.
"""
import json
import logging
import math
from typing import Dict

from gradysim.protocol.interface import IProtocol
from gradysim.protocol.messages.communication import (
    BroadcastMessageCommand,
    SendMessageCommand,
)
from gradysim.protocol.messages.mobility import SetSpeedMobilityCommand
from gradysim.protocol.messages.telemetry import Telemetry
from gradysim.protocol.plugin.follow_mobility import (
    MobilityFollowerPlugin,
    MobilityFollowerConfiguration,
)

from utils.common.config import (
    COMM_RANGE,
    CORE_RADIUS,
    CORE_RIM_NODES,
    CORE_SPIN_DT,
    FLIGHT_ALT,
    UAV_SPEED,
    dock_point_of,
)
from utils.common.agent_types import AgentType
from src.core_hub import HUB_SPIN_STATE
from src.dock_mixin import BEACON_MSG_TYPE

CORE_BEACON_INTERVAL = 0.25


def make_core_rim_protocol(k: int) -> type:
    """
    Factory returning a CoreRimProtocol class for rim node index k (0-7).

    The rim angle for node k is k * 45° (0° = +X) within the hub's rotating
    frame.  Absolute angle = current_orientation + k * 45°.
    """

    _k = k
    _base_angle_deg = k * (360.0 / CORE_RIM_NODES)   # local angle in the formation frame

    class CoreRimProtocol(IProtocol):
        """
        Rim node k.
        Routing mixins are applied on top in uav.py via make_core_rim().
        """

        # ── attributes required by routing mixins ─────────────────────
        packets: Dict[str, int]
        _current_position: tuple

        def initialize(self) -> None:
            self.packets = {}
            self._current_position = (0.0, 0.0, FLIGHT_ALT)

            # ── Follow-mobility follower plugin ────────────────────────
            self._follower = MobilityFollowerPlugin(
                self,
                MobilityFollowerConfiguration(
                    auto_follow=True,
                    scanning_interval=0.5,
                    leader_timeout=2.0,
                ),
            )

            # Set initial relative position (hub frame angle + k offset)
            angle_rad = math.radians(HUB_SPIN_STATE.current_angle + _base_angle_deg)
            self._follower.set_relative_position((
                CORE_RADIUS * math.cos(angle_rad),
                CORE_RADIUS * math.sin(angle_rad),
                0.0,
            ))

            # Set speed once
            self.provider.send_mobility_command(SetSpeedMobilityCommand(UAV_SPEED))

            # ── Core message buffer ────────────────────────────────────
            self._core_buffer: Dict[str, dict] = {}

            # ── Spin + beacon timers ───────────────────────────────────
            self.provider.schedule_timer("rim_spin", self.provider.current_time() + CORE_SPIN_DT)
            self.provider.schedule_timer("core_beacon", self.provider.current_time() + CORE_BEACON_INTERVAL)

            # ── Periodic sample log ────────────────────────────────────
            if _k == 0:
                self._last_pos_sample = None
                self._last_sample_time = None
                self.provider.schedule_timer("rim_sample", self.provider.current_time() + 5.0)

        # ── Timers ─────────────────────────────────────────────────────

        def handle_timer(self, timer: str) -> None:
            if timer == "rim_spin":
                angle_rad = math.radians(HUB_SPIN_STATE.current_angle + _base_angle_deg)
                new_rel = (
                    CORE_RADIUS * math.cos(angle_rad),
                    CORE_RADIUS * math.sin(angle_rad),
                    0.0,
                )
                self._follower.set_relative_position(new_rel)
                self.provider.schedule_timer("rim_spin", self.provider.current_time() + CORE_SPIN_DT)

            elif timer == "core_beacon":
                self._send_beacon()
                self.provider.schedule_timer(
                    "core_beacon", self.provider.current_time() + CORE_BEACON_INTERVAL
                )

            elif timer == "rim_sample" and _k == 0:
                self._do_sample()
                self.provider.schedule_timer("rim_sample", self.provider.current_time() + 5.0)

            else:
                super().handle_timer(timer)

        # ── Telemetry ──────────────────────────────────────────────────

        def handle_telemetry(self, telemetry: Telemetry) -> None:
            pos = tuple(telemetry.current_position)
            self._current_position = pos

            # Hub distance check (acceptance criterion)
            if self._follower.current_leader_position is not None:
                lp = self._follower.current_leader_position
                dist = math.sqrt(
                    (pos[0] - lp[0]) ** 2 + (pos[1] - lp[1]) ** 2 + (pos[2] - lp[2]) ** 2
                )
                if dist > COMM_RANGE:
                    logging.warning(
                        f"[RIM{_k} {self.provider.get_id()}] hub distance {dist:.2f} > COMM_RANGE!"
                    )

            super().handle_telemetry(telemetry)

        # ── Packets ────────────────────────────────────────────────────

        def handle_packet(self, message: str) -> None:
            try:
                raw = json.loads(message)
            except json.JSONDecodeError:
                return

            # Ignore raw sensor packets
            if raw.get("sender_type") == AgentType.SENSOR.value:
                return

            # Update neighbor table from core beacons (for geocasting)
            if raw.get("msg_type") == BEACON_MSG_TYPE:
                peer_id = raw.get("sender_id")
                position = raw.get("position")
                if peer_id is not None and position is not None:
                    if not hasattr(self, "_geo_neighbors"):
                        self._geo_neighbors = {}
                    self._geo_neighbors[peer_id] = tuple(position)
                # Don't return — pass to super for potential geo usage
                return super().handle_packet(message)

            super().handle_packet(message)

        # ── Beacon ─────────────────────────────────────────────────────

        def _send_beacon(self) -> None:
            msg = json.dumps({
                "msg_type": BEACON_MSG_TYPE,
                "sender_id": self.provider.get_id(),
                "role": f"rim{_k}",
                "position": list(self._current_position),
            })
            self.provider.send_communication_command(BroadcastMessageCommand(msg))

        # ── Acceptance sample (rim 0 only) ─────────────────────────────

        def _do_sample(self) -> None:
            t = self.provider.current_time()
            pos = self._current_position
            lp = self._follower.current_leader_position
            if lp is not None:
                r = math.sqrt((pos[0] - lp[0]) ** 2 + (pos[1] - lp[1]) ** 2)
                expected_angle = math.radians(HUB_SPIN_STATE.current_angle + _base_angle_deg)
                expected_x = lp[0] + CORE_RADIUS * math.cos(expected_angle)
                expected_y = lp[1] + CORE_RADIUS * math.sin(expected_angle)
                angle_err = math.sqrt((pos[0] - expected_x) ** 2 + (pos[1] - expected_y) ** 2)

                if self._last_sample_time is not None:
                    dt = t - self._last_sample_time
                    if self._last_pos_sample is not None and dt > 0:
                        lp0 = self._last_pos_sample
                        # Estimate angular movement
                        prev_angle = math.atan2(lp0[1] - lp[1], lp0[0] - lp[0])
                        curr_angle = math.atan2(pos[1] - lp[1], pos[0] - lp[0])
                        da = math.degrees(curr_angle - prev_angle)
                        rate = abs(da) / dt
                        logging.info(
                            f"[RIM0 sample] t={t:.1f} r={r:.2f}m "
                            f"err={angle_err:.2f}m rate≈{rate:.2f}°/s"
                        )

                self._last_sample_time = t
                self._last_pos_sample = pos

        # ── finish ─────────────────────────────────────────────────────

        def finish(self) -> None:
            logging.info(
                f"[RIM{_k} {self.provider.get_id()}] finish | buffer={len(self._core_buffer)}"
            )
            super().finish()

    CoreRimProtocol.__name__ = f"CoreRimProtocol_k{k}"
    return CoreRimProtocol
