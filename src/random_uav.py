"""
Random-topology core UAV protocol.

Each of the NUM_RANDOM_UAVS nodes:
  - Flies random waypoints inside the obstacle zone (gradysim RandomMobilityPlugin)
    at a fixed FLIGHT_ALT, at UAV_SPEED.
  - Broadcasts core beacons (for neighbor tables / geocasting).
  - Is "aligned" with a dock point per CoreDockMixin (within DOCK_RANGE, 2-D).
  - Participates in dock handshake and routing via mixins applied in uav.py.
  - Ignores raw sensor packets.
"""
import json
import logging
from typing import Dict

from gradysim.protocol.interface import IProtocol
from gradysim.protocol.messages.communication import BroadcastMessageCommand
from gradysim.protocol.messages.mobility import SetSpeedMobilityCommand
from gradysim.protocol.messages.telemetry import Telemetry
from gradysim.protocol.plugin.random_mobility import (
    RandomMobilityPlugin,
    RandomMobilityConfig,
)

from utils.common.agent_types import AgentType
from utils.common.config import (
    FLIGHT_ALT,
    UAV_SPEED,
    RANDOM_X_RANGE,
    RANDOM_Y_RANGE,
)
from src.dock_mixin import BEACON_MSG_TYPE

CORE_BEACON_INTERVAL = 0.25    # seconds

# Only the first node to finish prints the metrics summary.
_SUMMARY_PRINTED = False


class RandomUAVProtocol(IProtocol):
    """Randomly moving core node. Routing mixins are applied in uav.py."""

    packets: Dict[str, int]
    _current_position: tuple

    def initialize(self) -> None:
        self.packets = {}
        self._current_position = (0.0, 0.0, FLIGHT_ALT)
        self._core_buffer: Dict[str, dict] = {}

        # Random waypoints inside the obstacle zone, fixed altitude
        self.provider.send_mobility_command(SetSpeedMobilityCommand(UAV_SPEED))
        self._mobility = RandomMobilityPlugin(
            self,
            RandomMobilityConfig(
                x_range=RANDOM_X_RANGE,
                y_range=RANDOM_Y_RANGE,
                z_range=(FLIGHT_ALT, FLIGHT_ALT),
            ),
        )
        self._mobility.initiate_random_trip()

        self.provider.schedule_timer(
            "core_beacon", self.provider.current_time() + CORE_BEACON_INTERVAL
        )

    # ── Timers ────────────────────────────────────────────────────────────

    def handle_timer(self, timer: str) -> None:
        if timer == "core_beacon":
            self._send_beacon()
            self.provider.schedule_timer(
                "core_beacon", self.provider.current_time() + CORE_BEACON_INTERVAL
            )

    # ── Telemetry ─────────────────────────────────────────────────────────

    def handle_telemetry(self, telemetry: Telemetry) -> None:
        self._current_position = tuple(telemetry.current_position)

    # ── Packets ───────────────────────────────────────────────────────────

    def handle_packet(self, message: str) -> None:
        try:
            raw = json.loads(message)
        except json.JSONDecodeError:
            return

        # Ignore raw sensor packets — satellites collect data, not the core
        if raw.get("sender_type") == AgentType.SENSOR.value:
            return

        super().handle_packet(message)

    # ── Beacon ────────────────────────────────────────────────────────────

    def _send_beacon(self) -> None:
        msg = json.dumps({
            "msg_type": BEACON_MSG_TYPE,
            "sender_id": self.provider.get_id(),
            "role": "random",
            "position": list(self._current_position),
        })
        self.provider.send_communication_command(BroadcastMessageCommand(msg))

    # ── finish ────────────────────────────────────────────────────────────

    def finish(self) -> None:
        global _SUMMARY_PRINTED
        logging.info(
            f"[RANDOM {self.provider.get_id()}] finish | buffer={len(self._core_buffer)}"
        )
        if not _SUMMARY_PRINTED:
            _SUMMARY_PRINTED = True
            from utils.metrics.stats import print_summary
            print_summary()
