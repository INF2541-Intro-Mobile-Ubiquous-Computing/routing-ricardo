"""
Core hub protocol — the stationary leader of the rotating wheel formation.

The hub:
  - Uses MobilityLeaderPlugin to broadcast its position every
    CORE_BROADCAST_INTERVAL seconds so rim nodes can follow.
  - Never moves itself.
  - Fires a timer every CORE_SPIN_DT seconds to update the shared orientation
    angle, which hub_hub_spin_state.current_angle is read by rim nodes to
    recompute their relative positions.
  - Participates in routing (core dock mixin, applied by the factory in uav.py).
  - Ignores raw sensor packets (sensors broadcast to anyone nearby).

Rotation implementation
-----------------------
Because the installed MobilityLeaderPlugin broadcasts only the hub's position
(no orientation field), we implement rotation by having the HUB update a
global shared state object that all rim nodes poll every CORE_SPIN_DT seconds
to recompute their set_relative_position() offset.
"""
import json
import logging
import math
from typing import Dict, List

from gradysim.protocol.interface import IProtocol
from gradysim.protocol.messages.communication import BroadcastMessageCommand
from gradysim.protocol.messages.telemetry import Telemetry
from gradysim.protocol.plugin.follow_mobility import (
    MobilityLeaderPlugin,
    MobilityLeaderConfiguration,
)

from utils.common.config import (
    COMM_RANGE,
    CORE_BROADCAST_INTERVAL,
    CORE_INITIAL_ORIENTATION_DEG,
    CORE_RIM_NODES,
    CORE_RADIUS,
    CORE_SPIN_DT,
    ROT_RATE_DEG,
    DOCK_RADIUS,
    ALIGN_EPS,
    FACES,
    FLIGHT_ALT,
    dock_point_of,
)
from utils.common.agent_types import AgentType

# ---------------------------------------------------------------------------
# Shared spin state — one per simulation process
# ---------------------------------------------------------------------------

BEACON_MSG_TYPE = "core_beacon"  # re-exported; canonical is dock_mixin.BEACON_MSG_TYPE


class _SpinState:
    """Mutable container holding the current orientation angle (degrees)."""
    def __init__(self):
        self.current_angle: float = CORE_INITIAL_ORIENTATION_DEG   # degrees, unwrapped

# Single global instance shared by hub and all rim nodes.
HUB_SPIN_STATE = _SpinState()


# ---------------------------------------------------------------------------
# Core beacon (for neighbor tables)
# ---------------------------------------------------------------------------
CORE_BEACON_INTERVAL = 0.25    # seconds


class CoreHubProtocol(IProtocol):
    """
    Hub (leader) protocol.
    Routing mixins (flooding / dtn / geocasting) are applied on top in uav.py.
    """

    # ── attributes set by routing mixins ──────────────────────────────────
    packets: Dict[str, int]          # sensor→count  (required by mixins)
    _current_position: tuple         # XYZ           (required by geo mixin)

    def initialize(self) -> None:
        # ── Packet buffer (routing mixins read/write this) ─────────────
        self.packets = {}
        self._current_position = (0.0, 0.0, FLIGHT_ALT)

        # ── Follow-mobility leader plugin ──────────────────────────────
        self._leader = MobilityLeaderPlugin(
            self,
            MobilityLeaderConfiguration(
                broadcast_interval=CORE_BROADCAST_INTERVAL,
            ),
        )

        # ── Core message buffer for dock delivery ──────────────────────
        # Keyed by msg_id → message dict (dest_face, payload, hops, …)
        self._core_buffer: Dict[str, dict] = {}

        # ── Spin timer ─────────────────────────────────────────────────
        self.provider.schedule_timer("hub_spin", self.provider.current_time() + CORE_SPIN_DT)

        # ── Beacon timer ───────────────────────────────────────────────
        self.provider.schedule_timer("core_beacon", self.provider.current_time() + CORE_BEACON_INTERVAL)

        # ── Periodic sample log for acceptance check ───────────────────
        self.provider.schedule_timer("hub_sample", self.provider.current_time() + 10.0)

    # ── Timers ────────────────────────────────────────────────────────────

    def handle_timer(self, timer: str) -> None:
        if timer == "hub_spin":
            t = self.provider.current_time()
            HUB_SPIN_STATE.current_angle = CORE_INITIAL_ORIENTATION_DEG + ROT_RATE_DEG * t
            self.provider.schedule_timer("hub_spin", t + CORE_SPIN_DT)

        elif timer == "core_beacon":
            self._send_beacon()
            self.provider.schedule_timer(
                "core_beacon", self.provider.current_time() + CORE_BEACON_INTERVAL
            )

        elif timer == "hub_sample":
            logging.info(
                f"[HUB {self.provider.get_id()}] orientation={HUB_SPIN_STATE.current_angle:.1f}° "
                f"buffer_size={len(self._core_buffer)}"
            )
            self.provider.schedule_timer("hub_sample", self.provider.current_time() + 10.0)

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

        # Ignore follow-mobility leader/follower protocol messages
        # (those are intercepted by the dispatcher before reaching here)

        # Dispatch: dock handshake and routing messages fall to super()
        super().handle_packet(message)

    # ── Beacon ────────────────────────────────────────────────────────────

    def _send_beacon(self) -> None:
        msg = json.dumps({
            "msg_type": BEACON_MSG_TYPE,
            "sender_id": self.provider.get_id(),
            "role": "hub",
            "position": list(self._current_position),
        })
        self.provider.send_communication_command(BroadcastMessageCommand(msg))

    # ── Alignment check ───────────────────────────────────────────────────

    def _aligned_with(self, dock_2d: tuple) -> bool:
        """
        The hub is stationary at the center — it is never physically near
        a dock point (which is at DOCK_RADIUS from center).  Only rim nodes
        that have rotated into angular alignment should respond to dock_hello.
        """
        return False

    # ── finish ────────────────────────────────────────────────────────────

    def finish(self) -> None:
        from utils.metrics.stats import print_summary
        logging.info(
            f"[HUB {self.provider.get_id()}] finish | buffer={len(self._core_buffer)} messages"
        )
        print_summary()
