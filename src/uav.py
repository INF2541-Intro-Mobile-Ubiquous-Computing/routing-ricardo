"""
UAV protocol factories for the core + satellite topology.

Public API
----------
make_core_hub(routing)      → hub leader class (with viz if enabled)
make_core_rim(k, routing)   → rim follower class k=0..7 (with viz)
make_random_uav(routing)    → randomly moving core node (random topology)
make_satellite(face)        → satellite class for face ∈ {N,E,S,W}
"""
from typing import List, Tuple

from utils.viz_templates.uav_viz_template import UAVVizMixin
from utils.viz_templates.constants import COLOR_CORE_HUB, COLOR_CORE_RIM, COLOR_SATELLITE
from gradysim.simulator.extension.visualization_controller import VisualizationController
from utils.viz_templates.constants import VIZ_NODE_SIZE

from src.core_hub import CoreHubProtocol
from src.core_rim import make_core_rim_protocol
from src.dock_mixin import CoreDockMixin
from src.neighbor_mixin import NeighborMixin
from src.satellite import make_satellite_protocol
from src.random_uav import RandomUAVProtocol

from src.flooding_mixin import FloodingMixin
from src.dtn_mixin import DTNMixin
from src.geocasting_mixin import GeocastingMixin


_ROUTING_MIXINS = {
    "flooding":   FloodingMixin,
    "dtn":        DTNMixin,
    "geocasting": GeocastingMixin,
}


def _routing_mixin(name: str) -> type:
    try:
        return _ROUTING_MIXINS[name.lower()]
    except (AttributeError, KeyError) as exc:
        choices = ", ".join(sorted(_ROUTING_MIXINS))
        raise ValueError(f"Unknown routing strategy {name!r}; choose {choices}") from exc


# ---------------------------------------------------------------------------
# Hub viz mixin
# ---------------------------------------------------------------------------

class HubVizMixin:
    def initialize(self) -> None:
        super().initialize()
        self._vc = VisualizationController(self)
        self._vc.resize_nodes(VIZ_NODE_SIZE)
        self.provider.schedule_timer("hub_repaint", self.provider.current_time() + 1)

    def handle_timer(self, timer: str) -> None:
        super().handle_timer(timer)
        if timer == "hub_repaint":
            self._vc.paint_node(self.provider.get_id(), COLOR_CORE_HUB)
            self.provider.schedule_timer("hub_repaint", self.provider.current_time() + 1)


class RimVizMixin:
    def initialize(self) -> None:
        super().initialize()
        self._vc = VisualizationController(self)
        self._vc.resize_nodes(VIZ_NODE_SIZE)
        self.provider.schedule_timer("rim_repaint", self.provider.current_time() + 1)

    def handle_timer(self, timer: str) -> None:
        super().handle_timer(timer)
        if timer == "rim_repaint":
            self._vc.paint_node(self.provider.get_id(), COLOR_CORE_RIM)
            self.provider.schedule_timer("rim_repaint", self.provider.current_time() + 1)


class SatVizMixin:
    def initialize(self) -> None:
        super().initialize()
        self._vc = VisualizationController(self)
        self._vc.resize_nodes(VIZ_NODE_SIZE)
        self.provider.schedule_timer("sat_repaint", self.provider.current_time() + 1)

    def handle_timer(self, timer: str) -> None:
        super().handle_timer(timer)
        if timer == "sat_repaint":
            self._vc.paint_node(self.provider.get_id(), COLOR_SATELLITE)
            self.provider.schedule_timer("sat_repaint", self.provider.current_time() + 1)


# ---------------------------------------------------------------------------
# Factories
# ---------------------------------------------------------------------------

def make_core_hub(routing: str = "flooding", viz: bool = False) -> type:
    """
    Return a hub (leader) protocol class.

    MRO (most-derived first):
        [HubVizMixin,] CoreDockMixin, RoutingMixin, CoreHubProtocol
    """
    RoutingMixin = _routing_mixin(routing)

    if viz:
        class _HubViz(HubVizMixin, NeighborMixin, CoreDockMixin, RoutingMixin, CoreHubProtocol):
            pass
        _HubViz.__name__ = f"CoreHub_{routing}_viz"
        return _HubViz

    class _Hub(NeighborMixin, CoreDockMixin, RoutingMixin, CoreHubProtocol):
        pass
    _Hub.__name__ = f"CoreHub_{routing}"
    return _Hub


def make_core_rim(k: int, routing: str = "flooding", viz: bool = False) -> type:
    """
    Return a rim (follower) protocol class for slot k.

    MRO (most-derived first):
        [RimVizMixin,] CoreDockMixin, RoutingMixin, CoreRimProtocol_k
    """
    RoutingMixin = _routing_mixin(routing)
    BaseRim = make_core_rim_protocol(k)

    if viz:
        class _RimViz(RimVizMixin, NeighborMixin, CoreDockMixin, RoutingMixin, BaseRim):
            pass
        _RimViz.__name__ = f"CoreRim{k}_{routing}_viz"
        return _RimViz

    class _Rim(NeighborMixin, CoreDockMixin, RoutingMixin, BaseRim):
        pass
    _Rim.__name__ = f"CoreRim{k}_{routing}"
    return _Rim


def make_random_uav(routing: str = "flooding", viz: bool = False) -> type:
    """
    Return a randomly moving core node class (random topology).

    MRO (most-derived first):
        [RimVizMixin,] CoreDockMixin, RoutingMixin, RandomUAVProtocol
    """
    RoutingMixin = _routing_mixin(routing)

    if viz:
        class _RandViz(RimVizMixin, NeighborMixin, CoreDockMixin, RoutingMixin, RandomUAVProtocol):
            pass
        _RandViz.__name__ = f"RandomUAV_{routing}_viz"
        return _RandViz

    class _Rand(NeighborMixin, CoreDockMixin, RoutingMixin, RandomUAVProtocol):
        pass
    _Rand.__name__ = f"RandomUAV_{routing}"
    return _Rand


def make_satellite(face: str, viz: bool = False) -> type:
    """
    Return a satellite protocol class for the given face.

    Satellites do NOT use routing mixins.
    """
    BaseSat = make_satellite_protocol(face)

    if viz:
        class _SatViz(SatVizMixin, BaseSat):
            pass
        _SatViz.__name__ = f"Satellite_{face}_viz"
        return _SatViz

    return BaseSat