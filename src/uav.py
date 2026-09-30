"""
UAV protocol factories for the core + satellite topology.

Public API
----------
make_core_hub(routing)      → hub leader class (with viz if enabled)
make_core_rim(k, routing)   → rim follower class k=0..7 (with viz)
make_satellite(face)        → satellite class for face ∈ {N,E,S,W}

Legacy (unused but left so imports don't break if any test references them):
    make_uav, make_uav_viz
"""
from typing import List, Tuple

from utils.viz_templates.uav_viz_template import UAVVizMixin
from utils.viz_templates.constants import COLOR_CORE_HUB, COLOR_CORE_RIM, COLOR_SATELLITE
from gradysim.simulator.extension.visualization_controller import VisualizationController
from utils.viz_templates.constants import VIZ_NODE_SIZE

from src.core_hub import CoreHubProtocol
from src.core_rim import make_core_rim_protocol
from src.dock_mixin import CoreDockMixin
from src.satellite import make_satellite_protocol

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
        class _HubViz(HubVizMixin, CoreDockMixin, RoutingMixin, CoreHubProtocol):
            pass
        _HubViz.__name__ = f"CoreHub_{routing}_viz"
        return _HubViz

    class _Hub(CoreDockMixin, RoutingMixin, CoreHubProtocol):
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
        class _RimViz(RimVizMixin, CoreDockMixin, RoutingMixin, BaseRim):
            pass
        _RimViz.__name__ = f"CoreRim{k}_{routing}_viz"
        return _RimViz

    class _Rim(CoreDockMixin, RoutingMixin, BaseRim):
        pass
    _Rim.__name__ = f"CoreRim{k}_{routing}"
    return _Rim


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


# ---------------------------------------------------------------------------
# Legacy stubs (kept to avoid import errors in old tests)
# ---------------------------------------------------------------------------

def make_uav(cluster_index: int = 0, waypoints=None, group_size=None,
             cyclic: bool = True, routing_strategy: str = "flooding") -> type:
    """Deprecated — returns a core hub for backward compat."""
    import warnings
    warnings.warn("make_uav is deprecated; use make_core_hub/rim/satellite", DeprecationWarning)
    return make_core_hub(routing_strategy)


def make_uav_viz(cluster_index: int = 0, waypoints=None, group_size=None,
                 cyclic: bool = True, routing_strategy: str = "flooding") -> type:
    """Deprecated — returns a core hub viz for backward compat."""
    import warnings
    warnings.warn("make_uav_viz is deprecated; use make_core_hub/rim/satellite", DeprecationWarning)
    return make_core_hub(routing_strategy, viz=True)