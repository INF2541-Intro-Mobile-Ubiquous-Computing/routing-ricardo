from typing import List, Tuple

from utils.actor_templates.uav_template import make_uav_protocol
from utils.viz_templates.uav_viz_template import UAVVizMixin
from utils.common.config import NUM_UAVS

from src.dtn_mixin import DTNMixin
from src.flooding_mixin import FloodingMixin
from src.geocasting_mixin import GeocastingMixin


_ROUTING_MIXINS = {
    "flooding": FloodingMixin,
    "dtn": DTNMixin,
    "geocasting": GeocastingMixin,
}


def _routing_mixin(name: str) -> type:
    try:
        return _ROUTING_MIXINS[name.lower()]
    except (AttributeError, KeyError) as exc:
        choices = ", ".join(sorted(_ROUTING_MIXINS))
        raise ValueError(f"Unknown routing strategy {name!r}; choose {choices}") from exc


def make_uav(
    cluster_index: int,
    waypoints: List[Tuple[float, float, float]] = None,
    group_size: int = None,
    cyclic: bool = True,
    routing_strategy: str = "flooding",
) -> type:
    """
    Factory that returns a UAV protocol class for the given cluster slot.

    Args:
        cluster_index: This UAV's slot within its sub-group (0-based).
        waypoints:     Ordered waypoint path for this UAV's group.
                   Defaults to the configured fixed route.
        group_size:    Number of UAVs sharing this path (used for
                       formation spacing).  Defaults to NUM_UAVS.
        cyclic:        Patrol the supplied route as a closed cycle.
        routing_strategy: Global routing protocol name: flooding, dtn, or
                  geocasting.

    If you want to add behaviours to every UAV (e.g. battery drain, swap
    logic), do it here: add a mixin before make_uav_protocol, or subclass
    the returned class.

    Example — adding battery drain to every UAV:

        class BatteryMixin:
            def handle_telemetry(self, telemetry):
                super().handle_telemetry(telemetry)
                # compute distance moved, subtract from self.battery …

        class _UAV(BatteryMixin, FloodingMixin, make_uav_protocol(
            waypoints     = waypoints,
            cluster_index = cluster_index,
            num_uavs      = NUM_UAVS,
        )):
            pass
        return _UAV
    """
    if waypoints is None:
        raise ValueError("A fixed patrol route must be provided")
    if group_size is None:
        group_size = NUM_UAVS

    RoutingMixin = _routing_mixin(routing_strategy)

    class _UAV(RoutingMixin, make_uav_protocol(
        waypoints     = waypoints,
        cluster_index = cluster_index,
        num_uavs      = group_size,
        cyclic        = cyclic,
    )):
        pass

    _UAV.__name__ = f"UAV_slot{cluster_index}"
    return _UAV


def make_uav_viz(
    cluster_index: int,
    waypoints: List[Tuple[float, float, float]] = None,
    group_size: int = None,
    cyclic: bool = True,
    routing_strategy: str = "flooding",
) -> type:
    """
    Same as make_uav() but with GrADySim visualisation support mixed in.
    Use this class with VisualizationHandler enabled.

    Args:
        cluster_index: This UAV's slot within its sub-group (0-based).
        waypoints:     Ordered waypoint path for this UAV's group.
                   Defaults to the configured fixed route.
        group_size:    Number of UAVs sharing this path (used for
                       formation spacing).  Defaults to NUM_UAVS.
        cyclic:        Patrol the supplied route as a closed cycle.
        routing_strategy: Global routing protocol name: flooding, dtn, or
                  geocasting.
    """
    if waypoints is None:
        raise ValueError("A fixed patrol route must be provided")
    if group_size is None:
        group_size = NUM_UAVS

    RoutingMixin = _routing_mixin(routing_strategy)

    class _UAVViz(UAVVizMixin, RoutingMixin, make_uav_protocol(
        waypoints     = waypoints,
        cluster_index = cluster_index,
        num_uavs      = group_size,
        cyclic        = cyclic,
    )):
        """UAV protocol with visualization and the selected routing mixin."""
        pass

    _UAVViz.__name__ = f"UAV_slot{cluster_index}"
    return _UAVViz