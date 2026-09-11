"""
Simulation entry point.

Run with:
    python -m src.main
    python -m src.main --no-viz         # disable browser visualisation
    python -m src.main --routing-strategy geocasting
"""
import logging
import argparse

from gradysim.simulator.handler.communication import (
    CommunicationHandler,
    CommunicationMedium,
)
from gradysim.simulator.handler.mobility import MobilityHandler
from gradysim.simulator.handler.timer import TimerHandler
from gradysim.simulator.handler.visualization import (
    VisualizationHandler,
    VisualizationConfiguration,
)
from gradysim.simulator.simulation import SimulationBuilder, SimulationConfiguration

from utils.common.config import (
    COMM_RANGE,
    SIMULATION_DURATION,
    BASE_GROUND,
    ENDPOINT_GROUND,
    FLIGHT_ALT,
    SENSOR_POSITIONS,
    FIXED_UAV_GROUPS,
)

from src.sensor import Sensor
from src.uav import make_uav_viz

def main() -> None:
    parser = argparse.ArgumentParser(description="GrADySim UAV cluster simulation")
    parser.add_argument(
        "--no-viz",
        action="store_true",
        default=False,
        help="disable the browser visualisation",
    )
    parser.add_argument(
        "--force-regen",
        action="store_true",
        default=False,
        help="Force regeneration of sensor positions and cluster waypoints",
    )
    parser.add_argument(
        "--routing-strategy",
        choices=("flooding", "dtn", "geocasting"),
        default="flooding",
        help="Global UAV routing protocol",
    )
    args = parser.parse_args()

    # ── Simulation config ──────────────────────────────────────────────────
    config  = SimulationConfiguration(duration=SIMULATION_DURATION)
    builder = SimulationBuilder(config)
    num_uavs = sum(group_size for _, group_size, _, _ in FIXED_UAV_GROUPS)

    logging.info(f"Base:              ({BASE_GROUND[0]:.1f}, {BASE_GROUND[1]:.1f})")
    logging.info(f"Endpoint:          ({ENDPOINT_GROUND[0]:.1f}, {ENDPOINT_GROUND[1]:.1f})")
    logging.info(f"Sensors placed:    {len(SENSOR_POSITIONS)}")
    logging.info(f"Route groups:       {[(name, size) for name, size, _, _ in FIXED_UAV_GROUPS]}")
    logging.info(f"UAVs:              {num_uavs}")
    logging.info(f"Routing strategy:  {args.routing_strategy}")

    # ── Scene bounds for visualisation ────────────────────────────────────
    if not args.no_viz:
        padding = 50
        all_x   = [p[0] for p in SENSOR_POSITIONS.values()] + [BASE_GROUND[0], ENDPOINT_GROUND[0]]
        all_y   = [p[1] for p in SENSOR_POSITIONS.values()] + [BASE_GROUND[1], ENDPOINT_GROUND[1]]
        viz_config = VisualizationConfiguration(
            x_range      = (min(all_x) - padding, max(all_x) + padding),
            y_range      = (min(all_y) - padding, max(all_y) + padding),
            z_range      = (0, FLIGHT_ALT + 20),
            open_browser = True,
        )

    # ── Sensors (static nodes) ─────────────────────────────────────────────
    for sensor_pos in SENSOR_POSITIONS.values():
        builder.add_node(Sensor, sensor_pos)

    # ── Fixed UAV patrol groups ───────────────────────────────────────────
    for group_name, group_size, waypoints, start_position in FIXED_UAV_GROUPS:
        logging.info(
            f"{group_name.title()} group: {group_size} UAVs "
            f"({len(waypoints)} waypoints)"
        )
        for cluster_index in range(group_size):
            UAVClass = make_uav_viz(
                cluster_index=cluster_index,
                waypoints=waypoints,
                group_size=group_size,
                cyclic=True,
                routing_strategy=args.routing_strategy,
            )
            builder.add_node(UAVClass, start_position)

    # ── Handlers ───────────────────────────────────────────────────────────
    builder.add_handler(TimerHandler())
    builder.add_handler(MobilityHandler())
    builder.add_handler(
        CommunicationHandler(CommunicationMedium(transmission_range=COMM_RANGE))
    )
    if not args.no_viz:
        builder.add_handler(VisualizationHandler(viz_config))

    # ── Run ────────────────────────────────────────────────────────────────
    simulation = builder.build()
    simulation.start_simulation()

    logging.info("Simulation complete.")


if __name__ == "__main__":
    main()