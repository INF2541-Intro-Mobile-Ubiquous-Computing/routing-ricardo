"""
Simulation entry point — core + satellite topology.

Run with:
    python -m src.main
    python -m src.main --no-viz
    python -m src.main --routing-strategy geocasting
    python -m src.main --force-regen
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
    FLIGHT_ALT,
    SENSOR_POSITIONS,
    OBSTACLE_CENTER,
    CORE_RIM_NODES,
    CORE_HUB_START,
    CORE_RIM_STARTS,
    FACE_NAMES,
    NUM_UAVS,
    SAT_OUTER_DIST,
    SENSOR_FIELD_X,
    SENSOR_FIELD_Y,
    sat_start,
    _log_derived,
)

from src.sensor import Sensor
from src.uav import make_core_hub, make_core_rim, make_satellite


def main() -> None:
    parser = argparse.ArgumentParser(description="GrADySim core+satellite UAV simulation")
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
        help="Force regeneration of sensor positions",
    )
    parser.add_argument(
        "--routing-strategy",
        choices=("flooding", "dtn", "geocasting"),
        default="flooding",
        help="Global UAV routing protocol (applied to core nodes)",
    )
    args = parser.parse_args()

    # ── Simulation config ──────────────────────────────────────────────────
    config  = SimulationConfiguration(duration=SIMULATION_DURATION)
    builder = SimulationBuilder(config)

    _log_derived()
    logging.info(f"Obstacle center:   {OBSTACLE_CENTER}")
    logging.info(f"Sensors placed:    {len(SENSOR_POSITIONS)}")
    logging.info(f"UAVs:              {NUM_UAVS}  (1 hub + {CORE_RIM_NODES} rim + 4 satellites)")
    logging.info(f"Routing strategy:  {args.routing_strategy}")

    viz = not args.no_viz

    # ── Sensors (static nodes) ─────────────────────────────────────────────
    for sensor_pos in SENSOR_POSITIONS.values():
        builder.add_node(Sensor, sensor_pos)

    # ── Core hub ───────────────────────────────────────────────────────────
    HubClass = make_core_hub(routing=args.routing_strategy, viz=viz)
    logging.info(f"Hub start: {CORE_HUB_START}")
    builder.add_node(HubClass, CORE_HUB_START)

    # ── Core rim nodes 0..7 ────────────────────────────────────────────────
    for k in range(CORE_RIM_NODES):
        RimClass = make_core_rim(k=k, routing=args.routing_strategy, viz=viz)
        logging.info(f"Rim{k} start: {CORE_RIM_STARTS[k]}")
        builder.add_node(RimClass, CORE_RIM_STARTS[k])

    # ── Satellites N, E, S, W ─────────────────────────────────────────────
    for face in FACE_NAMES:
        SatClass = make_satellite(face=face, viz=viz)
        start = sat_start(face)
        logging.info(f"Sat-{face} start: {start}")
        builder.add_node(SatClass, start)

    # ── Scene bounds for visualisation ────────────────────────────────────
    if viz:
        padding = 20
        # Include the satellite outer turnaround points
        outer = SAT_OUTER_DIST + 10
        cx, cy = OBSTACLE_CENTER
        viz_config = VisualizationConfiguration(
            x_range=(cx - outer - padding, cx + outer + padding),
            y_range=(cy - outer - padding, cy + outer + padding),
            z_range=(0, FLIGHT_ALT + 20),
            open_browser=True,
        )

    # ── Handlers ───────────────────────────────────────────────────────────
    builder.add_handler(TimerHandler())
    builder.add_handler(MobilityHandler())
    builder.add_handler(
        CommunicationHandler(CommunicationMedium(transmission_range=COMM_RANGE))
    )
    if viz:
        builder.add_handler(VisualizationHandler(viz_config))

    # ── Run ────────────────────────────────────────────────────────────────
    simulation = builder.build()
    simulation.start_simulation()

    logging.info("Simulation complete.")


if __name__ == "__main__":
    main()