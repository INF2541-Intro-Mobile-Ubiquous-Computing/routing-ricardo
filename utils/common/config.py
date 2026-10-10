"""
Configuration for the core + satellite UAV topology simulation.

All derived values are computed here; nothing is hard-coded in protocol files.
"""
import logging
import math

from utils.common.sensor_positions import (
    ForbiddenZone,
)

# ---------------------------------------------------------------------------
# Simulation
# ---------------------------------------------------------------------------
SIMULATION_DURATION = 300         # seconds — one satellite cycle ~22 s

# ---------------------------------------------------------------------------
# Communication
# ---------------------------------------------------------------------------
COMM_RANGE = 30.0                   # metres  (was 32)
ANNOUNCE_WINDOW = 3.0

# ---------------------------------------------------------------------------
# UAV counts
# ---------------------------------------------------------------------------
NUM_UAVS = 13                       # 1 hub + 8 rim + 4 satellites
MAX_BUFFER_SIZE = 100

# ---------------------------------------------------------------------------
# Flight
# ---------------------------------------------------------------------------
FLIGHT_ALT = 15.0
PASS_ALT_OFFSET = 5.0
UAV_SPEED = 5                       # m/s — movement speed shared by all moving UAVs

# ---------------------------------------------------------------------------
# Sensor
# ---------------------------------------------------------------------------
SENSOR_SEED = 2056
SENSOR_FIELD_X = (-100, 100)
SENSOR_FIELD_Y = (-100, 100)
SENSOR_PACKET_INTERVAL = 4.0
SENSOR_MIN_SPACING = COMM_RANGE
SENSOR_JITTER = (SENSOR_MIN_SPACING / 2) - 1
SENSOR_CORRIDOR_HALF_WIDTH = 25.0   # half-width of each cardinal-axis corridor

# ---------------------------------------------------------------------------
# Lamport / TTL
# ---------------------------------------------------------------------------
PACKET_TTL_TICKS = 100

# ---------------------------------------------------------------------------
# Core formation
# ---------------------------------------------------------------------------
CORE_DIAMETER = 58.5                # metres
CORE_RADIUS = CORE_DIAMETER / 2    # = 29.25 m  (< COMM_RANGE = 30)
CORE_RIM_NODES = 8                  # rim nodes, 45° apart

ROT_RATE_DEG = 6.0                  # deg/s counter-clockwise
CORE_INITIAL_ORIENTATION_DEG = 0.0

CORE_SPIN_DT = 0.05                 # seconds between orientation updates
CORE_BROADCAST_INTERVAL = 0.1      # leader position broadcast period

# ---------------------------------------------------------------------------
# Dock
# ---------------------------------------------------------------------------
DOCK_GAP = 5.0                      # metres from rim outer perimeter
DOCK_RADIUS = CORE_RADIUS + DOCK_GAP  # = 34.25 m

# ---------------------------------------------------------------------------
# Satellite
# ---------------------------------------------------------------------------
SAT_OUTER_DIST = 90.0               # distance from center to turnaround point
EXCHANGE_TIMEOUT = 1.0              # seconds
NEIGHBOR_TIMEOUT = 1.0              # seconds before a neighbor is considered stale

# ---------------------------------------------------------------------------
# Obstacle (dock points must lie on faces)
# ---------------------------------------------------------------------------
OBSTACLE_CENTER = (
    (SENSOR_FIELD_X[0] + SENSOR_FIELD_X[1]) / 2,
    (SENSOR_FIELD_Y[0] + SENSOR_FIELD_Y[1]) / 2,
)
OBSTACLE_HALF_W = DOCK_RADIUS       # = 34.25
OBSTACLE_HALF_H = DOCK_RADIUS       # = 34.25

OBSTACLE_ZONE = ForbiddenZone(
    x_min=OBSTACLE_CENTER[0] - OBSTACLE_HALF_W,
    x_max=OBSTACLE_CENTER[0] + OBSTACLE_HALF_W,
    y_min=OBSTACLE_CENTER[1] - OBSTACLE_HALF_H,
    y_max=OBSTACLE_CENTER[1] + OBSTACLE_HALF_H,
)

# ---------------------------------------------------------------------------
# Random topology (--topology random)
# ---------------------------------------------------------------------------
NUM_RANDOM_UAVS = 9                 # UAVs roaming inside the obstacle zone
RANDOM_SEED = 2056                  # seeds start positions and random waypoints
DOCK_RANGE = COMM_RANGE             # 'aligned' = within this 2-D distance of a dock point
RANDOM_X_RANGE = (OBSTACLE_ZONE.x_min, OBSTACLE_ZONE.x_max)
RANDOM_Y_RANGE = (OBSTACLE_ZONE.y_min, OBSTACLE_ZONE.y_max)

# ---------------------------------------------------------------------------
# Routing Protocol Parameters
# ---------------------------------------------------------------------------
DTN_SESSION_COOLDOWN = 3.0          # seconds between epidemic sessions
DTN_MAX_HOPS = 6                    # max copies/hops for DTN routing
GEOCAST_REGION_RADIUS = DOCK_RANGE  # region radius for geocasting

# ---------------------------------------------------------------------------
# Derived geometry — logged at startup
# ---------------------------------------------------------------------------
_RIM_CHORD_ADJ = 2 * CORE_RADIUS * math.sin(math.pi / CORE_RIM_NODES)   # adjacent-node chord
_TANGENTIAL_SPEED = 2 * math.pi * CORE_RADIUS * (ROT_RATE_DEG / 360)    # m/s
_ALIGN_PERIOD = 45.0 / ROT_RATE_DEG                                       # seconds between alignments

def _log_derived() -> None:
    logging.info("=== Core topology derived values ===")
    logging.info(f"  CORE_RADIUS         = {CORE_RADIUS:.4f} m  (< COMM_RANGE={COMM_RANGE})")
    logging.info(f"  Rim adj. chord      = {_RIM_CHORD_ADJ:.4f} m  (< COMM_RANGE={COMM_RANGE})")
    logging.info(f"  Tangential speed    = {_TANGENTIAL_SPEED:.4f} m/s")
    logging.info(f"  Alignment period    = {_ALIGN_PERIOD:.2f} s  (45°/{ROT_RATE_DEG}°/s)")
    logging.info(f"  DOCK_RADIUS         = {DOCK_RADIUS:.4f} m")

# ---------------------------------------------------------------------------
# Startup assertions
# ---------------------------------------------------------------------------
assert CORE_RADIUS < COMM_RANGE, (
    f"CORE_RADIUS ({CORE_RADIUS}) must be < COMM_RANGE ({COMM_RANGE})"
)
assert _RIM_CHORD_ADJ < COMM_RANGE, (
    f"Rim adjacent chord ({_RIM_CHORD_ADJ:.3f}) must be < COMM_RANGE ({COMM_RANGE})"
)

# ---------------------------------------------------------------------------
# Cardinal faces
# ---------------------------------------------------------------------------
# Compass bearing: N=0°, E=90°, S=180°, W=270°
# Unit vector from center = (sin(bearing_rad), cos(bearing_rad))

def _face_unit(bearing_deg: float):
    rad = math.radians(bearing_deg)
    return (math.sin(rad), math.cos(rad))

def _dock_point(bearing_deg: float):
    ux, uy = _face_unit(bearing_deg)
    cx, cy = OBSTACLE_CENTER
    return (cx + DOCK_RADIUS * ux, cy + DOCK_RADIUS * uy)

# Each entry: (name, bearing_deg, unit_vec, dock_point_2d, opposite_name)
FACES = [
    ("N", 0.0,   _face_unit(0.0),   _dock_point(0.0),   "S"),
    ("E", 90.0,  _face_unit(90.0),  _dock_point(90.0),  "W"),
    ("S", 180.0, _face_unit(180.0), _dock_point(180.0), "N"),
    ("W", 270.0, _face_unit(270.0), _dock_point(270.0), "E"),
]
FACE_NAMES = [f[0] for f in FACES]
FACE_BY_NAME = {f[0]: f for f in FACES}

def opposite_face(name: str) -> str:
    return FACE_BY_NAME[name][4]

def dock_point_of(name: str):
    """Returns the 2-D dock point of the named face."""
    return FACE_BY_NAME[name][3]

# ---------------------------------------------------------------------------
# Core start positions
# ---------------------------------------------------------------------------
cx, cy = OBSTACLE_CENTER

CORE_HUB_START = (cx, cy, FLIGHT_ALT)

def _rim_start(k: int):
    """XYZ start for rim node k (already at initial orientation 0)."""
    angle_rad = math.radians(CORE_INITIAL_ORIENTATION_DEG + k * (360.0 / CORE_RIM_NODES))
    return (
        cx + CORE_RADIUS * math.cos(angle_rad),
        cy + CORE_RADIUS * math.sin(angle_rad),
        FLIGHT_ALT,
    )

CORE_RIM_STARTS = [_rim_start(k) for k in range(CORE_RIM_NODES)]

def random_uav_starts(n: int = NUM_RANDOM_UAVS, seed: int = RANDOM_SEED):
    """Seeded random start positions inside the obstacle zone at FLIGHT_ALT."""
    from random import Random
    rng = Random(seed)
    return [
        (rng.uniform(*RANDOM_X_RANGE), rng.uniform(*RANDOM_Y_RANGE), FLIGHT_ALT)
        for _ in range(n)
    ]

# ---------------------------------------------------------------------------
# Satellite start positions (= dock points at FLIGHT_ALT)
# ---------------------------------------------------------------------------
def sat_start(face_name: str):
    dp = dock_point_of(face_name)
    return (dp[0], dp[1], FLIGHT_ALT)

# ---------------------------------------------------------------------------
# Sensor positions (cross-shaped corridor)
# ---------------------------------------------------------------------------
_obs_margin = 5.0

def generate_cross_sensor_positions(force_regen: bool = False):
    """
    Generate sensor positions restricted to a cross-shaped corridor:
      |x - cx| <= SENSOR_CORRIDOR_HALF_WIDTH  OR  |y - cy| <= SENSOR_CORRIDOR_HALF_WIDTH
    outside OBSTACLE_ZONE (+ margin) and within SAT_OUTER_DIST from center.
    """
    from utils.common.cache import load as cache_load, save as cache_save
    from math import sqrt
    from random import Random

    params = {
        "fn": "generate_cross_sensor_positions",
        "seed": SENSOR_SEED,
        "x_range": SENSOR_FIELD_X,
        "y_range": SENSOR_FIELD_Y,
        "min_spacing": SENSOR_MIN_SPACING,
        "corridor_half_width": SENSOR_CORRIDOR_HALF_WIDTH,
        "sat_outer_dist": SAT_OUTER_DIST,
        "obstacle_center": OBSTACLE_CENTER,
        "obstacle_hw": OBSTACLE_HALF_W,
        "obstacle_hh": OBSTACLE_HALF_H,
        "forbidden_margin": _obs_margin,
        "jitter": SENSOR_JITTER,
    }

    if not force_regen:
        cached = cache_load("cross_sensor_positions", params)
        if cached is not None:
            return {k: tuple(v) for k, v in cached.items()}

    rng = Random(SENSOR_SEED)
    row_spacing = SENSOR_MIN_SPACING * sqrt(3) / 2
    col_spacing = SENSOR_MIN_SPACING

    placed = {}
    idx = 0
    y = SENSOR_FIELD_Y[0]
    row = 0
    while y <= SENSOR_FIELD_Y[1]:
        x_offset = (col_spacing / 2) if (row % 2 == 1) else 0.0
        x = SENSOR_FIELD_X[0] + x_offset
        while x <= SENSOR_FIELD_X[1]:
            jx = rng.uniform(-SENSOR_JITTER, SENSOR_JITTER)
            jy = rng.uniform(-SENSOR_JITTER, SENSOR_JITTER)
            px, py = x + jx, y + jy

            # Within satellite outer distance
            dist_center = sqrt((px - cx) ** 2 + (py - cy) ** 2)
            if dist_center > SAT_OUTER_DIST:
                x += col_spacing
                continue

            # Must be in the cross corridor
            in_x_corridor = abs(px - cx) <= SENSOR_CORRIDOR_HALF_WIDTH
            in_y_corridor = abs(py - cy) <= SENSOR_CORRIDOR_HALF_WIDTH
            if not (in_x_corridor or in_y_corridor):
                x += col_spacing
                continue

            # Must not be in obstacle zone (+ margin)
            if OBSTACLE_ZONE.contains(px, py, margin=_obs_margin):
                x += col_spacing
                continue

            placed[f"sensor_{idx}"] = (px, py, 0.0)
            idx += 1
            x += col_spacing
        y += row_spacing
        row += 1

    cache_save("cross_sensor_positions", params, {k: list(v) for k, v in placed.items()})
    logging.info(f"Cross sensor positions generated: {len(placed)} sensors")
    return placed


SENSOR_POSITIONS = generate_cross_sensor_positions()

SENSORS_HOVER = {
    name: (pos[0], pos[1], FLIGHT_ALT)
    for name, pos in SENSOR_POSITIONS.items()
}

SENSOR_WAYPOINT_NAMES = list(SENSORS_HOVER.keys())
