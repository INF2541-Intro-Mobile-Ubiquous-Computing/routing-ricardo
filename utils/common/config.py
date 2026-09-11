from utils.common.sensor_positions import (
    ForbiddenZone,
    generate_sensor_positions,
    compute_route_waypoints,
)

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
# Overall simulation configs
SIMULATION_DURATION   = 120

# UAV configs
NUM_UAVS = 8
MAX_BUFFER_SIZE = 100
FLIGHT_ALT      = 15.0
PASS_ALT_OFFSET = 5.0

# Communication configs
COMM_RANGE            = 32.0
D                   = COMM_RANGE
ANNOUNCE_WINDOW       = 3.0

# Sensor configs
SENSOR_SEED = 2056
SENSOR_FIELD_X = (-100, 100)
SENSOR_FIELD_Y = (-100, 100)
SENSOR_PACKET_INTERVAL = 4.0
SENSOR_MIN_SPACING  = COMM_RANGE
# must be <= SENSOR_MIN_SPACING / 2 to stay valid
SENSOR_JITTER       = (SENSOR_MIN_SPACING  / 2) - 1


# ── Lamport clock / packet TTL ──────────────────────────────────────────────
# Each sensor "gen" tick advances the sensor's own Lamport counter by 1.
# Packets undelivered for more than PACKET_TTL_TICKS are expired and counted.
# TODO Check if still needed
PACKET_TTL_TICKS = 100

# ── Positions ──────────────────────────────────────────────────────────────
BASE_GROUND  = (sum(SENSOR_FIELD_X)/2, SENSOR_FIELD_Y[0], 0)
BASE_HOVER   = (BASE_GROUND[0], BASE_GROUND[1], FLIGHT_ALT)

ENDPOINT_GROUND = (sum(SENSOR_FIELD_X)/2, SENSOR_FIELD_Y[1], 0)
ENDPOINT_HOVER = (ENDPOINT_GROUND[0], ENDPOINT_GROUND[1], FLIGHT_ALT)

OBSTACLE_CENTER   = (sum(SENSOR_FIELD_X)/2, sum(SENSOR_FIELD_Y)/2)
OBSTACLE_HALF_W   = (COMM_RANGE + 50) / 2   
OBSTACLE_HALF_H   = (COMM_RANGE + 50) / 2

OBSTACLE_ZONE = ForbiddenZone(
    x_min = OBSTACLE_CENTER[0] - OBSTACLE_HALF_W,
    x_max = OBSTACLE_CENTER[0] + OBSTACLE_HALF_W,
    y_min = OBSTACLE_CENTER[1] - OBSTACLE_HALF_H,
    y_max = OBSTACLE_CENTER[1] + OBSTACLE_HALF_H,
)

_margin = 10.0
_y_dist = ENDPOINT_GROUND[1] - BASE_GROUND[1]
_backtrack_start = max(0, OBSTACLE_HALF_H + _margin - abs(BASE_GROUND[1])) * 2
_backtrack_end = max(0, OBSTACLE_HALF_H + _margin - abs(ENDPOINT_GROUND[1])) * 2
_x_detour = 2 * (OBSTACLE_HALF_W + _margin)
_total_dist = _y_dist + _backtrack_start + _backtrack_end + _x_detour

UAV_SPEED = max(3.0, _total_dist / (SIMULATION_DURATION * 0.8))

SENSOR_POSITIONS = generate_sensor_positions(
    seed             = SENSOR_SEED,
    x_range          = SENSOR_FIELD_X,
    y_range          = SENSOR_FIELD_Y,
    min_spacing      = SENSOR_MIN_SPACING,
    forbidden_zones  = [OBSTACLE_ZONE],
    forbidden_margin = 5.0,
    jitter           = SENSOR_JITTER,
)

SENSORS_HOVER = {
    name: (pos[0], pos[1], FLIGHT_ALT)
    for name, pos in SENSOR_POSITIONS.items()
}

SENSOR_WAYPOINT_NAMES = list(SENSORS_HOVER.keys())

ALL_WAYPOINTS = {
    "base_ground":     BASE_GROUND,
    "base_hover":      BASE_HOVER,
    "endpoint_ground": ENDPOINT_GROUND,
    "endpoint_hover":  ENDPOINT_HOVER,
    **SENSORS_HOVER,
}

# ── Fixed three-group patrol layout ───────────────────────────────────────
# Each route is a closed local cycle. UAVs launch directly into their region
# and PathDecisionMixin advances cyclically without returning through base.
ROUTE_SPACING = COMM_RANGE
HORIZONTAL_TRACK_X_MIN = -2 * D
HORIZONTAL_TRACK_X_MAX = 2 * D
HORIZONTAL_TRACK_Y_MIN = ENDPOINT_GROUND[1] - D
HORIZONTAL_TRACK_Y_MAX = ENDPOINT_GROUND[1] - D / 4
PERPENDICULAR_TRACK_X_MIN = D
PERPENDICULAR_TRACK_X_MAX = D + D / 2
PERPENDICULAR_TRACK_Y_MIN = BASE_GROUND[1] + 2 * D
PERPENDICULAR_TRACK_Y_MAX = ENDPOINT_GROUND[1] - 2 * D
DIAGONAL_TRACK_X_MIN = -2 * D
DIAGONAL_TRACK_X_MAX = 2 * D
DIAGONAL_TRACK_Y_MIN = BASE_GROUND[1] + D / 2
DIAGONAL_TRACK_Y_MAX = BASE_GROUND[1] + 3 * D

HORIZONTAL_WAYPOINTS = compute_route_waypoints(
    [
        (HORIZONTAL_TRACK_X_MIN, HORIZONTAL_TRACK_Y_MIN),
        (HORIZONTAL_TRACK_X_MAX, HORIZONTAL_TRACK_Y_MIN),
        (HORIZONTAL_TRACK_X_MAX, HORIZONTAL_TRACK_Y_MAX),
        (HORIZONTAL_TRACK_X_MIN, HORIZONTAL_TRACK_Y_MAX),
        (HORIZONTAL_TRACK_X_MIN, HORIZONTAL_TRACK_Y_MIN),
    ],
    ROUTE_SPACING,
)

PERPENDICULAR_WAYPOINTS = compute_route_waypoints(
    [
        (PERPENDICULAR_TRACK_X_MIN + 20, PERPENDICULAR_TRACK_Y_MIN),
        (PERPENDICULAR_TRACK_X_MIN + 20, PERPENDICULAR_TRACK_Y_MAX),
        (PERPENDICULAR_TRACK_X_MAX + 20, PERPENDICULAR_TRACK_Y_MAX),
        (PERPENDICULAR_TRACK_X_MAX + 20, PERPENDICULAR_TRACK_Y_MIN),
        (PERPENDICULAR_TRACK_X_MIN + 20, PERPENDICULAR_TRACK_Y_MIN),
    ],
    ROUTE_SPACING,
)

PERPENDICULAR_RIGHT_WAYPOINTS = [
    (-2 * OBSTACLE_HALF_W + (x - 30), y, z)
    for x, y, z in PERPENDICULAR_WAYPOINTS
]

DIAGONAL_WAYPOINTS_R = compute_route_waypoints(
    [
        (DIAGONAL_TRACK_X_MIN, DIAGONAL_TRACK_Y_MIN),
        (0.0, DIAGONAL_TRACK_Y_MAX),
        (DIAGONAL_TRACK_X_MAX, DIAGONAL_TRACK_Y_MIN),
        (0.0, DIAGONAL_TRACK_Y_MIN),
        (DIAGONAL_TRACK_X_MIN, DIAGONAL_TRACK_Y_MIN),
    ],
    ROUTE_SPACING,
)

DIAGONAL_WAYPOINTS_L = compute_route_waypoints(
    [
        (DIAGONAL_TRACK_X_MAX, DIAGONAL_TRACK_Y_MIN),
        (0.0, DIAGONAL_TRACK_Y_MAX),
        (DIAGONAL_TRACK_X_MIN, DIAGONAL_TRACK_Y_MIN),
        (0.0, DIAGONAL_TRACK_Y_MIN),
        (DIAGONAL_TRACK_X_MAX, DIAGONAL_TRACK_Y_MIN),
    ],
    ROUTE_SPACING,
)

FIXED_UAV_GROUPS = (
    ("horizontal", 2, HORIZONTAL_WAYPOINTS, HORIZONTAL_WAYPOINTS[0]),
    ("perpendicular", 1, PERPENDICULAR_WAYPOINTS, PERPENDICULAR_WAYPOINTS[0]),
    ("perpendicular_right", 1, PERPENDICULAR_RIGHT_WAYPOINTS, PERPENDICULAR_RIGHT_WAYPOINTS[0]),
    ("diagonal_R", 4, DIAGONAL_WAYPOINTS_R, DIAGONAL_WAYPOINTS_R[0]),
    ("diagonal_L", 4, DIAGONAL_WAYPOINTS_L, DIAGONAL_WAYPOINTS_L[0])
)

