# UAV Routing Simulation — Core + Satellite Topology

This GrADySim simulation collects sensor data using a core of 9 UAVs
and **4 satellites** shuttling data between the core and the sensor field.

The core can use one of two topologies (`--topology`):

- `random` (default): the 9 core UAVs move randomly inside the obstacle area.
- `star`: the 9 core UAVs form a rotating hub + 8 rim wheel (the original design).

Each run selects one routing protocol globally:

- `flooding`: broadcasts each routing message once within the core with a strict TTL (8).
- `dtn`: Epidemic Routing (Vahdat & Becker, 2000). Nodes conduct anti-entropy sessions with peers to exchange missing messages.
- `geocasting`: GPSR-based geocasting. Greedy geographic forwarding toward the destination dock point, falling back to Perimeter Mode (right-hand rule on a Relative Neighborhood Graph) at local minima, and flooding inside the region (`GEOCAST_REGION_RADIUS`).

## Architecture

### Star topology (`--topology star`)

```
                    Sat-N (shuttles N axis)
                       |
Sat-W ── [Rim0..7 rotating @ 6°/s around Hub] ── Sat-E
                       |
                    Sat-S (shuttles S axis)
```

### Random topology (`--topology random`, default)

The 9 core UAVs are identical. Each flies random waypoints (gradysim
`RandomMobilityPlugin`) inside the obstacle zone square at fixed `FLIGHT_ALT`.
Start positions and waypoints are seeded (`RANDOM_SEED`). Satellites keep the same
state machine and docking handshake as in the star topology.

### Rules shared by both topologies

- **Speed:** all moving UAVs (satellites, rim, random core nodes) fly at a shared `UAV_SPEED = 5.0 m/s` (except the hub which is stationary).
- **Dock alignment:** a core UAV counts as aligned with a dock point when it is within
  `DOCK_RANGE` (= `COMM_RANGE`, 2-D distance) of it. This applies to hub, rim and random UAVs.

### Core, star topology (9 UAVs)
- **Hub** (1): stationary at obstacle center, acts as follow-mobility leader.
  Broadcasts its position every `CORE_BROADCAST_INTERVAL = 0.1 s`.
- **Rim** (8): orbit the hub at `CORE_RADIUS = 29.25 m`, 45° apart.
  Follow-mobility followers; each updates its relative offset every `CORE_SPIN_DT = 0.05 s`.
  Formation rotates counter-clockwise at `ROT_RATE_DEG = 6°/s` (one full rotation per 60 s).
  Rim nodes near a dock point (within `DOCK_RANGE`) can serve that dock.

### Satellites (4 UAVs — N, E, S, W)
Each satellite follows a 4-state cycle:
1. **COLLECT_OUT** – fly to outer turnaround point (`SAT_OUTER_DIST = 90 m`), collecting sensor data.
2. **COLLECT_IN** – fly back to its dock point (`DOCK_RADIUS = 34.25 m` from center).
3. **WAIT_ALIGN** – hover at dock, broadcast `dock_hello` every 0.25 s until an aligned core node replies.
4. **EXCHANGE** – upload packet bundle to the aligned core node; it floods/forwards to the opposite dock;
   destination satellite receives `dock_deliver`. Timeout = 1 s.

A satellite's **opposite face** receives all messages: N↔S, E↔W.

## Key Parameters

| Parameter | Value | Notes |
|---|---|---|
| `COMM_RANGE` | 30.0 m | Radio range |
| `CORE_RADIUS` | 29.25 m | Hub-to-rim (< 30 m) |
| `CORE_RIM_NODES` | 8 | 45° spacing |
| `ROT_RATE_DEG` | 6.0 °/s | CCW, 60 s/revolution |
| `DOCK_RADIUS` | 34.25 m | Face dock point distance |
| `DOCK_RANGE` | 30.0 m | Distance tolerance for dock alignment (all topologies) = `COMM_RANGE` |
| `NUM_RANDOM_UAVS` | 9 | Core UAVs in the random topology |
| `RANDOM_SEED` | 2056 | Seed for random starts and waypoints |
| `SAT_OUTER_DIST` | 90.0 m | Turnaround distance |
| `SIMULATION_DURATION` | 300 s | ~13 satellite cycles |
| `UAV_SPEED` | 5.0 m/s | Movement speed for all moving UAVs |
| `DTN_SESSION_COOLDOWN` | 3.0 s | Cooldown between DTN epidemic sessions |
| `DTN_MAX_HOPS` | 6 | Max replications for a message in Epidemic Routing |
| `GEOCAST_REGION_RADIUS` | 30.0 m | Radius for geocast region flooding (= `DOCK_RANGE`) |

Derived values logged at startup:
- Rim adjacent chord ≈ 22.36 m (< 30 m ✓)
- Tangential speed ≈ 3.08 m/s (star)
- Alignment period = 7.5 s (star)

## Setup

```bash
python -m venv venv
# Windows
.\venv\Scripts\activate
# Linux / macOS
source venv/bin/activate

pip install -r requirements.txt
```

## Running

Always run from the **project root** using the `-m` flag:

```bash
python -m src.main
```

### CLI flags

| Flag | Description |
|------|-------------|
| `--no-viz` | Disable the browser visualisation |
| `--force-regen` | Force regeneration of sensor positions |
| `--routing-strategy NAME` | Routing protocol: `flooding`, `dtn`, or `geocasting` (default: `flooding`) |
| `--topology NAME` | Core topology: `random` or `star` (default: `random`) |

### Example runs

```bash
# No visualisation, flooding:
python -m src.main --no-viz --routing-strategy flooding

# DTN routing:
python -m src.main --no-viz --routing-strategy dtn

# Geocasting with viz:
python -m src.main --routing-strategy geocasting

# Star (rotating wheel) topology:
python -m src.main --no-viz --topology star

# Random topology, DTN:
python -m src.main --no-viz --topology random --routing-strategy dtn
```

## Sensors

Sensors are placed in a **cross-shaped corridor** covering both cardinal axes:
- `|x| ≤ 25 m` (N-S corridor) OR `|y| ≤ 25 m` (E-W corridor)
- Outside the obstacle zone, within 90 m of center

This ensures each satellite sweeps past sensors on its outbound leg.

## Metrics (end of simulation)

At the end of each run, the following is logged:
- Total messages uploaded / delivered / delivery ratio
- Latency mean/median/max
- Mean hops per message
- Per route-pair breakdown (N→S, S→N, E→W, W→E)
- Per-satellite: dock count, mean wait time for alignment, mean cycle time

### Tracking hops

The hop counter is incremented on every radio transmission:
- **Hop 1**: satellite → rim node (dock upload)
- **Hop 2**: rim node → destination satellite (dock deliver)

With flooding/dtn, intermediate forwarding adds extra hops.
Expected hops = 2 for direct delivery (sender rim aligned + dest rim already aligned).

## Visualisation

The visualisation connects to the GrADySim web viewer at  
<https://project-gradys.github.io/gradys-sim-nextgen-visualization/>

Node colours:
- 🟠 Orange: Hub (star only)
- 🟡 Amber: Rim nodes (star) / random core UAVs (random)
- 🔵 Cyan: Satellites (4)
- 🟣 Violet: Sensors

> **Note (Windows):** The visualization handler spawns a subprocess for the WebSocket server.
> Any top-level code that should run only once must be inside `if __name__ == "__main__":`.

## Changes from Previous Version

- Replaced 8 patrol-loop UAVs with **13 UAVs** (1 hub + 8 rim + 4 satellites)
- Removed `FIXED_UAV_GROUPS`, track/waypoint constants, `ENDPOINT_GROUND`
- Added all core/satellite parameters to `config.py`; nothing hard-coded in protocols
- Routing mixins adapted to use `dest_face` (dock point) instead of `ENDPOINT_GROUND`
- Sensor corridor restricted to cross-shape matching satellite axes
- Full metrics table added (latency, hops, dock stats)
- `COMM_RANGE` reduced from 32 to 30 m; `SIMULATION_DURATION` reduced to 300 s
- Added `--topology {random,star}`; `random` (9 UAVs moving randomly in the obstacle area) is the new default, `star` is the previous formation
