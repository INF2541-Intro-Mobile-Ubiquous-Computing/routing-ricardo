# UAV Routing Simulation — Core + Satellite Topology

This GrADySim simulation collects sensor data using a **rotating core** of 9 UAVs
and **4 satellites** shuttling data between the core and the sensor field.

Each run selects one routing protocol globally:

- `flooding`: broadcasts each routing message once within the core with a strict TTL (8).
- `dtn`: bounded epidemic forwarding and store-and-carry between core nodes.
- `geocasting`: greedy geographic forwarding toward the destination dock point, buffering if stuck.

## Architecture

```
                    Sat-N (shuttles N axis)
                       |
Sat-W ── [Rim0..7 rotating @ 6°/s around Hub] ── Sat-E
                       |
                    Sat-S (shuttles S axis)
```

### Core (9 UAVs)
- **Hub** (1): stationary at obstacle center, acts as follow-mobility leader.
  Broadcasts its position every `CORE_BROADCAST_INTERVAL = 0.1 s`.
- **Rim** (8): orbit the hub at `CORE_RADIUS = 29.25 m`, 45° apart.
  Follow-mobility followers; each updates its relative offset every `CORE_SPIN_DT = 0.05 s`.
  Formation rotates counter-clockwise at `ROT_RATE_DEG = 6°/s` (one full rotation per 60 s).
  All 4 dock points align simultaneously every **7.5 s** (45°/6°/s).

### Satellites (4 UAVs — N, E, S, W)
Each satellite follows a 4-state cycle:
1. **COLLECT_OUT** – fly to outer turnaround point (`SAT_OUTER_DIST = 90 m`), collecting sensor data.
2. **COLLECT_IN** – fly back to its dock point (`DOCK_RADIUS = 34.25 m` from center).
3. **WAIT_ALIGN** – hover at dock, broadcast `dock_hello` every 0.25 s until an aligned rim node replies.
4. **EXCHANGE** – upload packet bundle to the rim node; rim floods/forwards to opposite dock;
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
| `ALIGN_EPS` | 3.0 m | Arc tolerance for alignment |
| `SAT_SPEED` | 5.0 m/s | Satellite constant speed |
| `SAT_OUTER_DIST` | 90.0 m | Turnaround distance |
| `SIMULATION_DURATION` | 300 s | ~13 satellite cycles |

Derived values logged at startup:
- Rim adjacent chord ≈ 22.36 m (< 30 m ✓)
- Tangential speed ≈ 3.08 m/s
- Alignment period = 7.5 s

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

### Example runs

```bash
# No visualisation, flooding:
python -m src.main --no-viz --routing-strategy flooding

# DTN routing:
python -m src.main --no-viz --routing-strategy dtn

# Geocasting with viz:
python -m src.main --routing-strategy geocasting
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
- 🟠 Orange: Hub
- 🟡 Amber: Rim nodes (8)
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
