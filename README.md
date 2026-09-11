# TODO
Track the number of hops to reach destination

# UAV Routing Simulation

This GrADySim simulation collects sensor data with a moving UAV cluster. Each run selects one routing protocol globally:

- `flooding`: broadcasts each routing message once with a strict TTL.
- `dtn`: uses bounded epidemic forwarding and store-and-carry behavior.
- `geocasting`: greedily forwards toward `ENDPOINT_GROUND`; when no neighbor is closer, it buffers the message until movement reveals a better path.

Leader election has been removed. Sensor collection, fixed-route formation movement, and optional visualization remain available. The simulation always creates eight UAVs in groups of `2 + 2 + 4` and continuously patrols until `SIMULATION_DURATION`.

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

Always run from the **project root** using the `-m` flag so that `utils/` is on the path:

```bash
python -m src.main
```

### CLI flags

| Flag | Description |
|------|-------------|
| `--no-viz` | Disable the browser visualisation |
| `--force-regen` | Force regeneration of sensor positions and cluster waypoints |
| `--routing-strategy NAME` | Global routing protocol: `flooding`, `dtn`, or `geocasting` (default: `flooding`) |

The fixed movement layout contains two horizontal UAV tracks near the endpoint corridor, two parallel perpendicular tracks at communication-range distance `D`, and four diagonal tracks in the lower region. `D` is configured as `COMM_RANGE`.

The visualisation connects to the GrADySim web viewer at  
<https://project-gradys.github.io/gradys-sim-nextgen-visualization/>

> **Note (Windows):** The visualization handler spawns a subprocess for the WebSocket server.
> Any top-level code that should run only once must be inside `if __name__ == "__main__":`.
