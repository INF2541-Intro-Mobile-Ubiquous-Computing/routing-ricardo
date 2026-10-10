"""
Process-wide metrics collector for the core+satellite topology simulation.

Usage
-----
From satellites (sender side):
    from utils.metrics.stats import record_upload
    record_upload(msg_id, origin_face, dest_face, created_at, hops=1)

From satellites (receiver side):
    from utils.metrics.stats import record_delivery
    record_delivery(msg_id, delivered_at, hops)

At simulation end (called from any finish()):
    from utils.metrics.stats import print_summary
    print_summary()
"""
import logging
import statistics
from dataclasses import dataclass, field
from typing import Dict, List, Optional

# ---------------------------------------------------------------------------
# Sensor stats (kept from original — written by SensorProtocol.finish())
# ---------------------------------------------------------------------------
from typing import Dict as _Dict

SENSOR_STATS: _Dict[str, _Dict[str, int]] = {}
# sensor_name → {"produced": N, "expired": M, "undelivered": K}


# ---------------------------------------------------------------------------
# Message-level metrics
# ---------------------------------------------------------------------------

@dataclass
class MsgRecord:
    msg_id: str
    origin_face: str
    dest_face: str
    created_at: float
    delivered_at: Optional[float] = None
    hops: int = 0


# Singleton registry keyed by msg_id
_MESSAGES: Dict[str, MsgRecord] = {}

# Per-satellite dock records
@dataclass
class SatRecord:
    dock_count: int = 0
    wait_times: List[float] = field(default_factory=list)     # time from WAIT_ALIGN start to first ack
    cycle_times: List[float] = field(default_factory=list)    # time for a full OUT->IN->EXCHANGE cycle

_SATELLITES: Dict[str, SatRecord] = {}


def _sat(face: str) -> SatRecord:
    if face not in _SATELLITES:
        _SATELLITES[face] = SatRecord()
    return _SATELLITES[face]


def record_upload(msg_id: str, origin_face: str, dest_face: str,
                  created_at: float, hops: int = 1) -> None:
    """Called when a satellite uploads a packet bundle to a core node."""
    if msg_id not in _MESSAGES:
        _MESSAGES[msg_id] = MsgRecord(
            msg_id=msg_id,
            origin_face=origin_face,
            dest_face=dest_face,
            created_at=created_at,
            hops=hops,
        )


def record_delivery(msg_id: str, delivered_at: float, hops: int) -> None:
    """Called when the destination satellite receives dock_deliver for msg_id."""
    if msg_id in _MESSAGES:
        rec = _MESSAGES[msg_id]
        if rec.delivered_at is None:
            rec.delivered_at = delivered_at
            rec.hops = hops
    else:
        # Message was delivered but not uploaded — create a partial record
        _MESSAGES[msg_id] = MsgRecord(
            msg_id=msg_id,
            origin_face="?",
            dest_face="?",
            created_at=delivered_at,
            delivered_at=delivered_at,
            hops=hops,
        )


def record_dock(face: str, wait_time: float, cycle_time: Optional[float] = None) -> None:
    """Called after each dock exchange."""
    s = _sat(face)
    s.dock_count += 1
    s.wait_times.append(wait_time)
    if cycle_time is not None:
        s.cycle_times.append(cycle_time)


def _safe_mean(lst):
    return statistics.mean(lst) if lst else float("nan")

def _safe_median(lst):
    return statistics.median(lst) if lst else float("nan")

def _safe_max(lst):
    return max(lst) if lst else float("nan")


def print_summary() -> None:
    """Print the end-of-simulation metrics table."""
    if not _MESSAGES:
        logging.info("[Metrics] No messages recorded.")
        return

    delivered = [r for r in _MESSAGES.values() if r.delivered_at is not None]
    total = len(_MESSAGES)
    n_del = len(delivered)

    logging.info("=" * 60)
    logging.info("[Metrics] END-OF-SIMULATION SUMMARY")
    logging.info(f"  Total messages uploaded : {total}")
    logging.info(f"  Delivered               : {n_del}  ({100*n_del/max(total,1):.1f}%)")

    if delivered:
        latencies = [r.delivered_at - r.created_at for r in delivered]
        hop_counts = [r.hops for r in delivered]
        logging.info(f"  Latency  mean/median/max: "
                     f"{_safe_mean(latencies):.2f} / "
                     f"{_safe_median(latencies):.2f} / "
                     f"{_safe_max(latencies):.2f} s")
        logging.info(f"  Hops     mean/median/max: "
                     f"{_safe_mean(hop_counts):.1f} / "
                     f"{_safe_median(hop_counts):.1f} / "
                     f"{_safe_max(hop_counts):.0f}")

    # Per (origin -> dest) breakdown
    pairs: Dict[tuple, List[MsgRecord]] = {}
    for r in _MESSAGES.values():
        key = (r.origin_face, r.dest_face)
        pairs.setdefault(key, []).append(r)

    logging.info("  --- Per route-pair ---")
    for (orig, dest), recs in sorted(pairs.items()):
        dels = [r for r in recs if r.delivered_at is not None]
        ratio = len(dels) / len(recs) if recs else 0
        logging.info(f"    {orig}->{dest}: {len(dels)}/{len(recs)} delivered ({100*ratio:.0f}%)")

    # Per-satellite
    logging.info("  --- Per-satellite dock stats ---")
    for face, s in sorted(_SATELLITES.items()):
        logging.info(
            f"    Sat {face}: docks={s.dock_count}  "
            f"wait_mean={_safe_mean(s.wait_times):.2f}s  "
            f"cycle_mean={_safe_mean(s.cycle_times):.2f}s"
        )
    logging.info("=" * 60)