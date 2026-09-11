# ---------------------------------------------------------------------------
# Message schemas
# ---------------------------------------------------------------------------
from typing import Dict, List, Tuple, TypedDict

class SimpleMessage(TypedDict):
    """Basic message type used for template classes"""
    packets:       Dict[str, int]   # sensor_name → packet count
    sender_type:   int
    sender_id:     int
    lamport:       int              # sender's Lamport time at send


class FloodingMessage(TypedDict):
    """TTL-limited broadcast message."""
    msg_type: str
    message_id: str
    sender_id: int
    origin_id: int
    payload: Dict[str, int]
    ttl: int


class DTNMessage(TypedDict):
    """Epidemic message retained until another UAV receives it."""
    msg_type: str
    message_id: str
    sender_id: int
    origin_id: int
    payload: Dict[str, int]
    copies: int


class GeocastingMessage(TypedDict):
    """Greedy geographic message with a Store-and-Wait fallback."""
    msg_type: str
    message_id: str
    sender_id: int
    origin_id: int
    payload: Dict[str, int]
    destination: Tuple[float, float, float]
    radius: float
    waiting: bool
    