"""Pure semantic-observation protocol and consensus logic."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import json
import math
from typing import Iterable


@dataclass(frozen=True)
class SemanticObservation:
    source_id: str
    observation_id: str
    stamp_ns: int
    received_ns: int
    class_id: int | None
    class_name: str
    confidence: float
    track_id: int | None
    track_state: str
    x: float | None
    y: float | None
    z: float | None
    frame_id: str
    geometry_valid: bool
    status: str = ""

    @property
    def semantic_key(self) -> str:
        if self.class_name:
            return self.class_name.lower()
        return f"class:{self.class_id}" if self.class_id is not None else "unknown"


@dataclass(frozen=True)
class ConsensusResult:
    accepted: bool
    reason: str
    semantic_key: str
    source_ids: tuple[str, ...]
    confidence: float
    x: float | None = None
    y: float | None = None
    z: float | None = None
    spread_m: float | None = None


def encode_observation(observation: SemanticObservation) -> str:
    payload = asdict(observation)
    payload["protocol"] = "eirax.semantic_observation.v1"
    return json.dumps(payload, separators=(",", ":"), sort_keys=True)


def decode_observation(data: str, received_ns: int) -> SemanticObservation:
    payload = json.loads(data)
    if payload.get("protocol") != "eirax.semantic_observation.v1":
        raise ValueError("unsupported semantic observation protocol")
    confidence = float(payload["confidence"])
    if not 0.0 <= confidence <= 1.0:
        raise ValueError("confidence must be within [0, 1]")
    source_id = str(payload["source_id"]).strip()
    if not source_id:
        raise ValueError("source_id must not be empty")
    geometry_valid = bool(payload.get("geometry_valid", False))
    coordinates = tuple(payload.get(axis) for axis in ("x", "y", "z"))
    if geometry_valid and any(value is None or not math.isfinite(float(value)) for value in coordinates):
        raise ValueError("valid geometry requires finite x, y, and z")
    return SemanticObservation(
        source_id=source_id,
        observation_id=str(payload["observation_id"]),
        stamp_ns=int(payload["stamp_ns"]),
        received_ns=int(received_ns),
        class_id=None if payload.get("class_id") is None else int(payload["class_id"]),
        class_name=str(payload.get("class_name", "")),
        confidence=confidence,
        track_id=None if payload.get("track_id") is None else int(payload["track_id"]),
        track_state=str(payload.get("track_state", "raw")),
        x=None if payload.get("x") is None else float(payload["x"]),
        y=None if payload.get("y") is None else float(payload["y"]),
        z=None if payload.get("z") is None else float(payload["z"]),
        frame_id=str(payload.get("frame_id", "")),
        geometry_valid=geometry_valid,
        status=str(payload.get("status", "")),
    )


def reach_consensus(
    observations: Iterable[SemanticObservation],
    *,
    now_ns: int,
    max_age_s: float,
    min_confidence: float,
    min_independent_sources: int,
    max_spread_m: float,
    allow_single_source: bool,
    single_source_confidence: float,
) -> ConsensusResult:
    max_age_ns = int(max_age_s * 1_000_000_000)
    eligible = [
        item
        for item in observations
        if item.geometry_valid
        and item.confidence >= min_confidence
        and 0 <= now_ns - item.received_ns <= max_age_ns
    ]
    if not eligible:
        return ConsensusResult(False, "no_fresh_metric_evidence", "", (), 0.0)

    groups: dict[tuple[str, str], list[SemanticObservation]] = {}
    for item in eligible:
        groups.setdefault((item.semantic_key, item.frame_id), []).append(item)

    best: ConsensusResult | None = None
    for (semantic_key, _frame_id), group in groups.items():
        # Retain only the newest evidence from each independent vehicle.
        newest = {}
        for item in group:
            previous = newest.get(item.source_id)
            if previous is None or item.received_ns > previous.received_ns:
                newest[item.source_id] = item
        values = list(newest.values())
        weight_sum = sum(item.confidence for item in values)
        x = sum(item.x * item.confidence for item in values) / weight_sum
        y = sum(item.y * item.confidence for item in values) / weight_sum
        z = sum(item.z * item.confidence for item in values) / weight_sum
        spread = max(math.dist((item.x, item.y, item.z), (x, y, z)) for item in values)
        confidence = 1.0 - math.prod(1.0 - item.confidence for item in values)
        enough_sources = len(values) >= min_independent_sources
        exceptional_single = (
            allow_single_source
            and len(values) == 1
            and values[0].confidence >= single_source_confidence
        )
        spatially_consistent = spread <= max_spread_m
        accepted = spatially_consistent and (enough_sources or exceptional_single)
        reason = (
            "accepted"
            if accepted
            else "spatial_disagreement"
            if not spatially_consistent
            else "insufficient_independent_sources"
        )
        candidate = ConsensusResult(
            accepted,
            reason,
            semantic_key,
            tuple(sorted(newest)),
            confidence,
            x,
            y,
            z,
            spread,
        )
        if best is None or (candidate.accepted, candidate.confidence) > (best.accepted, best.confidence):
            best = candidate
    assert best is not None
    return best
