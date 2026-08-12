from decentralized_swarm_integration.consensus import (
    SemanticObservation,
    decode_observation,
    encode_observation,
    reach_consensus,
)


NOW = 10_000_000_000


def observation(source, x, confidence=0.8, received_ns=NOW):
    return SemanticObservation(
        source_id=source,
        observation_id=f"{source}:1",
        stamp_ns=received_ns,
        received_ns=received_ns,
        class_id=0,
        class_name="hazard",
        confidence=confidence,
        track_id=1,
        track_state="tracked",
        x=x,
        y=2.0,
        z=0.0,
        frame_id="map",
        geometry_valid=True,
    )


def decide(items, **overrides):
    config = dict(
        now_ns=NOW,
        max_age_s=1.0,
        min_confidence=0.35,
        min_independent_sources=2,
        max_spread_m=3.0,
        allow_single_source=False,
        single_source_confidence=0.9,
    )
    config.update(overrides)
    return reach_consensus(items, **config)


def test_protocol_round_trip():
    original = observation("uav1", 1.0)
    decoded = decode_observation(encode_observation(original), NOW)
    assert decoded == original


def test_two_consistent_sources_are_accepted():
    result = decide([observation("uav1", 1.0), observation("uav2", 2.0)])
    assert result.accepted
    assert result.source_ids == ("uav1", "uav2")
    assert 1.0 < result.x < 2.0


def test_duplicate_source_does_not_fake_independence():
    result = decide([observation("uav1", 1.0), observation("uav1", 1.1)])
    assert not result.accepted
    assert result.reason == "insufficient_independent_sources"


def test_spatial_disagreement_is_rejected():
    result = decide([observation("uav1", 0.0), observation("uav2", 20.0)])
    assert not result.accepted
    assert result.reason == "spatial_disagreement"


def test_stale_evidence_is_rejected():
    stale = observation("uav1", 1.0, received_ns=NOW - 2_000_000_000)
    assert decide([stale]).reason == "no_fresh_metric_evidence"


def test_single_source_requires_explicit_policy_and_high_confidence():
    strong = observation("uav1", 1.0, confidence=0.95)
    assert not decide([strong]).accepted
    assert decide([strong], allow_single_source=True).accepted

