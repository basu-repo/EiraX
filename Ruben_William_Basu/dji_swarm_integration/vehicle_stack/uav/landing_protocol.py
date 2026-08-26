"""Pure landing-handshake policy shared by the PX4 follower and tests."""

from __future__ import annotations


DISARM_RETRY_SEC = 3.0
FORCE_DISARM_AFTER_SEC = 9.0
PX4_FORCE_DISARM_MAGIC = 21_196.0


def disarm_parameters(first_request_at: float | None, now: float) -> tuple[float, ...]:
    """Return normal or PX4 force-disarm parameters after verified touchdown.

    PX4 can acknowledge a normal disarm while its land detector is settling.
    The caller is responsible for enforcing bay-position, height and speed
    gates before using this result.
    """
    if first_request_at is None or now - first_request_at < FORCE_DISARM_AFTER_SEC:
        return (0.0,)
    return (0.0, PX4_FORCE_DISARM_MAGIC)
