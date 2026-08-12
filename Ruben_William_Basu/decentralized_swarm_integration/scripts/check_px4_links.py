#!/usr/bin/env python3
"""Read-only heartbeat and local-position preflight for multiple PX4 links."""

from __future__ import annotations

import argparse
import json
import sys
import time


def parse_links(values: list[str]) -> list[tuple[str, str]]:
    links = []
    identities = set()
    endpoints = set()
    for value in values:
        try:
            uav_id, endpoint = value.split("=", 1)
        except ValueError as exc:
            raise argparse.ArgumentTypeError(
                f"link must be UAV=ENDPOINT, received {value!r}"
            ) from exc
        uav_id, endpoint = uav_id.strip(), endpoint.strip()
        if not uav_id or not endpoint:
            raise argparse.ArgumentTypeError("UAV and ENDPOINT must be non-empty")
        if uav_id in identities or endpoint in endpoints:
            raise argparse.ArgumentTypeError("UAV identities and endpoints must be unique")
        identities.add(uav_id)
        endpoints.add(endpoint)
        links.append((uav_id, endpoint))
    if not links:
        raise argparse.ArgumentTypeError("at least one PX4 link is required")
    return links


def inspect_link(mavutil, uav_id: str, endpoint: str, timeout_s: float) -> dict:
    connection = mavutil.mavlink_connection(endpoint, source_system=250)
    started = time.monotonic()
    try:
        heartbeat = connection.wait_heartbeat(timeout=timeout_s)
        if heartbeat is None:
            raise TimeoutError("heartbeat timeout")
        remaining = max(0.1, timeout_s - (time.monotonic() - started))
        position = connection.recv_match(
            type="LOCAL_POSITION_NED", blocking=True, timeout=remaining
        )
        if position is None:
            raise TimeoutError("LOCAL_POSITION_NED timeout")
        return {
            "uav_id": uav_id,
            "endpoint": endpoint,
            "ok": True,
            "system_id": int(connection.target_system),
            "component_id": int(connection.target_component),
            "local_ned_m": {
                "north": float(position.x),
                "east": float(position.y),
                "down": float(position.z),
            },
        }
    except Exception as exc:  # report every link instead of stopping at the first
        return {
            "uav_id": uav_id,
            "endpoint": endpoint,
            "ok": False,
            "error": f"{type(exc).__name__}: {exc}",
        }
    finally:
        connection.close()


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "links",
        nargs="+",
        metavar="UAV=ENDPOINT",
        help="for example dji0=udpin:0.0.0.0:14540",
    )
    parser.add_argument("--timeout", type=float, default=8.0)
    args = parser.parse_args(argv)
    if args.timeout <= 0:
        parser.error("--timeout must be positive")
    try:
        links = parse_links(args.links)
    except argparse.ArgumentTypeError as exc:
        parser.error(str(exc))
    try:
        from pymavlink import mavutil
    except ModuleNotFoundError:
        print(json.dumps({"ok": False, "error": "pymavlink is not installed"}))
        return 2
    results = [inspect_link(mavutil, uav, endpoint, args.timeout) for uav, endpoint in links]
    system_ids = [item.get("system_id") for item in results if item["ok"]]
    unique_systems = len(system_ids) == len(set(system_ids))
    report = {
        "protocol": "eirax.px4_link_preflight.v1",
        "ok": all(item["ok"] for item in results) and unique_systems,
        "unique_system_ids": unique_systems,
        "links": results,
    }
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    sys.exit(main())
