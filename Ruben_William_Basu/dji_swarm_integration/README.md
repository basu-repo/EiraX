# DJI Swarm Integration

This is an isolated DJI-oriented integration workspace. It contains physical
copies of the required source and simulation assets; it does not use symbolic
links and does not modify or import from `../decentralized_swarm_integration` at
runtime.

## Layout

- `simulation/`: copied Baylands terrain, obstacles, DJI M100 and Husky models,
  plus the editable worlds used by the tested integration.
- `ground_stack/`: copied known-good Husky/UGV stack.
- `vehicle_stack/`: copied vehicle orchestration and navigation foundation. PX4
  modules remain reference code only until the DJI control adapter replaces
  them.
- `dji_stack/`: updated Halmstad ROS 2 package, including DJI M100 simulation,
  camera/gimbal, YOLO, follow logic and PSDK-compatible command topics.
- `omnet_team/`: UAV_UGV-main OMNeT++/INET mobility and network implementation.
- `docs/halmstad/`: upstream design, operating and handoff notes.
- `config/halmstad/` and `scripts/halmstad/`: upstream workspace-level support
  configuration and operator scripts.

## Source provenance

| Destination | Copied from |
| --- | --- |
| `simulation/` | `../decentralized_swarm_integration/simulation/` |
| `ground_stack/` | `../decentralized_swarm_integration/ground_stack/` |
| `vehicle_stack/` | `../decentralized_swarm_integration/vehicle_stack/` |
| `dji_stack/` | `../halmstad_ws-main_new/halmstad_ws-main/src/lrs_halmstad/` |
| `omnet_team/` | `../UAV_UGV-main/` |

## Integration boundary

The existing PX4 simulation remains the tested baseline and is intentionally
untouched. The DJI implementation will preserve the high-level decentralized
roles, YOLO observations, OMNeT++ link state and UGV/Nav2 fallback while using a
separate DJI adapter for flight authority, telemetry, position/yaw setpoints,
gimbal control, takeoff and landing.

The Halmstad code exposes the PSDK-compatible position/yaw command topic:

```text
/<uav>/psdk_ros2/flight_control_setpoint_ENUposition_yaw
```

Physical-aircraft actuation must eventually be protected by an explicit
hardware mode and preflight checks. No physical DJI command is enabled merely
by this initial copy.

## Current implementation status

The DJI-native ROS package now includes:

- a validated decentralized DJI command protocol;
- a simulation/hardware boundary publishing the tested absolute-ENU PSDK Joy
  contract;
- stale-pose and stale-command holds, sequence rejection, altitude and command
  step limits;
- copied decentralized semantic consensus, role assignment, mission geometry,
  OMNeT pose/metrics bridges and deterministic failure/reconnect state;
- a three-M100 spawn/pose/gimbal/agent launch file for `baylands_editable`.

Hardware output is deliberately locked unless `mode:=hardware` and
`hardware_authorized:=true` are both supplied. The default is simulation.

## Build and run

Build once:

```bash
bash scripts/build_all.sh
```

Run the complete normal scenario:

```bash
python3 scripts/run_everything.py
```

Failure scenarios use the same flags as the PX4 baseline:

```bash
python3 scripts/run_everything.py --permanent-failure
python3 scripts/run_everything.py --connection-failure-reconnect
```

The launcher starts Baylands, the known-good Husky/Nav2 route, three DJI M100
aircraft, DJI PSDK-compatible control adapters, independent camera/YOLO/3D
observers, decentralized semantic and role peers, and the OMNeT++ Wi-Fi
overlay. In GUI mode, click Play after all models load. Press Ctrl+C once in
the launcher terminal for coordinated shutdown.

Live camera topics:

```text
/dji0/camera0/image_raw
/dji1/camera0/image_raw
/dji2/camera0/image_raw
```
