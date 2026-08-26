# EiraX UGV-UAV communication boundary

The combined mission now uses a deterministic distance-limited baseline before
Simu5G radio propagation or a security scenario is enabled. Normal operation
keeps the UAV 20 m ahead, warns at 40 m, disconnects beyond 50 m, and
reconnects inside 45 m. These values are editable in `link_config.yaml`.

## Live data flow

The communication node is the only relay for data transferred between the
vehicles:

| Direction | Flow | Transmit topic | Receive topic |
|---|---|---|---|
| UGV to UAV | GNSS | `/husky/gps` | `/communication/uav/rx/ugv_gps` |
| UGV to UAV | odometry | `/odom` | `/communication/uav/rx/ugv_odom` |
| UGV to UAV | Nav2 path | `/plan` | `/cooperative/ugv_global_path` |
| UAV to UGV | aerial obstacles | `/communication/uav/tx/aerial_obstacles` | `/cooperative/aerial_obstacles` |
| UAV to UGV | UAV odometry | `/uav/px4_odom` | `/communication/ugv/rx/uav_odom` |

Live link-health topics are:

| Topic | Meaning |
|---|---|
| `/communication/link/status` | Whether vehicle data is currently delivered |
| `/communication/link/distance_m` | Horizontal UGV-UAV separation |
| `/communication/link/quality` | Deterministic normalized range margin, 0–1 |
| `/communication/link/state` | `starting`, `connected`, `warning`, or `disconnected` |

The UAV progressively maps a configurable lead segment toward each waypoint.
It does not need to reach a distant waypoint before the UGV starts moving.
If separation reaches the warning boundary, the UAV commands a recovery
toward the last received UGV position.

The IP addresses and UDP ports in the logs are stable logical simulation
identifiers. They do not claim that ROS 2 is currently sending these messages
through physical UDP sockets. OMNeT++/Simu5G will later use the same flow
identity and replace the perfect delivery step between the transmit and
receive topics.

## Per-run output

Each combined dataset contains:

```text
communication/
├── channel_contract.json
├── communication_events.csv
├── flow_windows.csv
└── communication_summary.json
```

- `channel_contract.json` records the exact flows, range thresholds and radio
  metadata and confirms that artificial delay, corruption, and noise are
  disabled.
- `communication_events.csv` contains one row per transferred ROS message,
  including sequence, direction, serialized payload bytes, timestamps, relay
  latency, delivery, and link state.
- `flow_windows.csv` aggregates each flow into one-second windows and supplies
  the packet, byte, rate, throughput, latency, jitter, loss, session, and
  activity fields needed by the UAS co-simulation data bridge.
- `communication_summary.json` gives run-level totals and reserves radio and
  security fields for measured Simu5G results and scenario manifests.

The normalized quality topic describes remaining distance margin; it is not
radio signal strength in dBm. The channel deliberately does not fabricate
received signal strength, handover, authentication, attack, severity, or
incident-label values. Those fields must come from Simu5G or the controlled
security experiment, as required by the UAS co-simulation instructions.
