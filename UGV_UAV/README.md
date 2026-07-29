# EiraX cooperative UGV and UAV

This directory is the development area for the combined mission. The proven
ground-vehicle implementation remains isolated in `UGV_Standalone/` and is
used without modifying its controller, local costmap, localization, mission
targets, or onboard collision protection.

The cooperative layer adds:

- PX4 UAV takeoff and following;
- UAV three-dimensional LiDAR and RTAB-Map mapping;
- UAV obstacle avoidance;
- a measured bidirectional UGV-UAV communication boundary;
- aerial obstacle projection into the UGV global costmap;
- UGV path sharing with the UAV;
- combined localization, mapping, path, costmap, and mission recording.

Shared resources remain at the project root:

- `simulation/` contains the saved Baylands world and models;
- `datasets/` contains timestamped results;
- `UGV_Standalone/` contains the protected UGV baseline.

## Combined runner

From the project root:

```bash
cd /home/basudeo/Documents/EiraX
./UGV_UAV/run_UGV_UAV_simulation.py
```

The combined mode is now the default. The first validation run can be limited
to the first waypoint:

```bash
./UGV_UAV/run_UGV_UAV_simulation.py --uav-test-waypoint-1
```

For automated headless validation without generating a large sensor bag:

```bash
./UGV_UAV/run_UGV_UAV_simulation.py \
  --headless --uav-test-waypoint-1 --no-recording
```

Normal runs still record all configured transferable sensor and localization
topics. `--no-recording` is intended only for repeated integration testing.
The communication log is small and remains enabled in both modes. Its
event-level, one-second flow, contract, and summary files are written under
`datasets/run_.../communication/`. See
[`communication/README.md`](communication/README.md) for the topic contract
and the future OMNeT++/Simu5G insertion point.

The first-waypoint headless integration test passed on 2026-07-27. It verified
UAV takeoff and corridor survey, aerial obstacle publication, activation of the
UGV controller/planner/navigator, UGV arrival at the dynamically read first
waypoint, UAV landing near the UGV, and clean process shutdown. The full
waypoint-1-to-goal mission also passed. After the UGV reaches the final goal,
the UAV now lands in the goal area instead of returning to its launch pad.

The standalone reference remains:

```bash
./UGV_Standalone/run_UGV_simulation.py
```
