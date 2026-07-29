# EiraX cooperative UGV-UAV simulation

EiraX is a research simulation for autonomous cooperation between an Uncrewed
Ground Vehicle (UGV) and an Uncrewed Aerial Vehicle (UAV). The system combines
Gazebo, ROS 2, Nav2, RTAB-Map, PX4 Software-in-the-Loop, OMNeT++, INET, and
Simu5G.

The current implementation provides:

- a protected standalone Husky UGV navigation baseline;
- dynamic waypoint and goal positions read from the saved Gazebo world;
- three-dimensional Light Detection and Ranging (LiDAR) mapping;
- localization and ground-truth error recording;
- Nav2 path planning and local obstacle avoidance;
- a PX4-controlled UAV with three-dimensional LiDAR;
- progressive UAV surveying ahead of the UGV;
- UAV obstacle avoidance and safety climbing;
- aerial obstacle transfer into the UGV costmap;
- a distance-limited UGV-UAV communication channel;
- repeatable live delay, jitter, packet loss, outage, and recovery;
- trace-driven single-cell and multi-cell Simu5G experiments;
- a validated 44-column network and security experiment dataset.

Security attacks are not enabled yet. The current repository establishes and
validates the benign robotics and network baseline that security experiments
will use.

## 1. Repository boundary

The Git repository contains the source code, configuration, test code, the
prebuilt PX4 runtime, and reproducibility metadata.

The following large or externally maintained content is not stored in Git:

- `simulation/` contents: Baylands world and Gazebo model assets;
- all model, mesh, material, and texture directories, including PX4 models;
- `datasets/` contents: ROS bags, logs, maps, trajectories, and run outputs;
- `report/` contents: LaTeX sources, generated PDFs, and build files;
- `PX4-Autopilot/`: the full PX4 source repository;
- `halmstad_ws-main/`: supplied reference repository;
- `UAV_UGV-main/`: supplied reference repository;
- `UGV_UAV_5G_CoSimulation/`: local network co-simulation experiment;
- supplied UAS co-simulation documents;
- generated OMNeT++ and Simu5G results;
- the upstream Simu5G source checkout;
- local editor and agent state.

Git retains `.gitkeep` placeholders in `simulation/`, `datasets/`, and
`report/`, so the required mount-point directories exist after cloning.

## 2. Codebase layout

| Path | Responsibility |
|---|---|
| `UGV_Standalone/` | Proven standalone Husky navigation, mapping, localization, and recording |
| `UGV_UAV/` | Cooperative UAV extension around the protected UGV baseline |
| `UGV_UAV_5G_CoSimulation/` | Local network experiment; entire directory is excluded from Git |
| `simulation/` | External Baylands world and model assets |
| `datasets/` | Generated run datasets; never committed |
| `integration/` | Rules and location for future compatibility adapters |
| `report/` | Local mount point for externally maintained research reports |
| `reproducibility/` | Source-preservation inventory, manifests, snapshots, and change log |
| `tools/` | Repository and environment helper utilities |

The standalone UGV, combined UGV-UAV, and network experiment are deliberately
separate. This prevents experimental UAV or network changes from silently
changing the validated standalone UGV controller.

## 3. System architecture

```text
Gazebo Baylands world
        │
        ├── Husky sensors ──► ROS 2 bridge
        │                       │
        │                       ├── wheel odometry + IMU + GNSS
        │                       │           │
        │                       │           └── Extended Kalman Filters
        │                       │
        │                       ├── 3D LiDAR ──► RTAB-Map ──► /map
        │                       │                              │
        │                       └──────────────────────────► Nav2
        │                                                      │
        │                                                 /cmd_vel
        │
        └── PX4 x500 UAV
                │
                ├── PX4 flight controller and odometry
                ├── 3D LiDAR ──► aerial RTAB-Map
                ├── UAV Nav2 obstacle planner
                └── aerial obstacle projection
                              │
                    communication channel
                              │
                    UGV global costmap and path
```

During cooperative operation, the UAV surveys ahead and shares aerial
obstacles. The UGV still uses its own LiDAR, localization, Nav2 controller,
local costmap, and collision monitor. A communication outage removes aerial
assistance but does not remove local UGV autonomy.

## 4. Required environment

The verified development environment is:

| Component | Verified version |
|---|---|
| Operating system | Ubuntu 24.04 |
| Robot middleware | ROS 2 Jazzy |
| Simulator | Gazebo Harmonic |
| Ground navigation | Nav2 for ROS 2 Jazzy |
| Mapping | RTAB-Map |
| UAV controller | PX4 Software-in-the-Loop |
| Network simulator | OMNeT++ 6.0.1 |
| Network framework | INET 4.5.4 |
| 5G simulator | Simu5G 1.2.2 |

The code also requires the ROS packages used by Nav2, robot localization,
NavSat transform, RTAB-Map, Gazebo bridges, and the message types imported by
the Python nodes.

The included PX4 binary was built for this Linux and Gazebo environment. Its
Gazebo model assets are stored separately and are not committed. Keep the full
PX4 source separately if PX4 must be rebuilt or upgraded.

## 5. Preparing a fresh clone

### 5.1 Install Git LFS

The repository contains the prebuilt PX4 executable, so Git Large File Storage
must be available before cloning or staging:

```bash
sudo apt-get install git-lfs
git lfs install
```

### 5.2 Restore the simulation assets

Copy the separately stored simulation package into the empty directory. The
minimum expected structure is:

```text
simulation/
├── models/
│   ├── baylands/
│   ├── husky/
│   ├── goal_marker/
│   ├── waypoint_marker/
│   └── ...
└── worlds/
    └── baylands_editable.world
```

The saved world can contain absolute model paths from the computer where it
was edited. Relocate them after copying:

```bash
python3 tools/relocate_world.py
```

This changes only the `file://.../simulation/models/` prefix. It does not
change model names, poses, waypoints, the Husky spawn, or the goal.

Also restore the PX4 x500 and x500-base model packages under:

```text
UGV_UAV/px4_runtime/models/
```

Model, mesh, material, and texture assets under these paths remain local and
are intentionally ignored by Git.

### 5.3 Restore Simu5G

Install OMNeT++ 6.0.1 and INET 4.5.4, then restore the verified Simu5G release:

```bash
git clone --branch v1.2.2 --depth 1 \
  https://github.com/Unipisa/Simu5G.git \
  UGV_UAV_5G_CoSimulation/sim/simu5g/Simu5G

ln -s /absolute/path/to/inet \
  UGV_UAV_5G_CoSimulation/sim/simu5g/inet4.5
```

The verified Simu5G commit is:

```text
4625689909f9c13ae6461087098de5f5459c932a
```

### 5.4 Run the environment audit

```bash
source /opt/ros/jazzy/setup.bash
./UGV_UAV_5G_CoSimulation/run_5g_cosimulation.py --preflight
```

The audit checks the operating system, ROS distribution, OMNeT++, INET,
Simu5G, local cooperative baseline, schema, validator, reference dataset, stock
Simu5G result, and Python YAML support.

## 6. Standalone UGV walkthrough

Run:

```bash
cd /path/to/EiraX
source /opt/ros/jazzy/setup.bash
./UGV_Standalone/run_UGV_simulation.py
```

Gazebo opens paused. Wait for all models to load and click **Play**. The runner
allows up to 300 seconds for the required sensor topics.

### 6.1 Startup order

`UGV_Standalone/run_UGV_simulation.py` performs the following sequence:

1. Rejects startup if another Gazebo server is already running.
2. Reads `simulation/worlds/baylands_editable.world`.
3. Reads the Husky, waypoint, and goal poses dynamically.
4. Calculates a suitable Nav2 planning-area size and resolution from the
   longest mission leg.
5. Creates a new timestamped dataset directory.
6. Starts Gazebo and the ROS-Gazebo bridge.
7. Starts the monotonic clock adapter and sensor transforms.
8. Starts local and global Extended Kalman Filter localization.
9. Starts the NavSat transform for Global Navigation Satellite System data.
10. Starts three-dimensional LiDAR odometry and RTAB-Map.
11. Checks LiDAR, odometry, transforms, inertial data, GNSS, and map topics.
12. Starts localization-error recording and the ROS bag.
13. Generates a run-specific `nav2_params.yaml`.
14. Starts Nav2.
15. Sends the mission:
    `spawn → waypoint_1 → waypoint_2 → waypoint_3 → goal`.
16. Stops recording when the mission finishes.
17. Leaves visible Gazebo open for inspection until the user closes it.

### 6.2 Dynamic waypoint behavior

Mission positions are not hardcoded in Python. The function
`baseline/mission/world_poses.py:named_pose()` reads each named entity from the
saved world. `world_target_in_enu_odom()` converts it into the Husky
start-relative East-North-Up odometry frame.

Moving a waypoint or the goal in Gazebo therefore changes the next mission
after the world is saved. No mission-source edit is required.

The planning area is also dynamic:

- up to 100 m: `0.05 m` map resolution;
- 100–300 m: `0.10 m` resolution;
- above 300 m: `0.20 m` resolution.

### 6.3 Standalone command options

| Option | Effect |
|---|---|
| `--no-motion` | Starts sensing, localization, mapping, and Nav2 without sending a mission |
| `--headless` | Runs Gazebo without the graphical interface and starts simulation immediately |
| `--view-3d-slam` | Opens the RTAB-Map interface for the live three-dimensional map |
| `--return-to-spawn` | Adds the saved Husky spawn as the final mission target |

Examples:

```bash
./UGV_Standalone/run_UGV_simulation.py --view-3d-slam
./UGV_Standalone/run_UGV_simulation.py --return-to-spawn
./UGV_Standalone/run_UGV_simulation.py --headless --no-motion
```

### 6.4 Standalone source files

| File | Job |
|---|---|
| `run_UGV_simulation.py` | Small top-level orchestrator and health loop |
| `baseline/config/baseline.yaml` | Required and recorded ROS topics, timeouts, and world path |
| `baseline/config/nav2_config.py` | Generates the run-specific Nav2 configuration |
| `baseline/config/ekf_params.yaml` | Main wheel, inertial, and GNSS Extended Kalman Filter |
| `baseline/config/local_ekf_params.yaml` | Local wheel and inertial estimator |
| `baseline/config/navsat_params.yaml` | GNSS-to-local-frame conversion |
| `baseline/config/ugv_rtabmap_gui.ini` | RTAB-Map viewer layout |
| `baseline/core/process_manager.py` | Starts processes, redirects logs, reports failures, and stops process groups |
| `baseline/core/monotonic_clock.py` | Provides stable timing for recorded measurements |
| `baseline/launchers/commands.py` | Builds commands for Gazebo, bridges, localization, SLAM, Nav2, recording, and missions |
| `baseline/monitoring/topic_health.py` | Waits for required nodes, topics, messages, and finite odometry |
| `baseline/mission/world_poses.py` | Reads dynamic world poses and calculates the planning area |
| `baseline/mission/waypoint_mission.py` | Sends Nav2 goals, retries failed goals up to three times, and logs results |
| `baseline/data_logging/run_dataset.py` | Creates the timestamped dataset and event log |
| `baseline/evaluation/localization_recorder.py` | Records ground truth and estimator trajectories |
| `baseline/evaluation/analyze_localization.py` | Summarizes localization accuracy |

## 7. Cooperative UGV-UAV walkthrough

Run:

```bash
./UGV_UAV/run_UGV_UAV_simulation.py
```

The original UGV behavior remains in `UGV_Standalone/`. The cooperative runner
imports that baseline and adds the UAV, aerial mapping, communication relay,
and aerial guidance.

### 7.1 Cooperative mission sequence

1. A temporary cooperative world is generated from the saved Baylands world.
2. The PX4 x500 mapping UAV is inserted near the Husky.
3. Gazebo starts paused.
4. The UGV bridge, localization, three-dimensional SLAM, and Nav2 start.
5. PX4 Software-in-the-Loop and the UAV ROS bridge start.
6. The UAV LiDAR transform, LiDAR odometry, aerial RTAB-Map, and obstacle
   filter start.
7. The five-flow UGV-UAV communication relay starts.
8. The UAV takes off to the configured 15 m working altitude.
9. The UAV progressively surveys approximately 20 m toward waypoint 1.
10. When that forward corridor is ready, the existing UGV Nav2 mission is
    allowed to drive to waypoint 1.
11. The UAV follows while continuing to map and avoid obstacles.
12. The same process repeats for waypoint 2, waypoint 3, and the goal.
13. After the UGV reaches the goal, the UAV lands near the UGV instead of
    returning to its launch point.

The UAV does not need to reach a distant waypoint before releasing the UGV.
The survey and UGV movement overlap after the progressive lead is established.

### 7.2 UAV navigation and safety

`uav/follow_husky.py` manages the cooperative flight:

- reads the latest UGV pose received through the communication channel;
- establishes a progressive lead toward the next mission target;
- follows the UGV at the configured separation;
- respects the maximum communication range;
- requests obstacle-aware paths from the UAV Nav2 planner;
- holds horizontal position and climbs when LiDAR detects an unsafe flight
  corridor;
- records the UAV trajectory;
- lands and disarms near the UGV final position.

`uav/aerial_obstacle_filter.py` filters the UAV point cloud and publishes
ground-relevant aerial obstacles. These obstacles are transferred through the
communication channel and added to the UGV global costmap. The UGV local
costmap and collision monitor remain responsible for immediate ground-level
safety.

### 7.3 Reverse and recovery behavior

`config/cooperative_navigation.yaml` enables reverse trajectories as a Nav2
capability:

- maximum reverse speed: `0.25 m/s`;
- velocity samples: `30`.

This is a dynamic motion option for the local planner, not a hardcoded reverse
command. Nav2 selects forward, rotational, or reverse trajectories from the
live costmap.

### 7.4 Cooperative command options

| Option | Effect |
|---|---|
| `--no-motion` | Starts both sensing stacks and Nav2 without flying or driving |
| `--headless` | Runs Gazebo without its graphical interface |
| `--no-recording` | Integration-test mode without the large ROS bag or localization CSV |
| `--view-3d-slam` | Opens the UGV three-dimensional RTAB-Map viewer |
| `--return-to-spawn` | Adds the UGV spawn after the goal |
| `--ugv-only` | Runs only the UGV side through the cooperative orchestrator |
| `--uav-test-waypoint-1` | Limits the cooperative test to the first waypoint |

Recommended quick integration test:

```bash
./UGV_UAV/run_UGV_UAV_simulation.py \
  --headless \
  --uav-test-waypoint-1 \
  --no-recording
```

### 7.5 Cooperative source files

| File | Job |
|---|---|
| `run_UGV_UAV_simulation.py` | Starts and supervises the combined mission |
| `cooperative_nav2_config.py` | Adds aerial guidance to the generated UGV Nav2 configuration |
| `process_manager.py` | Loads the protected process manager from the UGV baseline |
| `config/cooperative_navigation.yaml` | Combined-mode physical motion limits |
| `communication/channel.py` | Relays and records the five logical vehicle flows |
| `communication/link_config.yaml` | 20 m lead, 40 m warning, 45 m reconnect, and 50 m maximum range |
| `uav/cooperative_world.py` | Builds a temporary world containing the UAV |
| `uav/follow_husky.py` | Takeoff, progressive survey, following, recovery, landing, and trajectory logging |
| `uav/aerial_obstacle_filter.py` | Converts the UAV cloud into ground-relevant obstacle information |
| `uav/nav2_config.py` | Generates the UAV obstacle-planner configuration |
| `uav/obstacle_aware_route.py` | Requests and follows obstacle-aware aerial paths |
| `uav/mapping_commands.py` | Builds UAV bridge, transform, odometry, RTAB-Map, viewer, and Nav2 commands |
| `uav/ugv_path_relay.py` | Relays the UGV planned path to the UAV side |
| `uav/gazebo_pose.py` | Reads model poses from Gazebo |
| `uav/x500_mission.py` | Basic PX4 command and local-setpoint helpers |
| `uav/x500_waypoint_mission.py` | Standalone UAV waypoint flight |
| `uav/aerial_mapping_runner.py` | Standalone UAV mapping experiment |
| `uav/analyze_waypoint.py` | UAV waypoint trajectory analysis and plotting |
| `px4_runtime/` | Prebuilt PX4 executable, plugins, Python modules, runtime root, and local mount point for excluded models |

The smaller `run_uav_simulation.py`, `run_uav_waypoint.py`, and
`run_uav_mapping.py` files are standalone UAV entry points. The combined
research mission should normally use `run_UGV_UAV_simulation.py`.

## 8. Communication layer

The communication node is the only cooperative relay for transferred vehicle
data.

| Direction | Logical flow | Transmit topic | Receive topic |
|---|---|---|---|
| UGV → UAV | GNSS | `/husky/gps` | `/communication/uav/rx/ugv_gps` |
| UGV → UAV | odometry | `/odom` | `/communication/uav/rx/ugv_odom` |
| UGV → UAV | Nav2 global path | `/plan` | `/cooperative/ugv_global_path` |
| UAV → UGV | aerial obstacles | `/communication/uav/tx/aerial_obstacles` | `/cooperative/aerial_obstacles` |
| UAV → UGV | UAV odometry | `/uav/px4_odom` | `/communication/ugv/rx/uav_odom` |

The logical Internet Protocol addresses and User Datagram Protocol ports in
the logs are stable simulation identifiers. They do not claim that ROS 2 is
using a physical radio or those sockets.

### 8.1 Link-health topics

| Topic | Meaning |
|---|---|
| `/communication/link/status` | Whether new cooperative messages can be delivered |
| `/communication/link/distance_m` | Horizontal UGV-UAV separation |
| `/communication/link/quality` | Normalized remaining range margin |
| `/communication/link/state` | Link state such as connected, degraded, warning, outage, or disconnected |

### 8.2 Behavior during disconnection

When communication is unavailable:

- the UGV continues with its own LiDAR, odometry, localization, Nav2, local
  costmap, and collision monitor;
- the UGV no longer receives new aerial obstacles or UAV odometry;
- the UAV uses local sensors and the last safely received UGV information;
- the UAV avoids increasing separation and attempts to restore the link;
- stale packets are not replayed after recovery;
- new data transfer resumes when the link becomes available.

The current policy is therefore:

```text
connected    → cooperative navigation
disconnected → independent local autonomy
reconnected  → cooperative exchange resumes
```

## 9. Live benign network profiles

The normal `UGV_UAV/` channel is the distance-limited baseline. The local,
Git-ignored `UGV_UAV_5G_CoSimulation/UGV_UAV/` experiment adds deterministic
live network profiles:

| Profile | One-way delay | Jitter | Random loss | Scheduled outage |
|---|---:|---:|---:|---:|
| `baseline` | 0 ms | 0 ms | 0% | none |
| `low` | 10 ms | 2 ms | 0.1% | none |
| `medium` | 35 ms | 10 ms | 1% | none |
| `high` | 80 ms | 25 ms | 3% | none |
| `recovery` | 35 ms | 10 ms | 1% | 60–70 seconds |

Run a visible recovery experiment:

```bash
./UGV_UAV_5G_CoSimulation/UGV_UAV/run_UGV_UAV_simulation.py \
  --network-profile recovery
```

The outage clock starts only after the operational mission is ready, not while
Gazebo models are loading.

`communication/network_model.py` validates profiles and provides reproducible
delay and loss sampling. `communication/channel.py` uses an asynchronous
delivery queue so network delay does not block ROS callbacks.

These live profiles are network emulation for closed-loop vehicle testing.
They are not described as measured Simu5G radio results.

## 10. Per-run dataset

Every runner creates:

```text
datasets/run_YYYYMMDD_HHMMSS/
├── metadata.json
├── events.jsonl
├── logs/
├── world/
├── config/
│   ├── nav2_params.yaml
│   └── uav_nav2_params.yaml
├── localization/
│   └── trajectory_and_errors.csv
├── communication/
│   ├── channel_contract.json
│   ├── communication_events.csv
│   ├── flow_windows.csv
│   ├── link_state_events.csv
│   └── communication_summary.json
├── cooperative/
├── rosbag/
├── rtabmap3d.db
├── uav_aerial_rtabmap.db
└── uav_follow_trajectory.csv
```

Some files exist only in the relevant mode. For example, `--no-recording`
omits the large ROS bag and localization CSV but keeps the small communication
evidence.

Important outputs:

| Output | Meaning |
|---|---|
| `events.jsonl` | Timestamped startup, health, goal, waypoint, and shutdown events |
| `logs/*.log` | Standard output and errors from each managed component |
| `trajectory_and_errors.csv` | Ground truth and estimator poses and errors |
| `uav_follow_trajectory.csv` | UAV flight and following trajectory |
| `communication_events.csv` | One row per transmitted cooperative message |
| `flow_windows.csv` | One-second flow aggregation for co-simulation |
| `communication_summary.json` | Delivery, loss, latency, distance, and per-flow totals |
| `link_state_events.csv` | Connection, degradation, outage, and recovery transitions |
| `rosbag/` | Transferable ROS sensor and navigation topics |
| `*.db` | RTAB-Map databases |

Images and Gazebo ground truth are not used as navigation inputs. Simulator
ground truth is recorded only for evaluation.

## 11. Localization data

The localization recorder compares:

- raw wheel odometry;
- wheel and inertial local estimation;
- wheel, inertial, and GNSS Extended Kalman Filter estimation;
- LiDAR odometry when enabled;
- RTAB-Map pose;
- simulator ground truth for evaluation only.

The main real-vehicle-transferable pose estimate is the Extended Kalman Filter
output on `/odom`. Latitude and longitude from GNSS are converted into a local
metric East-North-Up frame. Roll, pitch, yaw, and three-dimensional position
remain part of the vehicle pose even when ground navigation primarily uses
horizontal `x`, `y`, and yaw.

Analyze a completed standalone run with:

```bash
PYTHONPATH=UGV_Standalone python3 -m baseline.evaluation.analyze_localization \
  /absolute/path/to/EiraX/datasets/RUN_DIRECTORY
```

## 12. Nav2 configuration

`nav2_params.yaml` is generated separately inside every run. It is not a
manually copied static file.

The generated configuration records:

- the dynamic global planning area;
- resolution selected from the mission size;
- Husky footprint;
- controller speed and acceleration limits;
- local and global costmap sources;
- three-dimensional LiDAR obstacle integration;
- aerial obstacle integration in cooperative mode;
- recovery and reverse-motion capabilities;
- waypoint goal tolerances.

Keeping this file in each dataset makes the exact navigation conditions
reproducible even if later source configuration changes.

The UAV receives a separate `uav_nav2_params.yaml` under the `/uav_nav`
namespace. It is a three-dimensional-flight obstacle planner, not the Husky
controller.

## 13. Trace-driven Simu5G pipeline

The post-run network pipeline is under the local
`UGV_UAV_5G_CoSimulation/` directory. That complete directory is excluded from
Git and must be preserved or distributed separately before these commands can
be used on another computer.

### 13.1 Pipeline flow

```text
completed robotics dataset
        │
        ├── UGV and UAV measured trajectories
        │          └── BonnMotion mobility trace
        │
        ├── measured communication flows
        │          └── packetized Simu5G application traffic
        │
        └── shared mission clock
                   │
            OMNeT++ / INET / Simu5G
                   │
          scalar and vector measurements
                   │
       one-second time-window assembler
                   │
          validated 44-column dataset
```

### 13.2 Preflight

```bash
cd UGV_UAV_5G_CoSimulation
./run_5g_cosimulation.py --preflight
```

### 13.3 Process a two-vehicle run

Single-cell baseline:

```bash
./run_5g_cosimulation.py \
  --process-run /absolute/path/to/EiraX/datasets/RUN_DIRECTORY
```

Background-load experiment:

```bash
./run_5g_cosimulation.py \
  --process-run /absolute/path/to/EiraX/datasets/RUN_DIRECTORY \
  --network-profile high
```

Two-cell handover experiment:

```bash
./run_5g_cosimulation.py \
  --process-run /absolute/path/to/EiraX/datasets/RUN_DIRECTORY \
  --network-profile baseline \
  --radio-topology multi-cell
```

Valid post-run background profiles are:

| Profile | Aggregate bidirectional background load |
|---|---:|
| `baseline` | 0 Mbit/s |
| `low` | 1 Mbit/s |
| `medium` | 5 Mbit/s |
| `high` | 20 Mbit/s |

### 13.4 Process the one-UAS reference experiment

The document-aligned one-UAS compatibility path remains available:

```bash
./run_5g_cosimulation.py \
  --process-uas-run /absolute/path/to/EiraX/datasets/RUN_DIRECTORY
```

The original UAS documents are intentionally not committed. Their provenance
and preserved experiment decisions are described in the repository reports
and reproducibility material.

### 13.5 Simu5G source files

| File | Job |
|---|---|
| `run_5g_cosimulation.py` | Minimal executable entry point |
| `cosimulation/runner.py` | Parses the selected pipeline operation |
| `cosimulation/preflight.py` | Audits software and dataset prerequisites |
| `cosimulation/mobility_export.py` | Converts measured trajectories to BonnMotion |
| `cosimulation/scenario_builder.py` | Generates single-cell, multi-cell, application-flow, and background-load configuration |
| `cosimulation/network_profiles.py` | Loads and validates benign background profiles |
| `cosimulation/simu5g_pipeline.py` | Executes the two-user-equipment pipeline |
| `cosimulation/uas_pipeline.py` | Executes the one-UAS compatibility pipeline |
| `cosimulation/dataset_assembler.py` | Assembles the one-UAS dataset |
| `cosimulation/two_ue_dataset_assembler.py` | Joins UGV, UAV, radio, flow, and handover data |
| `config/5g_cosimulation.yaml` | Software paths, flow contract, topology, and evidence policy |
| `config/network_profiles.yaml` | Versioned Simu5G background-load values |
| `schema_reference.json` | Exact reconstructed 44-column contract |
| `validate_dataset.py` | Validates schema, types, and categorical values |
| `sim/simu5g/run_eirax_scenario.sh` | Runs the generated two-vehicle scenario |
| `sim/simu5g/run_uas_scenario.sh` | Runs the one-UAS reference scenario |

### 13.6 Simu5G output

Generated output is stored under:

```text
UGV_UAV_5G_CoSimulation/results/RUN_DIRECTORY/
├── five_g/
├── five_g_low/
├── five_g_medium/
├── five_g_high/
└── five_g_multicell/
```

Each result normally includes:

```text
input/      generated mobility, flow contract, manifest, and omnetpp.ini
raw/        Simu5G scalar, vector, and exported network measurements
data/       validated 44-column dataset and compact metrics
pipeline_summary.json
```

These generated results are ignored by Git.

### 13.7 Evidence boundary

The two network layers have different purposes:

| Layer | Purpose | Evidence status |
|---|---|---|
| Live ROS 2 network profile | Make running vehicles experience delay, loss, and outage | Deterministic emulation |
| Post-run OMNeT++/Simu5G | Measure congestion, latency, jitter, radio quality, cells, and handover | Authoritative simulated network measurement |

The implementation does not claim that live emulated delay is a Simu5G radio
measurement. It also does not yet implement a real-time synchronized
Simu5G-in-the-loop scheduler.

## 14. Dataset validation and tests

Run the deterministic network-model tests:

```bash
python3 -m unittest discover \
  -s UGV_UAV_5G_CoSimulation/tests \
  -p 'test_*.py' \
  -v
```

Validate a generated dataset:

```bash
./UGV_UAV_5G_CoSimulation/validate_dataset.py \
  /path/to/drone_network_telemetry_cosim.csv
```

The live ROS channel probe is:

```text
UGV_UAV_5G_CoSimulation/tests/live_channel_probe.py
```

It requires a sourced ROS 2 environment and local ROS communication support.

## 15. Reports and reproducibility

| Path | Content |
|---|---|
| `report/` | Local report mount point; its contents are excluded from Git |
| `UGV_UAV_5G_CoSimulation/reports/` | Baseline, degradation, network completion, schema, and validation reports |
| `reproducibility/stage1/inventory.md` | Original supplied-material inventory and limitations |
| `reproducibility/stage1/change_log.csv` | Classification of compatibility repairs and EiraX extensions |
| `reproducibility/stage1/manifests/` | Preserved checksums for source provenance |
| `reproducibility/stage1/snapshots/` | Small configuration snapshot |

The supplied reference repositories and UAS files themselves are excluded from
Git as requested.

## 16. Common operating procedure

For a normal visible cooperative test:

1. Confirm the simulation world and models exist under `simulation/`.
2. Confirm no other Gazebo server is running.
3. Source ROS 2 Jazzy.
4. Start the selected runner.
5. Wait for Gazebo models to load.
6. Click **Play**.
7. Wait for all `[OK]` startup messages.
8. Allow the mission to finish.
9. Do not delete the active dataset while the runner is still shutting down.
10. Inspect `events.jsonl`, component logs, localization CSV, communication
    summary, and final return code.
11. Close Gazebo after the runner reports that recording has stopped.

## 17. Troubleshooting

### Another Gazebo server is already running

Close the existing Gazebo window. Verify before starting another mission:

```bash
ps -ef | grep -E 'gz sim|ign gazebo'
```

### Startup remains at the sensor timeout message

In visible mode, Gazebo is intentionally paused. Click **Play** and wait for
sensor publication. The configured timeout is 300 seconds.

### Missing model or mesh

Confirm the external `simulation/models/` package is complete and run:

```bash
python3 tools/relocate_world.py
```

### UAV does not start

Check:

- `UGV_UAV/px4_runtime/bin/px4` exists and is executable;
- PX4 Gazebo plugins are present under `px4_runtime/plugins/`;
- no earlier PX4 process remains active;
- `logs/uav_px4.log` and `logs/uav_follower.log`.

### UGV does not move

Check:

- Gazebo is playing;
- `/odom`, `/map`, and `/husky/lidar3d/points` are publishing;
- Nav2 controller, planner, and behavior-tree navigator are active;
- the latest `mission_leg_*.log`;
- collision-monitor warnings and local costmap sensor freshness.

### RTAB-Map appears crowded

The map is a dense three-dimensional point cloud. Use the provided RTAB-Map
viewer configuration, height coloring, suitable point size, and filtering.
Do not confuse the full cloud with the projected two-dimensional Nav2 map.

### Mission return code

| Code | General meaning |
|---:|---|
| `0` | Mission completed successfully |
| `1` | Startup conflict such as another Gazebo server |
| `2` | Required sensor, map, or environment input missing |
| `3` | Navigation, UAV, or managed component failure |

## 18. Git workflow

The root `.gitignore` keeps all model and mesh assets, large run data,
simulation content, external repositories, build products, caches, and local
tooling out of Git.

Before staging:

```bash
git lfs install
git status --short
```

Never use `git add -f` to force `simulation/`, `datasets/`, `report/`,
`UGV_UAV_5G_CoSimulation/`, external repositories, or generated Simu5G result
files into the repository.

## 19. Current validated state

The latest completed full cooperative recovery run verified:

- UGV arrival at waypoint 1, waypoint 2, waypoint 3, and the goal;
- UAV progressive surveying and obstacle-triggered safety climbing;
- UAV landing and PX4 disarming near the UGV;
- a 10-second communication outage followed by automatic recovery;
- continued UGV navigation during the outage;
- maximum UGV-UAV separation below the 50 m communication boundary;
- live communication event and transition logging;
- single-cell Simu5G dataset validation;
- multi-cell Simu5G serving-cell transitions and handover recording.

The network baseline is ready for controlled security-scenario design. Attack
generation, authentication experiments, and security labels must be added
through explicit scenario manifests rather than inferred from benign traffic.

## 20. License

No license has been assigned. Choose an appropriate license before making the
repository public.
