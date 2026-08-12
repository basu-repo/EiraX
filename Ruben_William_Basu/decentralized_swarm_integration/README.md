# Isolated EiraX UGV–UAV Integration

This folder is a self-contained copy-and-integrate workspace. It does not read
runtime code, models, worlds, PX4 binaries, perception packages or OMNeT++
bridge sources from the original projects.

The local development folder is runtime-complete, but Git intentionally tracks
only source, configuration, tests, documentation, model descriptors and the
small YOLO checkpoint. Installed Python packages, compiled PX4, large Gazebo
meshes/textures and experiment datasets are excluded. See
`requirements/runtime-assets.md` before preparing a clean clone on another
computer.

See [RUNBOOK.md](RUNBOOK.md) for the verified commands.

## Vehicle baseline and swarm extension

The accepted primary baseline remains unchanged:

- one `husky_cooperative` UGV;
- three independently controlled PX4 `x500_mapping` UAVs;
- the Baylands environment and obstacles;
- the proven Gazebo startup and renderer behavior; and
- the proven cooperative UGV/UAV navigation and mapping logic.

The copied sources are named `ground_stack` and `vehicle_stack`. Models and
worlds are under `simulation`, and the copied PX4 executable, modules, models
and plugins are under `px4_runtime`.

## Integration architecture

```text
copied Baylands world
        │
        ├── Husky UGV ── /odom ───────────────────┐
        │       └── optional front RGB-D → YOLO   │
        │                                          ├─ ROS pose bridge → OMNeT++/INET
        └── x500_mapping UAV ─ /uav/px4_odom ─────┘        │
                    │                                      └─ link metrics
                    ├── 3D LiDAR mapping                         │
                    └── local semantic/role/mission peers ◀──────┘
```

The runtime now spawns three UAV peers. The coordination protocol has no
permanent leader or central role allocator. UAV 0 keeps the proven mapping and
survey follower. UAVs 1 and 2 run separate PX4 formation controllers at
vertically and laterally separated slots around the UGV mission anchor.

## UAV cameras and YOLO usage boundary

Each isolated `x500_mapping` UAV now carries its own lightweight 416×256 RGB-D
camera, pitched 45 degrees downward and capped at 8 Hz. Each image stream feeds
an independent YOLO observer capped at 5 Hz. YOLO contributes semantic class
identity and visual continuity; it does not replace LiDAR, costmaps, collision
avoidance, landing checks or metric ranging.

The consensus policy can therefore require two genuinely independent,
spatially consistent UAV observations. YOLO remains advisory and cannot alter
navigation or command motion by itself.

## Network boundary

The network overlay contains exactly one UGV, one UAV and one INET 802.11 Wi-Fi
flow. It uses copied local mobility/scheduler/metrics sources. It does not use
Simu5G, FLORA or 5G components.

OMNeT++ ground-truth distance is used only inside propagation modelling. The
ROS layer consumes RSSI, SNIR, PER, PDR and available timing metrics.

## Build

```bash
cd /home/basudeo/Documents/EiraX/Ruben_William_Basu/decentralized_swarm_integration
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install \
  --base-paths . perception_stack/lrs_halmstad \
  --build-base build_combined \
  --install-base install_combined
source install_combined/setup.bash
```

## Start everything from one terminal

```bash
python3 scripts/run_everything.py
```

This starts the UGV, three UAVs, ROS/YOLO integration and OMNeT++ overlay. One
Ctrl+C shuts down the entire process tree. See `RUNBOOK.md` for modes.

## Start the safe ROS overlay

In another terminal after sourcing `install_combined/setup.bash`:

```bash
ros2 launch decentralized_swarm_integration full_swarm.launch.py \
  uav_names:=uav \
  start_px4:=false \
  start_yolo:=false \
  control_enabled:=false
```

The defaults already match those safe values. `start_px4` refers only to the
alternative ROS MAVLink adapter; it stays off because the accepted follower
owns PX4 control. Nothing in the integration overlay arms a vehicle.

## Main contracts

- UGV odometry: `/odom`
- UAV odometry from the accepted controller: `/uav/px4_odom`
- OMNeT++ pose server: TCP 5555 (`ugv`, `uav`)
- OMNeT++ metrics server: TCP 5556 (`uav`)
- Shared observations: `/coord/swarm/semantic_observations`
- Local role: `/coord/swarm/uav/role`
- Advisory consensus: `/coord/swarm/uav/consensus`
- Optional mission setpoint: `/coord/swarm/uav/setpoint`

## Safety

- Integration control is disabled by default.
- The accepted vehicle controller remains the only PX4 controller.
- YOLO is advisory with one observer.
- Stale roles, anchors, observations and commands are rejected.
- Coordinate conversions and setpoint bounds are unit-tested.
- The UGV costmap and UAV onboard sensing retain collision-safety authority.

## Verification

Run:

```bash
python3 -m pytest -q
```

Automated checks cover protocol validation, safety bounds, role calculation,
launcher contracts, copied asset presence, one-vehicle defaults and forbidden
references to the original projects. A GUI flight run remains an operator test.

Current automated status (9 August 2026): 59 pytest checks pass; both local ROS
packages build; the copied-source OMNeT++ executable builds and its NED files
validate; the copied OBB model loads as class `ugv` on the isolated CPU runtime;
and the control-disabled YOLO overlay starts and shuts down cleanly. The
accepted GUI vehicle run is intentionally not automated.
