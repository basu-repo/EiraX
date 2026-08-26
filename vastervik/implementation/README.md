# Västervik cooperative Husky–DJI system

Västervik is a ROS 2 Jazzy workspace for cooperative missions between a Clearpath Husky UGV and one or three DJI Matrice 100 UAVs. It retains the Halmstad/Linköping `lrs_halmstad` structure so the mission logic can be built in the same form used by the physical robots. Gazebo Baylands is the current integration and validation environment.

The simulation demonstrates Husky localization, RTAB-Map and Nav2; one- or three-DJI scouting; independent camera and ENU control; YOLO OBB detection; gimbal pitch tracking; decentralized link-failure recovery; and experiment recording.

Simulation success does not authorize physical operation. Hardware drivers, flight authority, emergency stop, geofence, return-to-home, calibration and physical limits must be validated on each robot.

## Quick start

```bash
cd /home/basudeo/Documents/EiraX/vastervik/implementation
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 simulation/run_baylands_husky.py --dji-swarm
```

Gazebo opens paused. Wait for all models to appear and click **Play**. A normal startup confirms simulation time, sensors, localization, SLAM and Nav2 before motion. Press `Ctrl+C` once to stop cleanly.

Available runs:

```bash
# Sensors, localization, SLAM and Nav2; no autonomous mission
python3 simulation/run_baylands_husky.py --no-motion

# Husky-only waypoint mission
python3 simulation/run_baylands_husky.py

# One DJI scout and escort
python3 simulation/run_baylands_husky.py --dji-follow

# One DJI follower; no scout or aerial survey gate
python3 simulation/run_baylands_husky.py --single-dji-follower

# Three-DJI normal mission
python3 simulation/run_baylands_husky.py --dji-swarm

# DJI0 -> DJI1 -> DJI2 -> Husky fallback
python3 simulation/run_baylands_husky.py --permanent-failure

# Temporary DJI0 loss, DJI1 handover, DJI0 rejoins as follower
python3 simulation/run_baylands_husky.py --connection-failure-reconnect
```

Use `--headless` without the Gazebo GUI, `--view-3d-slam` for the RTAB-Map viewer, or `--return-to-spawn` for a final Husky return leg.

## Mission behavior

`--single-dji-follower` is the lightweight autonomous-UGV baseline. Husky Nav2
and LiDAR own every route and obstacle decision. One DJI stays 15 m behind and
15 m above the Husky, faces it, and uses the physical aircraft's fixed -45°
camera geometry. Gimbal actuation and aerial survey gating are disabled. A
missing YOLO detection, including temporary tree occlusion, never changes the
formation command. Use `--dji-swarm` to restore the complete three-aircraft
scout/follower design.

For each route leg, the active scout moves toward the next waypoint while the Husky remains stopped. After the survey is ready, the Husky drives using its own Nav2 stack and the UAVs update their formation relative to it. Aircraft land at separated ground-safe positions when the mission finishes.

The scout does not directly drive the Husky. The Husky retains its planner, controller, collision monitor and emergency behavior. UAV observations are cooperative inputs; local robot safety remains authoritative.

YOLO saves bounded, annotated evidence under each run at
`datasets/run_<timestamp>/yolo_frames/dji0/` (and `dji1`/`dji2` in swarm mode).
Positive JPEGs contain the rectangular box, yellow OBB polygon, class and
confidence. `frame_index.csv` records timestamps and box coordinates; sampled
negative frames are retained separately for auditing. Saving is capped at 500
positives and 100 negatives per aircraft.

## Repository map

```text
implementation/
├── src/                 authoritative colcon source packages
├── simulation/          Baylands launcher, worlds and models
├── robots/              readable robot-specific deployment/reference layout
├── shared/              cross-robot contracts and design boundaries
├── scripts/             Halmstad-compatible operational utilities
├── config/              workspace-wide support configuration
├── deployment/          physical deployment templates and guidance
├── tools/               diagnostics and offline analysis
├── descriptions/        design, pipeline and handoff documentation
├── datasets/            generated experiment evidence
├── build/               generated colcon intermediate files
├── install/             generated runnable ROS overlay
├── log/                 generated colcon build logs
├── run.sh               Halmstad command dispatcher
└── BUILD.md             complete build instructions
```

## Source ownership

`src/lrs_halmstad/` is the deployable ROS package and primary source of truth:

- `lrs_halmstad/coordination/`: DJI role state, failover and Husky following.
- `lrs_halmstad/vastervik_husky/`: Husky mission and validation helpers.
- `lrs_halmstad/follow/`: following and active gimbal tracking.
- `lrs_halmstad/perception/`: YOLO detection, tracking and 3D estimation.
- `config/vastervik_husky/`: Husky localization and Nav2 parameters.
- `models/obb/mymodels/`: packaged Baylands YOLO weights.

`robots/` is an easy-to-understand robot-oriented layout and retains known-good helpers. The nested `robots/dji_m100/ros2_ws` has `COLCON_IGNORE` and is reference-only. Never edit generated copies in `build/` or `install/` expecting source code to change.

## Main ROS interfaces

Husky:

- `/a201_0000/cmd_vel`: retained Clearpath velocity input.
- `/husky/lidar3d/points`: primary 3D obstacle point cloud.
- `/husky/gps`, `/odometry/gps`: GNSS and fused GNSS odometry.
- `/odom`, `/tf`, `/tf_static`, `/imu`: localization and transforms.
- `/map`: projected RTAB-Map/Nav2 map.

Each DJI uses `dji0`, `dji1` or `dji2`:

- `/<name>/psdk_ros2/flight_control_setpoint_ENUposition_yaw`: absolute ENU position/yaw command.
- `/<name>/camera0/image_raw`: camera image sent to YOLO.
- `/<name>/camera0/camera_info`: camera calibration.
- `/<name>/camera0/update_tilt`: requested gimbal pitch.

Simulation implements these contracts without PX4. Physical deployment must connect them to the verified DJI SDK/PSDK adapter.

## YOLO model

The launcher uses:

```text
src/lrs_halmstad/models/obb/mymodels/baylands-leader-v9-tuned-full.pt
```

It is an oriented-bounding-box model trained for the Baylands leader/UGV appearance. Each UAV observes its own camera independently. The copy under `install/` is generated from this source file.

## Run output

Every run creates `datasets/run_YYYYMMDD_HHMMSS/` with chronological events, exact generated configuration, component logs, role markers, per-UAV perception metrics, localization errors, selected ROS topics, world snapshots and UAV trajectories. These are generated research artifacts and can become very large; they should normally remain outside Git.

## Building

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Rebuild `build/` and `install/` from `src/` on every target computer. See [BUILD.md](BUILD.md).

## Where to make changes

- Mission and roles: `src/lrs_halmstad/lrs_halmstad/coordination/`.
- Husky route execution: `robots/husky/missions/` and packaged equivalent.
- Husky Nav2 tuning: `robots/husky/navigation/config/nav2_config.py`.
- DJI gimbal behavior: `src/lrs_halmstad/lrs_halmstad/follow/`.
- YOLO behavior: `src/lrs_halmstad/lrs_halmstad/perception/`.
- Robot identity/hardware/network: `robots/*/config/`.
- Baylands objects and coordinates: `simulation/worlds/baylands/`.
- Simulation orchestration: `simulation/run_baylands_husky.py`.

Do not edit `build/`, `install/`, dataset snapshots or `__pycache__`; they are outputs.

## Further reading

- [Simulation guide](simulation/README.md)
- [Robot layout](robots/README.md)
- [Husky guide](robots/husky/README.md)
- [DJI guide](robots/dji_m100/README.md)
- [ROS source guide](src/README.md)
- [Detailed file and folder index](FILE_INDEX.md)
- [Dataset guide](datasets/README.md)
- [Current state](descriptions/CURRENT_STATE.md)
- [Pipeline description](descriptions/PIPELINE_DESCRIPTION.md)
