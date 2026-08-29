# Baylands simulation and integration testing

This layer validates the robot contracts in Gazebo. It replaces physical drivers with explicit simulation adapters while retaining Husky and DJI topic boundaries. Physical deployment must never depend on Gazebo.

`run_baylands_husky.py` is the supported all-in-one launcher. It creates a unique dataset, generates the world, starts Gazebo and bridges, validates an advancing clock and sensor data, starts localization/SLAM/Nav2, starts optional DJI/YOLO/gimbal nodes, runs the selected mission, records evidence, and owns clean shutdown.

`run_one_dji.py` builds the embedded M100 model and provides the small one-aircraft validation path used during model development.

The first supported world is Baylands. Simulation success is not proof of physical
flight or driving safety.

## First validation run

Start with the proven Husky stack and no mission motion:

```bash
cd vastervik/implementation
python3 simulation/run_baylands_husky.py --no-motion
```

## One DJI M100 validation

This starts Baylands with exactly one locally copied `dji0`, its DJI-compatible
absolute ENU command interface, and its camera bridge. After Gazebo is playing,
the aircraft performs a short climb-and-return flight and remains hovering for
inspection.

```bash
cd /home/basudeo/Documents/EiraX/vastervik/implementation
python3 simulation/run_one_dji.py
```

Camera topic: `/dji0/camera0/image_raw`

## One DJI following the Husky

For the lightweight follower-only scenario (no scout and no aerial survey
gate), run:

```bash
python3 simulation/run_baylands_husky.py --single-dji-follower
```

DJI0 takes off, remains 15 m behind and 15 m above the Husky along the active
route direction, and faces it. This matches the fixed -45° physical camera;
gimbal actuation is disabled. Husky Nav2 independently starts every leg, and a
missing YOLO detection never causes the aircraft to reposition. The full
current version remains selectable with `--dji-swarm`.

Annotated YOLO evidence is written automatically to
`datasets/run_<timestamp>/yolo_frames/dji0/positive/`, with sampled no-detection
frames in `sampled_negative/` and coordinates in `frame_index.csv`.

This is the DJI adaptation of the successful `UGV_UAV` mission sequence. The
M100 takes off, establishes a 20 m progressive lead toward each saved waypoint,
holds for the survey interval, escorts the Husky while its own Nav2 drives the
leg, and finally lands 4 m clear of the UGV.

```bash
cd /home/basudeo/Documents/EiraX/vastervik/implementation
python3 simulation/run_baylands_husky.py --dji-follow
```

The vehicle-specific boundary remains explicit: Husky motion is produced by
its Nav2 stack; DJI motion is produced through
`/dji0/psdk_ros2/flight_control_setpoint_ENUposition_yaw`.

## Three-DJI failure recovery

```bash
python3 simulation/run_baylands_husky.py --dji-swarm
python3 simulation/run_baylands_husky.py --permanent-failure
python3 simulation/run_baylands_husky.py --connection-failure-reconnect
```

Permanent loss transfers the scout role DJI0 -> DJI1 -> DJI2 -> Husky Nav2.
Each permanently failed aircraft returns to its own launch point and stops at
absolute Baylands `z=1.5 m`.

During temporary loss DJI0 holds position, DJI1 becomes scout, and DJI0
reconnects after 30 seconds as a follower without reclaiming leadership. The
permanence threshold is 60 seconds. All three aircraft retain independent DJI
ENU control and camera topics.

## Expected startup order

Gazebo opens paused. Wait for the terrain, Husky and requested UAVs, then click Play. The launcher must print these milestones in order:

1. Gazebo simulation time is advancing.
2. Bridges, LiDAR, GNSS, odometry, TF and IMU are ready.
3. RTAB-Map and the projected Nav2 map publish.
4. Nav2 becomes active.
5. DJI controllers, cameras, YOLO and coordination start.
6. The scout surveys before each UGV leg.

If a run fails, inspect its `datasets/run_*/events.jsonl` first. A missing `/odometry/gps` during startup usually means simulation time never advanced. A rejected waypoint after navigation was already moving can mean Nav2 lifecycle recovery is in progress; the mission now waits for recovery before retrying.

## Camera viewing

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run rqt_image_view rqt_image_view
```

Select `/dji0/camera0/image_raw`, `/dji1/camera0/image_raw` or `/dji2/camera0/image_raw`. Three cameras and three YOLO processes are computationally expensive, so swarm camera rates are intentionally lower than a single-aircraft inspection run.

## Assets

- `worlds/baylands/`: editable world, maps, route YAMLs and editor guide.
- `models/`: self-contained terrain, obstacle, Husky, M100 and gimbal assets.
- `scenarios/`: reproducible scenario definitions.

Edit source worlds/models here. Generated per-run world snapshots under `datasets/` exist only for reproducibility.
