# Isolated Three-UAV Integration Runbook

This folder is independent of the original projects. It contains copied
Baylands, Husky, PX4, UAV, YOLO and OMNeT++ assets.

## Start everything

Open one terminal only:

```bash
cd /home/basudeo/Documents/EiraX/Ruben_William_Basu/decentralized_swarm_integration
python3 scripts/run_everything.py
```

That one process starts and owns:

- the accepted Baylands world and `husky_cooperative` UGV;
- three copied PX4 `x500_mapping` UAVs (`x500_mapping_0..2`);
- three separate PX4 controllers with collision-separated formation slots;
- three decentralized ROS role/mission/network peers;
- three genuine YOLO observers using independent UAV RGB-D cameras; and
- one OMNeT++/INET Wi-Fi network with three independently measured links.

Press Ctrl+C once in that terminal to stop Gazebo, all PX4 instances, ROS and
OMNeT++. Component logs are saved in `runtime_logs/`.

Useful diagnostic modes:

```bash
python3 scripts/run_everything.py --no-motion
python3 scripts/run_everything.py --no-yolo
python3 scripts/run_everything.py --no-omnet
python3 scripts/run_everything.py --headless
python3 scripts/run_everything.py --help
```

## Runtime contract

- UAV identities: `uav0`, `uav1`, `uav2`
- Gazebo identities: `x500_mapping_0`, `x500_mapping_1`, `x500_mapping_2`
- PX4 MAVLink endpoints: UDP 14540, 14541, 14542
- Initial layout: three marked bays on an 8 m × 12 m level staging deck
- Pose bridge: TCP 5555
- OMNeT++ link metrics: TCP 5556, 5557, 5558
- Network type: INET 802.11 Wi-Fi (not 5G, Simu5G or FLORA)
- ROS coordination-layer flight output: disabled (the copied vehicle stack owns flight)

## Move or resize the UAV launch site

Edit only:

```text
vehicle_stack/config/swarm_launch_site.yaml
```

The important values are `center_x`, `center_y`, and `deck_top_z`, expressed in
Gazebo world metres. Changing them moves the entire deck and all three UAVs.
`offset_x` and `offset_y` move an individual bay relative to the deck centre.
Keep bays at least 3 m apart. The launcher validates that every bay remains
inside the platform before starting Gazebo.

To obtain coordinates, select an object or temporary marker in Gazebo and read
its World Pose X/Y/Z values. Choose open ground, then set `deck_top_z` high
enough that the platform and its supports remain above the terrain.

All three UAVs arm and take off before the UGV mission begins. UAV 0 retains
the accepted 15 m mapping/survey controller. UAV 1 holds a 19 m formation slot
7 m to one side; UAV 2 holds a 23 m slot 7 m to the other side. Each controller
owns a distinct MAVLink port and odometry stream. The UGV is their shared
mission anchor; there is no permanent UAV leader or centralized UAV role
allocator. A generated open-air staging deck sits approximately 12 m ahead of
the Husky, outside the canopy. Its blue, green and orange bays provide a common
level surface and 4 m launch spacing on the sloped Baylands terrain.

Each UAV has a distinct 416×256 RGB-D camera pitched 45 degrees downward. The
three model-scoped 2 Hz streams feed three separate 0.5 Hz YOLO observers, so
consensus can use independent evidence without duplicating frames or starving
Gazebo and Nav2. Detection, status and estimate topics are included in the bag.
YOLO remains semantic/advisory; LiDAR and costmaps retain geometric collision
authority.

In a graphical run, the launcher opens one `rqt_image_view` selector. Use its
topic drop-down to switch between:

```text
/swarm/uav0/camera/image_raw
/swarm/uav1/camera/image_raw
/swarm/uav2/camera/image_raw
```

The corresponding `/camera_info` and `/depth_image` streams are also bridged
for every UAV; only one RGB view is rendered in the viewer at a time to avoid
three additional GUI rendering loads.

The default one-command mission runs
`spawn -> waypoint_1 -> waypoint_2 -> waypoint_3 -> goal`. UAV 0 surveys each
upcoming corridor before the corresponding UGV leg begins, while UAV 1 and UAV
2 maintain their independent formation slots and perception/network roles.
After the goal, all aircraft return at cruise altitude to their own colored
launch bays and descend in a staggered sequence. They never select unknown
ground near the UGV as a landing site.

## Build after source changes

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install \
  --base-paths . perception_stack/lrs_halmstad \
  --build-base build_combined \
  --install-base install_combined
cd omnet && bash build.sh && cd ..
```

## Verify

```bash
python3 -m pytest -q
python3 scripts/check_yolo_runtime.py
```
## Fault-tolerant swarm experiments

Run the complete permanent-loss sequence (UAV0, then UAV1, then UAV2):

```bash
python3 scripts/run_everything.py --permanent-failure
```

Each failed aircraft returns to its own bay. UAV1 and UAV2 successively take
the scout role. After UAV2 fails, the UGV completes the remaining route with
its onboard Nav2 stack.

Run a temporary UAV0 outage and automatic reconnection:

```bash
python3 scripts/run_everything.py --connection-failure-reconnect
```

UAV1 takes over immediately. UAV0 holds rather than returning, reconnects
after 30 seconds, and rejoins as UAV1's follower. The permanent threshold is
60 seconds. These two scenario options are mutually exclusive. The isolated
UGV forward-speed ceiling is 1.0 m/s; Nav2 can still command less while
turning, approaching a goal, or avoiding an obstacle.
