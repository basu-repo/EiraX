# EiraX standalone UGV

This self-contained runtime verifies and records the interfaces that transfer
to a real UGV, starts online three-dimensional LiDAR SLAM, and runs the Nav2
waypoint mission. The Baylands world and models remain in the shared
`simulation/` directory, and run data remains in the shared `datasets/`
directory.

## Run

```bash
cd /home/basudeo/Documents/EiraX
source /opt/ros/jazzy/setup.bash
./UGV_Standalone/run_UGV_simulation.py
```

The runner opens Gazebo **paused**. Wait for all models to load and then click
Play in Gazebo. The runner starts the ROS bridge and sensor transforms, waits
up to five minutes for the sensor topics, and records a timestamped dataset.
Press `Ctrl+C` in the runner terminal for a clean shutdown.

The mission dynamically reads the saved world and follows `waypoint_1`,
`waypoint_2`, `waypoint_3`, and `goal` in
that order. Saved world positions are converted into the Husky's start-relative
`odom` frame. Nav2 uses an odom-based rolling costmap so SLAM corrections cannot
shift the physical mission targets. SLAM mapping continues for the dataset.

Useful switches:

```bash
./UGV_Standalone/run_UGV_simulation.py --view-3d-slam
./UGV_Standalone/run_UGV_simulation.py --return-to-spawn
./UGV_Standalone/run_UGV_simulation.py --no-motion
```

## Navigation baseline

Nav2 is the preserved reference implementation and remains the only component
allowed to publish mission motion to `/cmd_vel`. The discontinued hand-built
RGB-D traversability controller and its command-line modes have been removed.

The front camera bridge remains available for the next learned-traversability
implementation. Previous RGB-D datasets and the workspace-level
[historical analysis notebook](../UGV_Standalone_Visual_Navigation_Analysis.ipynb)
are retained only as experimental evidence; they are not part of the runtime.

## Verified interface

- `/husky/lidar3d/points`: three-dimensional LiDAR point cloud
  (`sensor_msgs/msg/PointCloud2`)
- `/wheel/odom`: raw wheel-encoder odometry (`nav_msgs/msg/Odometry`)
- `/odom`: wheel and IMU fused EKF odometry (`nav_msgs/msg/Odometry`)
- `/tf`, `/tf_static`: robot and sensor transforms
- `/imu`: inertial measurements (`sensor_msgs/msg/Imu`)
- `/cmd_vel`: velocity commands (`geometry_msgs/msg/Twist`)
- `/husky/camera_front/image_raw`: front colour image (`sensor_msgs/msg/Image`)
- `/rtabmap3d/cloud_map`: online three-dimensional SLAM point-cloud map
- `/map`: projected occupancy map used by navigation

Datasets are written under `datasets/run_YYYYMMDD_HHMMSS/`. Images, Gazebo
scene state, GUI information, perfect simulator poses, and high-rate simulation
clock messages are excluded from the primary dataset.

The three blue waypoint markers and green goal marker are movable in Gazebo.
Save their edited positions with `simulation/save_baylands_world.sh` while
Gazebo is still running.
