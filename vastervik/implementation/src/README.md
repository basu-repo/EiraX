# ROS 2 source workspace

This is the authoritative input to `colcon build`. Make deployable changes here, rebuild, then test through the resulting `install/` overlay.

## Packages

`lrs_halmstad/` is the main ROS 2 package:

- `launch/`: Halmstad-compatible launch descriptions.
- `lrs_halmstad/coordination/`: decentralized DJI roles and recovery.
- `lrs_halmstad/follow/`: target generation, following and gimbal tracking.
- `lrs_halmstad/perception/`: YOLO detector/tracker and 3D estimation.
- `lrs_halmstad/nav/`: UGV navigation drivers and motion profiles.
- `lrs_halmstad/sim/`: Gazebo-only adapters; never start on real robots.
- `lrs_halmstad/vastervik_husky/`: Västervik Husky helpers.
- `config/`: Halmstad and Västervik runtime parameters.
- `clearpath/`: team Clearpath description and sensor additions.
- `urdf/`, `xacro/`: DJI M100 and camera/gimbal descriptions.
- `models/`: DJI, gimbal and packaged YOLO assets.
- `setup.py`: package data and ROS executable registration.
- `package.xml`: ROS dependency declaration.

`lrs_halmstad_gui_plugins/` contains Gazebo UI extensions. It is not onboard autonomy and is unnecessary on a headless physical robot.

## Build flow

```text
src/ -> colcon -> build/   (intermediate work)
              -> install/ (runnable ROS overlay)
              -> log/     (build records)
```

If an installed file differs from the source, rebuild it. Never repair `install/` manually.

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select lrs_halmstad
source install/setup.bash
```

The YOLO weight is kept under `lrs_halmstad/models/obb/mymodels/` so deployment does not depend on another repository's absolute path.
