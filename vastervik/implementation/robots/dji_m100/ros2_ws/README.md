# DJI M100 local ROS 2 workspace

This workspace is copied inside the DJI deployment so the UAV does not depend on
the other team's directory. Its `lrs_halmstad` package contains the M100 generator,
camera/gimbal description, follow logic, perception nodes and simulator adapter.

Build on the companion computer with:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select lrs_halmstad
source install/setup.bash
```

The `simulator` node is Gazebo-only. A verified physical DJI SDK adapter must replace
it in physical mode while retaining the external command and telemetry contracts.
