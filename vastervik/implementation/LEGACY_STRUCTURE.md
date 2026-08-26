# Vastervik Multi-Robot Implementation

This project provides a reusable structure for deploying software to a Husky UGV and a DJI M100 UAV.

I created the structure so that each robot can operate from its own onboard computer. A robot folder can be copied to the corresponding computer, and the hardware-specific values can then be changed through YAML configuration files. The main navigation and mission code should not need to be rewritten when a sensor, network address or hardware interface changes.

The project is intended for physical robots. Baylands is used first as a safe simulation environment to verify the interfaces and behavior.

## Project structure

```text
implementation/
├── robots/
│   ├── husky/
│   └── dji_m100/
├── shared/
├── deployment/
├── simulation/
└── tools/
```

## Robot folders

The `robots/` directory contains one independent deployment folder for each robot.

### Husky

```text
robots/husky/
├── config/        # Identity, hardware and network settings
├── hardware/      # Clearpath platform and physical sensor drivers
├── navigation/    # Husky Nav2, localization, maps and routes
├── missions/      # Husky mission behavior
└── simulation/    # Husky Baylands overrides
```

This folder is intended to be copied to the Husky onboard computer. The Husky keeps its own Nav2 parameters because its footprint, skid-steer movement, LiDAR, speed and stopping behavior are specific to the ground robot.

### DJI M100

```text
robots/dji_m100/
├── config/        # Identity, flight hardware and network settings
├── hardware/      # DJI SDK, telemetry and flight-control adapter
├── navigation/    # UAV navigation and localization parameters
├── perception/    # Camera and YOLO configuration
├── missions/      # Survey, follow, return and landing behavior
└── simulation/    # DJI M100 Baylands overrides
```

This folder is intended to be copied to the UAV companion computer. The UAV has separate navigation parameters because its altitude, safety radius, speed, localization and flight-control requirements are different from the Husky.

Standard Nav2 can be used initially for fixed-altitude horizontal planning. Full three-dimensional flight still requires a DJI-compatible aerial planner and flight controller.

## Shared software

The `shared/` directory contains software contracts used by both robots:

- Communication and heartbeat formats
- Robot and mission status
- Leader or scout role information
- Observation and detection messages
- Common fault and safety-event definitions
- Monitoring interfaces

Shared does not mean that a third computer controls both robots. The Husky and UAV run independently and exchange information directly through the network. A ground station may be used for monitoring and logging, but it should not be required for normal operation.

## Physical and simulation modes

Physical operation is the default:

```yaml
runtime:
  mode: physical
  use_sim_time: false
```

For Baylands testing, the robot loads its simulation override:

```yaml
runtime:
  mode: simulation
  use_sim_time: true
```

Simulation replaces physical drivers with Gazebo bridges. Robot identity, message interfaces and high-level behavior should remain consistent between the two modes.

Simulation success does not by itself prove that the physical robot is safe. Motor commands, flight authority, emergency stopping, geofencing and landing must be validated separately on the real hardware.

## Configuration

Users should normally change YAML files for:

- Robot name, serial number and ROS namespace
- ROS domain and network interface
- Sensor models, device addresses and serial numbers
- Speed, acceleration and command timeouts
- UAV altitude and geofence limits
- Maps, routes and mission settings
- Physical or simulation mode

Absolute paths, passwords, private keys and DJI credentials must not be stored in the repository.

## Recommended implementation order

1. Start the physical Husky drivers and verify diagnostics.
2. Test manual Husky movement and emergency stopping.
3. Verify odometry, IMU, camera and LiDAR data.
4. Configure and test the Husky Nav2 stack.
5. Start the DJI interface in telemetry-only mode.
6. Verify the UAV camera, localization and safety limits.
7. Test restrained low-altitude UAV control.
8. Validate both robots independently in Baylands.
9. Enable direct Husky–UAV communication.
10. Add cooperative missions only after independent safety tests pass.

Each subfolder contains its own `README.md` with more specific instructions.
