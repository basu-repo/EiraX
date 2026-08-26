# Physical deployment

This directory converts validated ROS source into repeatable onboard services. It does not mean the system is approved for unattended physical operation.

- `environment/`: ROS distribution, overlay, domain and dependency environment.
- `network/`: hostnames, DDS discovery, firewall and time synchronization.
- `systemd/`: future boot services, restart limits and safe shutdown.

Deploy `src` and required configuration, then build locally on each onboard computer. Do not copy a developer computer's generated `build/`, `install/`, datasets or Gazebo processes to a robot. Install secrets separately with restricted permissions.

Recommended startup order is hardware/telemetry, safety, localization, local navigation, perception, communication and mission coordination. Loss of coordination must leave each vehicle locally safe.
