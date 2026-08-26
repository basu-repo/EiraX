# Baylands

This directory contains the copied standalone Baylands worlds, editor utilities,
static maps and Halmstad route files.

- `baylands_editable.world`: known-good editable Gazebo Classic world.
- `baylands_world.sdf`: SDF world variant.
- `maps/`: existing occupancy maps for comparison/localization tests.
- `routes/`: existing Baylands Nav2 waypoint sequences.

The first run should validate only spawning, manual stop, sensors and camera.
Autonomous missions remain disabled in `baylands_smoke_test.yaml` until those
checks pass.
