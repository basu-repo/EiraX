# Developer and analysis tools

These tools support setup and validation but are not part of the real-time safety path.

- `data_analysis/`: rosbag, trajectory, localization, perception and network analysis.
- `diagnostics/`: read-only topic, timing, sensor and process checks.
- `setup/`: dependency, build and configuration validation.

Analysis should record its input run and parameters. Large derived files belong with experiment storage, not Git. A diagnostic that publishes motion or changes hardware state must be clearly labeled and require deliberate operator action.
