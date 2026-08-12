# Swarm run evidence notebook

Open `analysis/swarm_run_evidence.ipynb` in VS Code and select the system
Python 3 kernel. Start VS Code from a ROS-sourced terminal so the native bag
reader is available:

```bash
cd /home/basudeo/Documents/EiraX/Ruben_William_Basu/decentralized_swarm_integration
source /opt/ros/jazzy/setup.bash
code analysis/swarm_run_evidence.ipynb
```

The `jupyter` command-line application is not currently installed, so do not
use `jupyter notebook` unless it is installed later. No additional Python
package is required when the notebook is run through the existing VS Code
notebook interface and system Python environment.

Run all cells from top to bottom. By default, the notebook selects the newest
`datasets/run_*` directory. Set `RUN_OVERRIDE` in the first code cell to make a
report reproducible for a specific run.

The notebook writes plots, extracted detection payloads, and the final evidence
table into that run's `analysis_output/` directory. It does not modify the bag.
