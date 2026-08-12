# Runtime assets excluded from Git

The Git repository contains source, launch files, configuration, tests, model
descriptors, the 6 MB YOLO checkpoint, and the analysis notebook. The following
large local assets are intentionally excluded.

| Local path | Approximate size | Purpose |
|---|---:|---|
| `third_party/python/` | 1.2 GB | Installed YOLO/PyTorch Python runtime |
| `px4_runtime/` (except README) | 112 MB | Compiled PX4 SITL, plugins and models |
| `simulation/models/**/{meshes,media,materials}/` | 617 MB total model tree | Baylands and vehicle geometry/textures |
| `analysis/python/` | 101 MB | Project-local Jupyter kernel packages |
| `datasets/` | Variable; currently 5.8 GB | ROS bags, RTAB-Map DBs and run artifacts |

These assets remain on the development machine and are not deleted by Git
cleanup. Before distributing a clean clone, publish the PX4 and simulation
asset bundles in research/project storage, record their SHA-256 hashes here,
and add a download/setup script. ROS bags should be published as experiment
artifacts (for example Zenodo or institutional storage), never in Git history.

The repository is source-complete but a fresh clone is not yet runtime-complete
until those external asset bundles are installed.
