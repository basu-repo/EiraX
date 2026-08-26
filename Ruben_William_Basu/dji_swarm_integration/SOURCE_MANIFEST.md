# Source Manifest

Created on 2026-08-13 as an independent DJI integration baseline.

The files in this directory are full copies. A scan performed after copying
found no symbolic links. Generated PX4 build/install/runtime directories and
old experiment datasets were not copied from the existing decentralized
project.

The updated Halmstad `dji_stack/models/` directory is retained because its
launch files refer to the DJI M100/Matrice, camera, gimbal, terrain and obstacle
assets stored there. Duplicate assets can be deduplicated later only after the
DJI launch dependency graph is tested from this isolated directory.

`vehicle_stack/uav/` currently contains PX4-era reference modules. They are not
the selected DJI hardware interface and must not be presented as such. The next
development stage is to introduce and test a DJI adapter without altering the
source baseline project.

