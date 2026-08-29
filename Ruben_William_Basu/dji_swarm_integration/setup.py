from glob import glob
import os

from setuptools import find_packages, setup


package_name = "dji_swarm_integration"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Basudeo",
    maintainer_email="basudeo@example.com",
    description="Decentralized DJI M100 swarm integration for EiraX.",
    license="Apache-2.0",
    entry_points={"console_scripts": [
        "dji_vehicle_agent = dji_swarm_integration.dji_vehicle_agent:main",
        "dji_mission_director = dji_swarm_integration.dji_mission_director:main",
        "semantic_peer = dji_swarm_integration.semantic_peer:main",
        "multi_pose_bridge = dji_swarm_integration.multi_pose_bridge:main",
        "multi_metrics_bridge = dji_swarm_integration.multi_metrics_bridge:main",
        "role_peer = dji_swarm_integration.role_peer:main",
        "mission_peer = dji_swarm_integration.mission_peer:main",
    ]},
)
