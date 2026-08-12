from glob import glob
import os

from setuptools import find_packages, setup


package_name = "decentralized_swarm_integration"

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
    description="Decentralized semantic consensus for a heterogeneous UAV swarm.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "semantic_peer = decentralized_swarm_integration.semantic_peer:main",
            "multi_pose_bridge = decentralized_swarm_integration.multi_pose_bridge:main",
            "multi_metrics_bridge = decentralized_swarm_integration.multi_metrics_bridge:main",
            "px4_agent = decentralized_swarm_integration.px4_agent:main",
            "role_peer = decentralized_swarm_integration.role_peer:main",
            "mission_peer = decentralized_swarm_integration.mission_peer:main",
            "isolated_leader_detector = decentralized_swarm_integration.halmstad_runners:leader_detector_main",
            "isolated_leader_estimator = decentralized_swarm_integration.halmstad_runners:leader_estimator_main",
        ],
    },
)
