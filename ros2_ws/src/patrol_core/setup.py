import os
from glob import glob

from setuptools import setup

package_name = "patrol_core"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ohinseop",
    maintainer_email="osos8528@gmail.com",
    description="순찰 로봇 공용 노드",
    license="MIT",
    entry_points={
        "console_scripts": [
            "cmd_vel_mux = patrol_core.cmd_vel_mux:main",
            "patrol_node = patrol_core.patrol_node:main",
            "patrol_scheduler = patrol_core.patrol_scheduler:main",
            "explore_node = patrol_core.explore_node:main",
            "inspect_node = patrol_core.inspect_node:main",
            "helmet_node = patrol_core.helmet_node:main",
            "fire_node = patrol_core.fire_node:main",
            "restricted_node = patrol_core.restricted_node:main",
            "extinguisher_expiry_node = patrol_core.extinguisher_expiry_node:main",
            "extinguisher_inspect_node = patrol_core.extinguisher_inspect_node:main",
        ],
    },
)
