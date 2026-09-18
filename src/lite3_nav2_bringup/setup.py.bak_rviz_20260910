from glob import glob
from setuptools import setup

package_name = "lite3_nav2_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="electron",
    maintainer_email="electron@localhost",
    description="Safe Nav2 bringup for DeepRobotics Lite3",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "nav2_goal_to_scan = lite3_nav2_bringup.nav2_goal_to_scan:main",
            "scan_body_odom_bridge = lite3_nav2_bringup.scan_body_odom_bridge:main",
            "scan_cmd_safety_gate = lite3_nav2_bringup.scan_cmd_safety_gate:main",
        ],
    },
)
