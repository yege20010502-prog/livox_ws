"""Static transforms for the DeepRobotics Lite3 MID360 installation."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="imu_to_base_tf",
                output="screen",
                arguments=[
                    "--x", "-0.1910", "--y", "-0.02329", "--z", "-0.0947",
                    "--roll", "0.0", "--pitch", "-0.4037", "--yaw", "0.0",
                    "--frame-id", "imu_link", "--child-frame-id", "base_link",
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_to_livox_tf",
                output="screen",
                arguments=[
                    "--x", "0.220", "--y", "0.000", "--z", "0.057",
                    "--roll", "0.0", "--pitch", "0.4037", "--yaw", "0.0",
                    "--frame-id", "base_link", "--child-frame-id", "livox_frame",
                ],
            ),
        ]
    )
