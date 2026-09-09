import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap


def generate_launch_description():
    package_share = get_package_share_directory("lite3_nav2_bringup")
    nav2_share = get_package_share_directory("nav2_bringup")

    default_map = "/root/nav/livox_ws/maps/map831/maps/map_obstacles/map.yaml"
    default_params = os.path.join(package_share, "config", "nav2_params.yaml")

    map_arg = DeclareLaunchArgument("map", default_value=default_map)
    params_arg = DeclareLaunchArgument("params_file", default_value=default_params)
    cmd_vel_arg = DeclareLaunchArgument(
        "cmd_vel_topic",
        default_value="/nav2_cmd_vel",
        description="Safety default. Use /cmd_vel only after localization and dry-run checks pass.",
    )
    rviz_arg = DeclareLaunchArgument("use_rviz", default_value="False")

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[{"yaml_filename": LaunchConfiguration("map"), "use_sim_time": False}],
    )
    map_lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_map",
        output="screen",
        parameters=[{"autostart": True, "node_names": ["map_server"]}],
    )

    navigation = GroupAction([
        SetRemap(src="/cmd_vel", dst=LaunchConfiguration("cmd_vel_topic")),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_share, "launch", "navigation_launch.py")
            ),
            launch_arguments={
                "use_sim_time": "False",
                "autostart": "True",
                "params_file": LaunchConfiguration("params_file"),
                "use_composition": "False",
            }.items(),
        ),
    ])

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_nav2",
        output="screen",
        arguments=["-d", os.path.join(nav2_share, "rviz", "nav2_default_view.rviz")],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    return LaunchDescription([
        map_arg,
        params_arg,
        cmd_vel_arg,
        rviz_arg,
        map_server,
        map_lifecycle,
        navigation,
        rviz,
    ])
