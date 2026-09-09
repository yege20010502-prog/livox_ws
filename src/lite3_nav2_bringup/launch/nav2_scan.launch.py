import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory("lite3_nav2_bringup")
    scan_share = get_package_share_directory("scan_planner")
    nav2_params = os.path.join(bringup_share, "config", "nav2_params.yaml")
    scan_defaults = os.path.join(scan_share, "config", "planner.yaml")
    controller_defaults = os.path.join(scan_share, "config", "controllers.yaml")
    scan_overrides = os.path.join(bringup_share, "config", "scan_lite3_params.yaml")

    map_arg = DeclareLaunchArgument(
        "map", default_value="/root/nav/livox_ws/maps/map831/maps/map_obstacles/map.yaml")
    output_arg = DeclareLaunchArgument(
        "output_cmd_vel", default_value="/scan_safe_cmd_vel",
        description="Dry-run default. Set /cmd_vel only after all checks pass.")

    nodes = [
        Node(package="nav2_map_server", executable="map_server", name="map_server",
             output="screen", parameters=[{"yaml_filename": LaunchConfiguration("map"),
                                            "use_sim_time": False}]),
        Node(package="nav2_planner", executable="planner_server", name="planner_server",
             output="screen", parameters=[nav2_params]),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
             name="lifecycle_manager_nav2_scan", output="screen",
             parameters=[{"autostart": True,
                          "node_names": ["map_server", "planner_server"]}]),
        Node(package="lite3_nav2_bringup", executable="scan_body_odom_bridge",
             output="screen"),
        Node(package="lite3_nav2_bringup", executable="nav2_goal_to_scan",
             output="screen"),
        Node(package="scan_planner", executable="scan_planner_node",
             name="scan_planner_node", output="screen",
             parameters=[scan_defaults, scan_overrides],
             remappings=[("body_pose", "/scan/body_odom"),
                         ("sensor_pose", "/fastlio2/lio_odom"),
                         ("cloud", "/fastlio2/body_cloud"),
                         ("initial_path", "/initial_path")]),
        Node(package="scan_planner", executable="closed_loop_controller",
             name="closed_loop_controller", output="screen",
             parameters=[controller_defaults, scan_overrides],
             remappings=[("body_pose", "/scan/body_odom"),
                         ("cmd_vel", "/scan_cmd_vel")]),
        Node(package="lite3_nav2_bringup", executable="scan_cmd_safety_gate",
             output="screen", parameters=[{"output_cmd": LaunchConfiguration("output_cmd_vel")}]),
    ]
    return LaunchDescription([map_arg, output_arg] + nodes)
