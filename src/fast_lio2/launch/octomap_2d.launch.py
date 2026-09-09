from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 启动 FAST-LIO
        Node(
            package='fast_lio',
            executable='laser_mapping_node',
            name='laser_mapping',
            output='screen',
            parameters=[{'config_file': 'path/to/your/config.yaml'}],
            remappings=[
                ('/cloud_registered', '/filtered_cloud')
            ]
        ),
        
        # 启动 octomap_server 生成 2D octomap
        Node(
            package='octomap_server',
            executable='octomap_server_node',
            name='octomap_server',
            output='screen',
            parameters=[{'frame_id': 'map', 
                        'resolution': 0.05,
                        'height_min': -1.0,
                        'height_max': 1.0,
                        'latch': False}],
            remappings=[
                ('cloud_in', '/cloud_registered'),
                ('octomap_out', '/octomap_full')
            ]
        ),
        
        # 可选：单独发布 2D octomap projection
        Node(
            package='octomap_server',
            executable='octomap_server_node',
            name='octomap_2d_server',
            output='screen',
            parameters=[{'frame_id': 'map',
                        'resolution': 0.05,
                        'height_min': -0.2,
                        'height_max': 0.2}],
            remappings=[
                ('cloud_in', '/cloud_registered'),
                ('octomap_out', '/octomap_2d')
            ]
        )
    ])
