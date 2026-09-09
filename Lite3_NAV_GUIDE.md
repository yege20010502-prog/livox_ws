# Lite3 Single-Floor Navigation Guide

See the ROS2 startup and localization procedure in the assistant response:
1. source /opt/ros/humble/setup.bash and /root/nav/livox_ws/install/setup.bash
2. start Livox driver, then exactly one FAST-LIO2
3. start map_server and activate its lifecycle
4. start localizer_node only (do not use localizer_launch.py)
5. call /relocalize, then require /relocalize_check valid=true
6. run RViz on the server with Fixed Frame map and /map Transient Local

Board maps:
/root/nav/livox_ws/maps/map831/map_indoor_clean.pcd
/root/nav/livox_ws/maps/map831/map_obstacles.pcd
/root/nav/livox_ws/maps/map831/maps/map_obstacles/map.yaml

Required TF:
map -> odom -> imu_link -> base_link -> livox_frame

Do not start PGO while localizer is running. FAST-LIO body_cloud and lio_odom
must each have Publisher count 1. valid=false means localization failed and
Nav2 must not be started.

## Board commands

    cd /root/nav/livox_ws
    source /opt/ros/humble/setup.bash
    source install/setup.bash

    ros2 topic hz /livox/lidar
    ros2 topic hz /livox/imu
    ros2 launch fastlio2 lio_launch.py

Check that body_cloud and lio_odom are about 10 Hz and each has one
publisher:

    ros2 topic hz /fastlio2/body_cloud
    ros2 topic hz /fastlio2/lio_odom
    ros2 topic info /fastlio2/body_cloud -v
    ros2 topic info /fastlio2/lio_odom -v

Start the map:

    ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=/root/nav/livox_ws/maps/map831/maps/map_obstacles/map.yaml -p use_sim_time:=false
    ros2 lifecycle set /map_server configure
    ros2 lifecycle set /map_server activate
    ros2 lifecycle get /map_server

Start localizer only (not localizer_launch.py):

    ros2 run localizer localizer_node --ros-args -p config_path:=/root/nav/livox_ws/install/localizer/share/localizer/config/localizer.yaml


    重定位流程
    1. 确认节点
        开发板执行：
        ros2 node list | sort | uniq -c | sort -nr
        ros2 service list | grep relocalize
        应看到：
        /localizer_node
        /relocalize
        /relocalize_check
        并确认：
        ros2 topic info /fastlio2/body_cloud -v
        ros2 topic info /fastlio2/lio_odom -v
        两个话题都必须只有一个发布者。
    2. 在 RViz 获取机器人在旧地图中的初始位置
        服务器执行：
        ros2 topic echo /initialpose --once
        然后在 RViz：
        1. Fixed Frame 设置为 map。
        2. 点击 2D Pose Estimate。
        3. 在地图中点击机器狗实际位置。
        4. 沿机器狗实际朝向拖动箭头。
        5. 松开鼠标。
        记录输出中的：
        position.x
        position.y
        orientation.z
        orientation.w
    3. 计算 yaw
        python3 -c "import math; z=填写四元数z; w=填写四元数w; print(math.atan2(2*w*z,1-2*z*z))"
        例如：
        python3 -c "import math; z=0.38; w=0.92; print(math.atan2(2*w*z,1-2*z*z))"
        计算结果单位是弧度。
    4. 调用重定位服务
        将 X、Y、YAW 替换成真实值：
        ros2 service call /relocalize \
        interface/srv/Relocalize \
        "{pcd_path: '/root/nav/livox_ws/maps/map831/map_indoor_clean.pcd',
        x: X,
        y: Y,
        z: 0.0,
        yaw: YAW,
        pitch: 0.0,
        roll: 0.0}"
        例如：
        ros2 service call /relocalize \
        interface/srv/Relocalize \
        "{pcd_path: '/root/nav/livox_ws/maps/map831/map_indoor_clean.pcd',
        x: 1.2,
        y: -0.8,
        z: 0.0,
        yaw: 1.57,
        pitch: 0.0,
        roll: 0.0}"
        上面的数字只是格式示例，不能直接套用。
    5. 判断是否成功
        等待5～10秒：
        ros2 service call /relocalize_check \
        interface/srv/IsValid \
        "{code: 0}"
        只有以下结果才算成功：
        valid: true
        再检查：
        ros2 run tf2_ros tf2_echo map odom
        单层导航要求：
        z ≈ 0
        roll ≈ 0°
        pitch ≈ 0°
        同时在 RViz 中确认：
        /map_cloud              绿色
        /fastlio2/world_cloud   红色
        红色实时点云中的墙体应与绿色地图墙体基本重合。
    6. 重定位失败时
        如果返回：
        valid: false
        不要启动 Nav2，也不要随意修改 FAST-LIO 参数。优先检查：
        - RViz点击的机器人位置是否正确；
        - 箭头朝向是否正确；
        - x/y 是否为地图坐标；
        - yaw 是否为弧度；
        - 机器狗是否在地图覆盖范围内；
        - 是否同时运行了 PGO 或第二个 LIO。
