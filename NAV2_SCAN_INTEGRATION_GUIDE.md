# Lite3 Nav2 + SCAN 启动与测试

## 架构

- Nav2 `planner_server`：在静态二维地图中计算全局路径。
- `nav2_goal_to_scan`：接收 RViz `/goal_pose`，调用 Nav2，并把 `map` 路径按最新 TF 转为 `odom` 下的 `/initial_path`。
- SCAN：`navi_mode=3`，使用 FAST-LIO 实时点云进行局部重规划和避障。
- `scan_body_odom_bridge`：生成 SCAN 所需的 `odom -> base_link` 机身里程计。
- `scan_cmd_safety_gate`：默认关闭；里程计、点云、TF或速度指令超时就输出零速。

## 1. 编译

```bash
cd /root/nav/livox_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select localizer lite3_nav2_bringup scan_planner --symlink-install
source install/setup.bash
```

如果 `scan_planner` 提示依赖尚未构建，改用：

```bash
colcon build --packages-up-to scan_planner lite3_nav2_bringup localizer --symlink-install
source install/setup.bash
```

## 2. 启动定位基础链路

先启动 Livox 驱动、FAST-LIO、静态 TF 和 localizer。不要启动旧的
`jie_path_node`、`d1_controller`、完整 Nav2 bringup 或 `dog_control_bridge`。

确认只有一个发布者负责每条 TF：

```bash
ros2 topic info /tf --verbose
ros2 run tf2_ros tf2_echo map base_link
ros2 topic hz /fastlio2/lio_odom
ros2 topic hz /fastlio2/body_cloud
```

重新编译后的 localizer 会对 `map -> odom` 使用平面约束、2 cm/0.015 rad
死区、低通滤波和单帧跳变拒绝。定位成功后先静止观察 20 秒，确认 TF 不再明显摆动。

## 3. 干跑启动 Nav2 + SCAN

默认输出是隔离话题 `/scan_safe_cmd_vel`，不会接到机器狗：

```bash
ros2 launch lite3_nav2_bringup nav2_scan.launch.py
```

检查节点和数据：

```bash
ros2 lifecycle get /map_server
ros2 lifecycle get /planner_server
ros2 topic hz /scan/body_odom
ros2 topic hz /grid_map/occupancy
ros2 topic echo /scan_navigation_ready
```

在 RViz 中设置 Fixed Frame 为 `map`，用 `2D Goal Pose` 发布目标。正常时应看到：

```bash
ros2 topic echo /initial_path --once
ros2 topic echo /planning/bspline --once
ros2 topic echo /scan_cmd_vel
```

`/initial_path.header.frame_id` 必须是 `odom`。

## 4. 验证安全门控

门控默认关闭。先在隔离输出下启用：

```bash
ros2 topic pub --once /scan_navigation_enable std_msgs/msg/Bool "{data: true}"
ros2 topic echo /scan_safe_cmd_vel
```

随时停止：

```bash
ros2 topic pub --once /scan_navigation_enable std_msgs/msg/Bool "{data: false}"
```

断开点云或里程计时，`/scan_navigation_ready` 应变为 `false`，隔离输出应立即归零。

## 5. 最后才接真实机器狗

先关闭旧启动，再显式把安全门控输出接到 `/cmd_vel`：

```bash
ros2 launch lite3_nav2_bringup nav2_scan.launch.py output_cmd_vel:=/cmd_vel
```

保持门控关闭，随后启动 `dog_control_bridge`，确认 `/cmd_vel` 为零；清空机器人周围，
设置一个 0.5 米以内的目标，最后才启用：

```bash
ros2 topic pub --once /scan_navigation_enable std_msgs/msg/Bool "{data: true}"
```

当前速度硬限制为 `vx=0.15 m/s`、`vy=0.10 m/s`、`wz=0.25 rad/s`。
