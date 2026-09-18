# Lite3 Nav3D + SCAN 启动流程

本文对应以下唯一控制链：

```text
MID360 → FAST-LIO2 → Localizer/TF
                       ↓
Nav3D → /nav3d/trajectory → topic_bridge → /initial_path
                                                ↓
                                             SCAN
                                                ↓
                                         /scan_cmd_vel
                                                ↓
                                topic_bridge 安全门与限速
                                                ↓
                                            /cmd_vel
                                                ↓
                                    dog_control_bridge
```

Nav3D 只生成全局路径，SCAN 是唯一轨迹跟踪器，`topic_bridge` 是唯一的
`/cmd_vel` 发布端。不要同时启动旧 Nav2+SCAN、`trajectory_tracker` 或其他
`/cmd_vel` 转发节点。

## 一、修改后首次编译

```bash
cd /root/nav/livox_ws
source /opt/ros/humble/setup.bash

colcon build \
  --packages-select \
  fastlio2 \
  localizer \
  topic_bridge \
  sensor_extrinsic \
  scan_planner \
  nav3d_ros2_bridge \
  dog_control_bridge \
  --symlink-install \
  --event-handlers console_direct+
```

每个新终端先加载环境：

```bash
source /root/nav/livox_ws/scripts/ros_env.bash
```

## 二、tmux 一键启动

第一次赋予执行权限：

```bash
chmod +x /root/nav/livox_ws/scripts/start_nav3d_scan_tmux.sh
```

默认不在 RK3588 上启动 RViz：

```bash
/root/nav/livox_ws/scripts/start_nav3d_scan_tmux.sh \
  /root/nav/livox_ws/maps/map9802/map_edited.pcd
```

一键脚本会自动加载 `scripts/ros_env.bash`，运行它之前不需要手动 `source`，
也不需要每次手动删除 `AMENT_PREFIX_PATH` 等环境变量。

如果确实需要在板卡上打开 Nav3D RViz，只打开这一个：

```bash
NAV3D_RVIZ=true \
/root/nav/livox_ws/scripts/start_nav3d_scan_tmux.sh \
  /root/nav/livox_ws/maps/map9802/map_edited.pcd
```

脚本只创建一个 `ALL-NODES` 窗口，整个屏幕分成 `3×3` 九宫格。9个
程序全部同时显示，每个分屏顶部都有 `1/9` 到 `9/9` 的步骤编号和名称。
重定位、目标点和停车命令请在另一个普通终端执行。

tmux操作：

```text
Ctrl-b n       下一个窗口
Ctrl-b p       上一个窗口
Ctrl-b 数字    选择窗口
Ctrl-b 方向键  选择分屏
Ctrl-b d       退出界面但保持节点运行
F12            直接退出tmux界面但保持节点运行
```

重新进入：

```bash
tmux attach -t lite3_nav3d
```

停止整套程序：先关闭导航门并停车，然后在tmux中逐个 `Ctrl-c`。确认后可执行：

```bash
tmux kill-session -t lite3_nav3d
```

## 三、逐个手动启动

每条命令使用一个独立终端，并先执行环境脚本。

### 1. MID360

```bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

### 2. FAST-LIO2和Lite3静态TF

```bash
ros2 launch fastlio2 lio_launch.py rviz:=false
```

`lio_launch.py` 已包含 `imu_link → base_link → livox_frame`，不要再重复启动
`lite3_tf.launch.py`。

### 3. Nav3D当前位姿桥接

```bash
python3 /root/nav/livox_ws/src/pose_bridge.py
```

### 4. SCAN传感器实时位姿

```bash
ros2 run sensor_extrinsic lidar_extrinsic_publisher
```

### 5. 路径、点云和安全控制桥接

```bash
ros2 run topic_bridge all_bridges
```

### 6. Nav3D全局规划器

```bash
ros2 launch nav3d_ros2_bridge nav3d_bridge.launch.py \
  pcd_path:=/root/nav/livox_ws/maps/map9802/map_edited.pcd \
  frame_id:=map \
  planning_traversability:=ground \
  rviz:=true \
  rosbridge:=false
```

### 7. SCAN局部规划和闭环控制器

```bash
ros2 launch scan_planner run.launch.py \
  is_real_world:=true \
  navi_mode:=3 \
  sensor_type:=lidar \
  controller_mode:=closed_loop \
  use_sim_time:=false
```

### 8. Lite3底盘控制桥接

```bash
ros2 run dog_control_bridge dog_control_bridge
```

### 9. 最后启动Localizer

```bash
ros2 run localizer localizer_node \
  --ros-args \
  -p config_path:=/root/nav/livox_ws/install/localizer/share/localizer/config/localizer.yaml
```

## 四、重定位

将示例中的 `x/y/yaw` 换成机器狗真实初始位姿：

```bash
ros2 service call /relocalize interface/srv/Relocalize \
  "{pcd_path: '/root/nav/livox_ws/maps/map9802/map_edited.pcd', x: 0.0, y: 0.0, z: 0.0, yaw: 0.0, pitch: 0.0, roll: 0.0}"

ros2 service call /relocalize_check interface/srv/IsValid "{code: 0}"
timeout 5 ros2 run tf2_ros tf2_echo map base_link
```

必须看到 `valid=True`，并且 `map → base_link` 连续有效。

## 五、导航前检查

机器狗先保持趴下或架空：

```bash
ros2 topic info /cmd_vel -v | \
grep -E 'Publisher count:|Node name:|Endpoint type:'

ros2 topic type /LIO/odom_imu

ros2 topic echo /nav3d/current_pose --once | grep frame_id
ros2 topic echo /LIO/odom_vehicle --once | grep -E 'frame_id:|child_frame_id:'
```

预期：

```text
/cmd_vel只有all_bridges一个发布者
/LIO/odom_imu是nav_msgs/msg/Odometry
/nav3d/current_pose的frame_id是map
/LIO/odom_vehicle是map → base_link
```

## 六、下发目标并使能

先选择前方 `0.3～0.5 m` 的无障碍短目标。路径生成后检查：

```bash
ros2 topic echo /initial_path --once | grep frame_id
ros2 topic echo /scan_navigation_ready --once
```

只有返回 `data: true` 才能使能：

```bash
ros2 topic pub --once /scan_navigation_enable \
  std_msgs/msg/Bool "{data: true}"
```

## 七、停车

```bash
ros2 topic pub --once /scan_navigation_enable \
  std_msgs/msg/Bool "{data: false}"

ros2 topic pub --once /dog_control/command \
  std_msgs/msg/String "{data: stop}"
```

底盘 `invert_yaw` 当前默认是 `false`。完成架空测试，确认正角速度实际向左转后，
再进行落地导航测试。
