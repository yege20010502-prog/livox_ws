# Lite3 + Livox MID-360 建图与导航操作手册

> ROS 2 Humble；工作空间 `/root/nav/livox_ws`；当前地图 `/root/nav/livox_ws/maps/map9802`

## 1. 系统链路

### 建图

```text
Livox 驱动 -> /livox/lidar、/livox/imu
-> 唯一一个 FAST-LIO2 lio_node -> PGO -> PCD 地图
```

### 导航

```text
Livox -> FAST-LIO2
-> /fastlio2/lio_odom、/fastlio2/body_cloud
-> localizer -> map→odom TF
-> Nav2 -> /initial_path
-> scan_planner -> /planning/bspline、/scan_cmd_vel
-> scan_cmd_safety_gate -> /scan_safe_cmd_vel
-> /cmd_vel -> dog_control_bridge_node -> UDP -> Lite3
```

## 2. 环境与编译

新终端先执行：

```bash
source /opt/ros/humble/setup.bash
source /root/nav/livox_ws/install/setup.bash
```

源码修改后：

```bash
cd /root/nav/livox_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

只构建导航相关包：

```bash
colcon build --packages-up-to \
  localizer lite3_nav2_bringup scan_planner \
  --symlink-install
```

若出现 `libbspline_opt.a: file too short`，仅清理损坏包再构建：

```bash
rm -rf /root/nav/livox_ws/build/bspline_opt \
       /root/nav/livox_ws/install/bspline_opt
colcon build --packages-up-to bspline_opt --symlink-install
```

## 3. 建图流程

### 3.1 启动前

1. 电池充足、雷达固定牢靠。
2. 启动后先静止数秒完成 IMU 初始化。
3. PGO launch 已包含 FAST-LIO2 时，不要额外启动 FAST-LIO2。
4. 确认只有一个 `lio_node`：

```bash
pgrep -af lio_node
ros2 node list | grep lio
```

### 3.2 启动 Livox 驱动

使用当前 MID-360 驱动 launch，然后检查：

```bash
ros2 topic info /livox/lidar -v
ros2 topic info /livox/imu -v
ros2 topic hz /livox/imu
ros2 topic echo /livox/imu --once
```

`/livox/lidar` 和 `/livox/imu` 均应只有一个 Publisher。

### 3.3 启动 PGO

```bash
cd /root/nav/livox_ws
source install/setup.bash
ros2 launch pgo pgo_launch.py
```

无显示器时关闭 PGO launch 中的 RViz，或使用其 `use_rviz:=false` 参数（以 launch 实际声明为准）。不要在 RK3588 上远程渲染高负载 RViz。

确认：

```bash
ros2 topic hz /fastlio2/lio_odom
ros2 topic hz /fastlio2/body_cloud
```

### 3.4 建图行走

- 先低速直行确认点云稳定；
- 转弯降低角速度，避免连续原地快速旋转；
- 尽量走闭环并回到起点附近；
- 避免剧烈俯仰、急转和快速场景切换；
- 一旦出现墙体多重重影、长线、点云跳飞，立即停止并保留日志/rosbag。

若出现：

```text
NO Effective Points!
VoxelGrid ... Integer indices would overflow
lio_node ... exit code -11
```

说明 LIO 已异常，飞掉后的地图不应继续使用。

### 3.5 保存地图

先确认服务：

```bash
ros2 service list | grep -E 'save|map'
ros2 service type /pgo/save_maps
```

若服务类型为 `interface/srv/SaveMaps`：

```bash
mkdir -p /root/nav/livox_ws/maps/map9802
ros2 service call /pgo/save_maps interface/srv/SaveMaps \
"{file_path: '/root/nav/livox_ws/maps/map9802', save_patches: true}"
```

必须等保存结束后再停止 launch 或断电。

## 4. PCD 地图处理

### 4.1 文件版本

建议保留：

```text
map_raw.pcd       原始地图
map_edited.pcd    手动清理明显畸变噪声后的地图
map_filtered.pcd  高度及离群点过滤后的地图
```

不要覆盖唯一原图。

### 4.2 检查 PCD 格式

```bash
head -n 15 map_edited.pcd
```

若为 `DATA binary_compressed`，而使用方不支持，转换为 ASCII：

```bash
pcl_convert_pcd_ascii_binary \
  map_edited.pcd map_ascii.pcd 0
```

### 4.3 高度裁剪

正确选项是 `-field`，不是 `-field_name`：

```bash
cd /root/nav/livox_ws/maps/map9802
pcl_passthrough_filter \
  map_edited.pcd map_no_ceiling.pcd \
  -field z -min -0.25 -max 1.50 \
  -inside 1 -keep 0
```

高度上下限需结合雷达安装高度调整：保留会碰到机身的固定障碍，删除地面和天花板。手动编辑时保留墙、柱和通道边界，只删除飞点、重复墙和孤立长线。

## 5. PCD 转二维地图

### 5.1 构建与运行

```bash
cd /root/nav/livox_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select pcd2pgm --symlink-install
source install/setup.bash
ros2 launch pcd2pgm pcd2pgm_launch.py
```

板卡无 X11 时 RViz 报 `could not connect to display` 不影响 `pcd2pgm_node`。关闭 launch 内 RViz，在有显示器的电脑单独运行 RViz。

当前一次有效处理数据约为：

```text
原始 30602 点
高度过滤后 12549 点
半径离群过滤后 12271 点
```

过滤后点数过少时检查高度范围、离群半径和最小邻居数。

### 5.2 保存 Nav2 地图

保持 pcd2pgm 正在发布 `/map`：

```bash
mkdir -p /root/nav/livox_ws/maps/map9802/nav2
ros2 run nav2_map_server map_saver_cli \
  -f /root/nav/livox_ws/maps/map9802/nav2/map
```

生成：

```text
map.pgm
map.yaml
```

保存后可停止 pcd2pgm。导航时由 map_server 直接加载 YAML/PGM，不需要运行 pcd2pgm。

## 6. 导航启动

导航不需要运行 PGO、pcd2pgm 或 map_saver。

### 6.1 启动顺序

1. Livox 驱动。
2. 唯一一个 FAST-LIO2。
3. localizer：

```bash
source /root/nav/livox_ws/install/setup.bash
ros2 run localizer localizer_node --ros-args \
  -p config_path:=/root/nav/livox_ws/install/localizer/share/localizer/config/localizer.yaml
```

4. Nav2/scan planner：

```bash
ros2 launch lite3_nav2_bringup nav2_scan.launch.py
```

等待 `Managed nodes are active`，然后检查：

```bash
ros2 lifecycle get /map_server
ros2 lifecycle get /planner_server
```

两者均应为 `active [3]`。

## 7. 重定位

localizer 当前不接收 RViz `/initialpose`，必须调用 `/relocalize`。

本次曾成功使用：

```bash
ros2 service call /relocalize interface/srv/Relocalize \
"{pcd_path: '/root/nav/livox_ws/maps/map9802/map_edited.pcd', \
x: 1.616, y: 2.533, z: 0.0, \
yaw: 3.000, pitch: 0.0, roll: 0.0}"
```

该初值只适用于当时的机器狗物理位置和当次 FAST-LIO `odom`。FAST-LIO 重启或机器狗搬动后不保证有效。

验证：

```bash
ros2 service call /relocalize_check interface/srv/IsValid "{code: 0}"
ros2 run tf2_ros tf2_echo map base_link
```

要求：`valid=True`、TF 连续稳定、点云与地图墙体和阳台方向一致。

RViz 用：

```text
Fixed Frame: map
PointCloud2: /fastlio2/body_cloud
Decay Time: 0
```

不要用累计 `/fastlio2/world_cloud` 判断当前对齐，否则旧 TF 会产生残影。

定位明显错误时禁止导航，关闭速度使能并重新确定初值。`Reject unstable map->odom correction` 表示修正超过保护阈值，不等同于 CPU 性能不足。

## 8. 安全门

安全门检查：里程计、当前点云、速度命令以及 `map→base_link` TF 的新鲜度。

当前运行时验证可用参数：

```bash
ros2 param set /scan_cmd_safety_gate sensor_timeout 2.0
ros2 param set /scan_cmd_safety_gate command_timeout 1.0
ros2 param set /scan_cmd_safety_gate tf_timeout 2.0
```

这些值重启后丢失，后续要永久写入 YAML/launch。

当前 `/scan_navigation_enable` 没有正式发布者。临时启用：

```bash
ros2 topic pub -r 2 /scan_navigation_enable std_msgs/msg/Bool \
"{data: true}"
```

测试后先停止持续发布，再关闭：

```bash
ros2 topic pub --once /scan_navigation_enable std_msgs/msg/Bool \
"{data: false}"
```

没有 `/scan_cmd_vel` 时，当前源码的 `/scan_navigation_ready=false` 是正常的，因为 ready 也要求命令新鲜。

## 9. 底盘 `/cmd_vel`

`dog_control_bridge_node` 订阅 `/cmd_vel`，并通过 UDP 向 `192.168.1.120:43893` 控制机器狗：

```cpp
vx  = linear.x;
vy  = linear.y;
yaw = -angular.z;
```

当前代码仅在收到零 Twist 时调用 `sendStop()`；没有实现 `/cmd_vel` 断流超时停止。心跳和状态查询也不是速度看门狗，所以保留上层安全门。

当前安全门输出 `/scan_safe_cmd_vel`，底盘监听 `/cmd_vel`。临时桥接：

```bash
ros2 run topic_tools relay /scan_safe_cmd_vel /cmd_vel
```

检查：

```bash
ros2 topic info /scan_safe_cmd_vel -v
ros2 topic info /cmd_vel -v
```

永久方案是把安全门设置为：

```yaml
input_cmd: /scan_cmd_vel
output_cmd: /cmd_vel
```

永久修改后不得再运行 relay，避免多个 Publisher 竞争。

底盘代码已发现的风险：

1. `yaw = -yaw` 必须验证左右方向；
2. 没有 ROS 命令超时看门狗；
3. 节点退出前没有明确发送停止；
4. `sendStop()` 调用 `sendMove(0,0,0)`，而 `sendMove()` 可能因趴下状态先触发自动站立。

## 10. 首次导航测试

### 10.1 测试前

```bash
ros2 lifecycle get /map_server
ros2 lifecycle get /planner_server
ros2 service call /relocalize_check interface/srv/IsValid "{code: 0}"
ros2 run tf2_ros tf2_echo map base_link
ros2 topic info /cmd_vel -v
```

确认 map/planner active、定位有效、TF 稳定、底盘节点订阅 `/cmd_vel`，并确保物理急停可用。

### 10.2 下发目标

1. 开启临时 navigation enable。
2. RViz Fixed Frame 设为 `map`。
3. 点击 `2D Goal Pose`。
4. 第一次只选前方 0.3～0.5 m 白色空闲区域。
5. 避开黑色障碍、膨胀区和地图边界。

监控：

```bash
ros2 topic echo /goal_pose --once
ros2 topic echo /initial_path --once
ros2 topic echo /planning/bspline --once
ros2 topic echo /scan_cmd_vel
ros2 topic echo /scan_safe_cmd_vel
ros2 topic echo /cmd_vel
```

正常链路：

```text
goal -> initial_path -> bspline -> scan_cmd_vel
-> safety gate -> cmd_vel -> 底盘
```

第一次只进行短直线和小角度转向测试。若要求左转但机器狗右转，检查底盘代码中的 `yaw = -yaw`。

## 11. 膨胀层与窄路

查看参数：

```bash
ros2 param list /planner_server | \
grep -E 'footprint|robot_radius|inflation_radius|cost_scaling'
```

调整原则：

- footprint/robot_radius 必须符合真实机器狗尺寸；
- 可适当降低 inflation_radius；
- 可增大 cost_scaling_factor 使代价更快衰减；
- 清理二维地图离散黑点；
- 不得把机器人半径设得小于真实半宽来强行通过。

## 12. 急停与关闭

异常时：先停止持续 enable，然后执行：

```bash
ros2 topic pub --once /scan_navigation_enable std_msgs/msg/Bool \
"{data: false}"
ros2 topic pub --once /dog_control/command std_msgs/msg/String \
"{data: stop}"
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

正常关闭顺序：关闭 enable、确认速度为零、停止导航、停止 localizer、停止 FAST-LIO2、停止驱动，最后断电。

## 13. 重启后

无需重做：地图、Git、SSH 密钥、源码和未损坏的编译产物。

需要重做：source 环境、启动驱动/FAST-LIO/localizer/Nav2、重新定位，以及尚未永久保存的安全门参数、enable 和 relay。

## 14. Git 记录

仓库：`https://github.com/yege20010502-prog/livox_ws`

修改前建议新建分支：

```bash
cd /root/nav/livox_ws
git status
git switch -c navigation-safety-fix
```

修改和测试后：

```bash
git add src docs
git diff --cached
git commit -m "Improve navigation safety and cmd_vel bridge"
git push -u origin navigation-safety-fix
```

## 15. 待完成的永久改进

1. 安全门直接输出 `/cmd_vel`；
2. 永久保存三个 timeout；
3. 增加正式的 navigation enable 管理；
4. 修复旧规划时间戳；
5. 为底盘桥接增加 cmd_vel 超时停止；
6. 修复 stop 可能触发自动站立；
7. 参数化 IP、端口、限速和 yaw 方向；
8. 校准 footprint/inflation；
9. 制作一键启动流程；
10. 测试定位、点云和底盘掉线保护。
