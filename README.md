# Lite3 + MID360 导航操作手册

当前已撤回 test1–test5 控制／规划实验，并与昨天 test1 源码差异中记录的原控制器／状态机对象核对一致；不是直接认定旧提交日期就是正常版本。没有完整实机环境快照，不保证效果已恢复。最新流程以 [当前导航基线](docs/navigation_baseline.md) 为准。贴墙和提前转向风险仍在，先验证开阔区域直线，不测墙角。

当前使用 **map108**。板卡工作目录为 `/root/nav/livox_ws`，ROS 2 为 Humble。本手册只记录目前的使用方法、功能限制和后续优化。

## 1. 当前功能

| 功能 | 状态 |
| --- | --- |
| FAST-LIO 里程计、PCD 地图定位 | 已接入，每次启动后需要重定位并检查对齐 |
| Nav3D 全局地面路径规划 | 已接入 |
| SCAN 实时局部规划与底盘控制 | 实机模式启动，需要现场验收避障和停车 |
| 红色 keepout 禁行区 | Nav3D 规划和轨迹安全检查已接入 |
| 蓝色 passable 可通行区 | 仅保存、预览，尚未影响规划 |
| 黄色 preferred 优先通行区 | 仅保存、预览，尚未影响规划 |
| SCAN 遵守人工禁行区 | 尚未接入，局部绕行可能进入红区 |
| 预置点位、多点任务、到点固定朝向 | 待完善和验收 |
| Web 导航控制台 | 待实现 |

Nav3D 当前关闭自身速度控制器，由 SCAN 控制机器狗。`/nav3d/navigate_to_pose` 返回成功表示规划结果，不能当成机器狗已经到点。

当前全局最终路径净空后校验关闭、离墙权重为 0；当前未启用新增机身转向保护。此基线存在贴墙及拐角风险。

SCAN 规划最高速度恢复 0.75 m/s，实际控制仍限速 vx=0.15、vy=0.10、角速度=0.25；控制器恢复连续时间推进与 0.5 秒预瞄，末端容差恢复 0.15 m。规划／执行速度不一致仍需独立排查，不叠加新转弯算法。

包装启动脚本的全局半径恢复为 0.20 m；局部双圆恢复为半径 0.25 m、前后偏移 0.18 m。全局半径不足以覆盖整机转弯范围，后续需要朝向相关的机身碰撞与路径校验，而不是直接扩大圆形半径堵住窄路。导航输入代码使用 map→base_link，雷达至机身中心静态 TF 仍需实测核验，定位代码和外参未改动。

重启后先在开阔场地测试连续直线，再测左右转弯；有墙拐角暂不作为首轮测试。发生碰撞、定位跳变或地图错位立即用遥控／急停终止导航，停稳后检查并重新定位，重新下发目标，不继续旧路径。当前有位姿／速度超时保护，但不能保证识别“仍持续发布的错误定位”，没有实现撞墙检测或完整定位健康联锁。

## 2. 设备分工与运行链路

### 实机测试录制（板卡执行）




无需人工解锁，原位姿超时与局部碰撞停车保留；新增健康心跳仅用于录制诊断，不再阻断控制。最新验证见 [当前导航基线](docs/navigation_baseline.md)。录制默认轻量话题，第二参数 full 才录全部。

导航启动后、下发目标前，在另一个终端运行：

```bash
cd /root/nav/livox_ws
bash scripts/record_nav_test.sh baseline_straight
```

脚本不启动导航、不发送速度。默认保存关键诊断 ROS topic 的 bag（full 模式保存全部）、节点列表、参数、Git 版本及源码差异到 `test_records/时间_corner_test_进程号/`。测试中观察剩余磁盘空间（全量点云占用较大）；先用遥控／急停停稳机器狗，再在录制终端按 Ctrl-C。Ctrl-C 只结束录制，不会停车。用 `ros2 bag info test_records/对应目录/bag` 检查录制时长与消息数量，将整个对应目录拷贝回来，并附上现场视频、目标点、撞墙／停顿发生时间。没有完整结束或磁盘耗尽可能导致录制不完整。离线分析／回放不得连接实机速度控制链路。


板卡运行 ROS 节点、地图处理和区域采集；可视化电脑运行 RViz 与 PCD Viewer。电脑与板卡需网络互通、ROS_DOMAIN_ID 一致。板卡无需图形界面。

```text
MID360 → FAST-LIO → Localizer → map坐标位姿
PCD地图 → Nav3D全局路径 → nav_data_bridge → SCAN → /cmd_vel → Lite3底盘
实时雷达点云 ────────────────────────────↑
```

板卡手动执行 ROS 命令时先加载环境：

```bash
cd /root/nav/livox_ws
source scripts/ros_env.bash
```

启动脚本自动加载环境。导航时不要同时运行 PGO 和 Localizer，两者都会发布 map→odom；不要重复启动 FAST-LIO。

## 3. 建图和导出

停止导航：

```bash
cd /root/nav/livox_ws
./scripts/start_map1022_nav.sh stop
source scripts/ros_env.bash
```

两个终端分别启动雷达和 PGO：

```bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

```bash
source /root/nav/livox_ws/scripts/ros_env.bash
ros2 launch pgo pgo_launch.py use_rviz:=false
```

PGO launch 自带 FAST-LIO 和静态 TF。慢速扫描通道、多角度回看；允许人员跟随，尽量减少长时间遮挡固定特征。

每次导出使用全新目录，以下名称仅作示例：

```bash
mkdir -m 700 /root/nav/livox_ws/maps/map_new_run01
ros2 service call /pgo/save_maps interface/srv/SaveMaps \
  "{file_path: '/root/nav/livox_ws/maps/map_new_run01', save_patches: true}"
```

确认 success，检查 map.pcd、poses.txt、patches/。已有目录会被覆盖，patches 可能被重建。保留原图。结束建图节点后再启动导航。

## 4. 地图处理：map108

原图 `maps/map108/map.pcd` 有 370614 点、90 个关键帧，估计地面 z≈−0.394 m。已生成 `map_nav_candidate.pcd`。

复用处理命令：

```bash
cd /root/nav/livox_ws
python3 scripts/prepare_pgo_map.py \
  maps/map108/map.pcd maps/map108/map_nav_candidate.pcd \
  --poses maps/map108/poses.txt --xy-margin 2.5 \
  --floor-z -0.394 --below 0.15 --above 2.0
```

保留扫描轨迹外扩 2.5 m、地面以下 0.15 m 到地面以上 2 m。保留原坐标，生成 PCD 和 JSON 报告。已有输出拒绝覆盖，重新处理应换 v2 等名称。其他地图需要重新确定地面高度。

复制到可视化电脑查看：

```bash
pcl_viewer /实际保存路径/map_nav_candidate.pcd
```

检查墙体重影、门洞、细柱、地面连续性。整体扭曲应检查建图，不能仅靠裁剪修复。

可选动态点过滤，另生成待验收候选：

```bash
cd /root/nav/livox_ws
python3 scripts/filter_dynamic_pgo_map.py \
  maps/map108/map_nav_candidate.pcd maps/map108/map_static_candidate.pcd \
  --poses maps/map108/poses.txt --patches maps/map108/patches \
  --resolution 0.15 --min-frame-gap 5 --ground-band 0.15 --floor-z -0.394
```

过滤依据是关键帧重复观测，地面带受保护。这不是人体识别，可能残留人影或删除少量真实细障碍；先对比点云和规划效果再采用。

## 5. 当前栅格参数

包装脚本名称仍为 `start_map1022_nav.sh`，传入完整 PCD 路径即可用于 map108。**不传路径会默认加载 map1022。**

| 参数 | 当前包装脚本值 |
| --- | --- |
| map.resolution | 0.1 m |
| map.min_points_per_voxel | 1 |
| planning.ground_robot_radius | 0.20 m |
| planning.ground_support_depth_cells | 3 |
| map.occupancy_grid_min_z | −0.2 m |
| map.occupancy_grid_max_z | 0.5 m |
| NAV_MOTION_ENABLED | false |
| NAV3D_RVIZ | false |

地图支撑与高度参数来自 map1022 测试，map108 需验证。当前回退半径仅用于基线对照，不保证 0.61×0.37 m 机身在拐角处无碰撞。

3D 体素绿色/橙红色是高度显示，不等于通行结论。查看二维规划层时启用 `Nav3D OccupancyGrid Fallback`：黑色占据、白色自由、灰色未知。

## 6. 安全模式启动 map108

```bash
cd /root/nav/livox_ws
./scripts/start_map1022_nav.sh stop
NAV_MOTION_ENABLED=false NAV3D_ZONE_FILE='' \
  ./scripts/start_map1022_nav.sh \
  /root/nav/livox_ws/maps/map108/map_nav_candidate.pcd
```

进入九宫格，各节点按顺序启动，Localizer 约 36 秒后启动。此模式只做定位和全局规划，不启动 SCAN 和底盘。空区域变量明确禁用区域。不指定 `NAV3D_ZONE_FILE` 时，仅自动加载所选 PCD 同目录下已存在的 `navigation_zones.yaml`；不存在则不加载，绝不套用其他地图的区域。

若看到 `bad file: .../map108/navigation_zones.yaml`，是区域文件尚未保存却使用了加载区域的命令，不是 PCD 缺失。使用本节的无区域命令重启即可；新脚本会在启动节点前检查显式指定的区域文件，缺失时直接报错，不会悄悄忽略禁行区。无需重新编译脚本修改。

F12 退出九宫格并保留运行。会话管理：

```bash
./scripts/start_map1022_nav.sh status
./scripts/start_map1022_nav.sh attach
./scripts/start_map1022_nav.sh stop
```

独立 tmux socket 为 /tmp/lite3_tmux_nav。RUNNING 只代表会话存在，需要 attach 查看节点报错。旧会话存在时不会自动重新加载参数；换图或改参数先 stop。

## 7. 电脑端 RViz

复制配置文件：

```text
src/3D-Nav-Dynamic-EGO/ros2_bridge/nav3d_ros2_bridge/rviz/nav3d.rviz
```

电脑执行：

```bash
rviz2 -d /实际配置路径/nav3d.rviz
```

Fixed Frame 设置 map。显示二维栅格、全局路径、当前位姿和实时点云。实时点云使用 /fastlio2/world_cloud 或抽样主题 /fastlio2/world_cloud_viz，QoS 使用 Best Effort、Volatile。抽样显示不等于全部定位输入。

## 8. 重定位

1. 等节点启动完成，保持机器狗静止。
2. 用 2D Pose Estimate 在真实位置按下、拖出狗头方向，Topic 必须为 /initialpose。
3. 检查定位状态和 TF。
4. 核对实时墙角、门洞等至少两处特征与地图重合。

```bash
source /root/nav/livox_ws/scripts/ros_env.bash
ros2 service call /relocalize_check interface/srv/IsValid "{code: 0}"
ros2 run tf2_ros tf2_echo map base_link
```

valid=true 还需结合真实对齐和 TF 持续更新。每次重启、换图重新定位；旧图点位、区域、初值和锚点不能直接沿用。

固定出发区可选自动定位。确认对齐后学习锚点：

```bash
python3 scripts/auto_relocalize.py maps/map108/map_nav_candidate.pcd --learn-start
```

同一出发区先试算再执行：

```bash
python3 scripts/auto_relocalize.py maps/map108/map_nav_candidate.pcd
python3 scripts/auto_relocalize.py maps/map108/map_nav_candidate.pcd --execute
```

搬到别处时用手动定位。地图与锚点必须一致。

## 9. 全局路径验证

安全模式完成定位后，使用 Topic 为 **/nav3d/goal** 的 2D Goal Pose 点击目标并拖出方向。配置里另有发送到 /nav3d/start 的工具，不要误用。普通 Publish Point /clicked_point 有起终点状态逻辑，日常优先使用 /nav3d/goal。

```bash
source /root/nav/livox_ws/scripts/ros_env.bash
ros2 topic echo /nav3d/status
```

先测近距离直线、再测转弯和绕行。路径首点应靠近当前位置，不穿墙，目标有足够空间。终点四元数输入不代表当前链路已保证固定朝向到点。

## 10. 实机导航和局部避障

只读验证通过后，现场准备好遥控器停车方式，执行：

```bash
cd /root/nav/livox_ws
./scripts/start_map1022_nav.sh stop
NAV_MOTION_ENABLED=true NAV3D_ZONE_FILE='' \
  ./scripts/start_map1022_nav.sh \
  /root/nav/livox_ws/maps/map108/map_nav_candidate.pcd
```

重新定位后发送近距离空旷目标。SCAN 接收实时点云、位姿和 /initial_path，输出 /cmd_vel。局部连续重规划失败上限配置为 5；绕行与停车仍需实机验收。

检查链路：

```bash
source /root/nav/livox_ws/scripts/ros_env.bash
ros2 topic hz /LIO/clouds_lidar
ros2 topic hz /LIO/odom_vehicle
ros2 topic echo /initial_path --once
ros2 topic info /cmd_vel -v
```

频率命令短时观察后 Ctrl-C 退出。低速、空旷场地提前摆放明显障碍，检查局部轨迹改变以及绕不开时停车。节点启动不代表避障已经验证。

异常运动优先使用真实遥控器/急停，停稳后执行包装脚本 stop。结束 tmux 不等于已验证底盘急停，单次发布零速度也不能作为唯一停止保障。

## 11. 人工区域配置

map108 使用独立文件 `maps/map108/navigation_zones.yaml`。电脑加载：

```text
src/3D-Nav-Dynamic-EGO/ros2_bridge/nav3d_ros2_bridge/rviz/nav_zone_editor.rviz
```

```bash
rviz2 -d /实际配置路径/nav_zone_editor.rviz
```

板卡采集红色禁行区，例如 6 个顶点：

```bash
cd /root/nav/livox_ws
source scripts/ros_env.bash
python3 scripts/nav_zone_editor.py maps/map108/navigation_zones.yaml add \
  --type keepout --name forbidden_01 --vertices 6
```

看到 Waiting 后，专用 RViz Publish Point 顺着边界点击；Topic 为 /nav3d/zone_point。至少 3 点，可超过 4 点，最后一点自动闭合；不要自交。内部和边界都禁行，避免覆盖希望可走的走廊。

```bash
python3 scripts/nav_zone_editor.py maps/map108/navigation_zones.yaml list
python3 scripts/nav_zone_editor.py maps/map108/navigation_zones.yaml preview
python3 scripts/nav_zone_editor.py maps/map108/navigation_zones.yaml delete --name forbidden_01
```

preview 在板卡发布，电脑 MarkerArray 订阅 /nav3d/navigation_zone_markers。Ctrl-C 保留文件。

只有先保存区域文件后，才使用下面的命令加载区域做安全规划（未画区域请使用第 6 节）：

```bash
cd /root/nav/livox_ws
./scripts/start_map1022_nav.sh stop
NAV_MOTION_ENABLED=false \
NAV3D_ZONE_FILE=/root/nav/livox_ws/maps/map108/navigation_zones.yaml \
  ./scripts/start_map1022_nav.sh \
  /root/nav/livox_ws/maps/map108/map_nav_candidate.pcd
```

启动日志应包含 Loaded ... keepout navigation zone(s)。修改区域需重启。红框可见只证明预览成功；还需确认规划器加载并测试绕行。

蓝/黄区域可用 --type passable / preferred 保存，但目前不影响路线。SCAN 尚未读取区域，局部绕行可能进入红框。

## 12. 排错与参数核对

| 问题 | 处理 |
| --- | --- |
| navigation_zone_file:= malformed | 更新启动脚本，空变量应省略该 launch 参数 |
| bad file: .../navigation_zones.yaml | 区域文件不存在或不可读；保存文件后加载，或用第 6 节 NAV3D_ZONE_FILE='' 明确禁用区域 |
| Missing PCD map | 检查完整路径、候选文件是否生成 |
| 旧 FAST-LIO / PGO 已运行 | 停止旧来源，避免重复节点与 TF 冲突 |
| map frame does not exist | 等启动完成并重定位，检查定位和 TF |
| lidar_buffer=0 但点云正常 | 瞬间队列为空不等于断流，结合实际更新判断 |
| 点云无显示 | 检查 Topic、QoS、时间戳、TF 和频率 |
| 加区域后 no_path | 禁用区域对照，检查是否封住端点或唯一通道 |
| 普通空地 no_path | 检查地面支撑、高度基准、静态误障碍、真实连通性 |
| 有路径不走 | 检查安全模式开关、SCAN/底盘、路径和速度输入 |
| SCAN 连续失败 | 先停稳，检查局部地图、位姿、路径入口和障碍 |
| GLSL sampler 报错但显示正常 | 可继续显示；实际无法显示时排查图形驱动 |

```bash
source /root/nav/livox_ws/scripts/ros_env.bash
ros2 param get /nav3d_bridge_node map.pcd_path
ros2 param get /nav3d_bridge_node map.resolution
ros2 param get /nav3d_bridge_node map.min_points_per_voxel
ros2 param get /nav3d_bridge_node planning.ground_robot_radius
ros2 param get /nav3d_bridge_node planning.ground_support_depth_cells
ros2 param get /nav3d_bridge_node map.occupancy_grid_min_z
ros2 param get /nav3d_bridge_node map.occupancy_grid_max_z
ros2 param get /nav3d_bridge_node planning.navigation_zone_file
ros2 param get /localizer_node map_path
```

## 13. 构建与文件位置

已有编译结果时启动不需重新构建。改脚本无需编译，改 C++ 节点需构建对应包并重启：

```bash
cd /root/nav/livox_ws
source scripts/ros_env.bash
colcon build --packages-select scan_planner --symlink-install
colcon build --base-paths src/3D-Nav-Dynamic-EGO/ros2_bridge/nav3d_ros2_bridge --packages-select nav3d_ros2_bridge --symlink-install
```

| 文件 | 用途 |
| --- | --- |
| scripts/start_map1022_nav.sh | 地图路径、固定参数与会话管理 |
| scripts/start_nav3d_scan_tmux.sh | 节点启动与实机开关 |
| scripts/prepare_pgo_map.py | 地图裁剪 |
| scripts/filter_dynamic_pgo_map.py | 动态点过滤候选 |
| scripts/nav_zone_editor.py | 区域记录和预览 |
| scripts/auto_relocalize.py | 自动定位 |
| src/3D-Nav-Dynamic-EGO/ros2_bridge/nav3d_ros2_bridge/config/nav3d_bridge.yaml | 全局规划和安全配置 |
| src/SCAN-Planner-Ros2/src/planner/plan_manage/config/planner.yaml | 局部规划配置 |
| src/FASTLIO2_ROS2-main/localizer/config/localizer.yaml | 定位配置 |
| maps/map108/ | 当前地图和关键帧 |

## 14. 后续优化

按以下顺序实施、验收：

1. map108 定位对齐、运动定位稳定、普通及窄通道规划；对比过滤前后地图。
2. SCAN 绕障、无路停车、输入断流停车、遥控器接管、速度超时停止。
3. 将 keepout 接入 SCAN；实现 passable 静态误障碍修补、preferred 软代价。实时障碍和禁行区始终优先。
4. 用整机长宽与转弯范围完善净空检查，在保留窄路的同时减少贴墙。
5. 改善动态点过滤，保护地面与真实细小障碍，提供处理前后差异报告。
6. 按地图绑定参数、区域、锚点和点位，提供名称清楚的通用启动入口。
7. 实现点位存取、多点任务、真实到点判断、四元数朝向调整、超时与任务取消。
8. 提供 Web 控制台：地图选择、点位管理、任务执行、状态、取消和停车。
