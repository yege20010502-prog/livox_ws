# Lite3 + MID360：Nav3D / SCAN 导航进度与操作

本 README 记录截至 **2026-09-22** 的实机定位、地图和导航排查。当前目标是在已知地图上完成固定出发区的多点导航。**当前 map918 在实际可通行区域含可疑的连续高处占据点，尚未验收多点实机运行；不要把 `valid=True` 当作可发目标的唯一条件。**

## 当前运行链路

```text
MID360 → FAST-LIO2 (odom → imu_link)
                  → localizer (map → odom)
                  → Nav3D 全局路径 → /nav3d/trajectory
                  → nav_data_bridge → /initial_path
                  → SCAN 局部规划/闭环控制 → /cmd_vel → Lite3 底盘桥
```

入口脚本：[scripts/start_nav3d_scan_tmux.sh](scripts/start_nav3d_scan_tmux.sh)。map918 当前应使用仅旋转调平且兼容 Nav3D 的 `maps/map918/map_height_leveled_rotation_only_nav.pcd`；重定位和 Nav3D 必须使用同一 PCD。不要同时运行 PGO 和 localizer 来发布 `map → odom`，也不要重复启动 FAST-LIO。脚本默认 `NAV_MOTION_ENABLED=false`，不启动 SCAN 和底盘桥；也不会自动完成重定位。

## 2026-09-22 地图问题与是否重扫

- 在用户标记的可通行位置，约 `map (1.05, 0.00)`，当前导航 PCD 的一个 0.2 m 栅格柱同时含地面点和 `z≈0.1–0.5 m` 的密集高点；其中 `z≈0.1 m` 有 39 个原始点、`z≈0.3 m` 有 107 个。这不是少数孤立点，Nav3D 会把它当作障碍。该处扩大至半径 0.3 m 后，10 帧实时 `/fastlio2/world_cloud` 只看到 `z≈-0.1 m` 的 210 个点，未重现静态地图的高点；但单一视角不能排除遮挡。因此暂称**可疑重影/高度不一致**，不能直接删点或把红格设为空地。
- 当前 `/nav3d/occupied_grid` 把 `z=0.2–0.9 m` 内任意占据体素投影为整格障碍；橙红色 Marker 则仅表示同一 XY 柱内“比最低占据点高超过 0.3 m”，**不是机器人可通行性的结论**。绿色也不保证能走。严格地面搜索下，对目标 `(1.81, 0.06)` 返回 `no_path`；把端点吸附半径再调大只会掩盖地图问题。
- 如从不同观察位置重复确认该通道没有对应高处实物，**建议重新扫描一张地图**，而不是在旧 PCD 上大面积强制清障。保留 map918，不覆盖、不删除，以便比较和回退。
- Lite3 已知尺寸：机身 `0.61 × 0.37 m`，站立机身中心离地约 `0.406 m`；雷达相对中心前方 `0.22 m`，俯仰实际为 `23.13°`。当前 TF 角度与 FAST-LIO 雷达–IMU 外参一致。`base_link` 物理原点高度尚未实测确认，**不要为了修地图直接改外参**；若以后确实改外参，需重新评估已经用旧外参建立的地图。

### 重扫前先核对 PGO

当前 PGO 配置在 `src/FASTLIO2_ROS2-main/pgo/config/pgo.yaml`：关键帧平移阈值 `0.5 m`、转角阈值 `10°`、回环搜索半径 `1.0 m`、最小时间间隔 `60 s`、ICP 分数阈值 `0.15`。实现使用 `ExactTime` 配对 `/fastlio2/body_cloud` 与 `/fastlio2/lio_odom`；导出地图只包含已接收且被选作关键帧的数据。回环实现从半径内挑第一个符合时间条件的候选，ICP 最大对应距离为 `10 m`，目前缺少独立的回环位姿跳变/几何一致性门槛。**这些是代码风险，不等于已经证实某个回环导致 map918 错误。**

**已确认并修复的 PGO 输入问题**：FAST-LIO 当前以 `SensorDataQoS`（`BEST_EFFORT`）发布 `/fastlio2/body_cloud`，旧 PGO 对点云请求 `RELIABLE`，二者不兼容，PGO 的 `ExactTime` 同步可能始终收不到云。2026-09-22 已把 PGO 的点云订阅改为 `SensorDataQoS`、保留里程计的可靠订阅，并通过 `colcon build --packages-select pgo --symlink-install` 编译。修改前备份：`src/FASTLIO2_ROS2-main/pgo/src/pgo_node.cpp.20260922_before_cloud_sensor_qos.bak`。**尚未实机重扫验证**，启动新 PGO 后必须确认关键帧数量随移动增长；仅看到服务存在或 `/pgo/loop_markers` 无数据不能算通过。

重新扫描时先用 `ros2 topic hz` 短时确认 `/livox/lidar`、`/fastlio2/body_cloud`、`/fastlio2/lio_odom` 持续约 10 Hz；用 `ros2 topic info /fastlio2/body_cloud -v` 确认 PGO 订阅端与发布端 QoS 匹配，并观察 PGO 关键帧随移动增长。`ros2 topic hz` 用完正常 `Ctrl-C` 退出，不要长时间叠加点云订阅。若 PGO 报输入队列满、时间乱序、回环后地图墙体突然重影/跳变，先停止该次建图并保留日志，不应靠放宽 ICP 阈值继续。采集路线要覆盖通道两侧和明显特征，低速、平稳、多角度回看，避免只在重复结构附近做快速回环。PGO 的 `map→odom` 与导航 localizer 不能同时发布，且不能同时运行两个 FAST-LIO。

### 新地图从采集到导航的处理顺序

1. **独立建图会话**：先停止 `lite3_nav3d` 导航会话，确认无旧 `/fastlio2/lio_node`、`/localizer_node`、`/pgo/pgo_node`。然后启动 `ros2 launch pgo pgo_launch.py use_rviz:=false`（此 launch 自带 FAST-LIO 和 Lite3 静态 TF，不要另外再启动一次）。若要复盘 PGO，应同时记录点云、里程计、TF 和 PGO 日志；保存原始记录便于离线重建。
2. **导出到全新目录**：先建立唯一的新目录，例如 `mkdir -m 700 /root/nav/livox_ws/maps/map_new_20260922_run01`，再调用：

   ```bash
   source /root/nav/livox_ws/scripts/ros_env.bash
   ros2 service call /pgo/save_maps interface/srv/SaveMaps \
     "{file_path: '/root/nav/livox_ws/maps/map_new_20260922_run01', save_patches: true}"
   ```

   服务只接受**已存在目录**，会写入 `map.pcd`、`poses.txt` 和 `patches/`。同一目录再次 `save_patches: true` 会删除并重建旧 `patches/`，也会覆盖地图，因此每次导出必须使用新目录，不要指向 map918 或已有成果目录。检查返回 `success: true`、文件大小、PCD `POINTS` 数及关键帧数量，保留原始导出文件不覆盖。
3. **先验收 PGO 原图**：在 RViz 用俯视和侧视查看墙面重影、地面双层、悬空点、通道被高点堵塞；特别复查 map918 的 `(1.05,0.00)` 类似位置。检查闭环前后地图有没有跳变，墙角/门洞是否对齐。若原图已扭曲，后续裁剪、调平或放宽规划参数都不能替代重新检查 PGO/FAST-LIO；不要立即生成导航图。
4. **再制作导航候选图**：从原始 `map.pcd` 复制生成新文件；需要裁剪高度时可参考 `src/pcd_height/filter_pcd_height.py`，但该脚本用全局最低 5% 点估计地面，斜坡/多层地面可能误判，必须人工检查结果。只在测得整体倾斜时拟合地面并对**完整图和高度图施加同一个刚体旋转**；不要照搬 map918 的 `2.979°` 旋转或 `+0.432958 m` Z 平移。Nav3D 当前内置 PCD 读取器要求普通 `DATA binary`，不要把 `binary_compressed` 直接交给它。导航与 localizer 必须加载同一份最终 PCD。
5. **只读验收后再行走**：用 `NAV_MOTION_ENABLED=false` 启动导航，重新执行 `2D Pose Estimate`，检查 `valid=True`、`map→base_link` 时间前进、实时点云在至少两处特征处与地图重合；用 RViz 仅点目标检查全局路径首点、长度和可走通道。新地图的坐标原点可能变化，旧初始位姿、自动重定位锚点、多点坐标**全部作废**，必须重新采集。通过静态与短距离单点验证后，才开启 SCAN 和底盘桥做实机测试。

当前没有可靠证据支持一个可直接用于导航的“自动清理红格”阈值。优先完成新图质量验收，再决定是否给 Nav3D 增加基于机身高度和宽度的专用通行层，而不是修改 RViz 颜色冒充地图已修好。

### map922_02：生成导航候选 PCD（2026-09-22）

完整 PGO 原图为 `maps/map922_02/map.pcd`（163 关键帧、696438 点），原件不要修改。它的高度直方图在约 `z=-0.175 m` 有地面峰、约 `z=2.5 m` 有天花板峰。使用下面的专用脚本保守地去掉地面以下噪声及高天花板，同时**保留 0–2 m 内的障碍物**：

```bash
cd /root/nav/livox_ws
python3 scripts/prepare_pgo_map.py \
  maps/map922_02/map.pcd \
  maps/map922_02/map_nav_candidate.pcd \
  --below 0.15 --above 2.0
```

脚本只接受普通 `DATA binary` PCD，保留原始 XY/Z 坐标和 intensity；不会自动调平、平移、降采样，也**不会把红格当作空地清除**。输出另有同名 `.json` 报告，包含估计地面 Z、裁剪上下界和点数；文件已存在时拒绝覆盖。若自动地面高度明显不符合现场，可先人工测量，再使用 `--floor-z <米>` 重新生成到**另一个新文件名**，不可盲目修改阈值。该脚本尚不能处理多层地面或局部斜坡的逐格高度分类。

处理后先用 PCD viewer/RViz 对比原图和候选图，特别检查先前被误标红的实际可通行通道是否还有密集高点、真实障碍是否仍在，以及地面是否连续。通过视觉和规划检查后，才将候选图同时提供给 localizer 和 Nav3D；启动脚本示例：

```bash
NAV_MOTION_ENABLED=false ./scripts/start_nav3d_scan_tmux.sh \
  /root/nav/livox_ws/maps/map922_02/map_nav_candidate.pcd
```

这一步只读规划、不启动 SCAN/底盘。新地图坐标系不同于 map918：重新重定位并重新采集点位，不复用旧图锚点和多点坐标。

## map918 地图调平（2026-09-19）

已确认白色 `/map_cloud` 本身存在约 `2.979°` 的整体倾斜，而非单纯 RViz
观察视角问题。原始 PGO 使用完整 6DoF 位姿图，但没有重力方向、roll/pitch
或地面高度约束，因此会保留并可能累积姿态倾斜。

现已保留原图并生成：

- `maps/map918/map_height_leveled_rotation_only_nav.pcd`：普通 binary 格式，仅旋转调平并保留原始 Z 高度基准，导航与定位使用；
- `maps/map918/map_height_leveled_nav.pcd`：包含额外 Z 平移的早期实验版本，不再用于当前平面 ICP localizer；
- `maps/map918/map_height_leveled.pcd`：binary_compressed 调平母版，Nav3D 不支持直接读取；
- `maps/map918/map_leveled.pcd`：完整调平点云；
- `maps/map918/*.before_leveling_20260919.bak`：原始文件备份。

调平后的高度地图重新拟合得到法向量约
`(-0.000002, -0.000001, 1.000000)`，共 271254 点。详细变换和复核记录见
[maps/map918/LEVELING.md](maps/map918/LEVELING.md)。启动命令：

```bash
./scripts/start_nav3d_scan_tmux.sh \
  /root/nav/livox_ws/maps/map918/map_height_leveled_rotation_only_nav.pcd
```

调平改变了 `map` 坐标系，因此旧初始位姿和旧导航点全部作废；需在新地图
上重新重定位并重新记录多点坐标。

## 已确认的定位现象

- 2026-09-20 冷启动验证：Nav3D 不支持 PCL 的 `binary_compressed` PCD，当时试用普通 binary 格式的 `maps/map918/map_height_leveled_nav.pcd`，Nav3D、localizer 均成功加载。**后续已改用仅旋转、不平移 Z 的 `map_height_leveled_rotation_only_nav.pcd`；旧验证不代表现用地图已通过通行验收。**
- 2026-09-20 冷启动时，Livox 原始帧正常（每帧 20064 点），但 FAST-LIO 的独立 lidar callback group 未及时收到帧。激光订阅恢复默认 callback group，并匹配驱动使用 `Reliable + KeepLast(10)` 后，`/fastlio2/lio_odom` 实测稳定约 10 Hz，`/fastlio2/world_cloud` 持续更新。修改前备份为 `src/FASTLIO2_ROS2-main/fastlio2/src/lio_node.cpp.20260920_before_lidar_reliable.bak`。
- 2026-09-19 已修正 FAST-LIO 点面残差姿态雅可比中将雷达外参 `t_il` 误写为世界位置 `t_wi` 的错误。IMU 初始化窗口改为400点，执行器使用6线程。清理遗留诊断订阅器后，在完整导航栈下相隔30秒的两次静止读数，平面位移约1.8毫米，IMU缓存约9至26条，没有再次发散。运动状态仍需继续验收。
- 排查期间发现多个由 `timeout ros2 topic hz/bw` 遗留的订阅进程，其中 `/fastlio2/world_cloud` 带宽测试会迫使 LIO 持续转换和发送大点云，并显著放大回调延迟。测试结束后应 `Ctrl-C` 正常退出并用 `ps` 确认没有遗留；不要同时长期运行多个 `hz`、`bw` 或点云录制命令。
- 新会话未重定位时 `/relocalize_check` 为 `valid=False`、没有 `map → odom`，这是预期状态。FAST-LIO 点云与里程计在同一 5 秒窗口有 **50 对完全相同的时间戳**，`odom → base_link` 在静止时持续更新。
- 之前一个长时间运行的会话中，localizer 持续报 `No fresh synchronized LIO cloud/odometry` 并停止发布 `map → odom`，但 `/relocalize_check` 仍返回 `valid=True`。这是历史成功状态残留；那次同步中断的根因**尚未确定**，不能说已经修好。
- 手工在 RViz 点选的 `map → base_link` 为约 `(0.086, -0.009, -0.038 rad)`；据此算出的 `map → odom` 初值约 `(0.298, 0.008, -0.040 rad)`。重定位后的 `map → base_link` 却在约 `(-0.47, 0.00, 0.14 rad)`，与点选位置相差约半米、朝向约 0.18 rad。**这说明点选或配准结果至少有一处需要核实，不应直接保存为正确出发位姿。**
- `map918` 存在重复结构：自动点云匹配的两个不同三维候选分数约为 `0.122` 和 `0.125`。仅凭当前静止点云无法可靠区分，程序会拒绝自动执行；不能通过关闭歧义检查来“解决”。

## 现在先做什么

1. 让机器狗保持静止，不发送导航目标；保留遥控器或实体急停。
2. 在 RViz 设置 `Fixed Frame = map`，显示 `/nav3d/planning_occupied_markers`（地图）和 `/fastlio2/world_cloud`（实时点云）。确认墙角、门洞、柱子等至少两处特征重合。若明显错位，先不要学习出发位置，也不要测试多点导航。
3. 检查定位不仅“成功”，而且**持续有效**：

   ```bash
   source /root/nav/livox_ws/scripts/ros_env.bash
   ros2 service call /relocalize_check interface/srv/IsValid "{code: 0}"
   ros2 run tf2_ros tf2_echo map base_link
   ros2 topic hz /LIO/odom_vehicle
   ```

   验收：`valid=True`；`map → base_link` 的 `At time` 持续前进；静止位姿稳定；`/LIO/odom_vehicle` 持续有数据；实时点云与地图重合。若 localizer 报 `No fresh synchronized...` 或时间戳停止，取消测试，不要仅凭 `valid=True` 继续。
4. 若 RViz 中的地图不易辨认，先确定一个有明显墙角/门洞特征的固定出发点。当前地图的重复结构决定了完全未知起点的一次静止扫描可能无法唯一定位。

## 使用 RViz 设置初始位置

2026-09-19 起，localizer 已直接订阅 `/initialpose`。启动脚本会把本次 Nav3D 使用的同一 PCD 路径通过 `map_path` 参数传给 localizer。在 RViz 中将 `Fixed Frame` 设为 `map`，点击 `2D Pose Estimate`，在地图上按住并拖出机器狗真实朝向。localizer 会使用实时 `odom → base_link`，将 RViz 给出的 `map → base_link` 换算成 `map → odom` 初值，然后执行现有 ICP 重定位。

点击后在第 9 个 tmux 窗格应看到 `Accepted RViz initial pose`。随后仍须按上一节检查 `valid=True`、TF 时间持续更新和地图点云真实重合。若日志提示没有新鲜 LIO、TF 不存在、地图路径为空或配准失败，不要发送导航目标。

修改前备份为：

- `src/FASTLIO2_ROS2-main/localizer/src/localizer_node.cpp.20260919_before_initialpose.bak`
- `scripts/start_nav3d_scan_tmux.sh.20260919_before_initialpose.bak`

## 自动重定位程序

[scripts/auto_relocalize.py](scripts/auto_relocalize.py) 从 `/fastlio2/world_cloud` 采集实时点云，与 PCD 做多候选全局匹配和三维复核。它只估计平面 `map → odom` 的 `x/y/yaw`，与当前 localizer 的约束一致。匹配弱、位置含糊、传感器缺失、服务失败或没有新鲜 TF 时会报错退出。程序本身不发送运动指令。

先只读试算：

```bash
source /root/nav/livox_ws/scripts/ros_env.bash
python3 /root/nav/livox_ws/scripts/auto_relocalize.py \
  /root/nav/livox_ws/maps/map918/map_height_leveled_rotation_only_nav.pcd
```

**首次只在确认当前定位与地图真实重合后**，将狗停在以后固定使用的出发区，保存一次出发位姿：

```bash
python3 /root/nav/livox_ws/scripts/auto_relocalize.py \
  /root/nav/livox_ws/maps/map918/map_height_leveled_rotation_only_nav.pcd --learn-start
```

此命令会在所用 PCD 旁生成同名 `.start_pose.json`。它只是记录当前 `map → base_link`，**不会替你判断这个 TF 是否物理正确**。如果地图和实时点云尚未重合，不要执行 `--learn-start`；新地图不能沿用旧地图的锚点文件。

之后每次在同一出发区启动，可让程序重新计算并调用 `/relocalize`：

```bash
python3 /root/nav/livox_ws/scripts/auto_relocalize.py \
  /root/nav/livox_ws/maps/map918/map_height_leveled_rotation_only_nav.pcd --execute
```

`--execute` 会要求候选位于已学习出发区附近，并等待 localizer 返回成功和新鲜 `map → odom` TF。若狗被搬到其他位置，可能拒绝执行；这时需要重新确认真实位置，而不是扩大 `--anchor-radius` 强行通过。当前已做语法检查、已知位姿的离线点云回归测试和实机**只读试算**；尚未在真机上测试 `--execute` 服务调用。

## 启动与 tmux

默认 tmux socket 曾报 `server exited unexpectedly`，独立 socket 可正常使用。在一个终端中：

```bash
export TMUX_TMPDIR=/tmp/lite3_tmux_nav
mkdir -p "$TMUX_TMPDIR"
chmod 700 "$TMUX_TMPDIR"
./scripts/start_nav3d_scan_tmux.sh \
  /root/nav/livox_ws/maps/map918/map_height_leveled_rotation_only_nav.pcd
```

另一个终端连接时同样先设置 `TMUX_TMPDIR=/tmp/lite3_tmux_nav`，再运行 `tmux attach -t lite3_nav3d`。若会话已存在，脚本会连接旧会话，**不会按新地图重新启动**。不要直接删除仍有进程监听的默认 tmux socket。

## 多点导航的下一步

1. **先过定位验收**：墙体重合、TF 持续更新，固定出发区自动重定位能重复成功。
2. **再过单点验收**：近距离空旷目标，检查 Nav3D 路径高度、SCAN 跟踪、底盘方向和真实到点误差。
3. **最后做多点任务层**：按序发送 A→B→C。当前 Nav3D 关闭自身控制器，因此其 `NavigateToPose` action 在**规划并发布路径后**即可返回成功，不能用这个结果判断狗已到点；应以实时 `map → base_link` 到点距离、停稳状态及超时判定后再发下一点。

现有详细启动说明见 [docs/NAV3D_SCAN_STARTUP_GUIDE.md](docs/NAV3D_SCAN_STARTUP_GUIDE.md)。该旧说明以 `map9802` 为示例，实际使用 `map918` 时不要照抄其中的地图路径或 `0,0,0` 初值。

历史参考：[旧版 Nav2 操作手册](docs/legacy_nav2_guide.md)（旧链路记录，当前操作以本页为准）。
