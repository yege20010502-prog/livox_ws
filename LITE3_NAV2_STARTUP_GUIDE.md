# Lite3 + Livox + FAST-LIO2 + Nav2 启动手册

工作空间：/root/nav/livox_ws  
当前阶段：定位、规划、动态避障联调。真机安全门控未完成。

## 0. 安全要求

- 当前不要启动 dog_control_bridge。
- 发目标前运行 ros2 topic info /cmd_vel -v，确认无Lite3控制桥订阅。
- 出现TF跳变、时间外推、Robot is out of bounds或局部地图异常时，立即取消目标并停止Nav2。
- 真机接入前必须增加人工使能、定位/传感器超时急停、限速和独立急停。

## 1. 修改源码后编译

~~~bash
cd ~/nav/livox_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select fastlio2 localizer lite3_nav2_bringup --symlink-install
source /opt/ros/humble/setup.bash
source ~/nav/livox_ws/install/setup.bash
~~~

每个新终端先执行：

~~~bash
source /opt/ros/humble/setup.bash
export AMENT_PREFIX_PATH="/root/nav/livox_ws/install/lite3_nav2_bringup:${AMENT_PREFIX_PATH}"
source ~/nav/livox_ws/install/setup.bash
~~~

下面每项使用独立终端，严格按顺序启动。

## 2. 终端1：Livox驱动

~~~bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
~~~

## 3. 终端2：Lite3静态TF

~~~bash
ros2 launch fastlio2 lite3_tf.launch.py
~~~

当前外参：

~~~text
imu_link -> base_link: xyz[-0.1910,-0.02329,-0.0947], pitch=-0.4037rad
base_link -> livox_frame: xyz[0.220,0,0.057], pitch=+0.4037rad
~~~

检查：

~~~bash
ros2 node list | grep -E 'imu_to_base_tf|base_to_livox_tf'
ros2 run tf2_ros tf2_echo imu_link base_link
ros2 run tf2_ros tf2_echo base_link livox_frame
~~~

不要同时启动其他同名TF。

## 4. 终端3：FAST-LIO2

启动时机器狗保持静止：

~~~bash
ros2 run fastlio2 lio_node --ros-args -p config_path:=/root/nav/livox_ws/install/fastlio2/share/fastlio2/config/lio.yaml
~~~

检查：

~~~bash
ros2 topic hz /fastlio2/lio_odom
ros2 topic hz /fastlio2/body_cloud
ros2 run tf2_ros tf2_echo odom base_link
~~~

odom到imu_link约23度是安装角；水平地面应检查odom到base_link的roll/pitch接近0度。出现NO Effective Points时不要导航。

## 5. 终端4：localizer

~~~bash
ros2 run localizer localizer_node --ros-args -p config_path:=/root/nav/livox_ws/install/localizer/share/localizer/config/localizer.yaml
ros2 service list | grep relocalize
~~~

应有/relocalize和/relocalize_check。新版localizer在PCD加载前不会执行ICP或发布无效map到odom。

## 6. 终端5：重定位

~~~bash
ls -lh /root/nav/livox_ws/maps/map831/map_indoor_clean.pcd
~~~

FAST-LIO启动位置和方向与建图原点一致时：

~~~bash
ros2 service call /relocalize interface/srv/Relocalize "{pcd_path: '/root/nav/livox_ws/maps/map831/map_indoor_clean.pcd',
x: 0.0, y: 0.0, z: 0.0, yaw: 0.0, pitch: 0.0, roll: 0.0}"
ros2 service call /relocalize_check interface/srv/IsValid "{code: 0}"
~~~

必须返回valid:true。x/y/yaw是map到odom初始猜测，不是map到base_link坐标。

~~~bash
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_tools view_frames
~~~

TF应为map->odom->imu_link->base_link->livox_frame。地图范围x=-2.8~5.9、y=-4.1~1.7，机器人必须在范围内且静止时稳定。

## 7. 终端6：Nav2

不要额外手动启动map_server。

~~~bash
ros2 launch lite3_nav2_bringup lite3_nav2.launch.py
ros2 lifecycle get /map_server
ros2 lifecycle get /planner_server
ros2 lifecycle get /controller_server
ros2 lifecycle get /bt_navigator
~~~

四项必须全部active[3]。二维地图为：

~~~text
/root/nav/livox_ws/maps/map831/maps/map_obstacles/map.yaml
~~~

排除重复：

~~~bash
ros2 node list | sort | uniq -d
ros2 topic info /map -v
~~~

## 8. 当前代价地图配置

~~~text
footprint约0.60x0.36m
inflation_radius=0.30
cost_scaling_factor=6.0
mark_threshold=2
min_obstacle_height=0.15
max_obstacle_height=1.20
obstacle_max_range=3.0
raytrace_max_range=4.0
~~~

## 9. 终端7：RViz2

~~~bash
rviz2
~~~

Fixed Frame设为map，添加：

~~~text
Static Map: /map（配色map）
Global Costmap: /global_costmap/costmap（配色costmap）
Local Costmap: /local_costmap/costmap（配色costmap）
Path: /plan
PointCloud2: /fastlio2/body_cloud
TF、RobotModel
~~~

诊断时一次只显示一张Map，避免颜色叠加。

## 10. 安全空载规划测试

~~~bash
ros2 node list | grep dog_control_bridge
ros2 topic info /cmd_vel -v
~~~

确认无控制桥。在RViz用2D Goal Pose选择0.5~0.8米外空旷位置，不使用2D Pose Estimate。

~~~bash
ros2 topic echo /plan
ros2 topic echo /nav2_cmd_vel
ros2 topic echo /cmd_vel
~~~

机器人未实际移动时，Nav2可能输出后退或旋转恢复，这是空载测试的正常结果，也是当前不能接控制桥的原因。

## 11. 动态避障测试

~~~bash
ros2 topic hz /fastlio2/body_cloud
ros2 topic hz /local_costmap/costmap
~~~

RViz只显示Local Costmap、PointCloud2和TF。前方放纸箱，1~2秒内应出现障碍与膨胀圈；移走后数秒内应清除。

## 12. 常见故障

- TF extrapolation：检查map->odom和odom->base_link；通常需重启localizer并重新定位。
- Robot is out of bounds：检查map->base_link是否处于地图范围；FAST-LIO重启后必须重新定位。
- Invalid or empty point cloud：重新编译已修复的localizer，再调用/relocalize。
- 局部地图全障碍：确认使用最新过滤参数，检查/body_cloud的frame_id及odom->imu_link。

## 13. 停止顺序

1. RViz取消目标。
2. 停止Nav2。
3. 停止localizer。
4. 停止FAST-LIO2。
5. 停止静态TF。
6. 停止Livox驱动。

## 14. 真机接入（当前禁止）

不要执行 ros2 run dog_control_bridge dog_control_bridge。接入前必须完成独立速度安全门控、人工使能、TF/点云超时急停、速度限制、取消目标后连续零速度以及遥控器或硬件急停。