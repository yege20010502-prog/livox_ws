/**********************************************************************
 Copyright (c) 2020-2026, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

#include <chrono>
#include <cmath>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include "common/ros2_sport_client.h"
#include "unitree_go/msg/sport_mode_state.hpp"

#define TOPIC_HIGHSTATE "/sportmodestate"

class Go2WTrueStateClosedLoopNode : public rclcpp::Node {
 public:
  explicit Go2WTrueStateClosedLoopNode() : Node("go2w_sport_bridge_node"), sport_client_(this) {
    
    // 1. 订阅 Go2W 状态话题
    suber_ = this->create_subscription<unitree_go::msg::SportModeState>(
        TOPIC_HIGHSTATE, 1,
        [this](const unitree_go::msg::SportModeState::SharedPtr data) {
          HighStateHandler(data);
        });

    // 2. 订阅控制/导航速度话题
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10,
        [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
          CmdVelHandler(msg);
        });

    RCLCPP_INFO(this->get_logger(), "宇树 Go2W [5-趴下 | 6-锁定 | 1-平衡] 状态真值闭环节点已部署。");
  }

 private:
  void CmdVelHandler(const geometry_msgs::msg::Twist::SharedPtr msg) {
    
    // 拦截 A：如果底盘反馈当前是趴下的 (mode == 5)
    if (current_robot_mode_ == 5) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
        "【安全拦截】检测到 Go2W 处于趴下状态(mode:5)。正在发送 StandUp()...");
      sport_client_.StandUp(req_);
      return;
    }

    // 拦截 B：【核心突破】如果底盘反馈当前是正常站立锁定 (mode == 6)
    if (current_robot_mode_ == 6) {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
        "【模式锁定中】检测到 Go2W 处于关节锁定状态(mode:6)。正在高频发送 BalanceStand() 强制解锁...");
      
      // 只要底层反馈不是 mode==1，这里就会一直发 BalanceStand，直到迫使其强行跳进 1 为止
      sport_client_.BalanceStand(req_); 
      return; 
    }

    // 通道放行：只有当底层状态 100% 反馈 current_robot_mode_ == 1 (balanceStand) 时，速度指令才被允许输入
    if (current_robot_mode_ == 1) {
      float vx = msg->linear.x;
      float vy = msg->linear.y;
      float yaw = msg->angular.z;

      sport_client_.Move(req_, vx, vy, yaw);
    }
  }

  void HighStateHandler(const unitree_go::msg::SportModeState::SharedPtr msg) {
    current_robot_mode_ = msg->mode;      // 实时同步底层真值模式
    double current_z    = msg->position[2]; // 使用正确的索引 [2] 获取高度

    // 映射模式名称，方便调试看清
    std::string mode_name = "未知模式";
    if (current_robot_mode_ == 0)  mode_name = "默认站立/空闲";
    if (current_robot_mode_ == 1)  mode_name = "balanceStand (平衡就绪，可动)";
    if (current_robot_mode_ == 5)  mode_name = "lieDown (趴下)";
    if (current_robot_mode_ == 6)  mode_name = "jointLock (站立死锁)";

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
      "【Go2W 状态】真实Mode代号: %d | 状态名称: %s | 真实高度 Z: %.3fm", 
      current_robot_mode_, mode_name.c_str(), current_z);
  }

  SportClient sport_client_;
  rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr suber_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  unitree_api::msg::Request req_;  

  int current_robot_mode_ = 5; // 默认初始化为趴下状态码 5
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Go2WTrueStateClosedLoopNode>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}

