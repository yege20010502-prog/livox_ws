#include <iostream>
#include <string>
#include <chrono>
#include <thread>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "unitree_api/msg/request.hpp"
#include "common/ros2_b2_sport_client.h"
#include "unitree_go/msg/sport_mode_state.hpp"

using namespace std::chrono_literals;

#define TOPIC_HIGHSTATE "/lf/sportmodestate"

class B2wSportClientNode : public rclcpp::Node
{
public:
    B2wSportClientNode() : Node("b2w_sport_bridge_node"), sport_client_(this)
    {
        // 1. 订阅 B2 状态话题
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

        RCLCPP_INFO(this->get_logger(), "宇树 B2 [5-趴下 | 6-锁定 | 3-起立88� | 18-经典行走] 状态真值闭环节点已部署。");
        RCLCPP_INFO(this->get_logger(), "监听话题: /cmd_vel (速度控制)");
        RCLCPP_INFO(this->get_logger(), "经典行走模式: 可接收速度指令");
    }

private:
    void CmdVelHandler(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // ===== 拦截 A：趴下状态 (mode == 5) =====
        if ( current_robot_mode_ == 7 || current_robot_mode_ == 5) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "【安全拦截】检测到 B2 处于趴下状态(mode:5)。正在发送 RecoveryStand() 起立...");
            sport_client_.RecoveryStand(req_);
            return;
        }

        // ===== 拦截 B：关节锁定状态 (mode == 6) =====
        if (current_robot_mode_ == 6) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "【模式锁定中】检测到 B2 处于关节锁定状态(mode:6)。正在发送 ClassicWalk() 切换到行走模式...");
            sport_client_.ClassicWalk(req_, true);
            return;
        }

        // ===== 拦截 C：起立过程中 (mode == 3) =====
        if (current_robot_mode_ == 3) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                "【起立中】检测到 B2 正在起立(mode:3)。等待完成...");
            return;
        }

        // ===== 通道放行：经典行走模式 (mode == 11) =====
        if (current_robot_mode_ == 18) {
            float vx = msg->linear.x;
            float vy = msg->linear.y;
            float yaw = msg->angular.z;

            // 检查是否有移动指令
            if (std::abs(vx) > 0.01 || std::abs(vy) > 0.01 || std::abs(yaw) > 0.01) {
                sport_client_.Move(req_, vx, vy, yaw);
            } else {
                // 停止移动
                sport_client_.StopMove(req_);
            }
        } else {
            // 如果不在行走模式，自动切换到经典行走
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "【模式切换】当前模式: %d，自动切换到经典行走模式(11)...", current_robot_mode_);
            sport_client_.ClassicWalk(req_, true);
        }
    }

    void HighStateHandler(const unitree_go::msg::SportModeState::SharedPtr msg)
    {
        current_robot_mode_ = msg->mode;
        current_position_z_ = msg->position[2];

        // 映射模式名称
        std::string mode_name = "未知模式";
        if (current_robot_mode_ == 0)  mode_name = "空闲/默认站立";
        if (current_robot_mode_ == 1)  mode_name = "balanceStand (平衡站立)";
        if (current_robot_mode_ == 2)  mode_name = "pose (姿态控制)";
        if (current_robot_mode_ == 3)  mode_name = "locomotion (起立/运动过渡中)";
        if (current_robot_mode_ == 5)  mode_name = "lieDown (趴下)";
        if (current_robot_mode_ == 6)  mode_name = "jointLock (关节锁定)";
        if (current_robot_mode_ == 7)  mode_name = "damping (阻尼)";
        if (current_robot_mode_ == 9)  mode_name = "FreeWalk (自由行走)";
        if (current_robot_mode_ == 18) mode_name = "ClassicWalk (经典行走) ⭐ 可运动";
        if (current_robot_mode_ == 19) mode_name = "FastWalk (快速行走)";
        if (current_robot_mode_ == 20) mode_name = "Euler (欧拉模式)";

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "【B2 状态】Mode: %d | %s | 高度: %.3fm",
            current_robot_mode_, mode_name.c_str(), current_position_z_);
    }

    SportClient sport_client_;
    rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr suber_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    unitree_api::msg::Request req_;

    int current_robot_mode_ = 7;  // 默认初始化为阻尼状态
    double current_position_z_ = 0.0;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<B2wSportClientNode>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
