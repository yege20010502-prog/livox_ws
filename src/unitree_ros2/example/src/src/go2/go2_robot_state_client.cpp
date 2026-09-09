#include <iostream>
#include <string>
#include <chrono>
#include <thread>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "unitree_api/msg/request.hpp"
#include "common/ros2_sport_client.h"
#include "unitree_go/msg/sport_mode_state.hpp"

using namespace std::chrono_literals;

#define TOPIC_HIGHSTATE "/sportmodestate"

class Go2SportClientNode : public rclcpp::Node
{
public:
    Go2SportClientNode() : Node("go2_sport_bridge_node"), sport_client_(this)
    {
        // 1. 订阅 Go2 状态话题
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

        RCLCPP_INFO(this->get_logger(), "宇树 Go2 状态机节点已部署");
        RCLCPP_INFO(this->get_logger(), "监听话题: /cmd_vel (速度控制)");
        RCLCPP_INFO(this->get_logger(), "目标模式: 100-灵动模式 (基础运动)");
        RCLCPP_INFO(this->get_logger(), "状态码: 1001-趴下 | 1002-站立锁定 | 100-灵动(可运动)");
        RCLCPP_INFO(this->get_logger(), "等待 /cmd_vel 指令自动唤醒...");
    }

private:
    void CmdVelHandler(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // 检查速度指令是否为零
        float vx = msg->linear.x;
        float vy = msg->linear.y;
        float yaw = msg->angular.z;
        bool is_zero_cmd = (std::abs(vx) < 0.001 && std::abs(vy) < 0.001 && std::abs(yaw) < 0.001);

        // ===== 拦截 A：阻尼/趴下状态 (error_code == 1001) =====
        if (current_robot_error_code_ == 1001) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "【安全拦截】检测到 Go2 处于阻尼/趴下状态(error_code:1001)。正在发送 StandUp() 起立...");
            sport_client_.StandUp(req_);
            last_mode_switch_time_ = this->now();
            return;
        }

        // ===== 拦截 B：站立锁定状态 (error_code == 1002) =====
        if (current_robot_error_code_ == 1002) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "【模式锁定中】检测到 Go2 处于站立锁定状态(error_code:1002)。正在切换到灵动模式(100)...");
            // 切换到灵动模式: 使用 FreeWalk (如果支持) 或 BalanceStand + Move
            // 根据 API，使用 StandUp 或 BalanceStand 进入可运动状态
            sport_client_.BalanceStand(req_);
            last_mode_switch_time_ = this->now();
            return;
        }

        // ===== 拦截 C：非运动状态，切换到灵动模式(100) =====
        // 灵动模式是 100，只有 100 才允许速度指令
        if (current_robot_error_code_ != 100) {
            auto now = this->now();
            double elapsed = (now - last_mode_switch_time_).seconds();
            
            if (elapsed > 2.0) {
                // 如果在 2010 经典模式，先切换到平衡站立再进入灵动
                if (current_robot_error_code_ == 2010) {
                    RCLCPP_INFO(this->get_logger(), 
                        "【模式切换】当前状态: 2010(经典模式)，切换到灵动模式(100)...");
                    // 先停止当前运动，再切换到灵动模式
                    sport_client_.StopMove(req_);
                    // 切换到灵动模式：通过 BalanceStand 进入可运动状态
                    sport_client_.BalanceStand(req_);
                } else {
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                        "【模式切换】当前状态: %d，切换到灵动模式(100)...", current_robot_error_code_);
                    sport_client_.BalanceStand(req_);
                }
                last_mode_switch_time_ = now;
            }
            return;
        }

        // ===== 通道放行：灵动模式 (error_code == 100) =====
        if (current_robot_error_code_ == 100) {
            // 如果速度指令全为零，发送 StopMove
            if (is_zero_cmd) {
                sport_client_.StopMove(req_);
                RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "【停止】收到零速度指令，停止运动");
            } else {
                // ★★★ 灵动模式速度限制 ★★★
                // 线速度: 0~0.8 m/s (灵动模式支持更快的速度)
                float safe_vx = std::clamp(vx, -0.8f, 0.8f);
                float safe_vy = std::clamp(vy, -0.5f, 0.5f);
                float safe_yaw = std::clamp(yaw, -0.8f, 0.8f);
                
                // 只允许前进和左转 (根据运控限制)
                safe_vx = std::max(safe_vx, 0.0f);   // 只能前进
                // safe_yaw = std::max(safe_yaw, 0.0f); // 只能左转
                
                sport_client_.Move(req_, safe_vx, safe_vy, safe_yaw);
            }
        }
    }

    void HighStateHandler(const unitree_go::msg::SportModeState::SharedPtr msg)
    {
        current_robot_error_code_ = msg->error_code;
        current_position_z_ = msg->position[2];

        // 映射状态机名称
        std::string mode_name = "未知状态";
        if (current_robot_error_code_ == 100)    mode_name = "灵动模式 ⭐ 可运动";
        if (current_robot_error_code_ == 1001)   mode_name = "阻尼/趴下";
        if (current_robot_error_code_ == 1002)   mode_name = "站立锁定";
        if (current_robot_error_code_ == 1004 || current_robot_error_code_ == 2006) mode_name = "蹲下";
        if (current_robot_error_code_ == 1006)   mode_name = "打招呼/伸懒腰/舞蹈/拜年/比心/开心";
        if (current_robot_error_code_ == 1007)   mode_name = "坐下";
        if (current_robot_error_code_ == 1008)   mode_name = "前跳";
        if (current_robot_error_code_ == 1009)   mode_name = "扑人";
        if (current_robot_error_code_ == 1013)   mode_name = "平衡站立";
        if (current_robot_error_code_ == 1015)   mode_name = "常规行走";
        if (current_robot_error_code_ == 1016)   mode_name = "常规跑步";
        if (current_robot_error_code_ == 1017)   mode_name = "常规续航";
        if (current_robot_error_code_ == 1091)   mode_name = "摆姿势";
        if (current_robot_error_code_ == 2007)   mode_name = "闪避";
        if (current_robot_error_code_ == 2008)   mode_name = "并腿跑";
        if (current_robot_error_code_ == 2009)   mode_name = "跳跃跑";
        if (current_robot_error_code_ == 2010)   mode_name = "经典模式";

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "【Go2 状态】error_code: %d | %s | 高度: %.3fm",
            current_robot_error_code_, mode_name.c_str(), current_position_z_);
    }

    SportClient sport_client_;
    rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr suber_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    unitree_api::msg::Request req_;

    int current_robot_error_code_ = 1001;
    double current_position_z_ = 0.0;
    
    // 模式切换防震荡
    rclcpp::Time last_mode_switch_time_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Go2SportClientNode>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
