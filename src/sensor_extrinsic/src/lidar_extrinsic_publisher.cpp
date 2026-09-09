// lidar_extrinsic_publisher.cpp
// 发布 MID360 激光雷达相对于机器人的静态外参（安装位置）
// 发布到 /LIO/odom_imu 话题，解决 [GridMap] no sensor_pose received 警告
//
// 编译：放在 SCAN-Planner 的某个包中，或单独建一个包
// 用法：在启动 SCAN-Planner 之前运行此节点

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "geometry_msgs/msg/twist_with_covariance.hpp"

class LidarExtrinsicPublisher : public rclcpp::Node
{
public:
    LidarExtrinsicPublisher() : Node("lidar_extrinsic_publisher")
    {
        // 发布到 /LIO/odom_imu，因为 run.launch.py 中 sensor_pose 映射到了这里
        publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/LIO/odom_imu", 10);
        
        // 10Hz 定时发布，与 sensor 数据频率匹配
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&LidarExtrinsicPublisher::publishExtrinsic, this));
        
        RCLCPP_INFO(this->get_logger(), "激光雷达外参发布节点已启动");
        RCLCPP_INFO(this->get_logger(), "发布话题: /LIO/odom_imu");
        RCLCPP_INFO(this->get_logger(), "请根据实际安装位置调整 x/y/z/rpy 参数");
    }

private:
    void publishExtrinsic()
    {
        auto msg = nav_msgs::msg::Odometry();
        
        // 时间戳
        msg.header.stamp = this->now();
        msg.header.frame_id = "body";      // 机器人本体坐标系
        msg.child_frame_id = "lidar";       // 激光雷达坐标系
        
        // ★★★ 请根据实际安装位置修改以下参数 ★★★
        // MID360 在机器狗背部的安装位置（示例值，单位：米）
        msg.pose.pose.position.x = 0.0;
        msg.pose.pose.position.y = 0.0;
        msg.pose.pose.position.z = 0.25;   // 假设装在背部，高度约 25cm
        
        // 姿态四元数（这里假设激光雷达朝前安装，无旋转）
        // 如果激光雷达有安装角度，请修改此处
        msg.pose.pose.orientation.x = 0.0;
        msg.pose.pose.orientation.y = 0.0;
        msg.pose.pose.orientation.z = 0.0;
        msg.pose.pose.orientation.w = 1.0;
        
        // 协方差（全零，静态外参无不确定性）
        for (int i = 0; i < 36; i++) {
            msg.pose.covariance[i] = 0.0;
        }
        
        // 速度为零（这是静态外参）
        msg.twist.twist.linear.x = 0.0;
        msg.twist.twist.linear.y = 0.0;
        msg.twist.twist.linear.z = 0.0;
        msg.twist.twist.angular.x = 0.0;
        msg.twist.twist.angular.y = 0.0;
        msg.twist.twist.angular.z = 0.0;
        
        publisher_->publish(msg);
    }
    
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarExtrinsicPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}