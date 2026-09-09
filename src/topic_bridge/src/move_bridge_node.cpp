#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

class MoveBridgeNode : public rclcpp::Node {
public:
    MoveBridgeNode() : Node("unitree_move_bridge_node") {
        RCLCPP_INFO(this->get_logger(), "宇树狗 /cmd_vel 运控桥接节点已启动，正在监听控制命令...");

        // 1. 订阅标准导航框架发出的 /cmd_vel 话题
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&MoveBridgeNode::cmdVelCallback, this, std::placeholders::_1));

        // 2. 把包装好的指令高频送给宇树官方的 SDK 控制节点
        unitree_cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/api/sport/cmd_vel", 10);

        // 3. 定时器：保持 20Hz 左右的高频心跳发送
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50), std::bind(&MoveBridgeNode::timerCallback, this));
    }

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        latest_twist_ = *msg;
        RCLCPP_DEBUG(this->get_logger(), "收到速度输入 -> 线速度 X: %.2f, Y: %.2f, 角速度 Z: %.2f",
                     msg->linear.x, msg->linear.y, msg->angular.z);
    }

    void timerCallback() {
        // 保持高频向狗子发送最新的速度，防止底层看门狗超时摔倒
        geometry_msgs::msg::Twist output_msg;
        output_msg.linear.x = latest_twist_.linear.x;
        output_msg.linear.y = latest_twist_.linear.y;
        output_msg.angular.z = latest_twist_.angular.z;
        unitree_cmd_pub_->publish(output_msg);
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr unitree_cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    geometry_msgs::msg::Twist latest_twist_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveBridgeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
