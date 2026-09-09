#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class AllBridges : public rclcpp::Node
{
public:
    AllBridges() : Node("all_bridges")
    {
        // [1] /fastlio2/lio_odom → /nav3d/current_pose (Odometry → PoseStamped)
        pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/fastlio2/lio_odom", 10,
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                auto pose = geometry_msgs::msg::PoseStamped();
                pose.header = msg->header;
                pose.pose = msg->pose.pose;
                pose_pub_->publish(pose);
            });
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/nav3d/current_pose", 10);
        RCLCPP_INFO(this->get_logger(), "[1/6] /fastlio2/lio_odom → /nav3d/current_pose");

        // [2] /nav3d/trajectory → /initial_path (Path → Path)
        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/nav3d/trajectory", 10,
            [this](const nav_msgs::msg::Path::SharedPtr msg) {
                path_pub_->publish(*msg);
            });
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
            "/initial_path", 10);
        RCLCPP_INFO(this->get_logger(), "[2/6] /nav3d/trajectory → /initial_path");

        // [3] /quad_0/cmd_vel → /cmd_vel (Twist → Twist)
        cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/quad_0/cmd_vel", 10,
            [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
                cmd_pub_->publish(*msg);
            });
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10);
        RCLCPP_INFO(this->get_logger(), "[3/6] /quad_0/cmd_vel → /cmd_vel");

        // [4] /fastlio2/body_cloud → /LIO/clouds_lidar (PointCloud2 → PointCloud2)
        cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/fastlio2/body_cloud", 10,
            [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                cloud_pub_->publish(*msg);
            });
        cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/LIO/clouds_lidar", 10);
        RCLCPP_INFO(this->get_logger(), "[4/6] /fastlio2/body_cloud → /LIO/clouds_lidar");

        // [5] /fastlio2/lio_odom → /LIO/odom_vehicle (Odometry → Odometry)
        odom_vehicle_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/fastlio2/lio_odom", 10,
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                odom_vehicle_pub_->publish(*msg);
            });
        odom_vehicle_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
            "/LIO/odom_vehicle", 10);
        RCLCPP_INFO(this->get_logger(), "[5/6] /fastlio2/lio_odom → /LIO/odom_vehicle");

        // [6] /livox/imu → /LIO/odom_imu (Imu → Imu)
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/livox/imu", 10,
            [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
                imu_pub_->publish(*msg);
            });
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
            "/LIO/odom_imu", 10);
        RCLCPP_INFO(this->get_logger(), "[6/6] /livox/imu → /LIO/odom_imu");
    }

private:
    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_vehicle_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

    // Subscribers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_vehicle_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AllBridges>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
