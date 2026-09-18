#include <memory>
#include <string>

#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

// Data-only adapter for SCAN. It never publishes a velocity command.
class NavDataBridge : public rclcpp::Node
{
public:
  NavDataBridge()
  : Node("nav_data_bridge"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    global_frame_ = declare_parameter<std::string>("global_frame", "map");
    body_frame_ = declare_parameter<std::string>("body_frame", "base_link");
    const auto odom_topic = declare_parameter<std::string>(
      "input_odom_topic", "/fastlio2/lio_odom");
    const auto cloud_topic = declare_parameter<std::string>(
      "input_cloud_topic", "/fastlio2/body_cloud");

    path_pub_ = create_publisher<nav_msgs::msg::Path>("/initial_path", 10);
    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/LIO/clouds_lidar", rclcpp::SensorDataQoS());
    body_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      "/LIO/odom_vehicle", 10);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, 10,
      [this](nav_msgs::msg::Odometry::SharedPtr msg) {
        try {
          const auto tf = tf_buffer_.lookupTransform(
            global_frame_, body_frame_, tf2::TimePointZero);
          nav_msgs::msg::Odometry output;
          output.header = tf.header;
          output.header.frame_id = global_frame_;
          output.child_frame_id = body_frame_;
          output.pose.pose.position.x = tf.transform.translation.x;
          output.pose.pose.position.y = tf.transform.translation.y;
          output.pose.pose.position.z = tf.transform.translation.z;
          output.pose.pose.orientation = tf.transform.rotation;
          output.pose.covariance = msg->pose.covariance;
          output.twist = msg->twist;
          body_odom_pub_->publish(output);
        } catch (const tf2::TransformException & ex) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Waiting for %s -> %s: %s",
            global_frame_.c_str(), body_frame_.c_str(), ex.what());
        }
      });

    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/nav3d/trajectory", 10,
      [this](nav_msgs::msg::Path::SharedPtr msg) {
        if (msg->header.frame_id != global_frame_) {
          RCLCPP_ERROR(
            get_logger(), "Rejecting trajectory frame '%s'; expected '%s'",
            msg->header.frame_id.c_str(), global_frame_.c_str());
          return;
        }
        auto output = *msg;
        for (auto & pose : output.poses) {
          if (pose.header.frame_id.empty()) {
            pose.header.frame_id = global_frame_;
          } else if (pose.header.frame_id != global_frame_) {
            RCLCPP_ERROR(
              get_logger(), "Rejecting pose frame '%s' in trajectory",
              pose.header.frame_id.c_str());
            return;
          }
        }
        path_pub_->publish(output);
      });

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        cloud_pub_->publish(*msg);
      });

    RCLCPP_INFO(
      get_logger(), "Data bridge ready: odom, cloud and path only; no /cmd_vel publisher");
  }

private:
  std::string global_frame_;
  std::string body_frame_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr body_odom_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavDataBridge>());
  rclcpp::shutdown();
  return 0;
}
