#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class AllBridges : public rclcpp::Node
{
public:
  AllBridges()
  : Node("all_bridges"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    global_frame_ = declare_parameter<std::string>("global_frame", "map");
    body_frame_ = declare_parameter<std::string>("body_frame", "base_link");
    input_odom_topic_ =
      declare_parameter<std::string>("input_odom_topic", "/fastlio2/lio_odom");
    input_cloud_topic_ =
      declare_parameter<std::string>("input_cloud_topic", "/fastlio2/body_cloud");
    input_cmd_topic_ =
      declare_parameter<std::string>("input_cmd_topic", "/scan_cmd_vel");
    output_cmd_topic_ = declare_parameter<std::string>("output_cmd_topic", "/cmd_vel");
    command_timeout_ = declare_parameter<double>("command_timeout", 0.30);
    sensor_timeout_ = declare_parameter<double>("sensor_timeout", 0.50);
    max_vx_ = declare_parameter<double>("max_vx", 0.15);
    max_vy_ = declare_parameter<double>("max_vy", 0.10);
    max_wz_ = declare_parameter<double>("max_wz", 0.25);
    navigation_enabled_ = declare_parameter<bool>("navigation_enabled", false);

    path_pub_ = create_publisher<nav_msgs::msg::Path>("/initial_path", 10);
    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/LIO/clouds_lidar", rclcpp::SensorDataQoS());
    body_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      "/LIO/odom_vehicle", 10);
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(output_cmd_topic_, 10);
    ready_pub_ = create_publisher<std_msgs::msg::Bool>("/scan_navigation_ready", 10);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      input_odom_topic_, 10,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) { onOdom(msg); });
    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/nav3d/trajectory", 10,
      [this](const nav_msgs::msg::Path::SharedPtr msg) { onPath(msg); });
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_cloud_topic_, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        cloud_pub_->publish(*msg);
        last_cloud_time_ = now();
        have_cloud_ = true;
      });
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      input_cmd_topic_, 10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        last_cmd_ = *msg;
        last_cmd_time_ = now();
        have_cmd_ = true;
      });
    enable_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/scan_navigation_enable", 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        navigation_enabled_ = msg->data;
        RCLCPP_WARN(
          get_logger(), "Navigation command gate %s",
          navigation_enabled_ ? "ENABLED" : "disabled; commanding zero");
      });

    const auto now_time = now();
    last_cmd_time_ = now_time;
    last_odom_time_ = now_time;
    last_cloud_time_ = now_time;
    gate_timer_ = create_wall_timer(
      std::chrono::milliseconds(50), [this]() { commandGateTick(); });

    RCLCPP_INFO(
      get_logger(),
      "Bridge ready: map-frame pose, path/cloud bridge, %s -> %s safety gate",
      input_cmd_topic_.c_str(), output_cmd_topic_.c_str());
  }

private:
  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    try {
      const auto transform = tf_buffer_.lookupTransform(
        global_frame_, body_frame_, tf2::TimePointZero);

      nav_msgs::msg::Odometry body_odom;
      body_odom.header = transform.header;
      body_odom.header.frame_id = global_frame_;
      body_odom.child_frame_id = body_frame_;
      body_odom.pose.pose.position.x = transform.transform.translation.x;
      body_odom.pose.pose.position.y = transform.transform.translation.y;
      body_odom.pose.pose.position.z = transform.transform.translation.z;
      body_odom.pose.pose.orientation = transform.transform.rotation;
      body_odom.pose.covariance = msg->pose.covariance;
      body_odom.twist = msg->twist;
      body_odom_pub_->publish(body_odom);

      last_odom_time_ = now();
      have_body_pose_ = true;
    } catch (const tf2::TransformException & ex) {
      have_body_pose_ = false;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Cannot publish map-frame body pose (%s -> %s): %s",
        global_frame_.c_str(), body_frame_.c_str(), ex.what());
    }
  }

  void onPath(const nav_msgs::msg::Path::SharedPtr msg)
  {
    if (msg->header.frame_id != global_frame_) {
      RCLCPP_ERROR(
        get_logger(), "Rejecting /nav3d/trajectory in frame '%s'; expected '%s'",
        msg->header.frame_id.c_str(), global_frame_.c_str());
      return;
    }

    auto output = *msg;
    for (auto & pose : output.poses) {
      if (pose.header.frame_id.empty()) {
        pose.header.frame_id = global_frame_;
      } else if (pose.header.frame_id != global_frame_) {
        RCLCPP_ERROR(
          get_logger(), "Rejecting trajectory containing pose in frame '%s'",
          pose.header.frame_id.c_str());
        return;
      }
    }
    path_pub_->publish(output);
  }

  static double clampFinite(double value, double limit)
  {
    if (!std::isfinite(value) || !std::isfinite(limit) || limit < 0.0) {
      return 0.0;
    }
    return std::clamp(value, -limit, limit);
  }

  void commandGateTick()
  {
    const auto current_time = now();
    const bool command_fresh =
      have_cmd_ && (current_time - last_cmd_time_).seconds() <= command_timeout_;
    const bool sensors_fresh =
      have_body_pose_ && have_cloud_ &&
      (current_time - last_odom_time_).seconds() <= sensor_timeout_ &&
      (current_time - last_cloud_time_).seconds() <= sensor_timeout_;

    std_msgs::msg::Bool ready;
    ready.data = command_fresh && sensors_fresh;
    ready_pub_->publish(ready);

    geometry_msgs::msg::Twist output;
    if (navigation_enabled_ && ready.data) {
      output.linear.x = clampFinite(last_cmd_.linear.x, max_vx_);
      output.linear.y = clampFinite(last_cmd_.linear.y, max_vy_);
      output.angular.z = clampFinite(last_cmd_.angular.z, max_wz_);
    }
    cmd_pub_->publish(output);
  }

  std::string global_frame_;
  std::string body_frame_;
  std::string input_odom_topic_;
  std::string input_cloud_topic_;
  std::string input_cmd_topic_;
  std::string output_cmd_topic_;
  double command_timeout_{0.30};
  double sensor_timeout_{0.50};
  double max_vx_{0.15};
  double max_vy_{0.10};
  double max_wz_{0.25};
  bool navigation_enabled_{false};
  bool have_cmd_{false};
  bool have_body_pose_{false};
  bool have_cloud_{false};

  geometry_msgs::msg::Twist last_cmd_;
  rclcpp::Time last_cmd_time_;
  rclcpp::Time last_odom_time_;
  rclcpp::Time last_cloud_time_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr body_odom_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;
  rclcpp::TimerBase::SharedPtr gate_timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AllBridges>());
  rclcpp::shutdown();
  return 0;
}
