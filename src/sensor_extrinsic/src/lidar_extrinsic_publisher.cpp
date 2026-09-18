#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

// SCAN needs the live global pose of the frame in which its input point cloud
// is expressed. FAST-LIO publishes /fastlio2/body_cloud in imu_link, so this
// node publishes map -> imu_link as nav_msgs/Odometry. The static mounting
// transforms remain in TF and are never published as a fake global pose.
class LidarExtrinsicPublisher : public rclcpp::Node
{
public:
  LidarExtrinsicPublisher()
  : Node("scan_sensor_pose_publisher"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    global_frame_ = declare_parameter<std::string>("global_frame", "map");
    sensor_frame_ = declare_parameter<std::string>("sensor_frame", "imu_link");
    output_topic_ = declare_parameter<std::string>("output_topic", "/LIO/odom_imu");
    publish_rate_ = declare_parameter<double>("publish_rate", 20.0);

    if (publish_rate_ <= 0.0) {
      throw std::invalid_argument("publish_rate must be positive");
    }

    publisher_ = create_publisher<nav_msgs::msg::Odometry>(output_topic_, 10);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / publish_rate_)),
      [this]() { publishSensorPose(); });

    RCLCPP_INFO(
      get_logger(), "Publishing live %s -> %s pose on %s",
      global_frame_.c_str(), sensor_frame_.c_str(), output_topic_.c_str());
  }

private:
  void publishSensorPose()
  {
    try {
      const auto transform = tf_buffer_.lookupTransform(
        global_frame_, sensor_frame_, tf2::TimePointZero);

      nav_msgs::msg::Odometry msg;
      msg.header = transform.header;
      msg.header.frame_id = global_frame_;
      msg.child_frame_id = sensor_frame_;
      msg.pose.pose.position.x = transform.transform.translation.x;
      msg.pose.pose.position.y = transform.transform.translation.y;
      msg.pose.pose.position.z = transform.transform.translation.z;
      msg.pose.pose.orientation = transform.transform.rotation;
      publisher_->publish(msg);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Waiting for %s -> %s: %s",
        global_frame_.c_str(), sensor_frame_.c_str(), ex.what());
    }
  }

  std::string global_frame_;
  std::string sensor_frame_;
  std::string output_topic_;
  double publish_rate_{20.0};
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarExtrinsicPublisher>());
  rclcpp::shutdown();
  return 0;
}
