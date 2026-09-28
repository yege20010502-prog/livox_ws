#include <algorithm>
#include <cstring>
#include <memory>
#include <optional>
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
    // map->odom is corrected by the localizer at roughly 1 Hz.  A 0.5 s
    // limit therefore rejected a valid transform for about half of every
    // cycle and could starve SCAN's body-pose input.  Keep the limit finite
    // so a genuinely stopped localizer still makes the controller fail safe.
    max_pose_age_sec_ = declare_parameter<double>("max_pose_age_sec", 1.5);
    scan_body_height_ = declare_parameter<double>("scan_body_height", 0.4);
    flatten_scan_path_ = declare_parameter<bool>("flatten_scan_path", true);
    const auto odom_topic = declare_parameter<std::string>(
      "input_odom_topic", "/fastlio2/lio_odom");
    const auto cloud_topic = declare_parameter<std::string>(
      "input_cloud_topic", "/fastlio2/body_cloud");
    const auto viz_cloud_topic = declare_parameter<std::string>(
      "viz_cloud_topic", "/fastlio2/world_cloud");
    viz_point_stride_ = std::max(
      1, static_cast<int>(declare_parameter<int64_t>("viz_point_stride", 16)));
    viz_frame_divider_ = std::max(
      1, static_cast<int>(declare_parameter<int64_t>("viz_frame_divider", 2)));

    path_pub_ = create_publisher<nav_msgs::msg::Path>("/initial_path", 10);
    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/LIO/clouds_lidar", rclcpp::SensorDataQoS());
    viz_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/fastlio2/world_cloud_viz", rclcpp::SensorDataQoS().keep_last(1));
    body_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      "/LIO/odom_vehicle", 10);

    // FAST-LIO is a live sensor stream.  SensorDataQoS avoids a DDS
    // reliable-reader stall seen after the navigation stack is restarted
    // while the FAST-LIO writer remains active.
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::SharedPtr msg) {
        try {
          const auto tf = tf_buffer_.lookupTransform(
            global_frame_, body_frame_, tf2::TimePointZero);
          const double pose_age =
            (this->now() - rclcpp::Time(tf.header.stamp, RCL_ROS_TIME)).seconds();
          if (pose_age < -0.2 || pose_age > max_pose_age_sec_) {
            RCLCPP_WARN_THROTTLE(
              get_logger(), *get_clock(), 2000,
              "Rejecting stale map pose: TF age %.3f s", pose_age);
            return;
          }
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
          latest_map_pose_ = output.pose.pose;
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
        // In flat ground mode SCAN expects route/ground height and adds
        // scan_body_height_ internally.  Use the latest real body height so
        // a path first generated while the dog is lying down is automatically
        // raised after the pre-stand transition.  Otherwise SCAN attempts a
        // vertical trajectory back to the stale lying height and reports the
        // first/last control points as occupied.
        const double scan_route_z =
          flatten_scan_path_ && latest_map_pose_.has_value() ?
          latest_map_pose_->position.z - scan_body_height_ :
          (output.poses.empty() ? 0.0 :
          output.poses.front().pose.position.z - scan_body_height_);
        for (auto & pose : output.poses) {
          if (pose.header.frame_id.empty()) {
            pose.header.frame_id = global_frame_;
          } else if (pose.header.frame_id != global_frame_) {
            RCLCPP_ERROR(
              get_logger(), "Rejecting pose frame '%s' in trajectory",
              pose.header.frame_id.c_str());
            return;
          }
          // Nav3D ground trajectories use supported body-cell heights. SCAN
          // expects ground/route heights and adds grid_map.body_height itself.
          // Remove that known offset here to avoid adding 0.4 m twice.
          pose.pose.position.z = flatten_scan_path_ ?
            scan_route_z : pose.pose.position.z - scan_body_height_;
        }
        if (latest_map_pose_.has_value() && !output.poses.empty()) {
          geometry_msgs::msg::PoseStamped anchor;
          anchor.header = output.header;
          anchor.pose = *latest_map_pose_;
          anchor.pose.position.z = scan_route_z;
          output.poses.insert(output.poses.begin(), anchor);
        }
        path_pub_->publish(output);
      });

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        cloud_pub_->publish(*msg);
      });

    viz_cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      viz_cloud_topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        if ((viz_frame_counter_++ % static_cast<std::size_t>(viz_frame_divider_)) != 0) {
          return;
        }
        const std::size_t point_count =
          static_cast<std::size_t>(msg->width) * static_cast<std::size_t>(msg->height);
        if (point_count == 0 || msg->point_step == 0 ||
          msg->data.size() < point_count * msg->point_step)
        {
          return;
        }

        sensor_msgs::msg::PointCloud2 output = *msg;
        output.height = 1;
        output.width = static_cast<uint32_t>(
          (point_count + static_cast<std::size_t>(viz_point_stride_) - 1) /
          static_cast<std::size_t>(viz_point_stride_));
        output.row_step = output.width * output.point_step;
        output.data.resize(output.row_step);

        std::size_t destination = 0;
        for (std::size_t source = 0; source < point_count;
          source += static_cast<std::size_t>(viz_point_stride_))
        {
          std::memcpy(
            output.data.data() + destination * output.point_step,
            msg->data.data() + source * msg->point_step,
            output.point_step);
          ++destination;
        }
        viz_cloud_pub_->publish(output);
      });

    RCLCPP_INFO(
      get_logger(),
      "Data bridge ready: odom, cloud and path only; no /cmd_vel publisher; "
      "max map-pose age %.2f s",
      max_pose_age_sec_);
  }

private:
  std::string global_frame_;
  std::string body_frame_;
  double max_pose_age_sec_{1.5};
  double scan_body_height_{0.4};
  bool flatten_scan_path_{true};
  int viz_point_stride_{16};
  int viz_frame_divider_{2};
  std::size_t viz_frame_counter_{0};
  std::optional<geometry_msgs::msg::Pose> latest_map_pose_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr viz_cloud_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr body_odom_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr viz_cloud_sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavDataBridge>());
  rclcpp::shutdown();
  return 0;
}
