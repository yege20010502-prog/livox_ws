#include <queue>
#include <deque>
#include <mutex>
#include <filesystem>
#include <cmath>
#include <algorithm>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include "localizers/commons.h"
#include "localizers/icp_localizer.h"
#include "interface/srv/relocalize.hpp"
#include "interface/srv/is_valid.hpp"
#include <yaml-cpp/yaml.h>

using namespace std::chrono_literals;

struct NodeConfig
{
    std::string cloud_topic = "/fastlio2/body_cloud";
    std::string odom_topic = "/fastlio2/lio_odom";
    std::string map_frame = "map";
    std::string local_frame = "lidar";
    std::string base_frame = "base_link";
    std::string map_path;
    double update_hz = 1.0;
    double transform_filter_alpha = 0.20;
    double translation_deadband = 0.02;
    double yaw_deadband = 0.015;
    double max_translation_jump = 0.25;
    double max_yaw_jump = 0.25;
    double max_initial_translation_jump = 0.60;
    int initial_consistency_count = 3;
    double initial_consistency_translation = 0.08;
    double initial_consistency_yaw = 0.08;
    int tracking_failure_count = 3;
};

struct NodeState
{
    std::mutex message_mutex;
    std::mutex service_mutex;

    bool message_received = false;
    bool service_received = false;
    bool localize_success = false;
    rclcpp::Time last_send_tf_time = rclcpp::Clock().now();
    builtin_interfaces::msg::Time last_message_time;
    rclcpp::Time last_sync_receive_time{0, 0, RCL_ROS_TIME};
    CloudType::Ptr last_cloud = std::make_shared<CloudType>();
    M3D last_r;                          // localmap_body_r
    V3D last_t;                          // localmap_body_t
    M3D last_offset_r = M3D::Identity(); // map_localmap_r
    V3D last_offset_t = V3D::Zero();     // map_localmap_t
    M4F initial_guess = M4F::Identity();
    int initial_consistent_matches = 0;
    int consecutive_tracking_failures = 0;
    M3D pending_offset_r = M3D::Identity();
    V3D pending_offset_t = V3D::Zero();
};

class LocalizerNode : public rclcpp::Node
{
public:
    LocalizerNode() : Node("localizer_node")
    {
        RCLCPP_INFO(this->get_logger(), "Localizer Node Started");
        loadParameters();
        // Match FAST-LIO's bounded best-effort sensor-cloud publisher so a
        // slow ICP/RViz consumer cannot back-pressure the estimator.  Keep
        // odometry reliable; ApproximateTime pairs both by header stamp.
        auto cloud_qos = rclcpp::SensorDataQoS().keep_last(1);
        auto odom_qos = rclcpp::SensorDataQoS().keep_last(20);
        m_cloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            m_config.cloud_topic, cloud_qos,
            std::bind(&LocalizerNode::cloudCB, this, std::placeholders::_1));
        m_odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            m_config.odom_topic, odom_qos,
            std::bind(&LocalizerNode::odomCB, this, std::placeholders::_1));

        m_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
        m_tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);

        m_localizer = std::make_shared<ICPLocalizer>(m_localizer_config);

        m_reloc_srv = this->create_service<interface::srv::Relocalize>("relocalize", std::bind(&LocalizerNode::relocCB, this, std::placeholders::_1, std::placeholders::_2));

        m_reloc_check_srv = this->create_service<interface::srv::IsValid>("relocalize_check", std::bind(&LocalizerNode::relocCheckCB, this, std::placeholders::_1, std::placeholders::_2));

        m_initial_pose_sub = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", rclcpp::QoS(10),
            std::bind(&LocalizerNode::initialPoseCB, this, std::placeholders::_1));

        auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
        m_map_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "map_cloud", map_qos);

        // The static map must be available before the first RViz initial pose.
        // Publishing it with transient-local durability also serves RViz
        // instances that are opened after the localizer starts.
        if (!m_config.map_path.empty() && std::filesystem::exists(m_config.map_path))
        {
            m_map_loaded = m_localizer->loadMap(m_config.map_path);
            if (m_map_loaded)
            {
                builtin_interfaces::msg::Time map_stamp = this->now();
                publishMapCloud(map_stamp);
                RCLCPP_INFO(this->get_logger(), "Loaded startup map: %s",
                            m_config.map_path.c_str());
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to load startup map: %s",
                             m_config.map_path.c_str());
            }
        }

        m_timer = this->create_wall_timer(10ms, std::bind(&LocalizerNode::timerCB, this));
        m_map_timer = this->create_wall_timer(1s, [this]() {
            if (!m_map_loaded)
                return;
            builtin_interfaces::msg::Time stamp = this->now();
            publishMapCloud(stamp);
        });
    }

    void loadParameters()
    {
        this->declare_parameter("config_path", "");
        std::string config_path;
        this->get_parameter<std::string>("config_path", config_path);
        YAML::Node config = YAML::LoadFile(config_path);
        if (!config)
        {
            RCLCPP_WARN(this->get_logger(), "FAIL TO LOAD YAML FILE!");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "LOAD FROM YAML CONFIG PATH: %s", config_path.c_str());

        m_config.cloud_topic = config["cloud_topic"].as<std::string>();
        m_config.odom_topic = config["odom_topic"].as<std::string>();
        m_config.map_frame = config["map_frame"].as<std::string>();
        m_config.local_frame = config["local_frame"].as<std::string>();
        this->declare_parameter("map_path", "");
        this->declare_parameter("base_frame", m_config.base_frame);
        this->get_parameter("map_path", m_config.map_path);
        this->get_parameter("base_frame", m_config.base_frame);
        m_config.update_hz = config["update_hz"].as<double>();
        if (config["transform_filter_alpha"])
            m_config.transform_filter_alpha = config["transform_filter_alpha"].as<double>();
        if (config["translation_deadband"])
            m_config.translation_deadband = config["translation_deadband"].as<double>();
        if (config["yaw_deadband"])
            m_config.yaw_deadband = config["yaw_deadband"].as<double>();
        if (config["max_translation_jump"])
            m_config.max_translation_jump = config["max_translation_jump"].as<double>();
        if (config["max_yaw_jump"])
            m_config.max_yaw_jump = config["max_yaw_jump"].as<double>();
        if (config["max_initial_translation_jump"])
            m_config.max_initial_translation_jump =
                config["max_initial_translation_jump"].as<double>();
        if (config["initial_consistency_count"])
            m_config.initial_consistency_count = config["initial_consistency_count"].as<int>();
        if (config["initial_consistency_translation"])
            m_config.initial_consistency_translation = config["initial_consistency_translation"].as<double>();
        if (config["initial_consistency_yaw"])
            m_config.initial_consistency_yaw = config["initial_consistency_yaw"].as<double>();
        if (config["tracking_failure_count"])
            m_config.tracking_failure_count = config["tracking_failure_count"].as<int>();

        m_localizer_config.rough_scan_resolution = config["rough_scan_resolution"].as<double>();
        m_localizer_config.rough_map_resolution = config["rough_map_resolution"].as<double>();
        m_localizer_config.rough_max_iteration = config["rough_max_iteration"].as<int>();
        m_localizer_config.rough_score_thresh = config["rough_score_thresh"].as<double>();

        m_localizer_config.refine_scan_resolution = config["refine_scan_resolution"].as<double>();
        m_localizer_config.refine_map_resolution = config["refine_map_resolution"].as<double>();
        m_localizer_config.refine_max_iteration = config["refine_max_iteration"].as<int>();
        m_localizer_config.refine_score_thresh = config["refine_score_thresh"].as<double>();
        if (config["rough_max_correspondence_distance"])
            m_localizer_config.rough_max_correspondence_distance =
                config["rough_max_correspondence_distance"].as<double>();
        if (config["refine_max_correspondence_distance"])
            m_localizer_config.refine_max_correspondence_distance =
                config["refine_max_correspondence_distance"].as<double>();
        if (config["min_overlap_ratio"])
            m_localizer_config.min_overlap_ratio = config["min_overlap_ratio"].as<double>();
        if (config["min_overlap_points"])
            m_localizer_config.min_overlap_points = config["min_overlap_points"].as<int>();
        if (config["remove_ground_plane"])
            m_localizer_config.remove_ground_plane = config["remove_ground_plane"].as<bool>();
        if (config["ground_plane_distance"])
            m_localizer_config.ground_plane_distance = config["ground_plane_distance"].as<double>();
        if (config["ground_plane_min_points"])
            m_localizer_config.ground_plane_min_points = config["ground_plane_min_points"].as<int>();
    }
    void timerCB()
    {
        if (!m_state.message_received || !m_map_loaded)
            return;

        {
            std::lock_guard<std::mutex> lock(m_state.message_mutex);
            if ((this->now() - m_state.last_sync_receive_time).seconds() > 1.0)
            {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                     "No fresh synchronized LIO cloud/odometry; pausing map->odom TF");
                return;
            }
        }

        // Do not run ICP before an explicit initial-pose/relocalization
        // request.  The static map is already published independently above.
        {
            std::lock_guard<std::mutex> lock(m_state.service_mutex);
            if (!m_state.service_received && !m_state.localize_success)
                return;
        }

        rclcpp::Duration diff = rclcpp::Clock().now() - m_state.last_send_tf_time;

        bool update_tf = diff.seconds() > (1.0 / m_config.update_hz) && m_state.message_received;

        if (!update_tf)
        {
            if (m_state.localize_success)
                sendBroadCastTF(m_state.last_message_time);
            return;
        }

        m_state.last_send_tf_time = rclcpp::Clock().now();

        M4F initial_guess = M4F::Identity();
        if (m_state.service_received)
        {
            std::lock_guard<std::mutex>(m_state.service_mutex);
            initial_guess = m_state.initial_guess;
            // m_state.service_received = false;
        }
        else
        {
            std::lock_guard<std::mutex>(m_state.message_mutex);
            initial_guess.block<3, 3>(0, 0) = m_state.last_offset_r.cast<float>();
            initial_guess.block<3, 1>(0, 3) = m_state.last_offset_t.cast<float>();
        }

        builtin_interfaces::msg::Time current_time;
        {
            std::lock_guard<std::mutex>(m_state.message_mutex);
            current_time = m_state.last_message_time;
            m_localizer->setInput(m_state.last_cloud);
        }

        const M4F alignment_seed = initial_guess;
        bool result = m_localizer->align(initial_guess);
        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "ICP diagnostics: accepted=%s rough_converged=%s rough_score=%.6f/%.6f "
            "refine_converged=%s refine_score=%.6f/%.6f overlap=%d/%d=%.3f/%.3f",
            result ? "true" : "false",
            m_localizer->roughConverged() ? "true" : "false",
            m_localizer->roughScore(), m_localizer_config.rough_score_thresh,
            m_localizer->refineConverged() ? "true" : "false",
            m_localizer->refineScore(), m_localizer_config.refine_score_thresh,
            m_localizer->overlapPoints(), m_localizer->inputPoints(),
            m_localizer->overlapRatio(), m_localizer_config.min_overlap_ratio);
        // Initial consistency means consecutive valid observations. A failed
        // ICP frame must break the sequence; otherwise intermittent matches
        // in a repeated scene eventually accumulate into a false success.
        if (!result && !m_state.localize_success)
            m_state.initial_consistent_matches = 0;
        if (result)
        {
            // ICP directly returns map -> odom. Keep this transform planar and
            // filter small scan-to-scan corrections so the map does not shake.
            const M3D candidate_r = initial_guess.block<3, 3>(0, 0).cast<double>();
            V3D candidate_t = initial_guess.block<3, 1>(0, 3).cast<double>();
            candidate_t.z() = 0.0;
            const double candidate_yaw = std::atan2(candidate_r(1, 0), candidate_r(0, 0));
            const double previous_yaw =
                std::atan2(m_state.last_offset_r(1, 0), m_state.last_offset_r(0, 0));
            const double delta_yaw =
                std::atan2(std::sin(candidate_yaw - previous_yaw),
                           std::cos(candidate_yaw - previous_yaw));
            const double delta_xy =
                (candidate_t.head<2>() - m_state.last_offset_t.head<2>()).norm();
            const bool already_localized = m_state.localize_success;

            if (already_localized &&
                (delta_xy > m_config.max_translation_jump ||
                 std::abs(delta_yaw) > m_config.max_yaw_jump))
            {
                RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 2000,
                    "Reject unstable map->odom correction: dxy=%.3f m dyaw=%.3f rad",
                    delta_xy, delta_yaw);
                result = false;
            }
            else if (!already_localized)
            {
                const M3D seed_r = alignment_seed.block<3, 3>(0, 0).cast<double>();
                const V3D seed_t = alignment_seed.block<3, 1>(0, 3).cast<double>();
                const double seed_yaw = std::atan2(seed_r(1, 0), seed_r(0, 0));
                const double initial_delta_xy =
                    (candidate_t.head<2>() - seed_t.head<2>()).norm();
                const double initial_delta_yaw = std::atan2(
                    std::sin(candidate_yaw - seed_yaw),
                    std::cos(candidate_yaw - seed_yaw));

                if (initial_delta_xy > m_config.max_initial_translation_jump ||
                    std::abs(initial_delta_yaw) > m_config.max_yaw_jump)
                {
                    RCLCPP_WARN_THROTTLE(
                        get_logger(), *get_clock(), 2000,
                        "Reject initial ICP jump from RViz guess: dxy=%.3f m dyaw=%.3f rad",
                        initial_delta_xy, initial_delta_yaw);
                    m_state.initial_consistent_matches = 0;
                    result = false;
                }
                else
                {
                    const double pending_yaw = std::atan2(
                        m_state.pending_offset_r(1, 0), m_state.pending_offset_r(0, 0));
                    const double pending_delta_xy =
                        (candidate_t.head<2>() - m_state.pending_offset_t.head<2>()).norm();
                    const double pending_delta_yaw = std::atan2(
                        std::sin(candidate_yaw - pending_yaw),
                        std::cos(candidate_yaw - pending_yaw));
                    if (m_state.initial_consistent_matches == 0 ||
                        pending_delta_xy > m_config.initial_consistency_translation ||
                        std::abs(pending_delta_yaw) > m_config.initial_consistency_yaw)
                    {
                        m_state.initial_consistent_matches = 1;
                    }
                    else
                    {
                        ++m_state.initial_consistent_matches;
                    }
                    m_state.pending_offset_t = candidate_t;
                    m_state.pending_offset_r = Eigen::AngleAxisd(
                        candidate_yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();

                    if (m_state.initial_consistent_matches >=
                        m_config.initial_consistency_count)
                    {
                        m_state.last_offset_t = candidate_t;
                        m_state.last_offset_r = m_state.pending_offset_r;
                        std::lock_guard<std::mutex> lock(m_state.service_mutex);
                        m_state.localize_success = true;
                        m_state.service_received = false;
                        m_state.initial_consistent_matches = 0;
                        m_state.consecutive_tracking_failures = 0;
                        RCLCPP_INFO(get_logger(),
                                    "Initial localization accepted after %d consistent ICP results",
                                    m_config.initial_consistency_count);
                    }
                    else
                    {
                        RCLCPP_INFO(get_logger(),
                                    "Initial ICP consistency %d/%d",
                                    m_state.initial_consistent_matches,
                                    m_config.initial_consistency_count);
                    }
                }
            }
            else if (delta_xy >= m_config.translation_deadband ||
                     std::abs(delta_yaw) >= m_config.yaw_deadband)
            {
                const double alpha = std::clamp(m_config.transform_filter_alpha, 0.0, 1.0);
                m_state.last_offset_t += alpha * (candidate_t - m_state.last_offset_t);
                const double filtered_yaw = previous_yaw + alpha * delta_yaw;
                m_state.last_offset_r =
                    Eigen::AngleAxisd(filtered_yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
            }
        }
        if (m_state.localize_success)
        {
            if (result)
            {
                m_state.consecutive_tracking_failures = 0;
            }
            else
            {
                ++m_state.consecutive_tracking_failures;
                if (m_state.consecutive_tracking_failures >=
                    m_config.tracking_failure_count)
                {
                    std::lock_guard<std::mutex> lock(m_state.service_mutex);
                    m_state.localize_success = false;
                    m_state.service_received = false;
                    RCLCPP_ERROR(
                        get_logger(),
                        "Localization invalidated after %d consecutive ICP quality failures",
                        m_state.consecutive_tracking_failures);
                }
            }
        }
        if (m_state.localize_success)
            sendBroadCastTF(current_time);
        publishMapCloud(current_time);
    }
    void odomCB(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg)
    {
        sensor_msgs::msg::PointCloud2::ConstSharedPtr matched_cloud;
        double best_delta = std::numeric_limits<double>::infinity();
        {
            std::lock_guard<std::mutex> lock(m_pair_mutex);
            m_odom_cache.push_back(odom_msg);
            while (m_odom_cache.size() > 50)
                m_odom_cache.pop_front();

            const rclcpp::Time odom_stamp(odom_msg->header.stamp);
            auto best_it = m_cloud_cache.end();
            for (auto it = m_cloud_cache.begin(); it != m_cloud_cache.end(); ++it)
            {
                const double delta = std::abs(
                    (odom_stamp - rclcpp::Time((*it)->header.stamp)).seconds());
                if (delta < best_delta)
                {
                    best_delta = delta;
                    best_it = it;
                }
            }
            if (best_it != m_cloud_cache.end() && best_delta <= 0.12)
            {
                matched_cloud = *best_it;
                m_cloud_cache.erase(best_it);
            }
        }
        if (matched_cloud)
            syncCB(matched_cloud, odom_msg);
    }

    void cloudCB(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg)
    {
        nav_msgs::msg::Odometry::ConstSharedPtr matched_odom;
        double best_delta = std::numeric_limits<double>::infinity();
        {
            std::lock_guard<std::mutex> lock(m_pair_mutex);
            m_cloud_cache.push_back(cloud_msg);
            while (m_cloud_cache.size() > 50)
                m_cloud_cache.pop_front();
            const rclcpp::Time cloud_stamp(cloud_msg->header.stamp);
            for (const auto &odom : m_odom_cache)
            {
                const double delta = std::abs(
                    (cloud_stamp - rclcpp::Time(odom->header.stamp)).seconds());
                if (delta < best_delta)
                {
                    best_delta = delta;
                    matched_odom = odom;
                }
            }
            if (matched_odom && best_delta <= 0.12)
                m_cloud_cache.pop_back();
        }
        if (!matched_odom || best_delta > 0.12)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "No odometry within 120 ms of body cloud (best=%.6f s)", best_delta);
            return;
        }
        syncCB(cloud_msg, matched_odom);
    }

    void syncCB(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud_msg, const nav_msgs::msg::Odometry::ConstSharedPtr &odom_msg)
    {

        std::lock_guard<std::mutex> lock(m_state.message_mutex);

        if (m_state.message_received &&
            rclcpp::Time(cloud_msg->header.stamp) <= rclcpp::Time(m_state.last_message_time))
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "Dropping out-of-order synchronized LIO frame");
            return;
        }

        CloudType::Ptr body_cloud = std::make_shared<CloudType>();
        pcl::fromROSMsg(*cloud_msg, *body_cloud);

        m_state.last_r = Eigen::Quaterniond(odom_msg->pose.pose.orientation.w,
                                            odom_msg->pose.pose.orientation.x,
                                            odom_msg->pose.pose.orientation.y,
                                            odom_msg->pose.pose.orientation.z)
                             .toRotationMatrix();
        m_state.last_t = V3D(odom_msg->pose.pose.position.x,
                             odom_msg->pose.pose.position.y,
                             odom_msg->pose.pose.position.z);

        // Transform the synchronized imu_link scan into odom before ICP.
        Eigen::Matrix4f odom_body = Eigen::Matrix4f::Identity();
        odom_body.block<3, 3>(0, 0) = m_state.last_r.cast<float>();
        odom_body.block<3, 1>(0, 3) = m_state.last_t.cast<float>();
        pcl::transformPointCloud(
            *body_cloud, *m_state.last_cloud, odom_body);
        m_state.last_message_time = cloud_msg->header.stamp;
        m_state.last_sync_receive_time = this->now();
        if (!m_state.message_received)
        {
            m_state.message_received = true;
            m_config.local_frame = odom_msg->header.frame_id;
        }
    }

    void sendBroadCastTF(builtin_interfaces::msg::Time &time)
    {
        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.frame_id = m_config.map_frame;
        transformStamped.child_frame_id = m_config.local_frame;
        transformStamped.header.stamp = time;
        Eigen::Quaterniond q(m_state.last_offset_r);
        V3D t = m_state.last_offset_t;
        transformStamped.transform.translation.x = t.x();
        transformStamped.transform.translation.y = t.y();
        transformStamped.transform.translation.z = t.z();
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();
        m_tf_broadcaster->sendTransform(transformStamped);
    }

    void relocCB(const std::shared_ptr<interface::srv::Relocalize::Request> request, std::shared_ptr<interface::srv::Relocalize::Response> response)
    {
        std::string pcd_path = request->pcd_path;
        float x = request->x;
        float y = request->y;
        float yaw = request->yaw;

        if (!std::filesystem::exists(pcd_path))
        {
            response->success = false;
            response->message = "pcd file not found";
            return;
        }

        Eigen::AngleAxisd yaw_angle = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
        bool load_flag = m_localizer->loadMap(pcd_path);
        if (!load_flag)
        {
            response->success = false;
            response->message = "load map failed";
            return;
        }
        m_map_loaded = true;
        {
            std::lock_guard<std::mutex>(m_state.service_mutex);
            m_state.initial_guess.setIdentity();
            m_state.initial_guess.block<3, 3>(0, 0) =
                yaw_angle.toRotationMatrix().cast<float>();
            m_state.initial_guess.block<3, 1>(0, 3) = V3F(x, y, 0.0f);
            m_state.service_received = true;
            m_state.localize_success = false;
            m_state.initial_consistent_matches = 0;
            m_state.consecutive_tracking_failures = 0;
        }

        response->success = true;
        response->message =
            "relocalize request accepted; wait for relocalize_check validation";
        return;
    }

    void initialPoseCB(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        if (msg->header.frame_id != m_config.map_frame)
        {
            RCLCPP_ERROR(get_logger(),
                         "Reject /initialpose in frame '%s'; expected '%s'",
                         msg->header.frame_id.c_str(), m_config.map_frame.c_str());
            return;
        }
        if (m_config.map_path.empty())
        {
            RCLCPP_ERROR(get_logger(),
                         "Reject /initialpose: map_path parameter is empty");
            return;
        }
        {
            std::lock_guard<std::mutex> lock(m_state.message_mutex);
            if (!m_state.message_received ||
                (this->now() - m_state.last_sync_receive_time).seconds() > 1.0)
            {
                RCLCPP_ERROR(get_logger(),
                             "Reject /initialpose: no fresh synchronized LIO input");
                return;
            }
        }

        geometry_msgs::msg::TransformStamped odom_base;
        try
        {
            odom_base = m_tf_buffer->lookupTransform(
                m_config.local_frame, m_config.base_frame, tf2::TimePointZero);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_ERROR(get_logger(),
                         "Reject /initialpose: cannot get %s -> %s TF: %s",
                         m_config.local_frame.c_str(), m_config.base_frame.c_str(), ex.what());
            return;
        }

        const auto &map_pose = msg->pose.pose;
        Eigen::Quaterniond q_map_base(map_pose.orientation.w,
                                      map_pose.orientation.x,
                                      map_pose.orientation.y,
                                      map_pose.orientation.z);
        const auto &odom_tf = odom_base.transform;
        Eigen::Quaterniond q_odom_base(odom_tf.rotation.w,
                                       odom_tf.rotation.x,
                                       odom_tf.rotation.y,
                                       odom_tf.rotation.z);
        if (q_map_base.norm() < 1e-6 || q_odom_base.norm() < 1e-6)
        {
            RCLCPP_ERROR(get_logger(), "Reject /initialpose: invalid quaternion");
            return;
        }
        q_map_base.normalize();
        q_odom_base.normalize();

        const M3D r_map_base = q_map_base.toRotationMatrix();
        const M3D r_odom_base = q_odom_base.toRotationMatrix();
        const V3D t_map_base(map_pose.position.x,
                             map_pose.position.y,
                             map_pose.position.z);
        const V3D t_odom_base(odom_tf.translation.x,
                              odom_tf.translation.y,
                              odom_tf.translation.z);
        const M3D r_map_odom = r_map_base * r_odom_base.transpose();
        const V3D t_map_odom = t_map_base - r_map_odom * t_odom_base;
        const double yaw = std::atan2(r_map_odom(1, 0), r_map_odom(0, 0));

        auto request = std::make_shared<interface::srv::Relocalize::Request>();
        auto response = std::make_shared<interface::srv::Relocalize::Response>();
        request->pcd_path = m_config.map_path;
        request->x = static_cast<float>(t_map_odom.x());
        request->y = static_cast<float>(t_map_odom.y());
        request->z = 0.0f;
        request->yaw = static_cast<float>(yaw);
        request->pitch = 0.0f;
        request->roll = 0.0f;
        relocCB(request, response);
        if (!response->success)
        {
            RCLCPP_ERROR(get_logger(), "RViz initial pose rejected: %s",
                         response->message.c_str());
            return;
        }
        RCLCPP_INFO(get_logger(),
                    "Accepted RViz initial pose; map->odom guess x=%.3f y=%.3f yaw=%.3f",
                    request->x, request->y, request->yaw);
    }

    void relocCheckCB(const std::shared_ptr<interface::srv::IsValid::Request> request, std::shared_ptr<interface::srv::IsValid::Response> response)
    {
        std::lock_guard<std::mutex>(m_state.service_mutex);
        if (request->code == 1)
            response->valid = true;
        else
            response->valid = m_state.localize_success;
        return;
    }
    void publishMapCloud(builtin_interfaces::msg::Time &time)
    {
        CloudType::Ptr map_cloud = m_localizer->refineMap();
        if (map_cloud->size() < 1)
            return;
        sensor_msgs::msg::PointCloud2 map_cloud_msg;
        pcl::toROSMsg(*map_cloud, map_cloud_msg);
        map_cloud_msg.header.frame_id = m_config.map_frame;
        map_cloud_msg.header.stamp = time;
        m_map_cloud_pub->publish(map_cloud_msg);
    }

private:
    bool m_map_loaded{false};
    NodeConfig m_config;
    NodeState m_state;

    ICPConfig m_localizer_config;
    std::shared_ptr<ICPLocalizer> m_localizer;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_cloud_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_odom_sub;
    std::mutex m_pair_mutex;
    std::deque<nav_msgs::msg::Odometry::ConstSharedPtr> m_odom_cache;
    std::deque<sensor_msgs::msg::PointCloud2::ConstSharedPtr> m_cloud_cache;
    rclcpp::TimerBase::SharedPtr m_timer;
    rclcpp::TimerBase::SharedPtr m_map_timer;
    std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;
    std::unique_ptr<tf2_ros::Buffer> m_tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> m_tf_listener;
    rclcpp::Service<interface::srv::Relocalize>::SharedPtr m_reloc_srv;
    rclcpp::Service<interface::srv::IsValid>::SharedPtr m_reloc_check_srv;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr m_initial_pose_sub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_map_cloud_pub;
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LocalizerNode>());
    rclcpp::shutdown();
    return 0;
}
