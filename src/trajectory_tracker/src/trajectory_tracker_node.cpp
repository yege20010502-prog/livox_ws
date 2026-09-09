#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <cmath>
#include <chrono>
#include <deque>

class Nav3DTracker : public rclcpp::Node {
public:
    Nav3DTracker() : Node("nav3d_tracker") {
        m_path_sub = this->create_subscription<nav_msgs::msg::Path>(
            "/nav3d/trajectory", rclcpp::QoS(10),
            std::bind(&Nav3DTracker::pathCallback, this, std::placeholders::_1));

        m_pose_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/nav3d/current_pose", rclcpp::QoS(10),
            std::bind(&Nav3DTracker::poseCallback, this, std::placeholders::_1));

        m_cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", rclcpp::QoS(10));

        m_timer = this->create_wall_timer(std::chrono::milliseconds(50),
            std::bind(&Nav3DTracker::controlLoop, this));

        RCLCPP_INFO(this->get_logger(), "Nav3D 轨迹跟踪器已启动 (20Hz)");
        RCLCPP_INFO(this->get_logger(), "速度范围: vx 0.1~1.0 | vyaw 0.1~1.0 (只左转+前进)");
        RCLCPP_INFO(this->get_logger(), "⚠️ 最小速度 0.1 (低于此值机器狗不响应)");
        RCLCPP_INFO(this->get_logger(), "📌 路径对比方式: 终点距离 > 0.5m 或 路径点数变化 > 3 视为新路径");
    }

private:
    // ========================================================================
    // 1. 路径回调 - 智能区分新旧路径
    // ========================================================================
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
        if (msg->poses.empty()) {
            RCLCPP_WARN(this->get_logger(), "收到空路径，忽略");
            return;
        }

        // --- 如果是第一次收到路径，直接采纳 ---
        if (!m_path) {
            acceptNewPath(msg);
            return;
        }

        // --- 判断是否为新路径 ---
        bool is_new_path = isPathDifferent(msg);

        if (is_new_path) {
            RCLCPP_INFO(this->get_logger(), "🔄 检测到新路径，替换当前路径");
            acceptNewPath(msg);
        } else {
            RCLCPP_DEBUG(this->get_logger(), "📌 路径与当前相同，忽略更新");
        }
    }

    // ========================================================================
    // 2. 路径差异判断函数（核心）
    // ========================================================================
    bool isPathDifferent(const nav_msgs::msg::Path::SharedPtr new_path) {
        if (!m_path) return true;

        auto& old_poses = m_path->poses;
        auto& new_poses = new_path->poses;

        // 条件1：路径点数变化超过3个，视为新路径
        int size_diff = std::abs((int)old_poses.size() - (int)new_poses.size());
        if (size_diff > 3) {
            RCLCPP_DEBUG(this->get_logger(),
                "路径点数变化: %d -> %zu (差值 %d > 3)，视为新路径",
                old_poses.size(), new_poses.size(), size_diff);
            return true;
        }

        // 条件2：比较路径终点（距离 > 0.5m 视为新路径）
        auto& old_last = old_poses.back().pose.position;
        auto& new_last = new_poses.back().pose.position;
        double dx = new_last.x - old_last.x;
        double dy = new_last.y - old_last.y;
        double dist = std::sqrt(dx*dx + dy*dy);

        if (dist > 0.5) {
            RCLCPP_DEBUG(this->get_logger(),
                "终点距离变化: %.2f m (> 0.5m)，视为新路径", dist);
            return true;
        }

        // 条件3：比较路径起点（距离 > 0.3m 视为新路径）
        auto& old_first = old_poses.front().pose.position;
        auto& new_first = new_poses.front().pose.position;
        dx = new_first.x - old_first.x;
        dy = new_first.y - old_first.y;
        dist = std::sqrt(dx*dx + dy*dy);

        if (dist > 0.3) {
            RCLCPP_DEBUG(this->get_logger(),
                "起点距离变化: %.2f m (> 0.3m)，视为新路径", dist);
            return true;
        }

        // 条件4：采样比较路径中间点（每5个点取1个）
        int sample_step = std::max(1, (int)std::min(old_poses.size(), new_poses.size()) / 10);
        int max_samples = std::min((int)old_poses.size(), (int)new_poses.size());
        int diff_count = 0;
        const int DIFF_THRESHOLD = 3;  // 超过3个采样点不同，视为新路径

        for (int i = 0; i < max_samples; i += sample_step) {
            auto& old_p = old_poses[i].pose.position;
            auto& new_p = new_poses[i].pose.position;
            dx = new_p.x - old_p.x;
            dy = new_p.y - old_p.y;
            if (std::sqrt(dx*dx + dy*dy) > 0.2) {
                diff_count++;
                if (diff_count > DIFF_THRESHOLD) {
                    RCLCPP_DEBUG(this->get_logger(),
                        "中间点差异超过 %d 个，视为新路径", DIFF_THRESHOLD);
                    return true;
                }
            }
        }

        // 所有条件都不满足，认为是同一条路径
        return false;
    }

    // ========================================================================
    // 3. 采纳新路径
    // ========================================================================
    void acceptNewPath(const nav_msgs::msg::Path::SharedPtr msg) {
        // --- 过滤冗余点，减少路径点数量 ---
        nav_msgs::msg::Path filtered_path;
        filtered_path.header = msg->header;
        filtered_path.poses.push_back(msg->poses[0]);

        for (size_t i = 1; i < msg->poses.size(); i++) {
            auto& prev = filtered_path.poses.back().pose.position;
            auto& curr = msg->poses[i].pose.position;
            double dx = curr.x - prev.x;
            double dy = curr.y - prev.y;
            if (std::sqrt(dx*dx + dy*dy) > 0.3) {
                filtered_path.poses.push_back(msg->poses[i]);
            }
        }

        // 确保最后一个点被包含
        if (filtered_path.poses.size() > 1) {
            auto& last_filtered = filtered_path.poses.back().pose.position;
            auto& last_original = msg->poses.back().pose.position;
            double dx = last_original.x - last_filtered.x;
            double dy = last_original.y - last_filtered.y;
            if (std::sqrt(dx*dx + dy*dy) > 0.1) {
                filtered_path.poses.push_back(msg->poses.back());
            }
        }

        if (filtered_path.poses.size() < 2) {
            RCLCPP_WARN(this->get_logger(), "过滤后路径点太少 (<2)，使用原始路径");
            filtered_path = *msg;
        }

        // --- 更新路径 ---
        m_path = std::make_shared<nav_msgs::msg::Path>(filtered_path);
        m_path_index = 0;
        m_tracking = true;
        m_rotating = false;
        m_consecutive_stuck = 0;

        RCLCPP_INFO(this->get_logger(), "✅ 采纳新路径，原始 %zu 个点 -> 过滤后 %zu 个点",
                    msg->poses.size(), m_path->poses.size());

        // 打印前5个目标点
        for (size_t i = 0; i < std::min(size_t(5), m_path->poses.size()); i++) {
            auto& p = m_path->poses[i];
            RCLCPP_INFO(this->get_logger(), "  目标点 %zu: (%.2f, %.2f)",
                        i+1, p.pose.position.x, p.pose.position.y);
        }
        if (m_path->poses.size() > 5) {
            RCLCPP_INFO(this->get_logger(), "  ... 共 %zu 个目标点", m_path->poses.size());
        }
    }

    // ========================================================================
    // 4. 位姿回调
    // ========================================================================
    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        m_pose = msg;
    }

    // ========================================================================
    // 5. 工具函数
    // ========================================================================
    double quatToYaw(double qx, double qy, double qz, double qw) const {
        double siny = 2.0 * (qw * qz + qx * qy);
        double cosy = 1.0 - 2.0 * (qy * qy + qz * qz);
        return std::atan2(siny, cosy);
    }

    double normalizeAngle(double a) const {
        while (a > M_PI) a -= 2.0 * M_PI;
        while (a < -M_PI) a += 2.0 * M_PI;
        return a;
    }

    // ========================================================================
    // 6. 主控制循环
    // ========================================================================
    void controlLoop() {
        if (!m_path || !m_pose || !m_tracking) {
            publishStop();
            return;
        }

        double rx = m_pose->pose.position.x;
        double ry = m_pose->pose.position.y;
        double ryaw = quatToYaw(
            m_pose->pose.orientation.x,
            m_pose->pose.orientation.y,
            m_pose->pose.orientation.z,
            m_pose->pose.orientation.w
        );

        auto& target = m_path->poses[m_path_index];
        double tx = target.pose.position.x;
        double ty = target.pose.position.y;

        double dx = tx - rx;
        double dy = ty - ry;
        double dist = std::sqrt(dx * dx + dy * dy);
        double goal_angle = std::atan2(dy, dx);
        double angle_err = normalizeAngle(goal_angle - ryaw);

        // ===== 到达判定 =====
        constexpr double GOAL_TOLERANCE = 0.3;

        if (dist < GOAL_TOLERANCE) {
            m_path_index++;
            if (m_path_index >= m_path->poses.size()) {
                RCLCPP_INFO(this->get_logger(), "✅ 路径跟踪完成！");
                m_tracking = false;
                m_path = nullptr;
                publishStop();
                return;
            }
            RCLCPP_INFO(this->get_logger(), "📍 到达目标点 %zu，切换到下一个", m_path_index);
            m_rotating = false;
            m_consecutive_stuck = 0;
            return;
        }

        // ===== 防卡死检测 =====
        static double last_dist = dist;
        static rclcpp::Time last_dist_time = this->now();

        if (std::abs(dist - last_dist) < 0.02) {
            auto now = this->now();
            if ((now - last_dist_time).seconds() > 3.0) {
                RCLCPP_WARN(this->get_logger(), "⚠️ 检测到卡死 (距离不变)，强制前进");
                geometry_msgs::msg::Twist force_cmd;
                force_cmd.linear.x = 0.3;
                force_cmd.angular.z = 0.0;
                m_cmd_vel_pub->publish(force_cmd);
                last_dist_time = now;
                return;
            }
        } else {
            last_dist = dist;
            last_dist_time = this->now();
        }

        // ===== 计算左转角度 =====
        double left_turn = angle_err >= 0 ? angle_err : (2.0 * M_PI + angle_err);

        geometry_msgs::msg::Twist cmd;

        // ===== 如果角度偏差 > 20°，原地旋转 =====
        constexpr double ROTATE_THRESHOLD = 20.0 * M_PI / 180.0;

        if (left_turn > ROTATE_THRESHOLD && left_turn < 2.0 * M_PI - ROTATE_THRESHOLD) {
            m_rotating = true;

            double angular_speed;
            if (left_turn > M_PI) {
                angular_speed = std::clamp((2.0 * M_PI - left_turn) * 0.6, 0.3, 0.8);
            } else {
                angular_speed = std::clamp(left_turn * 0.6, 0.3, 0.8);
            }

            cmd.angular.z = angular_speed;
            cmd.linear.x = 0.0;

            static int rotate_log = 0;
            if (++rotate_log % 5 == 0) {
                RCLCPP_INFO(this->get_logger(),
                    "[旋转] 目标 %zu/%zu | 距离 %.2f m | 左转 %.1f° | w=+%.2f",
                    m_path_index + 1, m_path->poses.size(),
                    dist, left_turn * 180.0 / M_PI, cmd.angular.z);
            }
        } else {
            // ===== 前进 =====
            if (m_rotating) {
                RCLCPP_INFO(this->get_logger(), "✅ 对准完成，开始前进 (偏差 %.1f°)",
                            angle_err * 180.0 / M_PI);
                m_rotating = false;
            }

            constexpr double MIN_LINEAR = 0.2;
            constexpr double MAX_LINEAR = 0.8;

            if (dist < 0.5) {
                cmd.linear.x = MIN_LINEAR;
            } else if (dist < 1.0) {
                cmd.linear.x = std::clamp(dist * 0.35, MIN_LINEAR, 0.4);
            } else if (dist < 2.0) {
                cmd.linear.x = std::clamp(dist * 0.3, MIN_LINEAR, 0.6);
            } else {
                cmd.linear.x = std::clamp(dist * 0.25, MIN_LINEAR, MAX_LINEAR);
            }

            constexpr double ANGLE_DEADZONE = 5.0 * M_PI / 180.0;
            if (left_turn > ANGLE_DEADZONE && left_turn < 2.0 * M_PI - ANGLE_DEADZONE) {
                cmd.angular.z = std::clamp(left_turn * 0.25, 0.1, 0.3);
            } else {
                cmd.angular.z = 0.0;
            }
        }

        // ===== 平滑限制 =====
        static double last_angular = 0.0;
        static double last_linear = 0.0;

        double max_angular_change = 0.2;
        double max_linear_change = 0.3;

        if (cmd.angular.z - last_angular > max_angular_change) {
            cmd.angular.z = last_angular + max_angular_change;
        } else if (last_angular - cmd.angular.z > max_angular_change) {
            cmd.angular.z = last_angular - max_angular_change;
        }

        if (cmd.linear.x - last_linear > max_linear_change) {
            cmd.linear.x = last_linear + max_linear_change;
        } else if (last_linear - cmd.linear.x > max_linear_change) {
            cmd.linear.x = last_linear - max_linear_change;
        }

        last_angular = cmd.angular.z;
        last_linear = cmd.linear.x;

        // ===== 发布 =====
        m_cmd_vel_pub->publish(cmd);

        static int log_count = 0;
        if (++log_count % 5 == 0) {
            RCLCPP_INFO(this->get_logger(),
                "目标 %zu/%zu | 距离 %.2f m | 角度 %.1f° | v=%.2f | w=+%.2f",
                m_path_index + 1, m_path->poses.size(),
                dist, angle_err * 180.0 / M_PI,
                cmd.linear.x, cmd.angular.z);
        }
    }

    // ========================================================================
    // 7. 停止
    // ========================================================================
    void publishStop() {
        geometry_msgs::msg::Twist stop;
        m_cmd_vel_pub->publish(stop);
    }

    // ========================================================================
    // 8. 成员变量
    // ========================================================================
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr m_path_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr m_pose_sub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_cmd_vel_pub;
    rclcpp::TimerBase::SharedPtr m_timer;

    nav_msgs::msg::Path::SharedPtr m_path;
    geometry_msgs::msg::PoseStamped::SharedPtr m_pose;
    size_t m_path_index = 0;
    bool m_tracking = false;
    bool m_rotating = false;
    int m_consecutive_stuck = 0;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Nav3DTracker>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
