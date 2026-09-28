#include <algorithm>
#include <iostream>
#include <string>
#include <thread>
#include <atomic>
#include <chrono>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <cstring>
#include <iomanip>
#include <vector>
#include <stdexcept>
#include <sstream>
#include <cmath>
#include <memory>
#include <functional>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/string.hpp>

/**
 * @brief 机器人状态结构体
 */
struct RobotState {
    int basic_state = 999;
    int gait_state = 999;
    int motion_state = 999;
    double vel_x = 0.0;
    double vel_y = 0.0;
    double vel_z = 0.0;
    double battery = 0.0;
    double pos_x = 0.0;
    double pos_y = 0.0;
    double yaw = 0.0;
    bool is_moving = false;
    bool state_updated = false;
};

/**
 * @brief 机器人UDP控制客户端
 */
class RobotUDPClient {
public:
    RobotUDPClient(uint16_t local_port = 43897,
                   const std::string& robot_ip = "192.168.1.120",
                   uint16_t ctrl_port = 43893)
        : m_robot_ip(robot_ip),
          m_ctrl_port(ctrl_port),
          m_running(true),
          m_state_updated(false) {
        
        initializeNetwork(local_port);
        
        m_recv_thread = std::thread(&RobotUDPClient::receiveLoop, this);
        m_heartbeat_thread = std::thread(&RobotUDPClient::heartbeatLoop, this);
        m_state_query_thread = std::thread(&RobotUDPClient::stateQueryLoop, this);  // 重新添加
        
        std::cout << "[INFO] UDP客户端初始化完成" << std::endl;
        std::cout << "[INFO] 机器人IP: " << robot_ip << std::endl;
        std::cout << "[INFO] 控制端口: " << ctrl_port << std::endl;
        std::cout << "[INFO] 本地端口: " << local_port << std::endl;
    }
    
    ~RobotUDPClient() {
        m_running = false;
        
        if (m_recv_thread.joinable()) m_recv_thread.join();
        if (m_heartbeat_thread.joinable()) m_heartbeat_thread.join();
        if (m_state_query_thread.joinable()) m_state_query_thread.join();
        
        close(m_sock_fd);
        std::cout << "[INFO] UDP客户端已关闭" << std::endl;
    }
    
    std::string getStateName(int basic) {
        switch(basic) {
            case 1: return "趴下";
            case 4: return "准备起立";
            case 5: return "正在起立";
            case 6: return "站立(力控)";
            case 7: return "正在趴下";
            case 8: return "失控保护";
            case 9: return "姿态调整";
            case 11: return "翻身";
            case 16: return "AI状态";
            case 17: return "回零";
            case 18: return "后空翻";
            case 20: return "打招呼";
            default: return "未知(" + std::to_string(basic) + ")";
        }
    }
    
    void sendMove(float vx, float vy, float yaw) {
        float max_speed = 1.0f;
        vx = std::max(-max_speed, std::min(max_speed, vx));
        vy = std::max(-max_speed, std::min(max_speed, vy));
        yaw = std::max(-max_speed, std::min(max_speed, yaw));
        
        int32_t vx_val = static_cast<int32_t>(vx * 32767);
        int32_t vy_val = static_cast<int32_t>(vy * 32767);
        int32_t yaw_val = static_cast<int32_t>(yaw * 32767);
        
        if (std::abs(vx) > 0.001) {
            sendSimple(0x21010130, vx_val, 0, true);
        } else {
            sendSimple(0x21010130, 0, 0, true);
        }
        
        if (std::abs(vy) > 0.001) {
            sendSimple(0x21010131, vy_val, 0, true);
        } else {
            sendSimple(0x21010131, 0, 0, true);
        }
        
        if (std::abs(yaw) > 0.001) {
            sendSimple(0x21010135, yaw_val, 0, true);
        } else {
            sendSimple(0x21010135, 0, 0, true);
        }
    }
    
    void sendStandUp() {
        sendSimple(0x21010202, 0, 0, false);
        std::cout << "[指令] 起立" << std::endl;
    }
    
    void sendLieDown() {
        sendSimple(0x21010202, 1, 0, false);  // 修正：趴下参数为1
        std::cout << "[指令] 趴下" << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
    
    void sendBalanceStand() {
        sendSimple(0x21010C02, 0, 0, false);
        std::cout << "[指令] 平衡站立" << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
    
    void sendStop() {
        sendMove(0.0f, 0.0f, 0.0f);
        std::cout << "[指令] 停止" << std::endl;
    }
    
    RobotState getCurrentState() {
        std::lock_guard<std::mutex> lock(m_state_mutex);
        return m_current_state;
    }
    
    int getCurrentMode() {
        std::lock_guard<std::mutex> lock(m_state_mutex);
        return m_current_state.basic_state;
    }
    
    std::string getStateString() {
        RobotState state = getCurrentState();
        std::stringstream ss;
        
        if (state.basic_state == 999) {
            return "等待状态数据...";
        }
        
        ss << "状态: " << getStateName(state.basic_state);
        ss << " | 电池: " << std::fixed << std::setprecision(0) << state.battery << "%";
        
        return ss.str();
    }

private:
    std::string m_robot_ip;
    uint16_t m_ctrl_port;
    int m_sock_fd;
    std::atomic<bool> m_running;
    
    std::thread m_recv_thread;
    std::thread m_heartbeat_thread;
    std::thread m_state_query_thread;
    
    RobotState m_current_state;
    std::mutex m_state_mutex;
    bool m_state_updated;
    
    // 用于限制状态处理频率
    std::chrono::steady_clock::time_point m_last_process_time;
    std::mutex m_process_mutex;
    
    bool sendSimple(uint32_t code, int32_t param1 = 0, int32_t param2 = 0, bool silent = false) {
        try {
            uint32_t p1 = (param1 < 0) ? (param1 & 0xFFFFFFFF) : static_cast<uint32_t>(param1);
            uint32_t p2 = (param2 < 0) ? (param2 & 0xFFFFFFFF) : static_cast<uint32_t>(param2);
            
            std::vector<uint8_t> payload(12);
            packUint32(payload.data(), code);
            packUint32(payload.data() + 4, p1);
            packUint32(payload.data() + 8, p2);
            
            sendTo(payload, m_robot_ip, m_ctrl_port);
            return true;
        } catch (const std::exception& e) {
            std::cerr << "[错误] 发送失败: " << e.what() << std::endl;
            return false;
        }
    }
    
    void initializeNetwork(uint16_t local_port) {
        m_sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
        if (m_sock_fd < 0) {
            throw std::runtime_error("创建套接字失败");
        }
        
        if (local_port > 0) {
            struct sockaddr_in local_addr;
            memset(&local_addr, 0, sizeof(local_addr));
            local_addr.sin_family = AF_INET;
            local_addr.sin_addr.s_addr = INADDR_ANY;
            local_addr.sin_port = htons(local_port);
            
            if (bind(m_sock_fd, (struct sockaddr*)&local_addr, sizeof(local_addr)) < 0) {
                close(m_sock_fd);
                throw std::runtime_error("绑定端口失败");
            }
        }
        
        int flags = fcntl(m_sock_fd, F_GETFL, 0);
        if (flags >= 0) {
            fcntl(m_sock_fd, F_SETFL, flags | O_NONBLOCK);
        }
    }
    
    void sendTo(const std::vector<uint8_t>& data, const std::string& ip, uint16_t port) {
        struct sockaddr_in target_addr;
        memset(&target_addr, 0, sizeof(target_addr));
        target_addr.sin_family = AF_INET;
        target_addr.sin_port = htons(port);
        
        if (inet_pton(AF_INET, ip.c_str(), &target_addr.sin_addr) <= 0) {
            throw std::runtime_error("无效的IP地址: " + ip);
        }
        
        ssize_t sent = sendto(m_sock_fd, reinterpret_cast<const char*>(data.data()), 
                             data.size(), 0, (struct sockaddr*)&target_addr, sizeof(target_addr));
        
        if (sent < 0) {
            throw std::runtime_error("发送数据失败");
        }
    }
    
    void receiveLoop() {
        std::vector<uint8_t> buffer(4096);
        struct sockaddr_in from_addr;
        socklen_t addr_len = sizeof(from_addr);
        
        auto last_print_time = std::chrono::steady_clock::now();
        int state_count = 0;
        const int PROCESS_INTERVAL_MS = 100;  // 每100ms处理一次状态（10Hz）
        
        while (m_running) {
            memset(&from_addr, 0, sizeof(from_addr));
            ssize_t recv_len = recvfrom(m_sock_fd, reinterpret_cast<char*>(buffer.data()), 
                                       buffer.size(), 0, (struct sockaddr*)&from_addr, &addr_len);
            
            if (recv_len > 0 && recv_len >= 12) {
                uint32_t code = unpackUint32(buffer.data());
                
                // 只处理状态数据 (0x0901) - 50Hz
                if (code == 0x0901) {
                    state_count++;
                    
                    // 限制处理频率：每100ms只处理最新的状态包
                    auto now = std::chrono::steady_clock::now();
                    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - m_last_process_time).count();
                    
                    if (elapsed >= PROCESS_INTERVAL_MS) {
                        std::lock_guard<std::mutex> lock(m_state_mutex);
                        parseStateData(buffer.data(), recv_len);
                        m_state_updated = true;
                        m_last_process_time = now;
                    }
                    
                    // 每秒打印一次状态（而不是每次更新都打印）
                    auto print_elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - last_print_time).count();
                    if (print_elapsed >= 1) {
                        // 使用std::cout输出，但只在实际有状态更新时打印
                        if (m_state_updated) {
                            // 使用\r实现同一行更新
                            std::cout << "\r" << getStateString() << "    \r" << std::flush;
                            last_print_time = now;
                        }
                    }
                }
            } else {
                if (errno != EAGAIN && errno != EWOULDBLOCK) {
                    // 真正的错误
                }
                // 短暂休眠避免CPU空转
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }
        std::cout << std::endl;
    }
    
    void heartbeatLoop() {
        while (m_running) {
            try {
                sendSimple(0x21040001, 0, 0, true);
            } catch (...) {
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    }
    
    // 重新添加状态查询线程
    void stateQueryLoop() {
        while (m_running) {
            try {
                // 每秒查询一次状态
                sendSimple(0x21020001, 0, 0, true);  // 查询完整状态
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                
                sendSimple(0x21020002, 0, 0, true);  // 查询速度
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                
                sendSimple(0x21020003, 0, 0, true);  // 查询模式
                std::this_thread::sleep_for(std::chrono::milliseconds(1800));  // 总共2秒一个周期
            } catch (...) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            }
        }
    }
    
    void parseStateData(const uint8_t* data, size_t len) {
        size_t offset = 12;
        
        // 1. basic_state - 偏移12
        if (len >= offset + 4) {
            m_current_state.basic_state = static_cast<int>(unpackInt32(data + offset));
            offset += 4;
        }
        
        // 2. gait_state - 偏移16
        if (len >= offset + 4) {
            m_current_state.gait_state = static_cast<int>(unpackInt32(data + offset));
            offset += 4;
        }
        
        // 3. policy_state - 偏移20 (跳过)
        if (len >= offset + 4) {
            offset += 4;
        }
        
        // 4-6. rpy, rpy_vel, xyz_acc (各24字节) - 跳过
        if (len >= offset + 72) {
            offset += 72;
        }
        
        // 7. pos_world[3] - 偏移96
        if (len >= offset + 24) {
            m_current_state.pos_x = unpackDouble(data + offset);
            m_current_state.pos_y = unpackDouble(data + offset + 8);
            m_current_state.yaw = unpackDouble(data + offset + 16);
            offset += 24;
        }
        
        // 8. vel_world[3] - 偏移120
        if (len >= offset + 24) {
            m_current_state.vel_x = unpackDouble(data + offset);
            m_current_state.vel_y = unpackDouble(data + offset + 8);
            m_current_state.vel_z = unpackDouble(data + offset + 16);
            offset += 24;
        }
        
        // 9-12. 跳过其他字段
        if (len >= offset + 24) offset += 24;
        if (len >= offset + 4) offset += 4;
        if (len >= offset + 1) offset += 1;
        if (len >= offset + 4) offset += 4;
        
        // 13. motion_state - 偏移180
        if (len >= offset + 4) {
            m_current_state.motion_state = static_cast<int>(unpackInt32(data + offset));
            offset += 4;
        }
        
        // 14. battery - 偏移180 (double)
        if (len >= 180 + 8) {
            double battery_val = unpackDouble(data + 180);
            if (battery_val >= 0 && battery_val <= 100) {
                m_current_state.battery = battery_val;
            }
        }
        
        m_current_state.is_moving = (std::abs(m_current_state.vel_x) > 0.01 || 
                                     std::abs(m_current_state.vel_y) > 0.01);
        m_current_state.state_updated = true;
        
        // 状态变化时打印（只在变化时输出）
        static int last_basic = -1;
        if (last_basic != m_current_state.basic_state) {
            std::cout << std::endl << "[状态变化] basic=" << m_current_state.basic_state 
                      << " gait=" << m_current_state.gait_state
                      << " motion=" << m_current_state.motion_state 
                      << " battery=" << m_current_state.battery << "%" << std::endl;
            last_basic = m_current_state.basic_state;
        }
    }
    
    void packUint32(uint8_t* buffer, uint32_t value) {
        buffer[0] = (value >> 0) & 0xFF;
        buffer[1] = (value >> 8) & 0xFF;
        buffer[2] = (value >> 16) & 0xFF;
        buffer[3] = (value >> 24) & 0xFF;
    }
    
    uint32_t unpackUint32(const uint8_t* buffer) {
        return (static_cast<uint32_t>(buffer[0]) << 0) |
               (static_cast<uint32_t>(buffer[1]) << 8) |
               (static_cast<uint32_t>(buffer[2]) << 16) |
               (static_cast<uint32_t>(buffer[3]) << 24);
    }
    
    int32_t unpackInt32(const uint8_t* buffer) {
        return static_cast<int32_t>(unpackUint32(buffer));
    }
    
    double unpackDouble(const uint8_t* buffer) {
        uint64_t bits = 0;
        for (int i = 0; i < 8; i++) {
            bits |= static_cast<uint64_t>(buffer[i]) << (i * 8);
        }
        double value;
        memcpy(&value, &bits, sizeof(value));
        return value;
    }
};

/**
 * @brief ROS2节点：机器狗控制桥接节点
 */
class DogControlBridgeNode : public rclcpp::Node {
public:
    explicit DogControlBridgeNode() 
        : Node("dog_control_bridge_node"),
          m_udp_client(43897, "192.168.1.120", 43893) {

        m_cmd_timeout = this->declare_parameter<double>("cmd_timeout", 1.00);
        m_vx_scale = this->declare_parameter<double>("vx_scale", 4.2);
        m_vy_scale = this->declare_parameter<double>("vy_scale", 6.2);
        m_yaw_scale = this->declare_parameter<double>("yaw_scale", 2.5);
        // Conservative limits for the first real-world navigation tests.
        m_max_vx = this->declare_parameter<double>("max_vx", 0.6);
        m_max_vy = this->declare_parameter<double>("max_vy", 0.3);
        m_max_yaw = this->declare_parameter<double>("max_yaw", 0.5);
        if (!std::isfinite(m_vx_scale) || m_vx_scale <= 0.0 ||
            !std::isfinite(m_vy_scale) || m_vy_scale <= 0.0 ||
            !std::isfinite(m_yaw_scale) || m_yaw_scale <= 0.0 ||
            !std::isfinite(m_max_vx) || m_max_vx <= 0.0 || m_max_vx > 1.0 ||
            !std::isfinite(m_max_vy) || m_max_vy <= 0.0 || m_max_vy > 1.0 ||
            !std::isfinite(m_max_yaw) || m_max_yaw <= 0.0 || m_max_yaw > 1.0) {
            throw std::invalid_argument("Speed scales must be positive and limits must be in (0, 1]");
        }
        m_last_cmd_time = std::chrono::steady_clock::now();
        
        m_cmd_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel",
            rclcpp::QoS(10),
            std::bind(&DogControlBridgeNode::cmdVelCallback, this, std::placeholders::_1));

        m_path_sub = this->create_subscription<nav_msgs::msg::Path>(
            "/initial_path",
            rclcpp::QoS(10),
            std::bind(&DogControlBridgeNode::pathCallback, this, std::placeholders::_1));
        
        m_command_sub = this->create_subscription<std_msgs::msg::String>(
            "/dog_control/command", 
            rclcpp::QoS(10),
            std::bind(&DogControlBridgeNode::commandCallback, this, std::placeholders::_1));
        
        m_state_pub = this->create_publisher<std_msgs::msg::String>(
            "/dog_control/state", 
            rclcpp::QoS(10));
        
        // 每2秒发布一次状态到ROS话题
        m_timer = this->create_wall_timer(
            std::chrono::milliseconds(2000),
            std::bind(&DogControlBridgeNode::publishState, this));
        m_watchdog_timer = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&DogControlBridgeNode::watchdogTick, this));
        
        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "云深处机器狗控制桥接节点已启动");
        RCLCPP_INFO(this->get_logger(), "机器人IP: 192.168.1.120");
        RCLCPP_INFO(this->get_logger(), "控制端口: 43893 | 监听端口: 43897");
        RCLCPP_INFO(this->get_logger(), "监听话题: /cmd_vel (速度控制)");
        RCLCPP_INFO(this->get_logger(), "监听话题: /initial_path (趴下时仅提前起立)");
        RCLCPP_INFO(
            this->get_logger(),
            "原始速度直通，角速度反向，指令断流 %.2fs 后停车",
            m_cmd_timeout);
        RCLCPP_INFO(this->get_logger(), "监听话题: /dog_control/command (指令控制)");
        RCLCPP_INFO(this->get_logger(), "发布话题: /dog_control/state (状态信息)");
        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "可用指令: stand, lie, balance, stop, state");
    }

private:
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
        if (!msg || msg->poses.empty()) {
            return;
        }

        const RobotState state = m_udp_client.getCurrentState();
        if (!state.state_updated || state.basic_state != 1) {
            return;
        }

        const auto now = std::chrono::steady_clock::now();
        bool request_stand = false;
        {
            std::lock_guard<std::mutex> lock(m_cmd_mutex);
            // A path is navigation intent, but never a velocity command.  It
            // may only wake the robot from lying down so SCAN can plan using
            // the standing body height.  Walking still requires fresh,
            // non-zero /cmd_vel and all existing watchdog checks.
            if (std::chrono::duration<double>(now - m_last_stand_request).count() > 1.0) {
                m_last_stand_request = now;
                m_waiting_for_stand = true;
                request_stand = true;
            }
        }
        if (request_stand) {
            RCLCPP_INFO(
                this->get_logger(),
                "收到非空导航路径且机器狗趴下，仅提前执行起立；等待速度后再行走");
            m_udp_client.sendStandUp();
        }
    }

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        float vx = static_cast<float>(msg->linear.x * m_vx_scale);
        float vy = static_cast<float>(msg->linear.y * m_vy_scale);
        float yaw = static_cast<float>(msg->angular.z * m_yaw_scale);

        if (!std::isfinite(vx) || !std::isfinite(vy) || !std::isfinite(yaw)) {
            RCLCPP_ERROR(this->get_logger(), "拒绝包含 NaN/Inf 的 /cmd_vel，并立即停车");
            vx = 0.0f;
            vy = 0.0f;
            yaw = 0.0f;
        }

        vx = std::clamp(vx, -static_cast<float>(m_max_vx), static_cast<float>(m_max_vx));
        vy = std::clamp(vy, -static_cast<float>(m_max_vy), static_cast<float>(m_max_vy));
        yaw = -std::clamp(yaw, -static_cast<float>(m_max_yaw), static_cast<float>(m_max_yaw));

        bool has_move = (std::abs(vx) > 0.01 || std::abs(vy) > 0.01 || std::abs(yaw) > 0.01);
        if (!has_move) {
            bool was_active = false;
            {
                std::lock_guard<std::mutex> lock(m_cmd_mutex);
                was_active = m_command_active;
                m_command_active = false;
                m_waiting_for_stand = false;
                m_pending_vx = 0.0f;
                m_pending_vy = 0.0f;
                m_pending_yaw = 0.0f;
                // A zero command is the acknowledgement that the previous
                // trajectory has ended.  Only then may a fault latch clear.
                m_motion_inhibited = false;
            }
            // SCAN may publish zero continuously while idle.  Repeating zero
            // UDP commands would override the hand controller, so stop only
            // when an autonomous command was previously active.
            if (was_active) {
                m_udp_client.sendStop();
                RCLCPP_INFO(this->get_logger(), "导航速度归零，底盘停车并归还遥控器控制");
            }
            return;
        }

        {
            std::lock_guard<std::mutex> lock(m_cmd_mutex);
            if (m_motion_inhibited) {
                return;
            }
            m_last_cmd_time = std::chrono::steady_clock::now();
            m_command_active = true;
            m_pending_vx = vx;
            m_pending_vy = vy;
            m_pending_yaw = yaw;
        }

        processPendingMotion();
    }

    void watchdogTick() {
        bool timed_out = false;
        {
            std::lock_guard<std::mutex> lock(m_cmd_mutex);
            const double elapsed = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - m_last_cmd_time).count();
            if (m_command_active && elapsed > m_cmd_timeout) {
                m_command_active = false;
                m_waiting_for_stand = false;
                m_pending_vx = 0.0f;
                m_pending_vy = 0.0f;
                m_pending_yaw = 0.0f;
                timed_out = true;
            }
        }
        if (timed_out) {
            RCLCPP_ERROR(this->get_logger(), "/cmd_vel 超时，强制停车");
            m_udp_client.sendStop();
            return;
        }

        processPendingMotion();
    }

    void processPendingMotion() {
        float vx = 0.0f;
        float vy = 0.0f;
        float yaw = 0.0f;
        {
            std::lock_guard<std::mutex> lock(m_cmd_mutex);
            if (!m_command_active) {
                return;
            }
            vx = m_pending_vx;
            vy = m_pending_vy;
            yaw = m_pending_yaw;
        }

        const RobotState state = m_udp_client.getCurrentState();
        if (!state.state_updated || state.basic_state == 999) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "收到导航速度，但尚未取得底盘状态；暂不下发运动");
            return;
        }

        if (state.basic_state == 6) {
            {
                std::lock_guard<std::mutex> lock(m_cmd_mutex);
                if (m_waiting_for_stand) {
                    RCLCPP_INFO(this->get_logger(), "检测到机器狗已站立，开始执行导航速度");
                }
                m_waiting_for_stand = false;
            }
            m_udp_client.sendMove(vx, vy, yaw);
            return;
        }

        if (state.basic_state == 1) {
            const auto now = std::chrono::steady_clock::now();
            bool request_stand = false;
            {
                std::lock_guard<std::mutex> lock(m_cmd_mutex);
                if (!m_waiting_for_stand ||
                    std::chrono::duration<double>(now - m_last_stand_request).count() > 1.0) {
                    m_waiting_for_stand = true;
                    m_last_stand_request = now;
                    request_stand = true;
                }
            }
            if (request_stand) {
                RCLCPP_INFO(this->get_logger(), "检测到导航需要运动且机器狗趴下，先执行起立");
                m_udp_client.sendStandUp();
            }
            return;
        }

        // States 4/5 are standing transitions.  Do not send velocity and do
        // not switch gait/mode; state 6 is sufficient for walking.
        if (state.basic_state == 4 || state.basic_state == 5 || state.basic_state == 24) {
            std::lock_guard<std::mutex> lock(m_cmd_mutex);
            m_waiting_for_stand = true;
            return;
        }

        bool newly_inhibited = false;
        {
            std::lock_guard<std::mutex> lock(m_cmd_mutex);
            newly_inhibited = !m_motion_inhibited;
            m_motion_inhibited = true;
            m_command_active = false;
            m_waiting_for_stand = false;
            m_pending_vx = 0.0f;
            m_pending_vy = 0.0f;
            m_pending_yaw = 0.0f;
        }
        if (newly_inhibited) {
            m_udp_client.sendStop();
            RCLCPP_ERROR(
                this->get_logger(),
                "底盘异常状态 basic=%d：停车并锁止旧导航速度，收到零速后才允许新任务",
                state.basic_state);
        }
    }
    
    void commandCallback(const std_msgs::msg::String::SharedPtr msg) {
        std::string cmd = msg->data;
        
        if (cmd == "stand" || cmd == "standup") {
            RCLCPP_INFO(this->get_logger(), "执行: 起立");
            m_udp_client.sendStandUp();
        } 
        else if (cmd == "lie" || cmd == "liedown") {
            RCLCPP_INFO(this->get_logger(), "执行: 趴下");
            cancelAutonomousMotion();
            m_udp_client.sendLieDown();
        }
        else if (cmd == "balance" || cmd == "balancestand") {
            RCLCPP_INFO(this->get_logger(), "执行: 平衡站立");
            m_udp_client.sendBalanceStand();
        }
        else if (cmd == "stop") {
            RCLCPP_INFO(this->get_logger(), "执行: 停止");
            cancelAutonomousMotion();
            m_udp_client.sendStop();
        }
        else if (cmd == "state" || cmd == "status") {
            RCLCPP_INFO(this->get_logger(), "执行: 查询状态");
            publishState();
        }
        else {
            RCLCPP_WARN(this->get_logger(), "未知指令: %s", cmd.c_str());
        }
    }
    
    void publishState() {
        auto msg = std_msgs::msg::String();
        msg.data = m_udp_client.getStateString();
        m_state_pub->publish(msg);
    }

    void cancelAutonomousMotion() {
        std::lock_guard<std::mutex> lock(m_cmd_mutex);
        m_command_active = false;
        m_waiting_for_stand = false;
        m_pending_vx = 0.0f;
        m_pending_vy = 0.0f;
        m_pending_yaw = 0.0f;
    }
    
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr m_cmd_vel_sub;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr m_path_sub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_command_sub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_state_pub;
    rclcpp::TimerBase::SharedPtr m_timer;
    rclcpp::TimerBase::SharedPtr m_watchdog_timer;

    bool m_command_active{false};
    bool m_waiting_for_stand{false};
    bool m_motion_inhibited{false};
    float m_pending_vx{0.0f};
    float m_pending_vy{0.0f};
    float m_pending_yaw{0.0f};
    double m_cmd_timeout{1.00};
    double m_vx_scale{4.2};
    double m_vy_scale{6.2};
    double m_yaw_scale{2.5};
    double m_max_vx{0.6};
    double m_max_vy{0.3};
    double m_max_yaw{0.5};
    std::chrono::steady_clock::time_point m_last_cmd_time;
    std::chrono::steady_clock::time_point m_last_stand_request{};
    std::mutex m_cmd_mutex;
    
    RobotUDPClient m_udp_client;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<DogControlBridgeNode>();
    
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}
