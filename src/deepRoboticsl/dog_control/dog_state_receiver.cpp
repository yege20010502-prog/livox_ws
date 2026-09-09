#include <iostream>
#include <string>
#include <thread>
#include <atomic>
#include <chrono>
#include <cstring>
#include <iomanip>
#include <vector>
#include <mutex>          // 添加 mutex 头文件
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>

/**
 * @brief 机器人状态结构体（对应协议中的 RobotStateUpload）
 */
struct RobotStateUpload {
    int robot_basic_state;        // 机器人基本运动状态
    int robot_gait_state;         // 机器人当前步态
    int robot_policy_state;       // 机器人当前AI步态
    double rpy[3];                // IMU角度信息
    double rpy_vel[3];            // IMU角速度信息
    double xyz_acc[3];            // IMU加速度信息
    double pos_world[3];          // 世界坐标系位姿 [x, y, yaw]
    double vel_world[3];          // 世界坐标系速度 [x_vel, y_vel, yaw_vel]
    double vel_body[3];           // 身体坐标系速度
    unsigned touch_down_and_stair_trot;
    bool is_charging;
    unsigned error_state;
    int robot_motion_state;       // 机器人动作状态
    double battery_level;         // 电池电量
    int task_state;
    bool is_robot_need_move;
    bool zero_position_flag;
    bool is_after_first_start;
    bool is_voice_ctrl_enable;
    double ultrasound[2];
};

/**
 * @brief 机器人状态接收器
 */
class RobotStateReceiver {
public:
    RobotStateReceiver(uint16_t local_port = 43897) {
        m_running = true;
        m_state_updated = false;
        
        // 初始化状态
        memset(&m_current_state, 0, sizeof(m_current_state));
        
        // 初始化socket
        initSocket(local_port);
        
        // 启动接收线程
        m_recv_thread = std::thread(&RobotStateReceiver::receiveLoop, this);
        
        std::cout << "[INFO] 状态接收器已启动" << std::endl;
        std::cout << "[INFO] 本地端口: " << local_port << std::endl;
        std::cout << "[INFO] 等待接收机器人状态数据(指令码0x0901)..." << std::endl;
        std::cout << std::endl;
    }
    
    ~RobotStateReceiver() {
        m_running = false;
        if (m_recv_thread.joinable()) {
            m_recv_thread.join();
        }
        close(m_sock_fd);
        std::cout << "[INFO] 状态接收器已关闭" << std::endl;
    }
    
    /**
     * @brief 获取状态名称
     */
    std::string getStateName(int basic, int gait, int motion) {
        if (basic == 1 && gait == 0 && motion == 0) return "趴下";
        if (basic == 1 && gait == 0 && motion == 11) return "向前跳";
        if (basic == 4 && gait == 0 && motion == 0) return "准备起立";
        if (basic == 5 && gait == 0 && motion == 0) return "正在起立";
        if (basic == 7 && gait == 0 && motion == 0) return "正在趴下";
        if (basic == 8 && gait == 0 && motion == 0) return "失控保护";
        if (basic == 9 && gait == 0 && motion == 0) return "姿态调整";
        if (basic == 11 && gait == 0 && motion == 0) return "翻身";
        if (basic == 17 && gait == 0 && motion == 0) return "回零";
        if (basic == 18 && gait == 0 && motion == 0) return "后空翻";
        if (basic == 20 && gait == 0 && motion == 0) return "打招呼";
        
        if (basic == 6) {
            if (motion == 0) {
                switch(gait) {
                    case 0: return "力控站立(平地低速)";
                    case 2: return "力控站立(通用越障)";
                    case 4: return "力控站立(平地中速)";
                    case 5: return "力控站立(平地高速)";
                    case 6: return "力控站立(抓地越障)";
                    case 13: return "力控站立(高踏步越障)";
                    default: return "力控站立(步态" + std::to_string(gait) + ")";
                }
            } else if (motion == 1) {
                switch(gait) {
                    case 0: return "踏步(平地低速)";
                    case 2: return "踏步(通用越障)";
                    case 4: return "踏步(平地中速)";
                    case 5: return "踏步(平地高速)";
                    case 6: return "踏步(抓地越障)";
                    case 13: return "踏步(高踏步越障)";
                    default: return "踏步(步态" + std::to_string(gait) + ")";
                }
            } else if (motion == 2) return "扭身体";
            else if (motion == 4) return "扭身跳";
            else if (motion == 12 && gait == 12) return "太空步";
        }
        
        if (basic == 16) {
            if (gait == 0) return "AI基础步态";
            if (gait == 16) return "AI跳跃步态";
            if (gait == 18) return "AI站立步态";
            if (gait == 20) return "AI极速步态";
            return "AI状态(步态" + std::to_string(gait) + ")";
        }
        
        return "未知(" + std::to_string(basic) + "," + std::to_string(gait) + "," + std::to_string(motion) + ")";
    }
    
    /**
     * @brief 打印状态
     */
    void printState() {
        std::lock_guard<std::mutex> lock(m_state_mutex);
        
        if (!m_state_updated) {
            std::cout << "[状态] 尚未收到状态数据" << std::endl;
            return;
        }
        
        std::cout << "========== 机器人状态 ==========" << std::endl;
        std::cout << "基本状态: " << getStateName(m_current_state.robot_basic_state, 
                                                   m_current_state.robot_gait_state, 
                                                   m_current_state.robot_motion_state) << std::endl;
        std::cout << "  - basic_state: " << m_current_state.robot_basic_state << std::endl;
        std::cout << "  - gait_state: " << m_current_state.robot_gait_state << std::endl;
        std::cout << "  - policy_state: " << m_current_state.robot_policy_state << std::endl;
        std::cout << "  - motion_state: " << m_current_state.robot_motion_state << std::endl;
        
        std::cout << "位置: (" << std::fixed << std::setprecision(2)
                  << m_current_state.pos_world[0] << ", " 
                  << m_current_state.pos_world[1] << ", " 
                  << m_current_state.pos_world[2] << ")" << std::endl;
        
        std::cout << "速度(世界): (" << std::fixed << std::setprecision(2)
                  << m_current_state.vel_world[0] << ", " 
                  << m_current_state.vel_world[1] << ", " 
                  << m_current_state.vel_world[2] << ")" << std::endl;
        
        std::cout << "速度(身体): (" << std::fixed << std::setprecision(2)
                  << m_current_state.vel_body[0] << ", " 
                  << m_current_state.vel_body[1] << ", " 
                  << m_current_state.vel_body[2] << ")" << std::endl;
        
        std::cout << "电池: " << std::fixed << std::setprecision(1) 
                  << (m_current_state.battery_level * 100) << "%" << std::endl;
        
        // 判断是否移动
        bool is_moving = (std::abs(m_current_state.vel_world[0]) > 0.01 || 
                          std::abs(m_current_state.vel_world[1]) > 0.01);
        std::cout << "是否移动: " << (is_moving ? "是" : "否") << std::endl;
        std::cout << "==================================" << std::endl;
        std::cout << std::endl;
    }
    
    /**
     * @brief 检查是否收到状态
     */
    bool hasState() {
        std::lock_guard<std::mutex> lock(m_state_mutex);
        return m_state_updated;
    }

private:
    int m_sock_fd;
    std::atomic<bool> m_running;
    std::thread m_recv_thread;
    
    RobotStateUpload m_current_state;
    std::mutex m_state_mutex;
    bool m_state_updated;
    
    void initSocket(uint16_t local_port) {
        m_sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
        if (m_sock_fd < 0) {
            throw std::runtime_error("创建套接字失败");
        }
        
        // 绑定本地端口
        struct sockaddr_in local_addr;
        memset(&local_addr, 0, sizeof(local_addr));
        local_addr.sin_family = AF_INET;
        local_addr.sin_addr.s_addr = INADDR_ANY;
        local_addr.sin_port = htons(local_port);
        
        if (bind(m_sock_fd, (struct sockaddr*)&local_addr, sizeof(local_addr)) < 0) {
            close(m_sock_fd);
            throw std::runtime_error("绑定端口失败，请检查端口是否被占用");
        }
        
        // 设置非阻塞
        int flags = fcntl(m_sock_fd, F_GETFL, 0);
        if (flags >= 0) {
            fcntl(m_sock_fd, F_SETFL, flags | O_NONBLOCK);
        }
    }
    
    void receiveLoop() {
        std::vector<uint8_t> buffer(4096);
        struct sockaddr_in from_addr;
        socklen_t addr_len = sizeof(from_addr);
        int recv_count = 0;
        
        while (m_running) {
            memset(&from_addr, 0, sizeof(from_addr));
            ssize_t recv_len = recvfrom(m_sock_fd, reinterpret_cast<char*>(buffer.data()), 
                                       buffer.size(), 0, (struct sockaddr*)&from_addr, &addr_len);
            
            if (recv_len > 0) {
                recv_count++;
                char ip_str[INET_ADDRSTRLEN];
                inet_ntop(AF_INET, &from_addr.sin_addr, ip_str, INET_ADDRSTRLEN);
                uint16_t port = ntohs(from_addr.sin_port);
                
                // 解析数据包
                if (recv_len >= 12) {
                    uint32_t code = unpackUint32(buffer.data());
                    
                    // 显示收到的数据包信息
                    std::cout << "[收到] 数据包 #" << recv_count 
                              << " | code=0x" << std::hex << code << std::dec
                              << " | size=" << recv_len 
                              << " | from=" << ip_str << ":" << port << std::endl;
                    
                    // 检查是否是状态数据 (0x0901)
                    if (code == 0x0901) {
                        std::lock_guard<std::mutex> lock(m_state_mutex);
                        parseStateData(buffer.data(), recv_len);
                        m_state_updated = true;
                        std::cout << "[成功] 解析到状态数据!" << std::endl;
                        printState();
                    }
                }
            } else {
                if (errno != EAGAIN && errno != EWOULDBLOCK) {
                    // 真正的错误
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }
    }
    
    void parseStateData(const uint8_t* data, size_t len) {
        size_t offset = 12;  // 跳过头部
        
        // 1. robot_basic_state
        if (len >= offset + 4) {
            m_current_state.robot_basic_state = unpackInt32(data + offset);
            offset += 4;
        }
        
        // 2. robot_gait_state
        if (len >= offset + 4) {
            m_current_state.robot_gait_state = unpackInt32(data + offset);
            offset += 4;
        }
        
        // 3. robot_policy_state
        if (len >= offset + 4) {
            m_current_state.robot_policy_state = unpackInt32(data + offset);
            offset += 4;
        }
        
        // 4. rpy[3]
        if (len >= offset + 24) {
            m_current_state.rpy[0] = unpackDouble(data + offset);
            m_current_state.rpy[1] = unpackDouble(data + offset + 8);
            m_current_state.rpy[2] = unpackDouble(data + offset + 16);
            offset += 24;
        }
        
        // 5. rpy_vel[3]
        if (len >= offset + 24) {
            m_current_state.rpy_vel[0] = unpackDouble(data + offset);
            m_current_state.rpy_vel[1] = unpackDouble(data + offset + 8);
            m_current_state.rpy_vel[2] = unpackDouble(data + offset + 16);
            offset += 24;
        }
        
        // 6. xyz_acc[3]
        if (len >= offset + 24) {
            m_current_state.xyz_acc[0] = unpackDouble(data + offset);
            m_current_state.xyz_acc[1] = unpackDouble(data + offset + 8);
            m_current_state.xyz_acc[2] = unpackDouble(data + offset + 16);
            offset += 24;
        }
        
        // 7. pos_world[3]
        if (len >= offset + 24) {
            m_current_state.pos_world[0] = unpackDouble(data + offset);
            m_current_state.pos_world[1] = unpackDouble(data + offset + 8);
            m_current_state.pos_world[2] = unpackDouble(data + offset + 16);
            offset += 24;
        }
        
        // 8. vel_world[3]
        if (len >= offset + 24) {
            m_current_state.vel_world[0] = unpackDouble(data + offset);
            m_current_state.vel_world[1] = unpackDouble(data + offset + 8);
            m_current_state.vel_world[2] = unpackDouble(data + offset + 16);
            offset += 24;
        }
        
        // 9. vel_body[3]
        if (len >= offset + 24) {
            m_current_state.vel_body[0] = unpackDouble(data + offset);
            m_current_state.vel_body[1] = unpackDouble(data + offset + 8);
            m_current_state.vel_body[2] = unpackDouble(data + offset + 16);
            offset += 24;
        }
        
        // 10. touch_down_and_stair_trot
        if (len >= offset + 4) {
            m_current_state.touch_down_and_stair_trot = unpackUint32(data + offset);
            offset += 4;
        }
        
        // 11. is_charging
        if (len >= offset + 1) {
            m_current_state.is_charging = (data[offset] != 0);
            offset += 1;
        }
        
        // 12. error_state
        if (len >= offset + 4) {
            m_current_state.error_state = unpackUint32(data + offset);
            offset += 4;
        }
        
        // 13. robot_motion_state
        if (len >= offset + 4) {
            m_current_state.robot_motion_state = unpackInt32(data + offset);
            offset += 4;
        }
        
        // 14. battery_level
        if (len >= offset + 8) {
            m_current_state.battery_level = unpackDouble(data + offset);
            offset += 8;
        }
        
        // 15. task_state
        if (len >= offset + 4) {
            m_current_state.task_state = unpackInt32(data + offset);
            offset += 4;
        }
        
        // 16. is_robot_need_move
        if (len >= offset + 1) {
            m_current_state.is_robot_need_move = (data[offset] != 0);
            offset += 1;
        }
        
        // 17. zero_position_flag
        if (len >= offset + 1) {
            m_current_state.zero_position_flag = (data[offset] != 0);
            offset += 1;
        }
        
        // 18. is_after_first_start
        if (len >= offset + 1) {
            m_current_state.is_after_first_start = (data[offset] != 0);
            offset += 1;
        }
        
        // 19. is_voice_ctrl_enable
        if (len >= offset + 1) {
            m_current_state.is_voice_ctrl_enable = (data[offset] != 0);
            offset += 1;
        }
        
        // 20. ultrasound[2]
        if (len >= offset + 16) {
            m_current_state.ultrasound[0] = unpackDouble(data + offset);
            m_current_state.ultrasound[1] = unpackDouble(data + offset + 8);
        }
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
 * @brief 主函数
 */
int main(int argc, char** argv) {
    std::cout << "========================================" << std::endl;
    std::cout << "云深处机器狗状态接收器" << std::endl;
    std::cout << "========================================" << std::endl;
    
    uint16_t local_port = 43897;
    
    // 解析命令行参数
    if (argc > 1) {
        local_port = static_cast<uint16_t>(std::stoi(argv[1]));
    }
    
    std::cout << "本地端口: " << local_port << std::endl;
    std::cout << "机器人IP: 192.168.1.120 (监听所有来源)" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << std::endl;
    std::cout << "提示: 请确保网线已连接，机器人已开机" << std::endl;
    std::cout << "提示: 机器人会主动发送状态数据到端口 " << local_port << std::endl;
    std::cout << "提示: 按 Ctrl+C 退出程序" << std::endl;
    std::cout << std::endl;
    
    try {
        RobotStateReceiver receiver(local_port);
        
        // 等待接收状态
        int wait_count = 0;
        while (true) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            wait_count++;
            
            if (receiver.hasState()) {
                std::cout << "[状态] 已成功接收机器人状态!" << std::endl;
                // 持续显示状态
                while (true) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
                    receiver.printState();
                }
            } else {
                if (wait_count % 5 == 0) {
                    std::cout << "[等待] 已等待 " << wait_count << " 秒，尚未收到状态数据..." << std::endl;
                    std::cout << "[提示] 请检查:" << std::endl;
                    std::cout << "  1. 网线是否正确连接" << std::endl;
                    std::cout << "  2. 机器人是否已开机" << std::endl;
                    std::cout << "  3. 本地端口 " << local_port << " 是否被占用" << std::endl;
                    std::cout << "  4. 机器人是否配置了正确的目标端口" << std::endl;
                    std::cout << std::endl;
                }
            }
        }
        
    } catch (const std::exception& e) {
        std::cerr << "[错误] " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
