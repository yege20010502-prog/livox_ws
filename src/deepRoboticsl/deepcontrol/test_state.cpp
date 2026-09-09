#include <iostream>
#include <cstring>
#include <iomanip>
#include <vector>
#include <thread>
#include <chrono>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>

class StateTester {
public:
    StateTester(uint16_t local_port = 43897) {
        // 创建UDP套接字
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
            throw std::runtime_error("绑定端口 " + std::to_string(local_port) + " 失败");
        }

        // 设置非阻塞
        int flags = fcntl(m_sock_fd, F_GETFL, 0);
        fcntl(m_sock_fd, F_SETFL, flags | O_NONBLOCK);

        m_running = true;
        m_recv_thread = std::thread(&StateTester::receiveLoop, this);
        
        std::cout << "[INFO] 状态测试程序启动" << std::endl;
        std::cout << "[INFO] 监听端口: " << local_port << std::endl;
        std::cout << "[INFO] 等待机器狗发送状态数据..." << std::endl;
        std::cout << "========================================" << std::endl;
    }

    ~StateTester() {
        m_running = false;
        if (m_recv_thread.joinable()) {
            m_recv_thread.join();
        }
        close(m_sock_fd);
        std::cout << "\n[INFO] 测试程序已关闭" << std::endl;
    }

private:
    void receiveLoop() {
        std::vector<uint8_t> buffer(4096);
        struct sockaddr_in from_addr;
        socklen_t addr_len = sizeof(from_addr);
        
        int packet_count = 0;
        auto last_print_time = std::chrono::steady_clock::now();

        while (m_running) {
            memset(&from_addr, 0, sizeof(from_addr));
            ssize_t recv_len = recvfrom(m_sock_fd, reinterpret_cast<char*>(buffer.data()),
                                       buffer.size(), 0, 
                                       (struct sockaddr*)&from_addr, &addr_len);

            if (recv_len > 0) {
                packet_count++;
                
                // 解析源地址
                char ip_str[INET_ADDRSTRLEN];
                inet_ntop(AF_INET, &from_addr.sin_addr, ip_str, INET_ADDRSTRLEN);
                uint16_t port = ntohs(from_addr.sin_port);

                // 获取数据包code
                if (recv_len >= 4) {
                    uint32_t code = unpackUint32(buffer.data());
                    
                    // 只处理状态数据 (0x0901)
                    if (code == 0x0901) {
                        std::cout << "\n========================================" << std::endl;
                        std::cout << "收到状态包 #" << packet_count << std::endl;
                        std::cout << "来源: " << ip_str << ":" << port << std::endl;
                        std::cout << "数据长度: " << recv_len << " 字节" << std::endl;
                        std::cout << "Code: 0x" << std::hex << std::setw(8) << std::setfill('0') << code << std::dec << std::endl;
                        
                        // 完整打印数据包内容
                        printHexDump(buffer.data(), recv_len);
                        
                        // 解析各个字段
                        parseAndPrintFields(buffer.data(), recv_len);
                        
                        // 特别关注速度字段
                        findVelocityFields(buffer.data(), recv_len);
                    }
                }
                
                // 每收到10个包打印一次统计
                if (packet_count % 10 == 0) {
                    auto now = std::chrono::steady_clock::now();
                    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - last_print_time).count();
                    if (elapsed >= 1) {
                        std::cout << "\r[统计] 已接收 " << packet_count << " 个数据包，其中状态包 " << packet_count << " 个" << std::flush;
                        last_print_time = now;
                    }
                }
                
            } else {
                if (errno != EAGAIN && errno != EWOULDBLOCK) {
                    // 真正的错误
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }
        std::cout << std::endl;
    }

    void printHexDump(const uint8_t* data, size_t len) {
        std::cout << "\n十六进制数据:" << std::endl;
        for (size_t i = 0; i < len; i++) {
            if (i % 16 == 0) {
                std::cout << std::setw(4) << std::setfill('0') << i << ": ";
            }
            std::cout << std::hex << std::setw(2) << std::setfill('0') 
                      << static_cast<int>(data[i]) << " ";
            if (i % 16 == 15) {
                std::cout << std::endl;
            }
        }
        if (len % 16 != 0) {
            std::cout << std::endl;
        }
        std::cout << std::dec;
    }

    void parseAndPrintFields(const uint8_t* data, size_t len) {
        std::cout << "\n解析字段:" << std::endl;
        size_t offset = 12;  // 跳过包头（code + 参数）
        
        // 1. basic_state - 偏移12 (4字节)
        if (len >= offset + 4) {
            int basic_state = unpackInt32(data + offset);
            std::cout << "  [偏移 " << std::setw(3) << offset << "] basic_state: " << basic_state 
                      << " (" << getStateName(basic_state) << ")" << std::endl;
            offset += 4;
        }
        
        // 2. gait_state - 偏移16
        if (len >= offset + 4) {
            int gait_state = unpackInt32(data + offset);
            std::cout << "  [偏移 " << std::setw(3) << offset << "] gait_state: " << gait_state << std::endl;
            offset += 4;
        }
        
        // 3. policy_state - 偏移20
        if (len >= offset + 4) {
            int policy_state = unpackInt32(data + offset);
            std::cout << "  [偏移 " << std::setw(3) << offset << "] policy_state: " << policy_state << std::endl;
            offset += 4;
        }
        
        // 4-6. rpy, rpy_vel, xyz_acc - 偏移24到96 (各24字节)
        if (len >= offset + 72) {
            double roll = unpackDouble(data + offset);
            double pitch = unpackDouble(data + offset + 8);
            double yaw = unpackDouble(data + offset + 16);
            std::cout << "  [偏移 " << std::setw(3) << offset << "] roll: " << std::fixed << std::setprecision(3) << roll 
                      << ", pitch: " << pitch << ", yaw: " << yaw << std::endl;
            offset += 24;
            
            double roll_vel = unpackDouble(data + offset);
            double pitch_vel = unpackDouble(data + offset + 8);
            double yaw_vel = unpackDouble(data + offset + 16);
            std::cout << "  [偏移 " << std::setw(3) << offset << "] roll_vel: " << roll_vel 
                      << ", pitch_vel: " << pitch_vel << ", yaw_vel: " << yaw_vel << std::endl;
            offset += 24;
            
            double acc_x = unpackDouble(data + offset);
            double acc_y = unpackDouble(data + offset + 8);
            double acc_z = unpackDouble(data + offset + 16);
            std::cout << "  [偏移 " << std::setw(3) << offset << "] acc_x: " << acc_x 
                      << ", acc_y: " << acc_y << ", acc_z: " << acc_z << std::endl;
            offset += 24;
        }
        
        // 7. pos_world - 偏移96
        if (len >= offset + 24) {
            double pos_x = unpackDouble(data + offset);
            double pos_y = unpackDouble(data + offset + 8);
            double pos_z = unpackDouble(data + offset + 16);
            std::cout << "  [偏移 " << std::setw(3) << offset << "] pos_x: " << std::fixed << std::setprecision(3) << pos_x 
                      << ", pos_y: " << pos_y << ", pos_z: " << pos_z << std::endl;
            offset += 24;
        }
        
        // 8. vel_world - 偏移120 (这是速度！)
        if (len >= offset + 24) {
            double vel_x = unpackDouble(data + offset);
            double vel_y = unpackDouble(data + offset + 8);
            double vel_z = unpackDouble(data + offset + 16);
            std::cout << "  [偏移 " << std::setw(3) << offset << "] >>> vel_x: " << std::fixed << std::setprecision(3) << vel_x 
                      << ", vel_y: " << vel_y << ", vel_z: " << vel_z << " <<< (速度)" << std::endl;
            offset += 24;
        }
        
        // 9. pos_relative - 偏移144
        if (len >= offset + 24) {
            double rel_x = unpackDouble(data + offset);
            double rel_y = unpackDouble(data + offset + 8);
            double rel_z = unpackDouble(data + offset + 16);
            std::cout << "  [偏移 " << std::setw(3) << offset << "] rel_x: " << rel_x 
                      << ", rel_y: " << rel_y << ", rel_z: " << rel_z << std::endl;
            offset += 24;
        }
        
        // 10. yaw_so3 - 偏移168
        if (len >= offset + 4) {
            int yaw_so3 = unpackInt32(data + offset);
            std::cout << "  [偏移 " << std::setw(3) << offset << "] yaw_so3: " << yaw_so3 << std::endl;
            offset += 4;
        }
        
        // 11. reserved - 偏移172
        if (len >= offset + 1) {
            uint8_t reserved = data[offset];
            std::cout << "  [偏移 " << std::setw(3) << offset << "] reserved: " << static_cast<int>(reserved) << std::endl;
            offset += 1;
        }
        
        // 12. version - 偏移173
        if (len >= offset + 4) {
            int version = unpackInt32(data + offset);
            std::cout << "  [偏移 " << std::setw(3) << offset << "] version: " << version << std::endl;
            offset += 4;
        }
        
        // 13. motion_state - 偏移177
        if (len >= offset + 4) {
            int motion_state = unpackInt32(data + offset);
            std::cout << "  [偏移 " << std::setw(3) << offset << "] motion_state: " << motion_state << std::endl;
            offset += 4;
        }
        
        // 14. battery - 偏移181
        if (len >= offset + 8) {
            double battery = unpackDouble(data + offset);
            std::cout << "  [偏移 " << std::setw(3) << offset << "] battery: " << std::fixed << std::setprecision(1) << battery << "%" << std::endl;
            offset += 8;
        }
        
        std::cout << std::dec << std::endl;
    }

    void findVelocityFields(const uint8_t* data, size_t len) {
        std::cout << "\n查找速度相关字段:" << std::endl;
        std::cout << "尝试在数据中寻找速度值 (vx, vy, vz)..." << std::endl;
        
        // 跳过包头，从偏移12开始搜索
        size_t start_offset = 12;
        
        // 搜索看起来像速度值的double数据
        for (size_t offset = start_offset; offset <= len - 8; offset += 8) {
            double value = unpackDouble(data + offset);
            
            // 如果值在合理范围内（-5到5 m/s），可能是速度
            if (std::abs(value) > 0.001 && std::abs(value) < 5.0) {
                std::cout << "  偏移 " << std::setw(3) << offset << ": " 
                          << std::fixed << std::setprecision(4) << value;
                
                // 检查是否三个连续的值都像速度
                if (offset + 16 <= len) {
                    double v1 = unpackDouble(data + offset);
                    double v2 = unpackDouble(data + offset + 8);
                    double v3 = unpackDouble(data + offset + 16);
                    if (std::abs(v1) < 5.0 && std::abs(v2) < 5.0 && std::abs(v3) < 5.0) {
                        std::cout << " (可能是速度: vx=" << v1 << ", vy=" << v2 << ", vz=" << v3 << ")";
                    }
                }
                std::cout << std::endl;
            }
        }
        std::cout << std::endl;
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
            default: return "未知";
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

private:
    int m_sock_fd;
    bool m_running;
    std::thread m_recv_thread;
};

int main() {
    try {
        StateTester tester(43897);  // 监听43897端口
        
        // 保持程序运行
        std::cout << "\n按Ctrl+C退出程序..." << std::endl;
        
        while (true) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    } catch (const std::exception& e) {
        std::cerr << "错误: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
