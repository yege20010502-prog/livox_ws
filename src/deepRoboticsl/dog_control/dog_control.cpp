#include <iostream>
#include <string>
#include <thread>
#include <atomic>
#include <chrono>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <map>
#include <functional>
#include <cstring>
#include <iomanip>
#include <vector>
#include <stdexcept>
#include <sstream>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib")
#else
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#endif

/**
 * @brief 机器人控制指令结构体
 * 
 * 定义UDP通信的数据包结构，包含指令码和两个参数
 */
struct RobotCommand {
    uint32_t code;      ///< 指令码
    uint32_t param1;    ///< 参数1
    uint32_t param2;    ///< 参数2
};

/**
 * @brief 接收数据包结构体
 */
struct ReceivedPacket {
    std::vector<uint8_t> data;  ///< 接收到的数据
    std::string address;        ///< 发送方地址
    uint16_t port;              ///< 发送方端口
};

/**
 * @brief 机器人控制器类
 * 
 * 负责与机器人进行UDP通信，发送控制指令，接收状态信息
 */
class RobotCommander {
public:
    /**
     * @brief 构造函数
     * @param local_port 本地端口
     */
    RobotCommander(uint16_t local_port = 20001)
        : m_ctrl_addr("192.168.1.120", 43893),          // 固定控制主机IP和端口
          m_perception_addr("192.168.1.120", 43899),    // 固定感知主机IP和端口
          m_running(true),
          m_step_in_place_state(false),
          m_expecting_speaker_status(false) {
        
        // 初始化网络
        initializeNetwork(local_port);
        
        // 启动接收线程
        m_recv_thread = std::thread(&RobotCommander::receiveLoop, this);
        
        // 启动心跳线程
        m_heartbeat_thread = std::thread(&RobotCommander::heartbeatLoop, this);
        
        std::cout << "[INFO] 机器人控制器初始化完成" << std::endl;
        std::cout << "[INFO] 机器人IP: 192.168.1.120" << std::endl;
        std::cout << "[INFO] 控制端口: 43893" << std::endl;
        std::cout << "[INFO] 感知端口: 43899" << std::endl;
        std::cout << "[INFO] 本地端口: " << local_port << std::endl;
        std::cout << std::endl;
        std::cout << "[提示] 请确保电脑IP在 192.168.1.x 网段" << std::endl;
        std::cout << "[提示] 建议设置电脑IP为 192.168.1.100" << std::endl;
    }
    
    /**
     * @brief 析构函数
     */
    ~RobotCommander() {
        m_running = false;
        
        // 唤醒接收线程
        m_queue_cv.notify_all();
        
        // 等待线程结束
        if (m_recv_thread.joinable()) {
            m_recv_thread.join();
        }
        if (m_heartbeat_thread.joinable()) {
            m_heartbeat_thread.join();
        }
        
        // 关闭套接字
#ifdef _WIN32
        closesocket(m_sock_fd);
        WSACleanup();
#else
        close(m_sock_fd);
#endif
        
        std::cout << "[INFO] 机器人控制器已关闭" << std::endl;
    }
    
    /**
     * @brief 发送简单指令
     * @param code 指令码
     * @param param1 参数1
     * @param param2 参数2
     * @param silent 是否静默发送（不打印日志）
     * @return 是否发送成功
     */
    bool sendSimple(uint32_t code, int32_t param1 = 0, int32_t param2 = 0, bool silent = false) {
        try {
            // 处理负值
            uint32_t p1 = (param1 < 0) ? (param1 & 0xFFFFFFFF) : static_cast<uint32_t>(param1);
            uint32_t p2 = (param2 < 0) ? (param2 & 0xFFFFFFFF) : static_cast<uint32_t>(param2);
            
            // 打包数据
            std::vector<uint8_t> payload(12);
            packUint32(payload.data(), code);
            packUint32(payload.data() + 4, p1);
            packUint32(payload.data() + 8, p2);
            
            // 发送数据
            sendTo(payload, m_ctrl_addr);
            
            // 只在非静默模式下打印日志
            if (!silent) {
                std::cout << "[发送] 0x" << std::hex << std::setw(8) << std::setfill('0') 
                         << code << std::dec << " p1=" << param1 << " p2=" << param2 << std::endl;
            }
            return true;
        } catch (const std::exception& e) {
            std::cerr << "[错误] 发送失败: " << e.what() << std::endl;
            return false;
        }
    }
    
    /**
     * @brief 发送复杂指令（带自定义数据）
     * @param code 指令码
     * @param data 自定义数据
     * @param silent 是否静默发送
     * @return 是否发送成功
     */
    bool sendComplex(uint32_t code, const std::vector<uint8_t>& data, bool silent = false) {
        try {
            uint32_t param_size = static_cast<uint32_t>(data.size());
            
            // 构建数据包：头部(12字节) + 数据
            std::vector<uint8_t> payload(12 + data.size());
            packUint32(payload.data(), code);
            packUint32(payload.data() + 4, param_size);
            packUint32(payload.data() + 8, 1);  // 复杂指令标志
            
            // 复制数据
            if (!data.empty()) {
                memcpy(payload.data() + 12, data.data(), data.size());
            }
            
            // 发送数据
            sendTo(payload, m_ctrl_addr);
            
            if (!silent) {
                std::cout << "[发送复杂指令] 0x" << std::hex << std::setw(8) << std::setfill('0') 
                         << code << std::dec << " 数据长度=" << param_size << std::endl;
            }
            return true;
        } catch (const std::exception& e) {
            std::cerr << "[错误] 复杂指令发送失败: " << e.what() << std::endl;
            return false;
        }
    }
    
    /**
     * @brief 发送到感知主机
     * @param code 指令码
     * @param param1 参数1
     * @param param2 参数2
     * @return 是否发送成功
     */
    bool sendToPerception(uint32_t code, int32_t param1 = 0, int32_t param2 = 0) {
        try {
            uint32_t p1 = (param1 < 0) ? (param1 & 0xFFFFFFFF) : static_cast<uint32_t>(param1);
            uint32_t p2 = (param2 < 0) ? (param2 & 0xFFFFFFFF) : static_cast<uint32_t>(param2);
            
            std::vector<uint8_t> payload(12);
            packUint32(payload.data(), code);
            packUint32(payload.data() + 4, p1);
            packUint32(payload.data() + 8, p2);
            
            sendTo(payload, m_perception_addr);
            
            std::cout << "[发送到感知] 0x" << std::hex << std::setw(8) << std::setfill('0') 
                     << code << std::dec << " p1=" << param1 << " p2=" << param2 << std::endl;
            return true;
        } catch (const std::exception& e) {
            std::cerr << "[错误] 感知发送失败: " << e.what() << std::endl;
            return false;
        }
    }
    
    /**
     * @brief 查询扬声器状态
     */
    void querySpeakerStatus() {
        m_expecting_speaker_status = true;
        sendSimple(0x2101030D, 2);
    }
    
    /**
     * @brief 切换原地踏步状态
     */
    void toggleStepInPlace() {
        m_step_in_place_state = !m_step_in_place_state;
        int32_t val = m_step_in_place_state ? -1 : 2;
        sendSimple(0x21010C06, val);
        std::cout << "[原地踏步] " << (m_step_in_place_state ? "开启" : "关闭") << std::endl;
    }
    
    /**
     * @brief 发送速度指令（自主模式）
     * @param linear_x X方向线速度 (m/s)
     * @param linear_y Y方向线速度 (m/s)
     * @param angular 角速度 (rad/s)
     */
    void sendVelocity(double linear_x = 0.0, double linear_y = 0.0, double angular = 0.0) {
        // 发送线速度X
        std::vector<uint8_t> data_x(8);
        packDouble(data_x.data(), linear_x);
        sendComplex(0x0140, data_x, true);  // 静默发送
        
        // 发送线速度Y
        std::vector<uint8_t> data_y(8);
        packDouble(data_y.data(), linear_y);
        sendComplex(0x0145, data_y, true);  // 静默发送
        
        // 发送角速度
        std::vector<uint8_t> data_a(8);
        packDouble(data_a.data(), angular);
        sendComplex(0x0141, data_a, true);  // 静默发送
        
        std::cout << "[速度设置] vx=" << linear_x << " vy=" << linear_y << " va=" << angular << std::endl;
    }
    
    /**
     * @brief 进入交互模式
     * 
     * 提供命令行交互接口
     */
    void interactiveMode() {
        std::cout << "\n========== 机器人控制台 ==========" << std::endl;
        std::cout << "输入 'help' 查看命令列表" << std::endl;
        std::cout << "输入 'quit' 退出程序" << std::endl;
        std::cout << "===================================" << std::endl;
        
        std::string line;
        while (m_running && std::getline(std::cin, line)) {
            if (line.empty()) continue;
            
            // 分割命令和参数
            std::vector<std::string> tokens;
            splitString(line, tokens, ' ');
            
            if (tokens.empty()) continue;
            
            if (tokens[0] == "quit" || tokens[0] == "q") {
                break;
            } else if (tokens[0] == "help" || tokens[0] == "h") {
                printHelp();
            } else if (tokens[0] == "heartbeat" || tokens[0] == "hb") {
                sendSimple(0x21040001);
            } else if (tokens[0] == "stand" || tokens[0] == "st") {
                sendSimple(0x21010202);  // 起立/趴下
            } else if (tokens[0] == "step" || tokens[0] == "sp") {
                toggleStepInPlace();
            } else if (tokens[0] == "manual" || tokens[0] == "man") {
                sendSimple(0x21010C02);  // 手动模式
            } else if (tokens[0] == "auto" || tokens[0] == "aut") {
                sendSimple(0x21010C03);  // 自主模式
            } else if (tokens[0] == "stop" || tokens[0] == "stp") {
                sendSimple(0x21010C0E);  // 软急停
            } else if (tokens[0] == "home") {
                sendSimple(0x21010C05);  // 回零
            } else if (tokens[0] == "forward" || tokens[0] == "fw") {
                if (tokens.size() > 1) {
                    double speed = std::stod(tokens[1]);
                    if (speed < -1.0) speed = -1.0;
                    if (speed > 1.0) speed = 1.0;
                    sendSimple(0x21010130, static_cast<int32_t>(speed * 32767));
                } else {
                    sendSimple(0x21010130, 32767);  // 前进
                }
            } else if (tokens[0] == "backward" || tokens[0] == "bw") {
                if (tokens.size() > 1) {
                    double speed = std::stod(tokens[1]);
                    if (speed < -1.0) speed = -1.0;
                    if (speed > 1.0) speed = 1.0;
                    sendSimple(0x21010130, static_cast<int32_t>(-speed * 32767));
                } else {
                    sendSimple(0x21010130, -32767);  // 后退
                }
            } else if (tokens[0] == "left") {
                if (tokens.size() > 1) {
                    double speed = std::stod(tokens[1]);
                    if (speed < -1.0) speed = -1.0;
                    if (speed > 1.0) speed = 1.0;
                    sendSimple(0x21010131, static_cast<int32_t>(-speed * 32767));
                } else {
                    sendSimple(0x21010131, -32767);  // 左移
                }
            } else if (tokens[0] == "right") {
                if (tokens.size() > 1) {
                    double speed = std::stod(tokens[1]);
                    if (speed < -1.0) speed = -1.0;
                    if (speed > 1.0) speed = 1.0;
                    sendSimple(0x21010131, static_cast<int32_t>(speed * 32767));
                } else {
                    sendSimple(0x21010131, 32767);  // 右移
                }
            } else if (tokens[0] == "turnleft" || tokens[0] == "tl") {
                if (tokens.size() > 1) {
                    double speed = std::stod(tokens[1]);
                    if (speed < -1.0) speed = -1.0;
                    if (speed > 1.0) speed = 1.0;
                    sendSimple(0x21010135, static_cast<int32_t>(-speed * 32767));
                } else {
                    sendSimple(0x21010135, -32767);  // 左转
                }
            } else if (tokens[0] == "turnright" || tokens[0] == "tr") {
                if (tokens.size() > 1) {
                    double speed = std::stod(tokens[1]);
                    if (speed < -1.0) speed = -1.0;
                    if (speed > 1.0) speed = 1.0;
                    sendSimple(0x21010135, static_cast<int32_t>(speed * 32767));
                } else {
                    sendSimple(0x21010135, 32767);  // 右转
                }
            } else if (tokens[0] == "gait") {
                if (tokens.size() > 1) {
                    int gait = std::stoi(tokens[1]);
                    uint32_t code = 0x21010300 + static_cast<uint32_t>(gait);
                    sendSimple(code);
                } else {
                    std::cout << "用法: gait <步态编号>" << std::endl;
                    std::cout << "  0: 平地低速  1: 平地中速  2: 平地高速" << std::endl;
                    std::cout << "  3: 正常/匍匐  4: 抓地越障  5: 通用越障" << std::endl;
                    std::cout << "  6: 高踏步" << std::endl;
                }
            } else if (tokens[0] == "action") {
                if (tokens.size() > 1) {
                    int action = std::stoi(tokens[1]);
                    uint32_t code = 0x21010200 + static_cast<uint32_t>(action);
                    sendSimple(code);
                } else {
                    std::cout << "用法: action <动作编号>" << std::endl;
                    std::cout << "  4: 扭身体  5: 翻身  0C: 太空步" << std::endl;
                    std::cout << "  02: 后空翻  07: 打招呼  0B: 向前跳" << std::endl;
                    std::cout << "  0D: 扭身跳" << std::endl;
                }
            } else if (tokens[0] == "voice") {
                if (tokens.size() > 1) {
                    int cmd = std::stoi(tokens[1]);
                    sendSimple(0x21010C0A, cmd);
                } else {
                    std::cout << "用法: voice <指令编号>" << std::endl;
                    std::cout << "  1:起立  2:坐下  3:前进  4:后退" << std::endl;
                    std::cout << "  5:左移  6:右移  8:低头  9:抬头" << std::endl;
                    std::cout << "  11:左看  12:右看  13:左转90°  14:右转90°" << std::endl;
                    std::cout << "  15:后转180°  22:打招呼  7:停止" << std::endl;
                }
            } else if (tokens[0] == "speaker") {
                if (tokens.size() > 1) {
                    if (tokens[1] == "on") {
                        sendSimple(0x2101030D, 1);
                    } else if (tokens[1] == "off") {
                        sendSimple(0x2101030D, 0);
                    } else if (tokens[1] == "query") {
                        querySpeakerStatus();
                    } else {
                        std::cout << "用法: speaker on|off|query" << std::endl;
                    }
                } else {
                    std::cout << "用法: speaker on|off|query" << std::endl;
                }
            } else if (tokens[0] == "perception") {
                if (tokens.size() > 1) {
                    if (tokens[1] == "stop") {
                        sendSimple(0x21012109, 0x00);
                    } else if (tokens[1] == "obstacle") {
                        sendSimple(0x21012109, 0x20);
                    } else if (tokens[1] == "follow") {
                        sendSimple(0x21012109, 0xC0);
                    } else if (tokens[1] == "nav") {
                        sendToPerception(0x21012109, 0x40);
                    } else {
                        std::cout << "用法: perception stop|obstacle|follow|nav" << std::endl;
                    }
                } else {
                    std::cout << "用法: perception stop|obstacle|follow|nav" << std::endl;
                }
            } else if (tokens[0] == "vel" || tokens[0] == "velocity") {
                double vx = 0.0, vy = 0.0, va = 0.0;
                if (tokens.size() > 1) vx = std::stod(tokens[1]);
                if (tokens.size() > 2) vy = std::stod(tokens[2]);
                if (tokens.size() > 3) va = std::stod(tokens[3]);
                sendVelocity(vx, vy, va);
            } else if (tokens[0] == "stopmove" || tokens[0] == "sm") {
                sendVelocity(0.0, 0.0, 0.0);  // 停止移动
            } else {
                std::cout << "[错误] 未知命令: " << line << std::endl;
                std::cout << "输入 'help' 查看命令列表" << std::endl;
            }
        }
    }

private:
    // 网络地址结构
    struct NetAddress {
        std::string ip;
        uint16_t port;
        NetAddress(const std::string& ip, uint16_t port) : ip(ip), port(port) {}
    };
    
    // 成员变量
    NetAddress m_ctrl_addr;          ///< 控制主机地址
    NetAddress m_perception_addr;    ///< 感知主机地址
    int m_sock_fd;                   ///< 套接字文件描述符
    std::atomic<bool> m_running;     ///< 运行状态标志
    
    std::thread m_recv_thread;       ///< 接收线程
    std::thread m_heartbeat_thread;  ///< 心跳线程
    
    std::queue<ReceivedPacket> m_recv_queue;  ///< 接收队列
    std::mutex m_queue_mutex;                  ///< 队列互斥锁
    std::condition_variable m_queue_cv;        ///< 队列条件变量
    
    bool m_step_in_place_state;      ///< 原地踏步状态
    std::atomic<bool> m_expecting_speaker_status;  ///< 等待扬声器状态
    
    /**
     * @brief 初始化网络
     * @param local_port 本地端口
     */
    void initializeNetwork(uint16_t local_port) {
#ifdef _WIN32
        // Windows下初始化Winsock
        WSADATA wsaData;
        if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
            throw std::runtime_error("WSAStartup failed");
        }
#endif
        
        // 创建UDP套接字
        m_sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
        if (m_sock_fd < 0) {
            throw std::runtime_error("创建套接字失败");
        }
        
        // 绑定本地端口
        if (local_port > 0) {
            struct sockaddr_in local_addr;
            memset(&local_addr, 0, sizeof(local_addr));
            local_addr.sin_family = AF_INET;
            local_addr.sin_addr.s_addr = INADDR_ANY;
            local_addr.sin_port = htons(local_port);
            
            if (bind(m_sock_fd, (struct sockaddr*)&local_addr, sizeof(local_addr)) < 0) {
#ifdef _WIN32
                closesocket(m_sock_fd);
#else
                close(m_sock_fd);
#endif
                throw std::runtime_error("绑定端口失败");
            }
        }
        
        // 设置非阻塞模式
#ifdef _WIN32
        u_long mode = 1;
        ioctlsocket(m_sock_fd, FIONBIO, &mode);
#else
        int flags = fcntl(m_sock_fd, F_GETFL, 0);
        if (flags >= 0) {
            fcntl(m_sock_fd, F_SETFL, flags | O_NONBLOCK);
        }
#endif
    }
    
    /**
     * @brief 发送数据到指定地址
     * @param data 要发送的数据
     * @param addr 目标地址
     */
    void sendTo(const std::vector<uint8_t>& data, const NetAddress& addr) {
        struct sockaddr_in target_addr;
        memset(&target_addr, 0, sizeof(target_addr));
        target_addr.sin_family = AF_INET;
        target_addr.sin_port = htons(addr.port);
        
        if (inet_pton(AF_INET, addr.ip.c_str(), &target_addr.sin_addr) <= 0) {
            throw std::runtime_error("无效的IP地址: " + addr.ip);
        }
        
        ssize_t sent = sendto(m_sock_fd, reinterpret_cast<const char*>(data.data()), 
                             data.size(), 0, (struct sockaddr*)&target_addr, sizeof(target_addr));
        
        if (sent < 0) {
            throw std::runtime_error("发送数据失败");
        }
    }
    
    /**
     * @brief 接收循环（在独立线程中运行）
     */
    void receiveLoop() {
        std::vector<uint8_t> buffer(1024);
        struct sockaddr_in from_addr;
        socklen_t addr_len = sizeof(from_addr);
        
        while (m_running) {
            memset(&from_addr, 0, sizeof(from_addr));
            ssize_t recv_len = recvfrom(m_sock_fd, reinterpret_cast<char*>(buffer.data()), 
                                       buffer.size(), 0, (struct sockaddr*)&from_addr, &addr_len);
            
            if (recv_len > 0) {
                // 提取发送方信息
                char ip_str[INET_ADDRSTRLEN];
                inet_ntop(AF_INET, &from_addr.sin_addr, ip_str, INET_ADDRSTRLEN);
                uint16_t port = ntohs(from_addr.sin_port);
                
                // 构建接收数据包
                ReceivedPacket packet;
                packet.data.assign(buffer.begin(), buffer.begin() + recv_len);
                packet.address = std::string(ip_str);
                packet.port = port;
                
                // 处理接收到的数据
                handleReceivedData(packet);
            } else {
#ifdef _WIN32
                if (WSAGetLastError() != WSAEWOULDBLOCK) {
#else
                if (errno != EAGAIN && errno != EWOULDBLOCK) {
#endif
                    // 真正的错误，但继续运行
                }
                // 短暂休眠避免CPU占用过高
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }
    }
    
    /**
     * @brief 心跳循环（在独立线程中运行）
     */
    void heartbeatLoop() {
        while (m_running) {
            try {
                // 使用静默模式发送心跳（不打印日志）
                sendSimple(0x21040001, 0, 0, true);
            } catch (...) {
                // 忽略心跳发送失败
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    }
    
    /**
     * @brief 处理接收到的数据包
     * @param packet 接收到的数据包
     */
    void handleReceivedData(const ReceivedPacket& packet) {
        if (packet.data.size() < 12) {
            return;  // 数据包太小
        }
        
        // 解析指令码
        uint32_t code = unpackUint32(packet.data.data());
        
        // 检查是否为扬声器状态响应
        if (m_expecting_speaker_status && code == 0x11050f08 && packet.data.size() >= 16) {
            uint32_t state = unpackUint32(packet.data.data() + 12);
            std::string status;
            if (state == 0) status = "已关闭";
            else if (state == 1) status = "已开启";
            else status = "未知(" + std::to_string(state) + ")";
            
            std::cout << "[扬声器状态] " << status << std::endl;
            m_expecting_speaker_status = false;
        }
        
        // 其他数据包处理可以在此扩展
    }
    
    /**
     * @brief 打包32位无符号整数（小端序）
     */
    void packUint32(uint8_t* buffer, uint32_t value) {
        buffer[0] = (value >> 0) & 0xFF;
        buffer[1] = (value >> 8) & 0xFF;
        buffer[2] = (value >> 16) & 0xFF;
        buffer[3] = (value >> 24) & 0xFF;
    }
    
    /**
     * @brief 解包32位无符号整数（小端序）
     */
    uint32_t unpackUint32(const uint8_t* buffer) {
        return (static_cast<uint32_t>(buffer[0]) << 0) |
               (static_cast<uint32_t>(buffer[1]) << 8) |
               (static_cast<uint32_t>(buffer[2]) << 16) |
               (static_cast<uint32_t>(buffer[3]) << 24);
    }
    
    /**
     * @brief 打包64位双精度浮点数
     */
    void packDouble(uint8_t* buffer, double value) {
        // 将double转换为字节数组（小端序）
        uint64_t bits;
        memcpy(&bits, &value, sizeof(bits));
        for (int i = 0; i < 8; i++) {
            buffer[i] = (bits >> (i * 8)) & 0xFF;
        }
    }
    
    /**
     * @brief 分割字符串
     */
    void splitString(const std::string& str, std::vector<std::string>& tokens, char delimiter) {
        tokens.clear();
        std::stringstream ss(str);
        std::string token;
        while (std::getline(ss, token, delimiter)) {
            if (!token.empty()) {
                tokens.push_back(token);
            }
        }
    }
    
    /**
     * @brief 打印帮助信息
     */
    void printHelp() {
        std::cout << "\n========== 命令列表 ==========" << std::endl;
        std::cout << "  [运动主机命令] 控制机器人的运动和姿态" << std::endl;
        std::cout << "  heartbeat/hb        - 发送心跳包" << std::endl;
        std::cout << "  stand/st            - 起立/趴下" << std::endl;
        std::cout << "  step/sp             - 切换原地踏步" << std::endl;
        std::cout << "  manual/man          - 切换到手动模式" << std::endl;
        std::cout << "  auto/aut            - 切换到自主模式" << std::endl;
        std::cout << "  home                - 回零" << std::endl;
        std::cout << "  stop/stp            - 软急停" << std::endl;
        std::cout << "  forward/fw [speed]  - 前进 (speed: 0-1)" << std::endl;
        std::cout << "  backward/bw [speed] - 后退 (speed: 0-1)" << std::endl;
        std::cout << "  left [speed]        - 左移 (speed: 0-1)" << std::endl;
        std::cout << "  right [speed]       - 右移 (speed: 0-1)" << std::endl;
        std::cout << "  turnleft/tl [speed] - 左转 (speed: 0-1)" << std::endl;
        std::cout << "  turnright/tr [speed]- 右转 (speed: 0-1)" << std::endl;
        std::cout << "  gait <编号>         - 切换步态" << std::endl;
        std::cout << "  action <编号>       - 执行动作" << std::endl;
        std::cout << "  voice <编号>        - 语音指令" << std::endl;
        std::cout << "  speaker on|off|query- 扬声器控制" << std::endl;
        std::cout << "  vel <vx> [vy] [va]  - 设置速度 (vx:线速度x, vy:线速度y, va:角速度)" << std::endl;
        std::cout << "  stopmove/sm         - 停止移动" << std::endl;
        std::cout << std::endl;
        std::cout << "  [感知主机命令] 控制机器人的感知和AI功能" << std::endl;
        std::cout << "  perception stop     - 关闭所有AI选项" << std::endl;
        std::cout << "  perception obstacle - 开启停障" << std::endl;
        std::cout << "  perception follow   - 开启跟随" << std::endl;
        std::cout << "  perception nav      - 开启导航避障" << std::endl;
        std::cout << std::endl;
        std::cout << "  [通用命令]" << std::endl;
        std::cout << "  help/h              - 显示此帮助" << std::endl;
        std::cout << "  quit/q              - 退出程序" << std::endl;
        std::cout << "================================" << std::endl;
    }
};

/**
 * @brief 主函数
 */
int main(int argc, char* argv[]) {
    std::cout << "========== 绝影Lite3 机器人控制器 ==========" << std::endl;
    std::cout << "目标机器人IP: 192.168.1.120" << std::endl;
    std::cout << "============================================" << std::endl;
    
    // 本地端口（可自定义）
    uint16_t local_port = 20001;
    
    // 如果命令行指定了端口
    if (argc > 1) {
        try {
            local_port = static_cast<uint16_t>(std::stoi(argv[1]));
        } catch (...) {
            std::cout << "用法: " << argv[0] << " [本地端口]" << std::endl;
            std::cout << "示例: " << argv[0] << " 20001" << std::endl;
            return 1;
        }
    }
    
    try {
        // 创建机器人控制器（固定IP为192.168.1.120）
        RobotCommander robot(local_port);
        
        // 进入交互模式
        robot.interactiveMode();
    } catch (const std::exception& e) {
        std::cerr << "[致命错误] " << e.what() << std::endl;
        std::cerr << std::endl;
        std::cerr << "请检查：" << std::endl;
        std::cerr << "1. 网线是否正确连接" << std::endl;
        std::cerr << "2. 电脑IP是否在 192.168.1.x 网段" << std::endl;
        std::cerr << "3. 机器人是否已上电并启动" << std::endl;
        std::cerr << "4. 防火墙是否允许UDP通信" << std::endl;
        return 1;
    }
    
    return 0;
}
