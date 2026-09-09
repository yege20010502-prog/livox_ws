#include <iostream>
#include <string>
#include <thread>
#include <atomic>
#include <chrono>
#include <cstring>
#include <iomanip>
#include <vector>
#include <mutex>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <cmath>

/**
 * @brief 调试程序 - 搜索速度值
 */
class DebugReceiver {
public:
    DebugReceiver(uint16_t port = 43897) : m_running(true) {
        m_sock = socket(AF_INET, SOCK_DGRAM, 0);
        if (m_sock < 0) throw std::runtime_error("socket failed");
        
        struct sockaddr_in addr;
        memset(&addr, 0, sizeof(addr));
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = INADDR_ANY;
        addr.sin_port = htons(port);
        
        if (bind(m_sock, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
            close(m_sock);
            throw std::runtime_error("bind failed");
        }
        
        int flags = fcntl(m_sock, F_GETFL, 0);
        if (flags >= 0) fcntl(m_sock, F_SETFL, flags | O_NONBLOCK);
        
        m_recv_thread = std::thread(&DebugReceiver::receiveLoop, this);
        std::cout << "[INFO] 监听端口 " << port << "，搜索速度数据..." << std::endl;
    }
    
    ~DebugReceiver() {
        m_running = false;
        if (m_recv_thread.joinable()) m_recv_thread.join();
        close(m_sock);
    }

private:
    int m_sock;
    std::atomic<bool> m_running;
    std::thread m_recv_thread;
    
    void receiveLoop() {
        std::vector<uint8_t> buf(4096);
        struct sockaddr_in from;
        socklen_t from_len = sizeof(from);
        int packet_count = 0;
        
        while (m_running) {
            memset(&from, 0, sizeof(from));
            ssize_t n = recvfrom(m_sock, buf.data(), buf.size(), 0, 
                                 (struct sockaddr*)&from, &from_len);
            if (n > 0 && n >= 12) {
                packet_count++;
                uint32_t code = unpackUint32(buf.data());
                
                if (code == 0x0901) {
                    std::cout << "\n===== 数据包 #" << packet_count << " =====" << std::endl;
                    
                    // 1. 读取 basic_state (偏移12)
                    int basic = unpackInt32(buf.data() + 12);
                    std::cout << "basic_state (偏移12): " << basic << std::endl;
                    
                    // 2. 读取 gait_state (偏移16)
                    int gait = unpackInt32(buf.data() + 16);
                    std::cout << "gait_state (偏移16): " << gait << std::endl;
                    
                    // 3. 读取 motion_state (偏移180)
                    int motion = unpackInt32(buf.data() + 180);
                    std::cout << "motion_state (偏移180): " << motion << std::endl;
                    
                    // 4. 读取 battery (偏移180 double)
                    double battery = unpackDouble(buf.data() + 180);
                    std::cout << "battery (偏移180): " << battery << "%" << std::endl;
                    
                    // 5. 搜索速度值 - 在偏移96到160之间搜索合理的速度值
                    std::cout << "\n搜索速度值 (应该在 -2.0 ~ 2.0 之间):" << std::endl;
                    
                    // 从偏移96开始，每4字节尝试解析double
                    for (size_t offset = 96; offset + 8 <= 200; offset += 4) {
                        double val = unpackDouble(buf.data() + offset);
                        // 只打印合理的速度值 (在 -2.0 到 2.0 之间)
                        if (std::abs(val) > 0.001 && std::abs(val) < 10.0) {
                            std::cout << "  偏移 " << offset << ": " << std::fixed << std::setprecision(3) << val << std::endl;
                        }
                    }
                    
                    // 6. 打印最后32字节的原始数据
                    std::cout << "\n最后32字节原始数据:" << std::endl;
                    size_t start = n > 32 ? n - 32 : 0;
                    for (size_t i = start; i < n; i++) {
                        std::cout << std::hex << std::setw(2) << std::setfill('0') 
                                  << (int)buf.data()[i] << " ";
                        if ((i - start + 1) % 16 == 0) std::cout << std::endl;
                    }
                    std::cout << std::dec << std::endl;
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
    
    uint32_t unpackUint32(const uint8_t* b) {
        return (uint32_t)b[0] | ((uint32_t)b[1]<<8) | ((uint32_t)b[2]<<16) | ((uint32_t)b[3]<<24);
    }
    
    int32_t unpackInt32(const uint8_t* b) {
        return (int32_t)unpackUint32(b);
    }
    
    double unpackDouble(const uint8_t* b) {
        uint64_t bits = 0;
        for (int i = 0; i < 8; i++) {
            bits |= (uint64_t)b[i] << (i*8);
        }
        double val;
        memcpy(&val, &bits, sizeof(val));
        return val;
    }
};

int main() {
    try {
        DebugReceiver receiver(43897);
        std::cout << "按 Ctrl+C 退出" << std::endl;
        while (true) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    } catch (const std::exception& e) {
        std::cerr << "错误: " << e.what() << std::endl;
    }
    return 0;
}
