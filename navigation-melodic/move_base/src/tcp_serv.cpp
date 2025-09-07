#include <iostream>      // 用于输入输出流操作
#include <thread>        // 用于多线程 std::thread
#include <mutex>         // 用于互斥锁 std::mutex
#include <cstring>       // 用于字符串处理函数，如 memset
#include <unistd.h>      // 提供 close、sleep、read 等 Unix 系统调用
#include <arpa/inet.h>   // 用于处理 IP 地址的转换等网络操作
#include <csignal>       // 用于处理信号，如 Ctrl+C
#include <ros/ros.h>
#include <std_msgs/Bool.h>

// 定义小车服务器类
class CarServer {
private:
    std::string serv_host;    // 服务器绑定的IP地址
    int serv_port;            // 服务器监听的端口号
    bool cs_connected = true; // 与客户端连接状态
    int client_fd = -1;       // 客户端 socket 文件描述符
    std::mutex mtx;           // 用于线程间互斥操作的锁
    bool stop_state = false;

    ros::Publisher pub;
    ros::Subscriber stop_sub;
    ros::NodeHandle nh;
public:
    // 构造函数，默认IP为192.168.1.100，端口为8080
    CarServer(ros::NodeHandle& nh_,const std::string& host = "192.168.1.100", int port = 6666)
        : nh(nh_), serv_host(host), serv_port(port) {

            pub = nh.advertise<std_msgs::Bool>("continue_signal", 1, true);
            stop_sub = nh.subscribe("stop_for_detect", 10, &CarServer::stopCallback, this);
        }
    
    // 启动服务器
    void start_server() {
        // 创建 socket，使用 IPv4 和 TCP
        int serv_fd = socket(AF_INET, SOCK_STREAM, 0);
        if (serv_fd < 0) {
            perror("socket error"); // 打印错误信息
            return;
        }

        // 配置服务器地址结构体
        sockaddr_in serv_addr{};
        serv_addr.sin_family = AF_INET;                    // 设置地址族为 IPv4
        serv_addr.sin_port = htons(serv_port);             // 设置端口号（主机序转网络序）
        inet_pton(AF_INET, serv_host.c_str(), &serv_addr.sin_addr); // 将IP字符串转为二进制格式

        // 绑定 IP 和端口
        if (bind(serv_fd, (sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
            perror("bind error");
            close(serv_fd); // 关闭 socket
            return;
        }

        // 开始监听客户端连接，最大连接数为1
        if (listen(serv_fd, 5) < 0) {
            perror("listen error");
            close(serv_fd);
            return;
        }

        // 输出启动成功提示
        std::cout << "服务端启动 | 初始状态: 行驶中 | 监听 " 
                  << serv_host << ":" << serv_port << std::endl;

        // 等待客户端连接
        sockaddr_in client_addr{};
        socklen_t client_len = sizeof(client_addr);
        
        while (true) {
            std::cout << "等待客户端连接中..." << std::endl;
            client_fd = accept(serv_fd, (sockaddr*)&client_addr, &client_len);
            if (client_fd >= 0) {
                std::cout << "客户端已连接" << std::endl;
                break;  // 成功后跳出等待
            } else {
                 perror("accept error");
                 std::this_thread::sleep_for(std::chrono::seconds(1)); // 1秒后重试
            }
        }


        // 启动后台线程，执行控制逻辑
        std::thread control_thread(&CarServer::control_loop, this);
        control_thread.detach(); // 设置为后台线程，主线程不等待它

        // 主线程持续保持运行，直到断开连接
        while (cs_connected) {
            std::this_thread::sleep_for(std::chrono::seconds(1)); // 每秒检查一次连接状态
        }

        // 清理资源
        close(client_fd);
        close(serv_fd);
    }

    // 控制主循环：周期性发送指令 + 等待客户端检测完成反馈
    void control_loop() {

        ros::Rate rate(10); 
        while (cs_connected) {
            ros::spinOnce(); 
            {
                if(stop_state == false)
                {
                    std::lock_guard<std::mutex> lock(mtx); // 加锁保护 共享变量
                    // car_running = true;                    // 设置为行驶状态
                    send_msg("状态:行驶中|禁止检测");       // 发送行驶状态消息
                    std::cout << "小车行驶中" << std::endl;
                }
            }


            // 等待客户端发送 "检测完成" 消息
            while (stop_state == true) {
                {
                    std::lock_guard<std::mutex> lock(mtx); // 加锁，切换为停止状态
                    // car_running = false;
                    send_msg("状态:已停止|开始检测");       // 通知客户端开始检测
                    std::cout << "小车已停止 (等待检测完成)" << std::endl;
                }
                
                ros::spinOnce(); 
                char buffer[1024] = {0}; // 接收缓冲区
                ssize_t len = recv(client_fd, buffer, sizeof(buffer) - 1, 0); // 接收数据
                if (len <= 0) {
                    cs_connected = false; // 客户端断开
                    break;
                }
                std::string data(buffer);
                if (data == "检测完成") {
                    std::cout << "收到检测完成通知" << std::endl;
                    //检测完成后--发布小车开始行驶话题
                    stop_state = false;
                    std_msgs::Bool msg;
                    msg.data = true;
                    pub.publish(msg);
                    ROS_INFO("[continue_signal_publisher] 发布: true");
                    break; // 收到完成消息，跳出等待
                }
                rate.sleep();
            }

            rate.sleep();
        }
    }


    // 发送消息给客户端
    void send_msg(const std::string& msg) {
        ssize_t sent = send(client_fd, msg.c_str(), msg.length(), 0);
        if (sent < 0) {
            perror("send error");
        }
    }

    // 停止服务器运行
    void stop() {
        cs_connected = false;
    }


    void stopCallback(const std_msgs::Bool::ConstPtr& msg) {
    if (msg->data) {
        stop_state = true;
        ROS_INFO("[Subscriber] 收到停车检测命令：停车并检测");
        // 在此添加你的小车停止逻辑
    } else {
        stop_state = false;
        ROS_INFO("[Subscriber] 收到继续前进命令");
    }
}
};

// 定义全局指针，用于信号处理时调用 stop
CarServer* global_server = nullptr;

// 信号处理函数：捕捉 Ctrl+C（SIGINT）关闭服务
void handle_sigint(int) {
    if (global_server) {
        global_server->stop(); // 设置连接标志为 false
        std::cout << "服务端安全关闭" << std::endl;
    }
    exit(0); // 退出程序
}

// 主函数入口
int main(int argc, char** argv) {
    ros::init(argc, argv, "tcp_server_node");
    ros::NodeHandle nh;

    signal(SIGINT, handle_sigint); // 注册 Ctrl+C 信号处理
    CarServer server(nh);              // 创建服务器对象
    global_server = &server;       // 设置全局指针
    server.start_server();         // 启动服务器主流程

    return 0;
}

