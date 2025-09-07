#ifndef TCP_SERV_H
#define TCP_SERV_H

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <string>
#include <thread>
#include <mutex>

class CarServer {
private:
    std::string serv_host;
    int serv_port;
    bool cs_connected;
    int client_fd;
    std::mutex mtx;
    bool stop_state;

    ros::Publisher pub;
    ros::Subscriber stop_sub;
    ros::NodeHandle nh;

public:
    CarServer(ros::NodeHandle& nh_, const std::string& host = "192.168.1.100", int port = 6666);

    void start_server();          // 启动服务端监听并处理逻辑
    void control_loop();          // 核心控制线程（状态发送 + 等待检测反馈）
    void send_msg(const std::string& msg); // 向客户端发送消息
    void stop();                  // 主动断开服务端
    void stopCallback(const std_msgs::Bool::ConstPtr& msg); // 接收检测指令（停车/前进）
};

extern CarServer* global_server;
void handle_sigint(int);  // Ctrl+C 信号处理函数

#endif 
