/*
 * @Author: wentao zhang && zwt190315@163.com
 * @Date: 2023-07-22
 * @LastEditTime: 2023-07-22
 * @Description: UDP communication bridge between ROS and other data source
 * @reference: 
 * @function lists:
 * 1. cv::Mat (CV_8UC1) <==> socket (UDP)       brief: obstacle map     reference: none
 * 2. geometry_msgs::Pose <==> socket (UDP)     brief: robot odometry   reference: http://docs.ros.org/en/api/geometry_msgs/html/msg/Pose.html
 * 3. geometry_msgs::Pose[] <==> socket (UDP)   brief: robot trajectory reference: none
 * Copyright (c) 2023 by wentao zhang, All Rights Reserved. 
 */

#ifndef UDP_BRIDGE_H
#define UDP_BRIDGE_H

#include <iostream>
#include <string>
#include <math.h>
#include <vector>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

////UDP communication parameters
#define BUFFER_SIZE_MAX 2048
#define BUFFER_SIZE 1024
#define UDP_HEADER 0xFF
#define UDP_END 0xEE
#define HEADER_SIZE 4 // header + length + struct_num = 4 bytes

//////  for test
// #define DEST_PORT 1024
// #define DSET_IP_ADDRESS  "192.168.2.130"

enum UDP_DATA_TYPE
{
    OBSTACLE_MAP    = (1 << 0),
    ROBOT_POSE      = (1 << 1),
    ROBOT_TRAJECTORY= (1 << 2),
};

struct geometry_msgs_Pose
{
    double x;
    double y;
    double z;
    double qx;
    double qy;
    double qz;
    double qw;
};

struct UDPHeader
{
    uint8_t header;
    uint16_t length;
    uint8_t struct_num;
};
//  char datas structure for UDP communication #################################################
/* header:      1 bytes | 0xFF      |
 * length:      2 bytes | uint16_t  | length of all data from header to end
 * struct num:  1 bytes | uint8_t
 * struct 1:    x bytes | data
 * ...
 * struct n:    x bytes | data
 * struct -----------------------------------------------------------------------------------
 * data type:   2 bytes | uint16_t   | OBSTACLE_MAP, ROBOT_POSE, ROBOT_TRAJECTORY
 * data size:   2 bytes | uint16_t   | size of data
 * data:        x bytes | data       | data
 * end:         1 bytes | 0xEE       |
*/


/*
 *
*/
class UDPBridge
{
    private:
    int socket_fd_;
    int port_;
    std::string ip_;
    int DATA_TYPE_; // data type
    int buffer_size_;
    char *buffer_;  // buffer for data
    char send_buffer_[BUFFER_SIZE_MAX];
    char receive_buffer_[BUFFER_SIZE_MAX];
    int data_size_; // data size in bytes
    int data_index_;// current data index
    struct sockaddr_in addr_serv;
    //// datas
    cv::Mat binary_image_;
    uint16_t cvMatBetySize_;
    geometry_msgs::Pose Pose_;
    uint16_t PoseSize_;
    nav_msgs::Path Path_;
    uint16_t PathSize_;

    public:
    geometry_msgs::Pose robot_pose_;
    std::vector<geometry_msgs::Pose> robot_trajectory_;
    

    UDPBridge() = default;
    ~UDPBridge() = default;
    bool setUDP(const int port,const char *ip);
    // bool setDataTypes(std::vector<UDP_DATA_TYPE> data_types);
    // bool setDataTypes(UDP_DATA_TYPE data_type);
    //TODO: add template for different data types
    bool setcvMat(cv::Mat &mat);
    bool setROSPose(geometry_msgs::Pose &pose);
    bool setROSNavseq(nav_msgs::Path &path);
    bool setHeaderandBail();
    bool send();
    bool receive();
    bool getcvMat(cv::Mat &mat);
    bool getROSPose(geometry_msgs::Pose &pose);
    bool getROSNavseq(nav_msgs::Path &path);
};

bool UDPBridge::setUDP(const int port,const char *ip)
{
    port_ = port;
    ip_ = std::string(ip);
    socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if(socket_fd_ < 0)
    {
        std::cout << "socket create failed" << std::endl;
        return false;
    }
    //将addr_serv数组全体内存空间按字节整体清零
    memset(&addr_serv, 0, sizeof(addr_serv));
    addr_serv.sin_family = AF_INET;
    addr_serv.sin_addr.s_addr = inet_addr(ip); //服务器ip，不是本机ip
    addr_serv.sin_port = htons(port);//服务器端口
    return true;
}

bool UDPBridge::setHeaderandBail()
{
    UDPHeader header;
    header.header = 0xFF;
    header.length = 0;
    header.struct_num = 0;
    memcpy(send_buffer_, &header, sizeof(header));
    data_index_ = sizeof(header);
    return true;
}

//ssize_t sendo(ints,const void *msg,size_t len,int flags,const struct sockaddr *to,socklen_ttolen);
/*
2>函数功能：
向目标主机发送消息
3>函数形参：
Ø  s:套接字描述符。
Ø  *msg:发送缓冲区
Ø  len:待发送数据的长度
Ø  flags:控制选项，一般设置为0或取下面的值
(1)MSG_OOB:在指定的套接字上发送带外数据(out-of-band data),该类型的套接字必须支持带外数据（eg:SOCK_STREAM）.
(2)MSG_DONTROUTE:通过最直接的路径发送数据，而忽略下层协议的路由设置。
Ø  to:用于指定目的地址
Ø  tolen:目的地址的长度。
4>函数返回值：
执行成功后返回实际发送数据的字节数，出错返回-1，错误代码存入errno中。
*/
bool UDPBridge::send() {
    auto send_flag = sendto(socket_fd_, send_buffer_, data_index_, 0, (struct sockaddr *)&addr_serv, sizeof(addr_serv));
    if (send_flag < 0)
        return false;
    else
        return true;
}
#endif // UDP_BRIDGE_H