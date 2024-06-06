#include "udp_master.h"

const int LOCAL_PORT = 40001;
const int REMOTE_PORT = 30001;
const char* REMOTE_IP = "192.168.31.103";

const std::vector<double> x_diff={0.0,0.0,0.0};
const std::vector<double> y_diff={0.0,-1.0,1.0};


UDPMaster::UDPMaster(int _idx) :idx(_idx) {
    recv_sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (recv_sock < 0) {
        ROS_ERROR("Socket creation failed");
        exit(1);
    }
    send_sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (send_sock < 0) {
        ROS_ERROR("Socket creation failed");
        exit(1);
    }

    // Local address setup
    memset(&local_addr, 0, sizeof(local_addr));
    local_addr.sin_family = AF_INET;
    local_addr.sin_addr.s_addr = INADDR_ANY;//监听所有来源
    local_addr.sin_port = htons(LOCAL_PORT);

    if (bind(recv_sock, (struct sockaddr*) &local_addr, sizeof(local_addr)) < 0) {
        ROS_ERROR("Socket bind failed");
        exit(1);
    }

    // Remote address setup
    memset(&remote_addr, 0, sizeof(remote_addr));
    remote_addr.sin_family = AF_INET;
    remote_addr.sin_addr.s_addr = inet_addr(REMOTE_IP);//向主机发送
    remote_addr.sin_port = htons(REMOTE_PORT);

    // if (bind(send_sock, (struct sockaddr*) &remote_addr, sizeof(remote_addr)) < 0) {
    //     ROS_ERROR("Socket bind failed");
    //     exit(1);
    // }

    // ROS subscribers and publishers
    nh = ros::NodeHandle("vehicle" + std::to_string(idx));
    twist_sub = nh.subscribe("cmd_vel", 1000, &UDPMaster::twistCallback, this);
    odom_pub = nh.advertise<nav_msgs::Odometry>("Odometry", 1000);

    ROS_INFO("UDP Communication Initialized.");
}

UDPMaster::~UDPMaster() {
    close(send_sock);
    close(recv_sock);
}

void UDPMaster::twistCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    ROS_INFO("relay slave %d twist",idx);
    uint32_t serial_size = ros::serialization::serializationLength(*msg);
    boost::shared_array<uint8_t> buffer(new uint8_t[serial_size]);
    ros::serialization::OStream stream(buffer.get(), serial_size);
    ros::serialization::serialize(stream, *msg);
    sendto(send_sock, buffer.get(), serial_size, 0, (struct sockaddr*) &remote_addr, sizeof(remote_addr));
}

void UDPMaster::receiveData() {
    nav_msgs::Odometry odom_msg;
    const uint32_t msg_len = ros::serialization::serializationLength(odom_msg)+16;
    // const uint32_t msg_len =1024;
    boost::shared_array<uint8_t> buffer(new uint8_t[msg_len]);
    struct sockaddr_in sender_addr;
    socklen_t sender_len = sizeof(sender_addr);
    int len = recvfrom(recv_sock, buffer.get(), msg_len, MSG_DONTWAIT, (struct sockaddr*) &sender_addr, &sender_len);
    // ROS_INFO("len = %d : %d, sender = %d:%d",len,msg_len,sender_addr.sin_addr.s_addr,sender_addr.sin_port);
    if (len > 0) {
        ros::serialization::IStream stream((uint8_t*) buffer.get(), msg_len);
        ros::serialization::deserialize(stream, odom_msg);
        odom_msg.pose.pose.position.x+=x_diff[idx-1];
        odom_msg.pose.pose.position.y+=y_diff[idx-1];
        // ROS_INFO("odom_msg.pose.pose.position = %.2lf,%.2lf",odom_msg.pose.pose.position.x,odom_msg.pose.pose.position.y);
        ROS_INFO("relay slave %d odom",idx);
        odom_pub.publish(odom_msg);
    }
}
