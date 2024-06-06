#include "udp_slave.h"

const int LOCAL_PORT = 30001;
const int REMOTE_PORT = 40001;
const char* REMOTE_IP = "192.168.31.101";

UDPSlave::UDPSlave() {
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
    // nh = ros::NodeHandle("vehicle" + std::to_string(idx));
    odom_sub = nh.subscribe("Odometry", 1000, &UDPSlave::odomCallback, this);
    twist_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

    ROS_INFO("UDP Communication Initialized.");
}

UDPSlave::~UDPSlave() {
    close(send_sock);
    close(recv_sock);
}

void UDPSlave::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    // std::string data = serializeOdom(*msg);
    // data += generateChecksum(data);
    uint32_t serial_size = ros::serialization::serializationLength(*msg);
    boost::shared_array<uint8_t> buffer(new uint8_t[serial_size]);
    ros::serialization::OStream stream(buffer.get(), serial_size);
    ros::serialization::serialize(stream, *msg);
    sendto(send_sock, buffer.get(), serial_size, 0, (struct sockaddr*) &remote_addr, sizeof(remote_addr));
}

void UDPSlave::receiveData() {
    geometry_msgs::Twist twist_msg;
    const uint32_t msg_len = ros::serialization::serializationLength(twist_msg);
    char buffer[msg_len];
    struct sockaddr_in sender_addr;
    socklen_t sender_len = sizeof(sender_addr);
    int len = recvfrom(recv_sock, buffer, msg_len, 0, (struct sockaddr*) &sender_addr, &sender_len);

    if (len > 0) {
        ros::serialization::IStream stream((uint8_t*) buffer, len);
        ros::serialization::deserialize(stream, twist_msg);
        twist_pub.publish(twist_msg);
    }
}
