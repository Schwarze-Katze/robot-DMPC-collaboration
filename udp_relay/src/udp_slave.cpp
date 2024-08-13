#include "udp_slave.h"


UDPSlave::UDPSlave() {
    std::string cfg_path;
    ros::param::get("cfg_path", cfg_path);
    YAML::Node cfg = YAML::LoadFile(cfg_path);
    auto self_cfg = cfg["self"];
    auto remote_cfg = cfg["remote"][0];

    int self_id = 0;
    if (self_cfg["id"]) {
        self_id = self_cfg["id"].as<int>();
    }
    else {
        ROS_ERROR("id undefined");
    }
    int LOCAL_PORT = remote_cfg["recv_port"].as<int>();
    std::string REMOTE_IP = remote_cfg["ip"].as<std::string>();
    int REMOTE_PORT = remote_cfg["send_port"].as<int>();

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

    int buffer_size = 1024; // 设置为1MB
    if (setsockopt(recv_sock, SOL_SOCKET, SO_RCVBUF, &buffer_size, sizeof(buffer_size)) < 0) {
        ROS_ERROR("Setting receive buffer size failed");
    }

    // Remote address setup
    memset(&remote_addr, 0, sizeof(remote_addr));
    remote_addr.sin_family = AF_INET;
    remote_addr.sin_addr.s_addr = inet_addr(REMOTE_IP.c_str());//向主机发送
    remote_addr.sin_port = htons(REMOTE_PORT);

    // if (bind(send_sock, (struct sockaddr*) &remote_addr, sizeof(remote_addr)) < 0) {
    //     ROS_ERROR("Socket bind failed");
    //     exit(1);
    // }

    // ROS subscribers and publishers
    nh = ros::NodeHandle();
    auto send_topics = remote_cfg["send_topics"];
    auto recv_topics = remote_cfg["recv_topics"];

    assert(send_topics.size() == 2);
    assert(recv_topics.size() == 2);

    subs.push_back(nh.subscribe(send_topics[0].as<std::string>(), 1000, &UDPSlave::odomCallback, this));
    subs.push_back(nh.subscribe(send_topics[1].as<std::string>(), 1000, &UDPSlave::catCmdCallback, this));

    pubs.push_back(nh.advertise<geometry_msgs::Twist>(recv_topics[0].as<std::string>(), 1000));
    pubs.push_back(nh.advertise<std_msgs::Bool>(recv_topics[1].as<std::string>(), 1000));

    ROS_INFO("UDP Communication Initialized.");
}

UDPSlave::~UDPSlave() {
    close(send_sock);
    close(recv_sock);
}

void UDPSlave::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    std::unique_lock<std::mutex> ulck(_mtx);
    odom_buf = *msg;
    return;
    // std::string data = serializeOdom(*msg);
    // data += generateChecksum(data);
    uint32_t serial_size = ros::serialization::serializationLength(*msg);
    boost::shared_array<uint8_t> buffer(new uint8_t[serial_size]);
    ros::serialization::OStream stream(buffer.get(), serial_size);
    ros::serialization::serialize(stream, *msg);
    sendto(send_sock, buffer.get(), serial_size, 0, (struct sockaddr*) &remote_addr, sizeof(remote_addr));
}


void UDPSlave::catCmdCallback(const std_msgs::Bool::ConstPtr& msg) {
    std::unique_lock<std::mutex> ulck(_mtx);
    bool_buf = *msg;
    return;
    ROS_INFO("relay slave command");
}

void UDPSlave::transferData() {
    // Send
    sendData();

    // Recv
    recvData();
    recvData();//receive twice to avoid unmatched frequency
}

void UDPSlave::sendData() {
std::unique_lock<std::mutex> ulck(_mtx);
    uint32_t serial_size = 16;
    serial_size += ros::serialization::serializationLength(odom_buf);
    serial_size += ros::serialization::serializationLength(bool_buf);
    boost::shared_array<uint8_t> send_buffer(new uint8_t[serial_size]);
    ros::serialization::OStream stream(send_buffer.get(), serial_size);
    ros::serialization::serialize(stream, odom_buf);
    ros::serialization::serialize(stream, bool_buf);
    ulck.unlock();
    sendto(send_sock, send_buffer.get(), serial_size, 0, (struct sockaddr*) &remote_addr, sizeof(remote_addr));
}

void UDPSlave::recvData() {
    // Recv
    geometry_msgs::Twist twist_msg;
    std_msgs::Bool bool_msg;
    uint32_t msg_len = 16;
    msg_len += ros::serialization::serializationLength(twist_msg);
    msg_len += ros::serialization::serializationLength(bool_msg);
    boost::shared_array<uint8_t> recv_buffer(new uint8_t[msg_len]);
    struct sockaddr_in sender_addr;
    socklen_t sender_len = sizeof(sender_addr);
    int len = recvfrom(recv_sock, recv_buffer.get(), msg_len, MSG_DONTWAIT, (struct sockaddr*) &sender_addr, &sender_len);

    if (len > 0) {
        ros::serialization::IStream stream((uint8_t*) recv_buffer.get(), msg_len);
        ros::serialization::deserialize(stream, twist_msg);
        pubs[0].publish(twist_msg);
        ros::serialization::deserialize(stream, bool_msg);
        pubs[1].publish(bool_msg);
    }
}
