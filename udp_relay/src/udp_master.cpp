#include "udp_master.h"

std::mutex _reconfigure_mtx;
std::vector<double> x_diff={0.0,0.0,0.0};
std::vector<double> y_diff={0.0,1.0,-1.0};
std::vector<double> yaw_diff={0.0,0.0,0.0};

UDPMaster::UDPMaster(int _i){
    std::string cfg_path;
    ros::param::get("cfg_path", cfg_path);
    YAML::Node cfg = YAML::LoadFile(cfg_path);
    auto self_cfg = cfg["self"];
    auto remote_cfg = cfg["remote"][_i];

    self_id = 0;
    if (self_cfg["id"]) {
        self_id = self_cfg["id"].as<int>();
    }
    else {
        ROS_ERROR("id undefined");
    }
    remote_id = remote_cfg["id"].as<int>();
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
    assert(send_topics[0].as<std::string>().find(std::to_string(remote_id)) != send_topics[0].as<std::string>().npos);

    subs.push_back(nh.subscribe(send_topics[0].as<std::string>(), 1000, &UDPMaster::twistCallback, this));
    subs.push_back(nh.subscribe(send_topics[1].as<std::string>(), 1000, &UDPMaster::catCmdCallback, this));

    pubs.push_back(nh.advertise<nav_msgs::Odometry>(recv_topics[0].as<std::string>(), 1000));
    pubs.push_back(nh.advertise<std_msgs::Bool>(recv_topics[1].as<std::string>(), 1000));

    ROS_INFO("UDP Connection Between #%d Initialized as Index %d", remote_id, _i);
}

UDPMaster::~UDPMaster() {
    close(send_sock);
    close(recv_sock);
}

void UDPMaster::twistCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    std::unique_lock<std::mutex> ulck(_mtx);
    twist_buf = *msg;
    return;
    ROS_INFO("relay slave %d twist", remote_id);
    
}

void UDPMaster::catCmdCallback(const std_msgs::Bool::ConstPtr& msg) {
    std::unique_lock<std::mutex> ulck(_mtx);
    bool_buf = *msg;
    return;
    ROS_INFO("relay slave %d command", remote_id);
}

void UDPMaster::transferData() {
    // Send
    std::unique_lock<std::mutex> ulck(_mtx);
    uint32_t serial_size = 16;
    serial_size += ros::serialization::serializationLength(twist_buf);
    serial_size += ros::serialization::serializationLength(bool_buf);
    boost::shared_array<uint8_t> send_buffer(new uint8_t[serial_size]);
    ros::serialization::OStream stream(send_buffer.get(), serial_size);
    ros::serialization::serialize(stream, twist_buf);
    ros::serialization::serialize(stream, bool_buf);
    ulck.unlock();
    sendto(send_sock, send_buffer.get(), serial_size, 0, (struct sockaddr*) &remote_addr, sizeof(remote_addr));

    // Recv
    nav_msgs::Odometry odom_msg;
    std_msgs::Bool bool_msg;
    uint32_t msg_len = 16;
    msg_len += ros::serialization::serializationLength(odom_msg);
    msg_len += ros::serialization::serializationLength(bool_msg);
    boost::shared_array<uint8_t> recv_buffer(new uint8_t[msg_len]);
    struct sockaddr_in sender_addr;
    socklen_t sender_len = sizeof(sender_addr);
    int len = recvfrom(recv_sock, recv_buffer.get(), msg_len, MSG_DONTWAIT, (struct sockaddr*) &sender_addr, &sender_len);
    // ROS_INFO("len = %d : %d, sender = %d:%d",len,msg_len,sender_addr.sin_addr.s_addr,sender_addr.sin_port);
    if (len > 0) {
        ros::serialization::IStream stream((uint8_t*) recv_buffer.get(), msg_len);
        ros::serialization::deserialize(stream, odom_msg);
        std::unique_lock<std::mutex> ulck(_reconfigure_mtx);
        odom_msg.pose.pose.position.x += x_diff[remote_id - 1];
        odom_msg.pose.pose.position.y += y_diff[remote_id - 1];
        //
        // auto quat = odom_msg.pose.pose.orientation;
        ulck.unlock();
        ROS_INFO("relay slave %d odom", remote_id);
        pubs[0].publish(odom_msg);
        ros::serialization::deserialize(stream, bool_msg);
        pubs[1].publish(bool_msg);
    }
}

void reconfigureCallback(udp_relay::initialPoseConfig& config, uint32_t level) {
    std::unique_lock<std::mutex> ulck(_reconfigure_mtx);
    x_diff[0] = config.robot1_x;
    x_diff[1] = config.robot2_x;
    x_diff[2] = config.robot3_x;
    y_diff[0] = config.robot1_y;
    y_diff[1] = config.robot2_y;
    y_diff[2] = config.robot3_y;
    yaw_diff[0] = config.robot1_yaw;
    yaw_diff[1] = config.robot2_yaw;
    yaw_diff[2] = config.robot3_yaw;
}