#include "udp_slave.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "udp_relay");
    std::shared_ptr<UDPSlave> udpComm = std::make_shared<UDPSlave>();
    ros::Rate loop_rate(50);

    while (ros::ok()) {
        udpComm->transferData();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
