#include "udp_slave.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "udp_relay");
    UDPSlave udpComm;
    ros::Rate loop_rate(50);

    while (ros::ok()) {
        udpComm.receiveData();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
