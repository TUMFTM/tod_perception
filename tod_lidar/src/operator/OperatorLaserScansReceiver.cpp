// Copyright 2020 Andreas Schimpe
#include "tod_network/tod_receiver.h"
#include <sensor_msgs/LaserScan.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "LaserScanReceiver");
    ros::NodeHandle n;
    tod_network::Receiver<sensor_msgs::LaserScan> receiver(n);
    std::string nodeName = ros::this_node::getName();

    int portFrom = tod_network::OperatorPorts::RX_LIDAR_DATA_RANGE_FROM;
    int portTo = tod_network::OperatorPorts::RX_LIDAR_DATA_RANGE_TO;
    int lidarCount{0};
    for (int i=0; i <= portTo - portFrom; ++i) {
        std::string name{""}, paramScan{""};
        bool is3D{false};
        if (n.getParam(std::string(nodeName + "/lidar" + std::to_string(i) + "/name"), name) &&
                n.getParam(std::string(nodeName + "/lidar" + std::to_string(i) + "/is_3D"), is3D)) {
            if (!is3D) {
                name.erase(name.begin());
                n.getParam(nodeName + "/laser_scan_name", paramScan);
                receiver.add_processer(name + paramScan,
                                       tod_network::OperatorPorts::RX_LIDAR_DATA_RANGE_FROM + lidarCount);
            }
            ++lidarCount;
        }
    }

    receiver.run();
    return 0;
}
