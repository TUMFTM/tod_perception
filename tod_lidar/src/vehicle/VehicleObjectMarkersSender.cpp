// Copyright 2020 Andreas Schimpe
#include "tod_network/tod_sender.h"
#include <visualization_msgs/Marker.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "VehicleObjectMarkersSender");
    ros::NodeHandle n;
    tod_network::Sender<visualization_msgs::Marker> sender(n, true);
    std::string nodeName = ros::this_node::getName();

    int portFrom = tod_network::OperatorPorts::RX_LIDAR_OBJECT_MARKER_RANGE_FROM;
    int portTo = tod_network::OperatorPorts::RX_LIDAR_OBJECT_MARKER_RANGE_TO;
    int lidarCount{0};
    for (int i=0; i <= portTo - portFrom; ++i) {
        std::string name{""};
        bool is3D{false};
        if (n.getParam(std::string(nodeName + "/lidar" + std::to_string(i) + "/name"), name) &&
                n.getParam(std::string(nodeName + "/lidar" + std::to_string(i) + "/is_3D"), is3D)) {
            if (!is3D) {
                name.erase(name.begin());
                sender.add_processer(name + "/object_marker",
                                     tod_network::OperatorPorts::RX_LIDAR_OBJECT_MARKER_RANGE_FROM + lidarCount);
            }
            ++lidarCount;
        }
    }

    sender.run();
    return 0;
}
