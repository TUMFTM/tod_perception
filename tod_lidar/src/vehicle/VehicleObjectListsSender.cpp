// Copyright 2020 Andreas Schimpe
#include "tod_network/tod_sender.h"
#include <tod_msgs/ObjectList.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "VehicleObjectListsSender");
    ros::NodeHandle n;
    tod_network::Sender<tod_msgs::ObjectList> sender(n, true);
    std::string nodeName = ros::this_node::getName();

    int portFrom = tod_network::OperatorPorts::RX_LIDAR_OBJECTS_RANGE_FROM;
    int portTo = tod_network::OperatorPorts::RX_LIDAR_OBJECTS_RANGE_TO;
    int lidarCount{0};
    for (int i=0; i <= portTo - portFrom; ++i) {
        std::string name{""};
        bool is3D{false};
        if (n.getParam(std::string(nodeName + "/lidar" + std::to_string(i) + "/name"), name) &&
                n.getParam(std::string(nodeName + "/lidar" + std::to_string(i) + "/is_3D"), is3D)) {
            if (!is3D) {
                name.erase(name.begin());
                sender.add_processer(name + "/object_list",
                                     tod_network::OperatorPorts::RX_LIDAR_OBJECTS_RANGE_FROM + lidarCount);
            }
            ++lidarCount;
        }
    }

    sender.run();

    return 0;
}
