// Copyright 2020 Andreas Schimpe
#include "tod_network/tod_receiver.h"
#include <tod_msgs/ObjectList.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "ObjectListReceiver");
    ros::NodeHandle n;
    tod_network::Receiver<tod_msgs::ObjectList> receiver(n);
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
                receiver.add_processer(name + "/object_list",
                                       tod_network::OperatorPorts::RX_LIDAR_OBJECTS_RANGE_FROM + lidarCount);
            }
            ++lidarCount;
        }
    }

    receiver.run();

    return 0;
}
