// Copyright 2020 Andreas Schimpe
#include "tod_network/tod_sender.h"
#include <tod_msgs/ObjectList.h>
#include <tod_core/LidarParameters.h>
#include <memory>

std::unique_ptr<tod_core::LidarParameters> _lidarParams{nullptr};

int main(int argc, char **argv) {
    ros::init(argc, argv, "ObjectListsSender");
    ros::NodeHandle nh;
    _lidarParams = std::make_unique<tod_core::LidarParameters>(nh);
    tod_network::Sender<tod_msgs::ObjectList> sender(nh, true);
    for (size_t lidarCount=0; lidarCount < _lidarParams->get_sensors().size(); ++lidarCount) {
        const auto& lidarSensor = _lidarParams->get_sensors().at(lidarCount);
        if (lidarSensor.detect_on) {
            std::string name = lidarSensor.name;
            name.erase(name.begin());
            sender.add_processer(name + "/object_list",
                                 tod_network::OperatorPorts::RX_LIDAR_OBJECTS_RANGE_FROM + lidarCount);
        }
    }
    sender.run();
    return 0;
}
