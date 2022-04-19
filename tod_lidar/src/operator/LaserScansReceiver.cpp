// Copyright 2020 Andreas Schimpe
#include "tod_network/tod_receiver.h"
#include <sensor_msgs/LaserScan.h>
#include <tod_core/LidarParameters.h>
#include <memory>

std::unique_ptr<tod_core::LidarParameters> _lidarParams{nullptr};

int main(int argc, char **argv) {
    ros::init(argc, argv, "LaserScanReceiver");
    ros::NodeHandle nh;
    _lidarParams = std::make_unique<tod_core::LidarParameters>(nh);
    tod_network::Receiver<sensor_msgs::LaserScan> receiver(nh);
    for (size_t lidarCount = 0; lidarCount < _lidarParams->get_sensors().size(); ++lidarCount) {
        const auto& lidar = _lidarParams->get_sensors().at(lidarCount);
        if (!lidar.is_3D) {
            std::string name = lidar.name;
            name.erase(name.begin());
            receiver.add_processer(name + _lidarParams->get_laser_scan_name(),
                                   tod_network::OperatorPorts::RX_LIDAR_DATA_RANGE_FROM + lidarCount);
        }
    }
    receiver.run();
    return 0;
}
