// Copyright 2020 Andreas Schimpe
#include <ros/ros.h>
#include "BandwidthManager.h"
int main(int argc, char **argv) {
    ros::init(argc, argv, "VehicleBandwidthManager");
    ros::NodeHandle n;
    BandwidthManager manager(n);
    manager.run();
    return 0;
}
