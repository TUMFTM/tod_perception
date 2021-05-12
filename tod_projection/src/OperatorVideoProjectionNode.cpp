// Copyright 2020 Andreas Schimpe
#include <ros/ros.h>
#include "OperatorVideoProjection.h"

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "OperatorVideoProjection");
    ros::NodeHandle nodeHandle;
    VehicleLaneProjection projection(nodeHandle);
    projection.run();
    return 0;
}
