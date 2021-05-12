// Copyright 2020 Andreas Schimpe
#include <ros/ros.h>
#include "LaserScansDetector.h"
int main(int argc, char *argv[]) {
    ros::init(argc, argv, "LaserScansDetector");
    ros::NodeHandle nodeHandle;
    LaserScansDetector detector(nodeHandle);
    detector.run();
    return 0;
}
