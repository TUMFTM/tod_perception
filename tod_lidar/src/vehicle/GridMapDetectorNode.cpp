// Copyright 2020 Andreas Schimpe
#include <ros/ros.h>
#include "GridMapDetector.h"
int main(int argc, char** argv) {
    ros::init(argc, argv, "GridMapDetector");
    ros::NodeHandle nodeHandle;
    GridMapDetector detector(nodeHandle);
    detector.run();
    return 0;
}
