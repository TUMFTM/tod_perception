// Copyright 2020 Andreas Schimpe
#include <ros/ros.h>
#include "RtspClients.h"
int main(int argc, char *argv[]) {
    ros::init(argc, argv, "OperatorRtspClients");
    ros::NodeHandle nodeHandle;
    RtspClients videoClients(nodeHandle);
    videoClients.run();
    return 0;
}
