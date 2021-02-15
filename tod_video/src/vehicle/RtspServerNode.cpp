// Copyright 2020 Andreas Schimpe
#include <ros/ros.h>
#include "RtspServer.h"
int main(int argc, char **argv) {
    ros::init(argc, argv, "VehicleRtspServer");
    ros::NodeHandle n;
    RtspServer server(n);
    server.run();
    return 0;
}
