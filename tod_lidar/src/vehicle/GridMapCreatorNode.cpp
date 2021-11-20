// Copyright 2020 Johannes Feiler
#include <ros/ros.h>
#include "GridMapCreator.h"
int main(int argc, char** argv) {
    ros::init(argc, argv, "GridMapCreator");
    ros::NodeHandle nh;
    GridMapCreator gridMapCreator(nh);
    ros::spin();
    return 0;
}
