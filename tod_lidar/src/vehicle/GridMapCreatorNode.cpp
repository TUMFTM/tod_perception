// Copyright 2020 Johannes Feiler
#include <ros/ros.h>
#include "GridMapCreator.h"
int main(int argc, char** argv) {
    ros::init(argc, argv, "ToDGridMap");
    ros::NodeHandle nh("~");
    GridMapCreator toDGridMap(nh);
    ros::Rate rate(100.0);
    while (nh.ok()) {
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
