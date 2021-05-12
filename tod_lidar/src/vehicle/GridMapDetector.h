// Copyright 2020 Andreas Schimpe
#pragma once
#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <nav_msgs/OccupancyGrid.h>
#include <tod_msgs/ObjectList.h>
#include <visualization_msgs/Marker.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "SimpleDBSCAN/dbscan.h"
#include "tod_lidar/ObjectListHelper.h"
#include <algorithm>
#include <string>

class GridMapDetector {
public:
    explicit GridMapDetector(ros::NodeHandle &nodeHandle);
    ~GridMapDetector() { }
    void run() { ros::spin(); }

private:
    ros::NodeHandle &_nodeHandle;
    ros::Publisher _pubObjectList, _pubObjectMarker;
    ros::Subscriber _subGridMap;
    std::string _nodeName;

    void callback_occupancy_grid(const nav_msgs::OccupancyGridConstPtr &msg);
    void publish_object_list_marker(const tod_msgs::ObjectList &objectList);
};
