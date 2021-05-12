// Copyright 2020 Andreas Schimpe
#pragma once

#include <vector>
#include <visualization_msgs/Marker.h>
#include <tod_msgs/ObjectData.h>
#include <tf2/LinearMath/Quaternion.h>

namespace ObjectListHelper {

    inline void create_markers_from_objects(const std::vector <tod_msgs::ObjectData> &objects,
                                            std::vector <visualization_msgs::Marker> &markers) {
        markers.clear();

        for (int i=0; i < objects.size(); ++i) {
            auto &cube = markers.emplace_back(visualization_msgs::Marker());
            tf2::Quaternion quaternion;
            quaternion.setRPY(0.0, 0.0, objects.at(i).yawAngle);
            cube.id = i;
            cube.type = visualization_msgs::Marker::CUBE;
            cube.scale.x = 0.1;
            cube.color.r = 1.0;
            cube.color.a = 1.0;
            cube.pose.orientation.x = quaternion.x();
            cube.pose.orientation.y = quaternion.y();
            cube.pose.orientation.z = quaternion.z();
            cube.pose.orientation.w = quaternion.w();
            cube.pose.position.x = objects.at(i).distCenterX;
            cube.pose.position.y = objects.at(i).distCenterY;
            cube.scale.x = objects.at(i).dimX;
            cube.scale.y = objects.at(i).dimY;
            cube.scale.z = objects.at(i).dimZ;
            cube.pose.position.z = -0.5 * cube.scale.z;
            cube.lifetime = ros::Duration(0.105);
        }
    }
} //namespace ObjectListHelper
