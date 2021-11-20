// Copyright 2020 Andreas Schimpe
#pragma once
#include <ros/ros.h>
#include <string>
#include <vector>
#include <limits>
#include <memory>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>
#include <tod_msgs/ObjectList.h>
#include <tod_helper/object_list/Helpers.h>

class LaserScansDetector {
public:
    explicit LaserScansDetector(ros::NodeHandle& nodeHandle);
    ~LaserScansDetector() {}
    void run();

private:
    ros::NodeHandle& _nodeHandle;
    std::string _nodeName;
    double _xmax2consider, _yminmax2consider, _maxDistanceBetweenPts;
    int _minNofPts;

    struct Detector {
        ros::Subscriber subScan;
        ros::Publisher pubObjectList, pubVizMarker;
        std::string name;
        explicit Detector(const std::string &myName) : name{myName} { }
    };
    struct Point { double x{0.0}, y{0.0}; };
    typedef std::vector<Point> PointList;
    typedef std::vector<int> IdxList;

    std::vector<std::shared_ptr<Detector>> _detectors;

    void callback_laser_scan(const sensor_msgs::LaserScanConstPtr& msg, std::shared_ptr<Detector> detector);
    void polar_to_cartesian(const sensor_msgs::LaserScanConstPtr& msg, PointList &pointsCartesian);
    void filter_driving_corridor(const PointList &points2filter, PointList &filteredPoints);
    void find_clusters(const PointList &points2cluster, std::vector<std::vector<int>> &all_clusters);
    void calc_bounding_boxes(const PointList &points, const std::vector<IdxList> &clusters, tod_msgs::ObjectList &msg);
    void publish_object_list_marker(const tod_msgs::ObjectList &msg, std::shared_ptr<Detector> detector);
    void get_points_in_cluster(const PointList &allPoints, const IdxList &cluster, PointList &points);
    void create_object(const PointList &pointsInCluster, const int clusterIndex, tod_msgs::ObjectData &object);
};
