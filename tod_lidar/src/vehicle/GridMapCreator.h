// Copyright 2020 Johannes Feiler
#include "ros/ros.h"
#include <tf2_ros/buffer.h>
#include <tf2/transform_datatypes.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <tf2_ros/transform_listener.h>
#include "tf2_ros/message_filter.h"
#include "message_filters/subscriber.h"
#include "std_msgs/String.h"
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <cmath>
#include "sensor_msgs/point_cloud_conversion.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point.h"
#include "nav_msgs/Odometry.h"
#include <vector>
#include <string>
#include <algorithm>
#include <memory>
#include <chrono>
#include <limits>

struct PolarPoint {
    double distance;
    double angle;
};

class GridMapCreator {
public:
    explicit GridMapCreator(ros::NodeHandle &nh);

private:
    ros::NodeHandle _nh;
    std::string _nn;
    ros::Publisher _gridMapPublisher, _gridMapVisualPublisher, _lidarPointCloudPublisher;
    ros::Subscriber _laserScanSubscriber, _odomSubscriber;
    tf2_ros::Buffer _tfBuffer;
    tf2_ros::TransformListener _tfListener;

    std::string _lidarFrameId{"none"};
    std::string _gridMapFrameId;
    grid_map::GridMap _lidarGridMap;
    std::string _occupancyLayer;
    grid_map_msgs::GridMap _gridMapRos;
    grid_map::Position _currentVehiclePosition;
    sensor_msgs::PointCloud2 _pointCloudGlobalCoord;
    std::vector<geometry_msgs::PoseStamped> _stampedPoses;
    std::vector<geometry_msgs::Point> _vehicleFixedLaserScanPoints;
    std::vector<geometry_msgs::Point> _globalLaserScanPoints;
    std::vector<geometry_msgs::Point32> _globalLaserScanPoints32;
    sensor_msgs::PointCloud _globalLaserScanPointCloud;

    bool _gridConstructed{false};
    double _gridMapWidthInM;
    double _gridMapHeightInM;
    double _gridMapCellSizeInM;
    double _mapBehindVehicleInM;
    double _lidarHeight;
    double _occupied;
    double _free;

    void processArrivedLidarScan(const sensor_msgs::LaserScan& msg);
    void initGridMap();
    grid_map::Position convertPolarToCartesianANY(const PolarPoint& polarPoint);
    void processArrivedOdomFrame(const nav_msgs::Odometry& msg);
    void convertLaserScanToCartesianCoords(const sensor_msgs::LaserScan& msg);
    geometry_msgs::Point convertPolarToCartesian(const PolarPoint& polarPoint);
    void convertVehicleFixedCartesianCoordsToGlobalCoords();
    void convertGlobalCoordsToPoint32();
    geometry_msgs::Point32 convertPointToPoint32(
        const geometry_msgs::Point& point);
    void storeAsPointCloud();
    void publishPointCloud();
    void storeIntoGridMap();
    void publishGridMap();
    grid_map::Position convertPoint32ToPosition(const geometry_msgs::Point32& point);
    bool isTransformMissing();
    void loadParamsFromParamServer();
    void printStandardRosWarn(const std::string& paramName);
};
