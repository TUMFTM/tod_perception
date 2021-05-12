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
    explicit GridMapCreator(ros::NodeHandle &nodeHandle);

    void addData();

private:
    ros::NodeHandle nodeHandle;
    std::string _nodeName{""};
    ros::Publisher gridMapPublisher;
    ros::Publisher gridMapVisualPublisher;
    ros::Subscriber lidarScanSubscriber;
    ros::Subscriber odomFrameSubscriber;
    grid_map::GridMap lidarGridMap;
    std::string itsOccupancyLayer;
    grid_map_msgs::GridMap lidarGridMapRos;
    grid_map::Position currentVehiclePosition;
    sensor_msgs::PointCloud2 pointCloudGlobalCoord;
    std::vector<geometry_msgs::PoseStamped> itsStampedPoses;
    tf2_ros::Buffer itsTf2Buffer;
    tf2_ros::TransformListener itsTf2Listener;
    std::vector<geometry_msgs::Point> vehicleFixedLaserScanPoints;
    std::vector<geometry_msgs::Point> globalLaserScanPoints;
    std::vector<geometry_msgs::Point32> globalLaserScanPoints32;
    sensor_msgs::PointCloud globalLaserScanPointCloud;
    ros::Publisher lidarPointCloudPublisher;
    std::string lidarFrameId{"none"};
    std::string gridMapFrameId;

    bool gridConstructed{false};
    double gridMapWidthInM;
    double gridMapHeightInM;
    double gridMapCellSizeInM;
    double mapBehindVehicleInM;
    double lidarHeight;
    double occupied;
    double free;

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
    void writeGridMapIntoMeshMsgAndPublishIt();
    void loadParamsFromParamServer();
    void printStandardRosWarn(const std::string& paramName);
};
