// Copyright 2020 Johannes Feiler
#include "GridMapCreator.h"

GridMapCreator::GridMapCreator(ros::NodeHandle &nodeHandle) :
    itsTf2Listener(itsTf2Buffer), nodeHandle(nodeHandle), _nodeName(ros::this_node::getName()) {
    for (int i = 0; i <= 10; ++i) {
        const std::string ns{_nodeName + "/lidar" + std::to_string(i)};
        std::string lidarName{""}, paramName{ns + "/name"};

        if (nodeHandle.getParam(paramName, lidarName)) {
            bool lidar3D{true};
            nodeHandle.getParam(ns + "/is_3D", lidar3D);

            if (!lidar3D) {
                std::string paramGrid{""};
                bool lidarGrid{false};
                nodeHandle.getParam(ns + "/construct_gridmap_from", lidarGrid);

                if (lidarGrid) {
                    if (!gridConstructed) {
                        std::string paramNs{""}, paramScan{""};
                        nodeHandle.getParam(_nodeName + "/lidar_topics_namespace", paramNs);
                        nodeHandle.getParam(_nodeName + "/laser_scan_name", paramScan);

                        lidarPointCloudPublisher =
                                nodeHandle.advertise<sensor_msgs::PointCloud>("laser_scan_pointcloud", 1, true);

                        gridMapPublisher = nodeHandle.advertise<grid_map_msgs::GridMap>("/grid_map", 1, true);

                        lidarScanSubscriber = nodeHandle.subscribe(paramNs + lidarName + paramScan, 1,
                                                                   &GridMapCreator::processArrivedLidarScan, this);

                        odomFrameSubscriber = nodeHandle.subscribe("/odometry", 1,
                                                                   &GridMapCreator::processArrivedOdomFrame, this);
                        gridMapFrameId = "ftm";
                        loadParamsFromParamServer();
                        initGridMap();
                        ROS_INFO("%s: Constructed a gridmap with lidar: %s.", _nodeName.c_str(), lidarName.c_str());
                        gridConstructed = true;
                    } else {
                        ROS_WARN("%s: Multiple data layers are not yet supported and additional "
                                 "gridmap request for %s will be skipped!", _nodeName.c_str(), lidarName.c_str());
                        break;
                    }
                }
            }
        }
    }
}

void GridMapCreator::loadParamsFromParamServer() {
    std::string paramName = "GRID_MAP_WIDTH_IN_M";
    if (!nodeHandle.getParam(ros::this_node::getName() + "/" + paramName,
                             gridMapWidthInM)) {
        printStandardRosWarn(paramName);
        gridMapWidthInM = 20.0;
    }
    paramName = "GRID_MAP_HEIGHT_IN_M";
    if (!nodeHandle.getParam(ros::this_node::getName() + "/" + paramName,
                             gridMapHeightInM)) {
        printStandardRosWarn(paramName);
        gridMapHeightInM = 20.0;
    }
    paramName = "GRID_CELL_SIZE_IN_M";
    if (!nodeHandle.getParam(ros::this_node::getName() + "/" + paramName,
                             gridMapCellSizeInM)) {
        printStandardRosWarn(paramName);
        gridMapCellSizeInM = 0.2;
    }
    paramName = "MAP_BEHIND_VEHICLE_IN_M";
    if (!nodeHandle.getParam(ros::this_node::getName() + "/" + paramName,
                             mapBehindVehicleInM)) {
        printStandardRosWarn(paramName);
        mapBehindVehicleInM = 7;
    }
    paramName = "LIDAR_HEIGHT";
    if (!nodeHandle.getParam(ros::this_node::getName() + "/" + paramName,
                             lidarHeight)) {
        printStandardRosWarn(paramName);
        lidarHeight = 0.172;
    }
    paramName = "OCCUPIED";
    if (!nodeHandle.getParam(ros::this_node::getName() + "/" + paramName,
                             occupied)) {
        printStandardRosWarn(paramName);
        occupied = 1.0;
    }
    paramName = "FREE";
    if (!nodeHandle.getParam(ros::this_node::getName() + "/" + paramName,
                             free)) {
        printStandardRosWarn(paramName);
        free = 0.0;
    }
}

void GridMapCreator::printStandardRosWarn(const std::string &paramName) {
    ROS_WARN("Could not find %s on param server at %s. Using default value.",
             paramName.c_str(), ros::this_node::getName().c_str());
}


void GridMapCreator::initGridMap() {
    itsOccupancyLayer = "occupancyProbability";
    lidarGridMap.add(itsOccupancyLayer);
    lidarGridMap.setFrameId(gridMapFrameId);
    lidarGridMap.setGeometry(grid_map::Length(gridMapWidthInM, gridMapHeightInM),
                             gridMapCellSizeInM);
    ROS_INFO("%s: Created map with size %f x %f m (%i x %i cells).", _nodeName.c_str(),
             lidarGridMap.getLength().x(), lidarGridMap.getLength().y(),
             lidarGridMap.getSize()(0), lidarGridMap.getSize()(1));
}

void GridMapCreator::processArrivedLidarScan(const sensor_msgs::LaserScan &msg) {
    convertLaserScanToCartesianCoords(msg);
    convertVehicleFixedCartesianCoordsToGlobalCoords();
    convertGlobalCoordsToPoint32();
    storeAsPointCloud();
    publishPointCloud();
    storeIntoGridMap();
    publishGridMap();
}

void GridMapCreator::processArrivedOdomFrame(const nav_msgs::Odometry &msg) {
    geometry_msgs::TransformStamped transformStamped;
    if (isTransformMissing()) {
        return;
    }
    try {
        transformStamped = itsTf2Buffer.lookupTransform(gridMapFrameId,
                                                        msg.header.frame_id, ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
    }
    geometry_msgs::Point outPoint;
    geometry_msgs::Point inPoint = msg.pose.pose.position;
    tf2::doTransform(inPoint, outPoint, transformStamped);

    double headingSelf{0.0};
    double factor{1.0};
    if (msg.pose.pose.orientation.z > 0.0) {
        factor = 1.0;
    } else {
        factor = -1.0;
    }
    headingSelf = factor * 2 * std::acos(msg.pose.pose.orientation.w);
    double xMapBehindVehicle = std::cos(headingSelf) * mapBehindVehicleInM;
    double yMapBehindVehicle = std::sin(headingSelf) * mapBehindVehicleInM;
    currentVehiclePosition.x() = outPoint.x;
    currentVehiclePosition.y() = outPoint.y;
    lidarGridMap.move(grid_map::Position(currentVehiclePosition.x() +
                                         xMapBehindVehicle, currentVehiclePosition.y() + yMapBehindVehicle));
}

void GridMapCreator::convertLaserScanToCartesianCoords(const sensor_msgs::LaserScan &msg) {
    vehicleFixedLaserScanPoints.clear();
    PolarPoint currentPolarPoint;
    for (int rangeIterator = 0; rangeIterator < msg.ranges.size(); ++rangeIterator) {
        currentPolarPoint.angle = msg.angle_min + rangeIterator * msg.angle_increment;
        currentPolarPoint.distance = msg.ranges.at(rangeIterator);
        vehicleFixedLaserScanPoints.push_back(convertPolarToCartesian(currentPolarPoint));
    }
    lidarFrameId = msg.header.frame_id;
}

void GridMapCreator::convertVehicleFixedCartesianCoordsToGlobalCoords() {
    globalLaserScanPoints.clear();
    geometry_msgs::TransformStamped transformStamped;
    if (isTransformMissing()) {
        return;
    }
    try {
        transformStamped = itsTf2Buffer.lookupTransform(gridMapFrameId,
                                                        lidarFrameId, ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
    }

    for (auto point = vehicleFixedLaserScanPoints.begin(); point !=
                                                           vehicleFixedLaserScanPoints.end(); ++point) {
        geometry_msgs::Point outPoint;
        tf2::doTransform(*point, outPoint, transformStamped);
        globalLaserScanPoints.push_back(outPoint);
    }
}

bool GridMapCreator::isTransformMissing() {
    return !itsTf2Buffer.canTransform(gridMapFrameId, lidarFrameId, ros::Time(0));
}

void GridMapCreator::convertGlobalCoordsToPoint32() {
    globalLaserScanPoints32.clear();
    for (auto point = globalLaserScanPoints.begin(); point != globalLaserScanPoints.end();
         ++point) {
        globalLaserScanPoints32.push_back(convertPointToPoint32(*point));
    }
}

geometry_msgs::Point32 GridMapCreator::convertPointToPoint32(
        const geometry_msgs::Point &point) {
    geometry_msgs::Point32 returnPoint;
    returnPoint.x = (float) point.x;
    returnPoint.y = (float) point.y;
    returnPoint.z = (float) point.z;
    return returnPoint;
}

void GridMapCreator::storeAsPointCloud() {
    globalLaserScanPointCloud.points.clear();
    for (auto point32 = globalLaserScanPoints32.begin();
         point32 != globalLaserScanPoints32.end();
         ++point32) {
        globalLaserScanPointCloud.points.push_back(*point32);
    }
    globalLaserScanPointCloud.header.stamp = ros::Time::now();
    globalLaserScanPointCloud.header.frame_id = lidarFrameId;
}

void GridMapCreator::publishPointCloud() {
    lidarPointCloudPublisher.publish(globalLaserScanPointCloud);
}

void GridMapCreator::storeIntoGridMap() {
    for (auto laserScanPoint = globalLaserScanPointCloud.points.begin();
         laserScanPoint != globalLaserScanPointCloud.points.end(); ++laserScanPoint) {
        grid_map::Position tmpPosition = convertPoint32ToPosition(*laserScanPoint);
        if (std::isnan(tmpPosition.x()) || std::isnan(tmpPosition.y())) {
            continue;
        }
        if (lidarGridMap.isInside(tmpPosition)) {
            lidarGridMap.atPosition(itsOccupancyLayer, tmpPosition) = occupied;
        }
    }
}

grid_map::Position GridMapCreator::convertPoint32ToPosition(
        const geometry_msgs::Point32 &point) {
    grid_map::Position returnPosition;
    returnPosition.x() = static_cast<double>(point.x);
    returnPosition.y() = static_cast<double>(point.y);
    return returnPosition;
}

void GridMapCreator::publishGridMap() {
    ros::Time time = ros::Time::now();
    lidarGridMap.setTimestamp(time.toNSec());
    grid_map::GridMapRosConverter::toMessage(lidarGridMap, lidarGridMapRos);
    gridMapPublisher.publish(lidarGridMapRos);
}

geometry_msgs::Point GridMapCreator::convertPolarToCartesian(const PolarPoint &polarPoint) {
    geometry_msgs::Point currentPosition;
    if (isnan(polarPoint.distance)) {
        currentPosition.x = currentPosition.y = std::numeric_limits<double>::quiet_NaN();
    } else {
        currentPosition.x = polarPoint.distance * std::cos(polarPoint.angle);
        currentPosition.y = polarPoint.distance * std::sin(polarPoint.angle);
    }
    currentPosition.z = lidarHeight;
    return currentPosition;
}

grid_map::Position GridMapCreator::convertPolarToCartesianANY(const PolarPoint &polarPoint) {
    grid_map::Position currentPosition;
    if (isnan(polarPoint.distance)) {
        currentPosition.x() = currentPosition.y() = std::numeric_limits<double>::quiet_NaN();
    } else {
        currentPosition.x() = polarPoint.distance * std::cos(polarPoint.angle);
        currentPosition.y() = polarPoint.distance * std::sin(polarPoint.angle);
    }
    return currentPosition;
}

void GridMapCreator::addData() {
    ros::Time time = ros::Time::now();
    for (grid_map::GridMapIterator it(lidarGridMap); !it.isPastEnd(); ++it) {
        grid_map::Position position;
        lidarGridMap.getPosition(*it, position);
        lidarGridMap.at("occupancyProbability", *it) =
                -0.04 + 0.2 * std::sin(3.0 * time.toSec() + 5.0 * position.y()) * position.x();
    }
}
