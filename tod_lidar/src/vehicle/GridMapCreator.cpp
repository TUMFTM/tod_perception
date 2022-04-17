// Copyright 2020 Johannes Feiler
#include "GridMapCreator.h"

GridMapCreator::GridMapCreator(ros::NodeHandle &nh) :
    _nh(nh),
    _nn(ros::this_node::getName()),
    _lidarParams{std::make_unique<tod_core::LidarParameters>(_nh)},
    _tfListener(_tfBuffer) {
    bool constructingGridMap{false};
    for (const auto& lidar : _lidarParams->get_sensors()) {
        if (lidar.construct_gridmap_from) {
            if (lidar.is_3D) {
                ROS_WARN("%s: Constructing grid map from 3D Lidar %s is not yet supported - continuing loop",
                         _nn.c_str(), lidar.name.c_str());
                continue;
            }
            if (constructingGridMap) {
                ROS_WARN("%s: Constructing grid map with additional layer %s is not yet supported - breaking loop",
                         _nn.c_str(), lidar.name.c_str());
                break;
            }

            _laserScanSubscriber = nh.subscribe(
                _lidarParams->get_lidar_topics_namespace() + lidar.name + _lidarParams->get_laser_scan_name(), 1,
                                                &GridMapCreator::processArrivedLidarScan, this);

            _odomSubscriber = nh.subscribe("/Vehicle/VehicleBridge/odometry", 1,
                                           &GridMapCreator::processArrivedOdomFrame, this);

            _lidarPointCloudPublisher = nh.advertise<sensor_msgs::PointCloud>("point_cloud", 1, true);

            _gridMapPublisher = nh.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);

            _gridMapFrameId = "ftm";
            loadParamsFromParamServer();
            initGridMap();
            constructingGridMap = true;
            ROS_INFO("%s: Constructing a gridmap from %s.", _nn.c_str(), lidar.name.c_str());
        }
    }
}

void GridMapCreator::loadParamsFromParamServer() {
    std::string paramName = "GRID_MAP_WIDTH_IN_M";
    if (!_nh.getParam(ros::this_node::getName() + "/" + paramName,
                      _gridMapWidthInM)) {
        printStandardRosWarn(paramName);
        _gridMapWidthInM = 20.0;
    }
    paramName = "GRID_MAP_HEIGHT_IN_M";
    if (!_nh.getParam(ros::this_node::getName() + "/" + paramName,
                      _gridMapHeightInM)) {
        printStandardRosWarn(paramName);
        _gridMapHeightInM = 20.0;
    }
    paramName = "GRID_CELL_SIZE_IN_M";
    if (!_nh.getParam(ros::this_node::getName() + "/" + paramName,
                      _gridMapCellSizeInM)) {
        printStandardRosWarn(paramName);
        _gridMapCellSizeInM = 0.2;
    }
    paramName = "MAP_BEHIND_VEHICLE_IN_M";
    if (!_nh.getParam(ros::this_node::getName() + "/" + paramName,
                      _mapBehindVehicleInM)) {
        printStandardRosWarn(paramName);
        _mapBehindVehicleInM = 7;
    }
    paramName = "LIDAR_HEIGHT";
    if (!_nh.getParam(ros::this_node::getName() + "/" + paramName,
                      _lidarHeight)) {
        printStandardRosWarn(paramName);
        _lidarHeight = 0.172;
    }
    paramName = "OCCUPIED";
    if (!_nh.getParam(ros::this_node::getName() + "/" + paramName,
                      _occupied)) {
        printStandardRosWarn(paramName);
        _occupied = 1.0;
    }
    paramName = "FREE";
    if (!_nh.getParam(ros::this_node::getName() + "/" + paramName,
                      _free)) {
        printStandardRosWarn(paramName);
        _free = 0.0;
    }
}

void GridMapCreator::printStandardRosWarn(const std::string &paramName) {
    ROS_WARN("Could not find %s on param server at %s. Using default value.",
             paramName.c_str(), ros::this_node::getName().c_str());
}


void GridMapCreator::initGridMap() {
    _occupancyLayer = "occupancyProbability";
    _lidarGridMap.add(_occupancyLayer);
    _lidarGridMap.setFrameId(_gridMapFrameId);
    _lidarGridMap.setGeometry(grid_map::Length(_gridMapWidthInM, _gridMapHeightInM),
                              _gridMapCellSizeInM);
    ROS_INFO("%s: Created map with size %f x %f m (%i x %i cells).", _nn.c_str(),
             _lidarGridMap.getLength().x(), _lidarGridMap.getLength().y(),
             _lidarGridMap.getSize()(0), _lidarGridMap.getSize()(1));
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
        transformStamped = _tfBuffer.lookupTransform(_gridMapFrameId,
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
    double xMapBehindVehicle = std::cos(headingSelf) * _mapBehindVehicleInM;
    double yMapBehindVehicle = std::sin(headingSelf) * _mapBehindVehicleInM;
    _currentVehiclePosition.x() = outPoint.x;
    _currentVehiclePosition.y() = outPoint.y;
    _lidarGridMap.move(grid_map::Position(_currentVehiclePosition.x() +
                                              xMapBehindVehicle, _currentVehiclePosition.y() + yMapBehindVehicle));
}

void GridMapCreator::convertLaserScanToCartesianCoords(const sensor_msgs::LaserScan &msg) {
    _vehicleFixedLaserScanPoints.clear();
    PolarPoint currentPolarPoint;
    for (int rangeIterator = 0; rangeIterator < msg.ranges.size(); ++rangeIterator) {
        currentPolarPoint.angle = msg.angle_min + rangeIterator * msg.angle_increment;
        currentPolarPoint.distance = msg.ranges.at(rangeIterator);
        _vehicleFixedLaserScanPoints.push_back(convertPolarToCartesian(currentPolarPoint));
    }
    _lidarFrameId = msg.header.frame_id;
}

void GridMapCreator::convertVehicleFixedCartesianCoordsToGlobalCoords() {
    _globalLaserScanPoints.clear();
    geometry_msgs::TransformStamped transformStamped;
    if (isTransformMissing()) {
        return;
    }
    try {
        transformStamped = _tfBuffer.lookupTransform(_gridMapFrameId,
                                                     _lidarFrameId, ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
    }

    for (auto point = _vehicleFixedLaserScanPoints.begin(); point !=
                                                            _vehicleFixedLaserScanPoints.end(); ++point) {
        geometry_msgs::Point outPoint;
        tf2::doTransform(*point, outPoint, transformStamped);
        _globalLaserScanPoints.push_back(outPoint);
    }
}

bool GridMapCreator::isTransformMissing() {
    return !_tfBuffer.canTransform(_gridMapFrameId, _lidarFrameId, ros::Time(0));
}

void GridMapCreator::convertGlobalCoordsToPoint32() {
    _globalLaserScanPoints32.clear();
    for (auto point = _globalLaserScanPoints.begin(); point != _globalLaserScanPoints.end();
         ++point) {
        _globalLaserScanPoints32.push_back(convertPointToPoint32(*point));
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
    _globalLaserScanPointCloud.points.clear();
    for (auto point32 = _globalLaserScanPoints32.begin();
         point32 != _globalLaserScanPoints32.end();
         ++point32) {
        _globalLaserScanPointCloud.points.push_back(*point32);
    }
    _globalLaserScanPointCloud.header.stamp = ros::Time::now();
    _globalLaserScanPointCloud.header.frame_id = _lidarFrameId;
}

void GridMapCreator::publishPointCloud() {
    _lidarPointCloudPublisher.publish(_globalLaserScanPointCloud);
}

void GridMapCreator::storeIntoGridMap() {
    for (auto laserScanPoint = _globalLaserScanPointCloud.points.begin();
         laserScanPoint != _globalLaserScanPointCloud.points.end(); ++laserScanPoint) {
        grid_map::Position tmpPosition = convertPoint32ToPosition(*laserScanPoint);
        if (std::isnan(tmpPosition.x()) || std::isnan(tmpPosition.y())) {
            continue;
        }
        if (_lidarGridMap.isInside(tmpPosition)) {
            _lidarGridMap.atPosition(_occupancyLayer, tmpPosition) = _occupied;
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
    _lidarGridMap.setTimestamp(time.toNSec());
    grid_map::GridMapRosConverter::toMessage(_lidarGridMap, _gridMapRos);
    _gridMapPublisher.publish(_gridMapRos);
}

geometry_msgs::Point GridMapCreator::convertPolarToCartesian(const PolarPoint &polarPoint) {
    geometry_msgs::Point currentPosition;
    if (isnan(polarPoint.distance)) {
        currentPosition.x = currentPosition.y = std::numeric_limits<double>::quiet_NaN();
    } else {
        currentPosition.x = polarPoint.distance * std::cos(polarPoint.angle);
        currentPosition.y = polarPoint.distance * std::sin(polarPoint.angle);
    }
    currentPosition.z = _lidarHeight;
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
