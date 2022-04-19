// Copyright 2020 Andreas Schimpe
#include "LaserScansDetector.h"

LaserScansDetector::LaserScansDetector(ros::NodeHandle& nodeHandle) :
    _nh(nodeHandle),
    _nn(ros::this_node::getName()),
    _lidarParams{std::make_unique<tod_core::LidarParameters>(_nh)} {
    for (const auto& lidarSensor : _lidarParams->get_sensors()) {
        if (!lidarSensor.is_3D && lidarSensor.detect_on) {
            std::string topicNs{_lidarParams->get_lidar_topics_namespace() + lidarSensor.name};
            // lidar name for frame id without '/'
            auto dtc = _detectors.emplace_back(
                std::make_shared<Detector>(std::string(&lidarSensor.name.at(1), lidarSensor.name.size()-1)));
            dtc->subScan = _nh.subscribe<sensor_msgs::LaserScan>(
                topicNs + _lidarParams->get_laser_scan_name(), 1,
                boost::bind(&LaserScansDetector::callback_laser_scan, this, _1, _detectors.back()));
            dtc->pubObjectList = _nh.advertise<tod_msgs::ObjectList>(topicNs + "/object_list", 1);
            dtc->pubVizMarker = _nh.advertise<visualization_msgs::Marker>(topicNs + "/object_marker", 100);
            ROS_INFO("%s: Created detector for lidar: %s.", _nn.c_str(), lidarSensor.name.c_str());
        }
    }

    _reconfigServer.setCallback(boost::bind(&LaserScansDetector::callback_reconfigure, this, _1, _2));
}

void LaserScansDetector::run() {
    ros::MultiThreadedSpinner spinner(uint32_t(_detectors.size()));
    spinner.spin();
}

void LaserScansDetector::callback_reconfigure(tod_lidar::LaserScansDetectorConfig &config, uint32_t level) {
    if (config.min_nof_pts > config.max_nof_pts) {
        ROS_WARN("%s: min_nof_pts (%d) cannot be larger than max_nof_pts (%d), set min_nof_pts to max_nof_pts",
                 _nn.c_str(), config.min_nof_pts, config.max_nof_pts);
        config.min_nof_pts = config.max_nof_pts;
    }
    if (config.x_min + 0.5 > config.x_max) {
        const double bad_x_min = config.x_min;
        config.x_min = config.x_max - 0.5;
        ROS_WARN("%s: x_min (%f) close to or larger than x_max (%f), set x_min to %f",
                 _nn.c_str(), bad_x_min, config.x_max, config.x_min);
    }
    _detectorParams = config;
}

void LaserScansDetector::callback_laser_scan(
    const sensor_msgs::LaserScanConstPtr& msg, std::shared_ptr<Detector> detector) {
    std::vector<Point> pointsCartesian;
    polar_to_cartesian(msg, pointsCartesian);
    if (pointsCartesian.empty())
        return;

    std::vector<Point> filteredPoints;
    filter_driving_corridor(pointsCartesian, filteredPoints);
    if (filteredPoints.empty())
        return;

    std::vector<std::vector<int>> all_clusters;
    find_clusters(filteredPoints, all_clusters);
    if (all_clusters.empty())
        return;

    tod_msgs::ObjectList objectListMsg;
    calc_bounding_boxes(filteredPoints, all_clusters, objectListMsg);
    objectListMsg.header.stamp = ros::Time::now();
    objectListMsg.header.frame_id = detector->name;
    detector->pubObjectList.publish(objectListMsg);
    publish_object_list_marker(objectListMsg, detector);
}

void LaserScansDetector::polar_to_cartesian(const sensor_msgs::LaserScanConstPtr& msg, PointList &pointsCartesian) {
    pointsCartesian.clear();
    for (int i=0; i < msg->ranges.size(); ++i) {
        Point& pt = pointsCartesian.emplace_back(Point());
        double angle = msg->angle_min + i*msg->angle_increment;
        double range = msg->ranges.at(i);
        if (isnan(range)) {
            pt.x = pt.y = std::numeric_limits<double>::quiet_NaN();
        } else {
            pt.x = range*std::cos(angle);
            pt.y = range*std::sin(angle);
        }
    }
}

void LaserScansDetector::filter_driving_corridor(const PointList &points2filter, PointList &filteredPoints) {
    filteredPoints.clear();
    for (const Point &pt : points2filter) {
        if (-_detectorParams.y_min_max <= pt.y && pt.y <= _detectorParams.y_min_max &&
            _detectorParams.x_min <= pt.x && pt.x <= _detectorParams.x_max) {
            filteredPoints.emplace_back(pt);
        }
    }
}

void LaserScansDetector::find_clusters(const PointList &points2cluster, std::vector<std::vector<int> > &all_clusters) {
    all_clusters.clear();
    std::vector<double> distsBetweenPts;
    distsBetweenPts.push_back(0.0);
    for (int i=1; i < points2cluster.size(); ++i) {
        const double distX = points2cluster.at(i).x - points2cluster.at(i-1).x;
        const double distY = points2cluster.at(i).y - points2cluster.at(i-1).y;
        const double dist = std::sqrt(distX*distX + distY*distY);
        distsBetweenPts.push_back(dist);
    }

    std::vector<int> newCluster;
    for (int i=0; i < points2cluster.size(); ++i) {
        if (distsBetweenPts.at(i) <= _detectorParams.max_distance) {
            newCluster.push_back(i);
        } else {
            if (_detectorParams.min_nof_pts <= newCluster.size() && newCluster.size() <= _detectorParams.max_nof_pts) {
                all_clusters.push_back(newCluster);
            }
            newCluster.clear();
            newCluster.push_back(i);
        }
    }

    if (_detectorParams.min_nof_pts <= newCluster.size() && newCluster.size() <= _detectorParams.max_nof_pts) {
        all_clusters.push_back(newCluster);
    }
}

void LaserScansDetector::calc_bounding_boxes(
    const PointList &points, const std::vector<IdxList> &clusters, tod_msgs::ObjectList &msg) {
    msg.objectList.clear();
    for (int i=0; i < clusters.size(); ++i) {
        std::vector<Point> pointsInCluster;
        get_points_in_cluster(points, clusters.at(i), pointsInCluster);
        tod_msgs::ObjectData &object = msg.objectList.emplace_back(tod_msgs::ObjectData());
        create_object(pointsInCluster, i, object);
    }
}

void LaserScansDetector::publish_object_list_marker(
    const tod_msgs::ObjectList &msg, std::shared_ptr<Detector> detector) {
    std::vector<visualization_msgs::Marker> markers = tod_helper::ObjectList::to_marker_vector(msg.objectList);
    for (auto& marker : markers) {
        marker.header.frame_id = detector->name;
        detector->pubVizMarker.publish(marker);
    }
}

void LaserScansDetector::get_points_in_cluster(const PointList &allPoints, const IdxList &cluster, PointList &points) {
    points.clear();
    for (int idx : cluster) {
        points.emplace_back(allPoints.at(idx));
    }
}

void LaserScansDetector::create_object(
    const PointList &pointsInCluster, const int clusterIndex, tod_msgs::ObjectData &object) {
    const Point &ptLeft = pointsInCluster.front();
    const Point &ptRight = pointsInCluster.back();
    Point tangent{ptRight.x - ptLeft.x,
                  ptRight.y - ptLeft.y};

    Point tangentCenter{ptLeft.x + 0.5 * tangent.x,
                        ptLeft.y + 0.5 * tangent.y};

    double tangentLength = std::sqrt(tangent.x*tangent.x + tangent.y*tangent.y);
    Point tangentNormalized{tangent.x / tangentLength,
                            tangent.y / tangentLength};

    Point normal{tangentNormalized.y, -tangentNormalized.x};

    Point clusterCenter{tangentCenter.x + 0.5 * tangentLength * normal.x,
                        tangentCenter.y + 0.5 * tangentLength * normal.y};

    double objectLength{0.0};
    for (const Point &pt : pointsInCluster) {
        Point distances{pt.x - clusterCenter.x,
                        pt.y - clusterCenter.y};
        double dist = std::sqrt(distances.x*distances.x + distances.y*distances.y);
        objectLength = (2*dist >= objectLength) ? 2*dist : objectLength;
    }

    object.distCenterX = float(clusterCenter.x);
    object.distCenterY = float(clusterCenter.y);
    object.yawAngle = float(std::atan2(normal.y, normal.x));
    object.dimY = float(tangentLength);
    object.dimX = float(objectLength);
    object.dimZ = 1.0;
    object.id = clusterIndex;
}
