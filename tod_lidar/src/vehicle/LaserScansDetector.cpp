// Copyright 2020 Andreas Schimpe
#include "LaserScansDetector.h"

LaserScansDetector::LaserScansDetector(ros::NodeHandle& nodeHandle) :
    _nodeHandle(nodeHandle), _nodeName(ros::this_node::getName()),
    _xmax2consider{15.0}, _yminmax2consider{8.5},
    _maxDistanceBetweenPts{0.49}, _minNofPts{3} {
    for (int i=0; i <= 10; ++i) {
        std::string lidarName{""}, paramName{_nodeName + "/lidar" + std::to_string(i) + "/name"};

        if (_nodeHandle.getParam(paramName, lidarName)) {
            std::string is3Dname{_nodeName + "/lidar" + std::to_string(i) + "/is_3D"},
                        detectOn{_nodeName + "/lidar" + std::to_string(i) + "/detect_on"};
            bool lidarIs3d{true}, detectIsOn{false};

            if (!_nodeHandle.getParam(is3Dname, lidarIs3d))
                ROS_ERROR("%s: Object detector not supported yet for the 3D Lidar: %s.",
                          _nodeName.c_str(), is3Dname.c_str());

            if (!lidarIs3d) {
                _nodeHandle.getParam(detectOn, detectIsOn);
                if (detectIsOn) {
                    std::string paramNs{""}, paramScan{""};
                    nodeHandle.getParam(_nodeName + "/lidar_topics_namespace", paramNs);
                    nodeHandle.getParam(_nodeName + "/laser_scan_name", paramScan);

                    // lidar name for frame id without '/'
                    auto dtc = _detectors.emplace_back(
                        std::make_shared<Detector>(std::string(&lidarName.at(1), lidarName.size()-1)));
                    std::string laserScanTopic{paramNs + lidarName + paramScan};

                    std::cout << paramNs + lidarName + paramScan << std::endl;
                    dtc->subScan = _nodeHandle.subscribe<sensor_msgs::LaserScan>(
                            laserScanTopic, 1, boost::bind(&LaserScansDetector::callback_laser_scan, this, _1,
                                                           _detectors.back()));
                    std::string objectListTopic{paramNs + lidarName + "/object_list"};
                    dtc->pubObjectList = _nodeHandle.advertise<tod_msgs::ObjectList>(objectListTopic, 1);
                    std::string objectMarkerTopic{paramNs + lidarName + "/object_marker"};
                    dtc->pubVizMarker = _nodeHandle.advertise<visualization_msgs::Marker>(objectMarkerTopic, 100);
                    ROS_INFO("%s: Created detector for lidar: %s.", _nodeName.c_str(), lidarName.c_str());
                }
                else
                    continue;
            }
        }
    }
}

void LaserScansDetector::run() {
    ros::MultiThreadedSpinner spinner(uint32_t(_detectors.size())); // multi threading
    spinner.spin();
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
        if (-_yminmax2consider <= pt.y && pt.y <= _yminmax2consider &&
            0.0 <= pt.x && pt.x <= _xmax2consider) {
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
        if (distsBetweenPts.at(i) <= _maxDistanceBetweenPts) {
            newCluster.push_back(i);
        } else {
            if (newCluster.size() >= _minNofPts) {
                all_clusters.push_back(newCluster);
            }
            newCluster.clear();
            newCluster.push_back(i);
        }
    }

    if (newCluster.size() >= _minNofPts)
        all_clusters.push_back(newCluster);
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
    std::vector<visualization_msgs::Marker> markers;
    ObjectListHelper::create_markers_from_objects(msg.objectList, markers);
    for (auto& marker : markers) {
        marker.header.frame_id = detector->name;
        detector->pubVizMarker.publish(marker);
    }
}

void LaserScansDetector::get_points_in_cluster(const PointList &allPoints, const IdxList &cluster, PointList &points) {
    points.clear();
    for (int idx : cluster)
        points.emplace_back(allPoints.at(idx));
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
