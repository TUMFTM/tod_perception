// Copyright 2020 Andreas Schimpe
#include "GridMapDetector.h"

GridMapDetector::GridMapDetector(ros::NodeHandle &nodeHandle)
    : _nodeHandle{nodeHandle}, _nodeName{ros::this_node::getName()} {
    _subGridMap = _nodeHandle.subscribe<nav_msgs::OccupancyGrid>(
        "elevation_grid", 1, boost::bind(&GridMapDetector::callback_occupancy_grid, this, _1));
    _pubObjectMarker = _nodeHandle.advertise<visualization_msgs::Marker>("object_marker", 100);
    _pubObjectList = _nodeHandle.advertise<tod_msgs::ObjectList>("object_list", 5);
}

void GridMapDetector::callback_occupancy_grid(const nav_msgs::OccupancyGridConstPtr &msg) {
    struct vec2f {
        double data[2];
        double operator[](int idx) const { return data[idx]; }
    };

    auto t0 = ros::Time::now();

    // grid map to data vector with occupied grid cell coordinates
    std::vector<vec2f> data;
    data.reserve(msg->info.width * msg->info.height);
    for (size_t h = 0; h < msg->info.height; ++h) {
        for (size_t w = 0; w < msg->info.width; ++w) {
            int occupancy = msg->data.at(h * msg->info.height + w);
            if (occupancy > 0) {
                double x = msg->info.origin.position.x + msg->info.resolution * w;
                double y = msg->info.origin.position.y + msg->info.resolution * h;
                data.emplace_back(vec2f{x, y});
            }
        }
    }
    if (data.empty()) return;

    /**
    * @describe: Run DBSCAN clustering alogrithm
    * @param: V {std::vector<T>} : data
    * @param: dim {unsigned int} : dimension of T (a vector-like struct)
    * @param: eps {Float} : epsilon or in other words, radian
    * @param: min {unsigned int} : minimal number of points in epsilon radian, then the point is cluster core point
    * @param: disfunc {DistanceFunc} :!!!! only used in bruteforce mode.  Distance function recall. Euclidian distance is recommanded, but you can replace it by any metric measurement function
    * @usage: Object.Run() and get the cluster and noise indices from this->Clusters & this->Noise.
    * @pitfall: If you set big eps(search range) and huge density V, then kdtree will be a bottleneck of performance
    * @pitfall: You MUST ensure the data's identicality (TVector* V) during Run(), because DBSCAN just use the reference of data passed in.
    * @TODO: customize kdtree algorithm or rewrite it ,stop further searching when minimal number which indicates cluster core point condition is satisfied
    */
    static auto dbscan{DBSCAN<vec2f, float>()};
    int ret = dbscan.Run(&data, 2, 0.45f, 4);
    if (ret) ROS_ERROR("error in db scan clustering");
    auto dt = ros::Time::now() - t0;

    // clusters to object data
    auto clusters = dbscan.Clusters;
    tod_msgs::ObjectList objectListMsg;
    objectListMsg.header.stamp = ros::Time::now();
    objectListMsg.header.frame_id = "ftm";
    int id{0};
    for (auto& cluster : clusters) {
        double minX{10000000.0}, maxX{-10000000.0}, minY{10000000.0}, maxY{-10000000.0};
        for (size_t ptIdx : cluster) {
            const vec2f& pt = data[ptIdx];
            minX = std::min(minX, pt[0]);
            maxX = std::max(maxX, pt[0]);
            minY = std::min(minY, pt[1]);
            maxY = std::max(maxY, pt[1]);
        }
        auto& obj = objectListMsg.objectList.emplace_back(tod_msgs::ObjectData());
        obj.id = id++;
        obj.distCenterX = minX + 0.5 * (maxX - minX);
        obj.distCenterY = minY + 0.5 * (maxY - minY);
        obj.dimX = float(std::max(0.25, maxX - minX));
        obj.dimY = float(std::max(0.25, maxY - minY));
        obj.dimZ = 1.0f;
    }
    _pubObjectList.publish(objectListMsg);
    publish_object_list_marker(objectListMsg);
}

void GridMapDetector::publish_object_list_marker(const tod_msgs::ObjectList &objectList) {
    std::vector<visualization_msgs::Marker> markers;
    ObjectListHelper::create_markers_from_objects(objectList.objectList, markers);
    for (auto &marker : markers) {
        marker.header.frame_id = objectList.header.frame_id;
        marker.header.stamp = ros::Time::now();
        _pubObjectMarker.publish(marker);
    }
}
