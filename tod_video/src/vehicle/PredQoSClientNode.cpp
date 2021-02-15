// Copyright 2020 Andreas Schimpe
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/NavSatFix.h>

#include <tod_video/BitrateConfig.h>
#include "tod_msgs/StatusMsg.h"
#include "tod_msgs/connectionStatus.h"
#include "tod_msgs/controlMode.h"
#include "KDTree/KDTree.hpp"
#include <map>
#include <dynamic_reconfigure/server.h>


static bool _connectedToOperator{false};
static bool _inAutomaticMode{false};
static sensor_msgs::NavSatFixConstPtr _gpsMsg{nullptr};

void gps_to_euclid(const double lon_deg, const double lat_deg, geometry_msgs::Point &euclid);
void gps_to_euclid(const sensor_msgs::NavSatFixConstPtr &gpsMsg, geometry_msgs::Point &euclid);
int min_prediction_in_radius_of_gps_position(const sensor_msgs::NavSatFixConstPtr &gpsMsg,
                                             const double radius, KDTree::KDTree &tree,
                                             std::map<size_t, int> indexedBitrateMap);
bool init_bandwidth_map_gps(ros::NodeHandle &n, KDTree::KDTree &tree, std::map<size_t, int> &predictions);


int main(int argc, char **argv) {
    ros::init(argc, argv, "VehiclePredQoSClient");
    ros::NodeHandle n;
    std::string nodeName = ros::this_node::getName();

    bool debug = false;
    n.getParam(nodeName + "/debug", debug);
    if (debug) // print ROS_DEBUG
        if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
            ros::console::notifyLoggerLevelsChanged();

    // initialize bandwidth map
    // gps positions as KDTree and indexed map for prediction values
    std::map<size_t, int> predictions;
    KDTree::KDTree tree;
    if (!init_bandwidth_map_gps(n, tree, predictions)) {
        ROS_ERROR("%s: initialization of KDTree failed - terminating node!",
                  nodeName.c_str());
        return 1;
    }

    ros::Subscriber subStatusMsg = n.subscribe<tod_msgs::StatusMsg>(
        "/Vehicle/Manager/status_msg", 5, [&](const auto &msg) {
            _connectedToOperator = (ConnectionStatus(msg->tod_status) != ConnectionStatus::IDLE);
            _inAutomaticMode = (VideoRateControlMode(msg->operator_video_rate_mode)
                                      == VideoRateControlMode::AUTOMATIC);
        });
    ros::Subscriber subGps = n.subscribe<sensor_msgs::NavSatFix>(
        "/Vehicle/VehicleConnection/gps/fix", 5, [&](const auto &msg) {
            _gpsMsg = msg;
        });
    ros::Publisher pubBrPred = n.advertise<geometry_msgs::PointStamped>("bitratePredOnGps", 5);

    // check periodically for change in bandwidth prediction
    ros::Rate r(20);
    while (ros::ok()) {
        ros::spinOnce();
        static bool prevInAutomaticMode{false};
        if (_gpsMsg && _connectedToOperator && _inAutomaticMode) {
            static int previousPrediction{0};
            double radius = 20.0; // could be a function of current vehicle speed
            int currentPrediction = min_prediction_in_radius_of_gps_position(
                _gpsMsg, radius, tree, predictions);

            // reconfigure BandWidthManager on change in prediction
            if (currentPrediction != previousPrediction
                || (!prevInAutomaticMode && _inAutomaticMode)) {
                tod_video::BitrateConfig cfg;
                cfg.bitrate = currentPrediction;
                dynamic_reconfigure::ReconfigureRequest req;
                dynamic_reconfigure::ReconfigureResponse resp;
                cfg.__toMessage__(req.config);
                ROS_DEBUG("%s: calling pQoS reconfigure with available bitrate of %d",
                          nodeName.c_str(), cfg.bitrate);
                if (!ros::service::call("/Vehicle/Video/BandwidthManager/set_parameters", req, resp))
                    ROS_ERROR("%s: calling ros service to set available bitrate of %d failed",
                              nodeName.c_str(), cfg.bitrate);
            }

            // publish current prediction at gps location
            geometry_msgs::PointStamped predMsg;
            predMsg.header.frame_id = _gpsMsg->header.frame_id;
            predMsg.header.stamp = ros::Time::now();
            predMsg.point.x = _gpsMsg->longitude;
            predMsg.point.y = _gpsMsg->latitude;
            predMsg.point.z = currentPrediction;
            pubBrPred.publish(predMsg);

            previousPrediction = currentPrediction;
            _gpsMsg.reset();
        }
        prevInAutomaticMode = _inAutomaticMode;
        r.sleep();
    }

    return 0;
}

void gps_to_euclid(const double lon_deg, const double lat_deg, geometry_msgs::Point &euclid) {
    static const double R = 6371000.0; // m
    double lon_rad = 3.1415 * lon_deg / 180.0;
    double lat_rad = 3.1415 * lat_deg / 180.0;
    euclid.x = R * std::cos(lat_rad) * std::cos(lon_rad);
    euclid.y = R * std::cos(lat_rad) * std::sin(lon_rad);
    euclid.z = R * std::sin(lat_rad);
}

void gps_to_euclid(const sensor_msgs::NavSatFixConstPtr &gpsMsg, geometry_msgs::Point &euclid) {
    gps_to_euclid(gpsMsg->longitude, gpsMsg->latitude, euclid);
}

int min_prediction_in_radius_of_gps_position(const sensor_msgs::NavSatFixConstPtr &gpsMsg,
                                             const double radius, KDTree::KDTree &tree,
                                             std::map<size_t, int> indexedBitrateMap) {
    geometry_msgs::Point ptEuclid;
    gps_to_euclid(gpsMsg, ptEuclid);
    KDTree::point_t pt{ptEuclid.x, ptEuclid.y, ptEuclid.z};
    auto idxs = tree.neighborhood_indices(pt, radius);
    int minPred{1000000};
    for (const auto idx : idxs) {
        minPred = (indexedBitrateMap[idx] < minPred)
                      ? indexedBitrateMap[idx] : minPred;
    }
    return minPred;
}

bool init_bandwidth_map_gps(ros::NodeHandle &n, KDTree::KDTree &tree, std::map<size_t, int> &predictions) {
    // get bandwidth map values from ros parameter server
    std::string nodeName = ros::this_node::getName();
    int bwMapRows{0}, bwMapCols{0};
    std::vector<double> bwMapData;
    if (!n.getParam(nodeName + "/bandwidth_map/cols", bwMapCols)
        || !n.getParam(nodeName + "/bandwidth_map/rows", bwMapRows)
        || !n.getParam(nodeName + "/bandwidth_map/data", bwMapData)) {
        ROS_ERROR("%s: could not get rows, cols or data from bandwidth map - terminating",
                  nodeName.c_str());
        return false;
    }
    ROS_DEBUG("%s: bw map data of size %d has %d rows and %d cols",
              nodeName.c_str(), bwMapData.size(), bwMapRows, bwMapCols);
    for (int i=0; i < 3*5; i = i + 3) {
        ROS_DEBUG("%s: BW Map Point %d at (%f,%f) has prediction %d", nodeName.c_str(),
                  i, bwMapData.at(i), bwMapData.at(i+1), int(bwMapData.at(i+2)));
    }

    // initialize KDTree and std::map
    KDTree::pointVec euclidPoints;
    for (size_t i=0; i < bwMapRows; ++i) {
        geometry_msgs::Point ptEuclid;
        gps_to_euclid(bwMapData.at(i*bwMapCols), bwMapData.at(i*bwMapCols + 1), ptEuclid);
        KDTree::point_t pt = {ptEuclid.x, ptEuclid.y, ptEuclid.z};
        euclidPoints.push_back(pt);
        predictions[i] = int(bwMapData.at(i * bwMapCols + 2));
        ROS_DEBUG("%s: put node %d in map at (%f,%f) with prediction %d", nodeName.c_str(),
                  i, euclidPoints.back().at(0), euclidPoints.back().at(1), predictions[i]);
    }
    tree = KDTree::KDTree(euclidPoints);
    return true;
}
