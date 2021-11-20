// Copyright 2021 Andreas Schimpe
#pragma once
#include <ros/ros.h>
#include <string>
#include <vector>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <nav_msgs/Path.h>
#include <tod_msgs/Status.h>
#include <tod_helper/camera_models/PinholeModel.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <sstream>
#include <memory>
#include <numeric>
#include <mutex>
#include <thread>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <chrono>
#include <map>

enum ProjectionType {
    POLYLINES = 0,
    CIRCLE = 1,
    NONE = 99
};

class VehicleLaneProjection {
public:
    explicit VehicleLaneProjection(ros::NodeHandle& nodeHandle);
    ~VehicleLaneProjection() {}
    void run();

private:
    struct ProjectionElement;
    struct ProjectionStream;
    typedef std::vector<geometry_msgs::PoseStamped> LaserScan;

    std::string _nodeName{""};
    nav_msgs::Path _vehicleLaneLeft, _vehicleLaneRight;
    std::vector<std::shared_ptr<LaserScan>> _scansToProject;
    tod_msgs::Status _statusMsg;
    tf2_ros::Buffer _tfBuffer;
    tf2_ros::TransformListener _transformListener;
    std::mutex _mutex;
    std::map<std::string, ros::Subscriber> _subMap;
    std::vector<std::shared_ptr<ProjectionStream>> _projections;
    bool _projectVehicleLanes{false}, _projectLaserScans{false};

    void project_stream_loop(std::shared_ptr<ProjectionStream> stream);
    bool project_on_stream(std::shared_ptr<ProjectionStream> stream, sensor_msgs::Image &out_msg);
    bool project_elements(const std::shared_ptr<ProjectionStream> stream, std::vector<ProjectionElement> &elements,
                          sensor_msgs::Image &out_msg);
    bool calc_px_coords_for_pose_vector(std::shared_ptr<ProjectionStream> stream,
                                        const LaserScan &poses,
                                        std::vector<cv::Point> &pointsPX);
    void callback_raw_image(const sensor_msgs::ImageConstPtr& msg, std::shared_ptr<ProjectionStream> stream);
    void callback_laser_scan_msg(const sensor_msgs::LaserScanConstPtr &msg,
                                 std::shared_ptr<LaserScan> scanToProject);
    void callback_path(const nav_msgs::PathConstPtr &msg, nav_msgs::Path* _path2set) {
        std::lock_guard<std::mutex> lock(_mutex);
        *_path2set = *msg;
    }
    void callback_status_msg(const tod_msgs::StatusConstPtr &msg) { _statusMsg = *msg; }

    bool hasVehicleLanes() { return (_vehicleLaneLeft.poses.size() && _vehicleLaneRight.poses.size()); }
    bool hasLaserScan() {
        for (auto &scan : _scansToProject)
            if (scan->size()) return true;
        return false;
    }

    struct ProjectionElement {
        std::vector<std::vector<cv::Point>> pointVectors;
        ProjectionType type{ProjectionType::NONE};
    };
    struct ProjectionStream {
        ros::Publisher publisher;
        std::string name{""};
        PinholeModel cameraMdl;
        int scaled_width{-1}, scaled_height{-1};
        double scaling_factor_width{-1.0}, scaling_factor_height{-1.0};
        ProjectionStream(const std::string &cameraName, const std::string &vehicleID)
            : name(cameraName), cameraMdl(cameraName, vehicleID) { }
    };
};
