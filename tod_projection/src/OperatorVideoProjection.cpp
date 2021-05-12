// Copyright 2021 Andreas Schimpe
#include "OperatorVideoProjection.h"

VehicleLaneProjection::VehicleLaneProjection(ros::NodeHandle& nodeHandle)
    : _nodeName(ros::this_node::getName()), _transformListener(_tfBuffer) {
    std::string topic{"/Operator/Manager/status_msg"};
    _subMap[topic] = nodeHandle.subscribe<tod_msgs::Status>(
        topic, 1, &VehicleLaneProjection::callback_status_msg, this);

    std::string vehicleID{""};
    if (!nodeHandle.getParam(std::string(_nodeName + "/vehicleID"), vehicleID))
        ROS_ERROR("%s: Could not get parameter vehicleID!", _nodeName.c_str());

    // intialize videos to project on
    for (int i=0; i <= 10; ++i) {
        std::string name{""};
        const std::string ns{_nodeName + "/camera" + std::to_string(i)};
        if (nodeHandle.getParam(std::string(ns + "/name"), name)) {
            bool isFisheye{true}, projectOn{false};
            if (!nodeHandle.getParam(ns + "/is_fisheye", isFisheye))
                ROS_ERROR("%s: could not get is_fisheye for %s", _nodeName.c_str(), name.c_str());
            nodeHandle.getParam(ns + "/project_on", projectOn);
            if (projectOn) {
                if (!isFisheye) {
                    name.erase(name.begin()); // removes '/'
                    auto stream = _projections.emplace_back(std::make_shared<ProjectionStream>(name, vehicleID));
                    std::string topic{"/Operator/Video/" + stream->name + "/image_raw"};
                    _subMap[topic] = nodeHandle.subscribe<sensor_msgs::Image>(
                        topic, 1, boost::bind(&VehicleLaneProjection::callback_raw_image, this, _1, stream));
                    stream->publisher = nodeHandle.advertise<sensor_msgs::Image>(stream->name + "/image_raw", 1);
                    ROS_INFO("%s: Projecting on %s", _nodeName.c_str(), topic.c_str());
                } else {
                    ROS_WARN("%s: Video projection not yet supported for fish eye camera %s",
                             _nodeName.c_str(), name.c_str());
                }
            }
        }
    }

    nodeHandle.getParam(_nodeName + "/do_project_vehicle_lane", _projectVehicleLanes);
    if (_projectVehicleLanes) {
        std::string topicFL{"vehicle_lane_front_left"};
        _subMap[topicFL] = nodeHandle.subscribe<nav_msgs::Path>(
            topicFL, 1, boost::bind(&VehicleLaneProjection::callback_path, this, _1, &_vehicleLaneLeft));
        std::string topicFR{"vehicle_lane_front_right"};
        _subMap[topicFR] = nodeHandle.subscribe<nav_msgs::Path>(
            topicFR, 1, boost::bind(&VehicleLaneProjection::callback_path, this, _1, &_vehicleLaneRight));
    }

    nodeHandle.getParam(_nodeName + "/do_project_laser_scans", _projectLaserScans);
    if (_projectLaserScans) {
        for (int i=0; i <= 10; ++i) {
            std::string name{""};
            bool is3d{false};
            if (nodeHandle.getParam(std::string(_nodeName + "/lidar" + std::to_string(i) + "/name"), name)) {
                if (!nodeHandle.getParam(std::string(_nodeName + "/lidar" + std::to_string(i) + "/is_3D"), is3d))
                    ROS_WARN("%s: could not get param is_3D for lidar %s", _nodeName.c_str(), name.c_str());
                if (!is3d) {
                    std::string topic{"/Operator/Lidar/" + name + "/scan"};
                    auto scanToProject = _scansToProject.emplace_back(std::make_shared<LaserScan>());
                    _subMap[topic] = nodeHandle.subscribe<sensor_msgs::LaserScan>(
                        topic, 1, boost::bind(&VehicleLaneProjection::callback_laser_scan_msg,
                                              this, _1, scanToProject));
                }
            }
        }
    }
}

void VehicleLaneProjection::run() {
    std::vector<std::thread> threads;
    for (auto stream : _projections)
        threads.emplace_back(std::thread(&VehicleLaneProjection::project_stream_loop, this, stream));
    ros::spin();
    for (auto &thread : threads) thread.join();
}

void VehicleLaneProjection::project_stream_loop(std::shared_ptr<ProjectionStream> stream) {
    ros::Rate r(20);
    sensor_msgs::Image out_msg;
    out_msg.data.reserve(stream->cameraMdl.width_raw * stream->cameraMdl.height_raw);
    while (ros::ok()) {
        if (_statusMsg.tod_status != tod_msgs::Status::TOD_STATUS_IDLE) {
            out_msg.step = stream->scaled_width;
            out_msg.height = stream->scaled_height;
            out_msg.width = stream->scaled_width;
            if (project_on_stream(stream, out_msg)) {
                out_msg.encoding = sensor_msgs::image_encodings::MONO8;
                out_msg.header.frame_id = stream->name;
                out_msg.header.stamp = ros::Time::now();
                stream->publisher.publish(out_msg);
            }
        }
        r.sleep();
    }
}

bool VehicleLaneProjection::project_on_stream(std::shared_ptr<ProjectionStream> stream, sensor_msgs::Image &out_msg) {
    if (stream->scaled_width < 0 || stream->scaled_height < 0)
        return false;
    cv::Rect roi(0, 0, stream->scaled_width, stream->scaled_height);

    std::vector<ProjectionElement> projectionElements;
    if (_projectVehicleLanes && hasVehicleLanes()) {
        std::vector<cv::Point> leftLanePX, rightLanePX;
        std::lock_guard<std::mutex> lock(_mutex);
        bool canProjectLeft = calc_px_coords_for_pose_vector(stream, _vehicleLaneLeft.poses, leftLanePX);
        bool canProjectRight = calc_px_coords_for_pose_vector(stream, _vehicleLaneRight.poses, rightLanePX);
        if (canProjectLeft && canProjectRight) {
            projectionElements.emplace_back(ProjectionElement());
            projectionElements.back().type = ProjectionType::POLYLINES;
            projectionElements.back().pointVectors.push_back(leftLanePX);
            projectionElements.back().pointVectors.push_back(rightLanePX);
        }
    }

    if (_projectLaserScans && hasLaserScan()) {
        std::lock_guard<std::mutex> lock(_mutex);
        for (const auto& scanToProject : _scansToProject) {
            std::vector<cv::Point> laserScanPX;
            bool canProjectLaserScan = calc_px_coords_for_pose_vector(
                stream, *scanToProject, laserScanPX);
            if (canProjectLaserScan) {
                projectionElements.emplace_back(ProjectionElement());
                projectionElements.back().type = ProjectionType::CIRCLE;
                projectionElements.back().pointVectors.push_back(laserScanPX);
            }
        }
    }

    bool ret = project_elements(stream, projectionElements, out_msg);
    return ret;
}


bool VehicleLaneProjection::project_elements(const std::shared_ptr<ProjectionStream> stream,
                                             std::vector<ProjectionElement>& elements,
                                             sensor_msgs::Image &out_msg) {
    cv::Mat mask = cv::Mat::zeros(out_msg.height, out_msg.width, CV_8UC1);
    for (auto &element : elements) {
        if (element.type == ProjectionType::POLYLINES) {
            const int nofLines = int(element.pointVectors.size());
            std::vector<int> npt(nofLines);
            std::vector<cv::Point*> ppt(nofLines);
            for (int i=0; i < nofLines; ++i) {
                ppt.at(i) = &element.pointVectors.at(i).at(0);
                npt.at(i) = int(element.pointVectors.at(i).size());
            }
            cv::polylines(mask, ppt.data(), npt.data(), nofLines, false,
                          cv::Scalar(255), int(stream->scaling_factor_width * 6.0));
        } else if (element.type == ProjectionType::CIRCLE) {
            for (const auto &pointVector : element.pointVectors) {
                for (const auto &pt : pointVector) {
                    cv::circle(mask, pt, int(stream->scaling_factor_width * 3.0),
                               cv::Scalar(255), int(stream->scaling_factor_width * 4.0));
                }
            }
        }
    }
    out_msg.data = std::vector<u_char>(
        &mask.at<uint8_t>(0), &mask.at<uint8_t>(0) + out_msg.width * out_msg.height);
    return true;
}

bool VehicleLaneProjection::calc_px_coords_for_pose_vector(
    std::shared_ptr<ProjectionStream> stream, const LaserScan &poses,
    std::vector<cv::Point> &pointsPX) {
    for (auto &pose : poses)  {
        geometry_msgs::PoseStamped poseOutRight;
        try {
            poseOutRight = _tfBuffer.transform(pose, stream->name);
        } catch (tf2::TransformException &ex) {
            ROS_WARN("%s: Transform of PoseStamped failed with %s.", _nodeName.c_str(), ex.what());
            return false;
        }
        if (!(poseOutRight.pose.position.z > 0.0)) continue; // only z > can be projected

        int x_px, y_px;
        if (stream->cameraMdl.point_on_image(poseOutRight.pose.position, x_px, y_px,
                                             stream->scaling_factor_width, stream->scaling_factor_height))
            pointsPX.emplace_back(cv::Point(x_px, y_px));
    }
    if (!pointsPX.size()) return false;
    return true;
}

void VehicleLaneProjection::callback_raw_image(const sensor_msgs::ImageConstPtr& msg,
                                               std::shared_ptr<ProjectionStream> stream) {
    stream->scaling_factor_width = (double) msg->width / stream->cameraMdl.width_raw;
    stream->scaling_factor_height = (double) msg->height / stream->cameraMdl.height_raw;
    stream->scaled_width = msg->width;
    stream->scaled_height = msg->height;
}

void VehicleLaneProjection::callback_laser_scan_msg(const sensor_msgs::LaserScanConstPtr &msg,
                                                    std::shared_ptr<LaserScan> scanToProject) {
    std::lock_guard lock(_mutex);
    scanToProject->clear();
    for (int i=0; i < msg->ranges.size(); ++i) {
        double range = msg->ranges.at(i);
        if (!isnan(range)) {
            double angle = msg->angle_min + i * msg->angle_increment;
            auto& pose = scanToProject->emplace_back(geometry_msgs::PoseStamped());
            pose.header.seq = msg->header.seq;
            pose.header.frame_id = msg->header.frame_id;
            pose.header.stamp = ros::Time::now();
            pose.pose.orientation.w = 1.0;
            pose.pose.position.x = range * std::cos(angle);
            pose.pose.position.y = range * std::sin(angle);
            pose.pose.position.z = 0.0;
        }
    }
}
