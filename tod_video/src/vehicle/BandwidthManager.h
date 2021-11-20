// Copyright 2020 Andreas Schimpe
#pragma once
#include <ros/ros.h>
#include <tod_msgs/Status.h>
#include <dynamic_reconfigure/server.h>
#include <tod_video/BitrateConfig.h>
#include <tod_video/VideoConfig.h>
#include <tod_helper/camera_models/PinholeModel.h>
#include <tod_helper/camera_models/OcamModel.h>
#include <numeric>
#include <algorithm>
#include <string>
#include <vector>
#include <memory>

class BandwidthManager {
private:
    struct VideoStream {
        std::string name;
        int rawWidth{-1}, rawHeight{-1};
        std::vector<int> transitionBitrates;
        std::vector<std::string> scalings;
        tod_video::VideoConfig config;
        int bitrateDemand{0}, bitrateAllocated{0};
        int optimalResolutionIndex{-1};
        explicit VideoStream(const std::string &myName) : name(myName) { }
    };

public:
    explicit BandwidthManager(ros::NodeHandle &n);
    ~BandwidthManager() { }
    void run();

private:
    ros::NodeHandle &_nodeHandle;
    ros::Subscriber _subStatus;
    dynamic_reconfigure::Server<tod_video::BitrateConfig> _reconfigServer;
    std::string _nodeName;
    std::vector<std::shared_ptr<VideoStream>> _streams;
    bool _connected{false};
    uint8_t _currentVidCtrlMode{tod_msgs::Status::VIDEO_RATE_CONTROL_MODE_SINGLE};

    void callback_status_msg(const tod_msgs::Status &msg);
    void callback_bitrate_config(tod_video::BitrateConfig &bitrateCfg, uint32_t level);
    void allocate_bitrate_for_each_stream(tod_video::BitrateConfig &bitrateCfg);
    void select_scaling_factor_for_each_stream();
    void update_stream_config(std::shared_ptr<VideoStream> stream2update);
    tod_video::VideoConfig reconfig_stream(const tod_video::VideoConfig &desiredCfg);
    bool entered_control_mode_collective_or_automatic(const uint8_t newMode);
};
