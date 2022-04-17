// Copyright 2020 Andreas Schimpe
#include "BandwidthManager.h"

BandwidthManager::BandwidthManager(ros::NodeHandle &n) :
    _nh{n},
    _nn{ros::this_node::getName()},
    _camParams{std::make_unique<tod_core::CameraParameters>(_nh)} {
    bool debug = false;
    n.getParam(_nn + "/debug", debug);
    if (debug) // print ROS_DEBUG
        if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
            ros::console::notifyLoggerLevelsChanged();

    for (const auto& cam : _camParams->get_sensors()) {
        auto stream = _streams.emplace_back(std::make_shared<VideoStream>(cam));
        if (cam.is_fisheye) {
            OcamModel mdl(cam.operator_name, _camParams->get_vehicle_id());
            stream->rawWidth = mdl.width_raw;
            stream->rawHeight = mdl.height_raw;
        } else {
            PinholeModel mdl(cam.operator_name, _camParams->get_vehicle_id());
            stream->rawWidth = mdl.width_raw;
            stream->rawHeight = mdl.height_raw;
        }
        ROS_DEBUG("%s: Initialized stream to reconfigure for %s", _nn.c_str(), stream->name.c_str());
    }

    _subStatus = _nh.subscribe("/Vehicle/Manager/status_msg", 5, &BandwidthManager::callback_status_msg, this);
    _reconfigServer.setCallback(boost::bind(&BandwidthManager::callback_bitrate_config, this, _1, _2));
}

void BandwidthManager::run() {
    ros::spin();
}

void BandwidthManager::callback_status_msg(const tod_msgs::Status &msg) {
    static bool connectedPrevious{false};
    _connected = (msg.tod_status != tod_msgs::Status::TOD_STATUS_IDLE);
    uint8_t newVidCtrlMode = msg.operator_video_rate_mode;
    if ((_connected && !connectedPrevious) || entered_control_mode_collective_or_automatic(newVidCtrlMode)) {
        ros::Duration(1.0).sleep(); // on connect, sleep to give clients time to connect
        // request current cropping on on/off setting from all cameras
        ROS_DEBUG("%s: video rate control mode %d engaged",
                  _nn.c_str(), newVidCtrlMode);
        for (auto stream : _streams) update_stream_config(stream);

        // calculate bitrate demand for each camera given current cropping and on / off setting
        for (auto stream : _streams) {
            if (stream->config.paused) {
                stream->bitrateDemand = 0;
                ROS_DEBUG("%s: %s demands a bitrate of %d", _nn.c_str(),
                          stream->name.c_str(), stream->bitrateDemand);
            } else {
                int pixelSet = stream->config.width * stream->config.height;
                int maxBitrate = stream->transitionBitrates.back();
                int pixelMax = stream->rawWidth * stream->rawHeight;
                float demand = (float) pixelSet * (float) maxBitrate / (float) pixelMax;
                stream->bitrateDemand = (int) demand;
                ROS_DEBUG("%s: for %s pixel set (%dx%d)/(%dx%d), optimal bitrate demand is bitrate (%d/%d)",
                          _nn.c_str(), stream->name.c_str(), stream->config.width,
                          stream->config.height, stream->rawWidth, stream->rawHeight,
                          stream->bitrateDemand, stream->transitionBitrates.back());
            }
        }
    }
    _currentVidCtrlMode = newVidCtrlMode;
    connectedPrevious = _connected;
}

void BandwidthManager::callback_bitrate_config(tod_video::BitrateConfig &bitrateCfg, uint32_t level) {
    if (bitrateCfg.bitrate < 0) return;

    static tod_video::BitrateConfig oldConfig;
    ROS_DEBUG("%s: received new bitrate prediction of %d", _nn.c_str(), bitrateCfg.bitrate);
    if (!_connected || _currentVidCtrlMode == tod_msgs::Status::VIDEO_RATE_CONTROL_MODE_SINGLE) {
        // catch cases which should not happen
        ROS_WARN("%s: ignoring reconfig req - not connected to operator or in mode single",
                 _nn.c_str());
        bitrateCfg = oldConfig;
        return;
    }
    allocate_bitrate_for_each_stream(bitrateCfg);
    select_scaling_factor_for_each_stream();

    // reconfigure streams with update parameters
    std::cout << std::endl;
    for (auto stream : _streams) {
        if (stream->bitrateAllocated > 0) {
            ROS_DEBUG("%s: Allocated bitrate for %s is %d of max %d yielding optimal resolution of %s",
                      _nn.c_str(), stream->name.c_str(), stream->bitrateAllocated,
                      stream->bitrateDemand, stream->scalings.at(stream->optimalResolutionIndex).c_str());
            tod_video::VideoConfig desCfg = stream->config;
            desCfg.bitrate = stream->bitrateAllocated;
            desCfg.scaling = stream->scalings.at(stream->optimalResolutionIndex);
            stream->config = reconfig_stream(desCfg);
        } else {
            ROS_INFO("%s: Stream %s demands bitrate of zero", _nn.c_str(), stream->name.c_str());
        }
    }

    oldConfig = bitrateCfg;
}

void BandwidthManager::allocate_bitrate_for_each_stream(tod_video::BitrateConfig &bitrateCfg) {
    int totalBitrateDemand{0};
    for (const auto &stream : _streams) totalBitrateDemand += stream->bitrateDemand;
    bitrateCfg.bitrate = std::min(totalBitrateDemand, bitrateCfg.bitrate);
    float compromise = (float) bitrateCfg.bitrate / (float) totalBitrateDemand;
    for (auto &stream : _streams)
        stream->bitrateAllocated = int(compromise * (float) stream->bitrateDemand);
    ROS_DEBUG("%s: total bitrate demand is %d", _nn.c_str(), totalBitrateDemand);
}

void BandwidthManager::select_scaling_factor_for_each_stream() {
    for (auto stream : _streams) {
        if (stream->bitrateAllocated == 0) continue;
        stream->optimalResolutionIndex = -1;
        int fullPx = stream->rawWidth * stream->rawHeight;
        int setPx = stream->config.width * stream->config.height;
        float cropFactor = (float) setPx / (float) fullPx;
        ROS_DEBUG("for %s, px is %d / %d = %f", stream->name.c_str(), setPx, fullPx, cropFactor);
        int allocatedBitrateWithoutCrop = int((float) stream->bitrateAllocated / cropFactor);
        for (int i=0; i < stream->scalings.size(); ++i) {
            int brLowerBound = (i == 0) ? 0 : stream->transitionBitrates.at(i-1);
            int brUpperBound = stream->transitionBitrates.at(i);
            if (brLowerBound <= allocatedBitrateWithoutCrop && allocatedBitrateWithoutCrop <= brUpperBound) {
                stream->optimalResolutionIndex = i;
                ROS_DEBUG("%s: selected %s for br %d in [%d,%d]", _nn.c_str(),
                          stream->scalings.at(stream->optimalResolutionIndex).c_str(),
                          allocatedBitrateWithoutCrop, brLowerBound, brUpperBound);
                break;
            } else {
                ROS_DEBUG("%s: did not select %s for br %d in [%d,%d]", _nn.c_str(),
                          stream->scalings.at(i).c_str(),
                          allocatedBitrateWithoutCrop, brLowerBound, brUpperBound);
            }
        }
        if (stream->optimalResolutionIndex == -1) {
            ROS_ERROR("%s: bitrate demand of %s in no bounds of quality curves - sets minimum scaling",
                      _nn.c_str(), stream->name.c_str());
            stream->optimalResolutionIndex = 0;
        }
    }
}

void BandwidthManager::update_stream_config(std::shared_ptr<VideoStream> stream2update) {
    tod_video::VideoConfig cfg;
    cfg.camera_name = stream2update->name;
    cfg.width = cfg.height = -1; // if server receives this, it replies with current cfg
    stream2update->config = reconfig_stream(cfg);
}

tod_video::VideoConfig BandwidthManager::reconfig_stream(const tod_video::VideoConfig &desiredCfg) {
    dynamic_reconfigure::ReconfigureRequest req;
    dynamic_reconfigure::ReconfigureResponse resp;
    desiredCfg.__toMessage__(req.config);
    if (!ros::service::call("/Vehicle/Video/RtspServer/set_parameters", req, resp))
        ROS_ERROR("%s: ROS service call to get video setting failed.", _nn.c_str());
    tod_video::VideoConfig actCfg;
    actCfg.__fromMessage__(resp.config);
    return actCfg;
}
bool BandwidthManager::entered_control_mode_collective_or_automatic(const uint8_t newMode) {
    return ( (_currentVidCtrlMode == tod_msgs::Status::VIDEO_RATE_CONTROL_MODE_SINGLE) &&
            ( (newMode == tod_msgs::Status::VIDEO_RATE_CONTROL_MODE_AUTOMATIC)
             || (newMode == tod_msgs::Status::VIDEO_RATE_CONTROL_MODE_COLLECTIVE) ) );
}
