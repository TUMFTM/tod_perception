// Copyright 2020 Andreas Schimpe
#pragma once
#include <ros/ros.h>
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <gst/rtsp-server/rtsp-server.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <thread>
#include <mutex>
#include <tod_network/tod_network.h>
#include <tod_video/VideoConfig.h>
#include <tod_msgs/StatusMsg.h>
#include <tod_msgs/connectionStatus.h>
#include <dynamic_reconfigure/server.h>
#include <string>
#include <algorithm>
#include <vector>
#include <memory>

class RtspServer {
public:
    explicit RtspServer(ros::NodeHandle &n);
    ~RtspServer() { }
    void run();

private:
    ros::NodeHandle &_nodeHandle;
    std::string _nodeName{""};
    ros::Subscriber _subsStatus;
    dynamic_reconfigure::Server<tod_video::VideoConfig> _reconfigServer;
    struct RtspStream;
    std::vector<std::shared_ptr<RtspStream>> _streams;
    bool _connected{false};
    int _defaultBitrate{1000};

    void factory_gst_video_pipeline(std::shared_ptr<RtspStream> stream,
                                    GstRTSPMountPoints *_gstMounts);
    void push_data(std::shared_ptr<RtspStream> stream);
    void callback_raw_image(const sensor_msgs::ImageConstPtr& msg, std::shared_ptr<RtspStream> stream);
    void callback_stream_reconfig(tod_video::VideoConfig &config, uint32_t level);
    void set_bitrate(tod_video::VideoConfig &config,
                     std::shared_ptr<RtspStream> stream2reconfigure);
    void set_cropping_and_scaling(tod_video::VideoConfig &new_config,
                                  std::shared_ptr<RtspStream> stream2reconfigure);
    static void need_data(GstElement* appSrc, guint unused, RtspStream *stream);
    static void media_configure(GstRTSPMediaFactory *factory, GstRTSPMedia *media,
                                RtspStream *stream);

    struct RtspStream {
        bool clientConnected{false};
        bool newDataAvailable{false};
        bool needData{false};
        ros::Time lastNeedDataStamp{ros::Time::now()};
        int rawHeight{-1}, rawWidth{-1}, rawStep{-1};
        std::string encoding{""};
        std::vector<uint8_t> imgData;
        std::string name{""};
        ros::Subscriber subsImg;
        GstRTSPMediaFactory* factory{nullptr};
        GstElement* appsrc{nullptr};
        GstElement* encoder{nullptr};
        GstElement* videocrop{nullptr};
        GstElement* scalingFilter{nullptr};
        tod_video::VideoConfig currentConfig;
        std::mutex mutex;
        RtspStream(const std::string &myName, const int defaultBitrate) : name(myName) {
            currentConfig.camera_name = name;
            currentConfig.bitrate = defaultBitrate;
            currentConfig.offset_width = currentConfig.offset_height = 0;
            currentConfig.scaling = tod_video::Video_1p000;
        }
        void reset(const int defaultBitrate) {
            currentConfig.bitrate = defaultBitrate;
            currentConfig.width = rawWidth;
            currentConfig.height = rawHeight;
            currentConfig.offset_width = 0;
            currentConfig.offset_height = 0;
            currentConfig.scaling = tod_video::Video_1p000;
            needData = false;
            newDataAvailable = false;
            clientConnected = false;
            encoder = nullptr;
            videocrop = nullptr;
            scalingFilter = nullptr;
            appsrc = nullptr;
        }
    };
};
