// Copyright 2020 Andreas Schimpe
#pragma once
#include <ros/ros.h>
#include <string>
#include <thread>
#include <memory>
#include <vector>
#include <mutex>
#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <gst/rtp/gstrtpbuffer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <dynamic_reconfigure/server.h>
#include "tod_msgs/Status.h"
#include "tod_video/ClientConfig.h"
#include "tod_msgs/PaketInfo.h"
#include "tod_msgs/VideoInfo.h"
#include "tod_network/connection_configs.h"

class RtspClients {
public:
    explicit RtspClients(ros::NodeHandle &nodeHandle);
    ~RtspClients() { }
    void run();

private:
    ros::NodeHandle &_nodeHandle;
    std::string _nodeName{""};
    dynamic_reconfigure::Server<tod_video::ClientConfig> _reconfigServer;
    ros::Subscriber _subsStatus;
    struct RtspStream;
    std::vector<std::shared_ptr<RtspStream>> _streams;
    int _latency;
    bool _connected{false};

    void callback_status_msg(const tod_msgs::Status &msg);
    void connect_video_client(std::shared_ptr<RtspStream> stream, const std::string &vehicleIp);
    void disconnect_video_client(std::shared_ptr<RtspStream> stream);
    void toggle_video_stream(tod_video::ClientConfig &config, uint32_t level);
    static void new_rtp_packet(GstElement *identity, GstBuffer *buffer, RtspStream *stream);
    static void new_image_sample(GstAppSink* appSink, RtspStream *stream);

    struct RtspStream {
        std::string name{""};
        ros::Publisher pubImage, pubVideoInfo, pubPaketInfo;
        std::mutex mutex;
        GstElement *pipeline{nullptr};
        GstRTPBuffer rtpPaket = GST_RTP_BUFFER_INIT;
        int rtpPaketCount{0}, frameCount{0}, pktSizeSum_bit{0};
        int bitrate_kbit{0}, framerate{0}, imgHeight_px{0}, imgWidth_px{0};
        ros::Time lastVideoInfoCalc;
        std::string imageOutputFormat;
    };
};
