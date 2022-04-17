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
#include "tod_core/CameraParameters.h"

class RtspClients {
public:
    explicit RtspClients(ros::NodeHandle &nodeHandle);
    ~RtspClients() { }
    void run();

private:
    ros::NodeHandle &_nh;
    std::string _nn{""};
    dynamic_reconfigure::Server<tod_video::ClientConfig> _reconfigServer;
    ros::Subscriber _subsStatus;
    std::unique_ptr<tod_core::CameraParameters> _camParams;
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
        std::string vehicle_name{""};
        std::string operator_name{""};
        int ip_offset{0};
        ros::Publisher pubImage, pubVideoInfo, pubPaketInfo;
        std::mutex mutex;
        GstElement *pipeline{nullptr};
        GstRTPBuffer rtpPaket = GST_RTP_BUFFER_INIT;
        int rtpPaketCount{0}, frameCount{0}, pktSizeSum_bit{0};
        int bitrate_kbit{0}, framerate{0}, imgHeight_px{0}, imgWidth_px{0};
        ros::Time lastVideoInfoCalc;
        std::string imageOutputFormat;
        bool isJpeg{false};
        RtspStream(const tod_core::CameraParameters::CameraSensor &camera, const std::string &outputFormat) :
            vehicle_name{camera.vehicle_name},
            operator_name{camera.operator_name},
            imageOutputFormat{outputFormat},
            ip_offset{camera.ip_offset},
            isJpeg{camera.is_jpeg} {
            // support for camera names with dots: ros topic names with 'DOT', uris with '.'
            std::string str2find = "DOT";
            std::size_t found;
            while ((found = vehicle_name.find(str2find)) != std::string::npos) {
                vehicle_name.replace(found, str2find.length(), ".");
            }
            while ((found = operator_name.find(str2find)) != std::string::npos) {
                operator_name.replace(found, str2find.length(), ".");
            }
        }
    };
};
