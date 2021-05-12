// Copyright 2020 Andreas Schimpe
#include "RtspClients.h"

RtspClients::RtspClients(ros::NodeHandle &nodeHandle) : _nodeHandle(nodeHandle),
    _nodeName(ros::this_node::getName()), _latency(500) {
    bool debug{false};
    _nodeHandle.getParam(_nodeName + "/debug", debug);
    if (debug) // print ROS_DEBUG
        if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
            ros::console::notifyLoggerLevelsChanged();

    std::string imgOutputFormat{"RGB"};
    _nodeHandle.getParam(_nodeName + "/imageOutputFormat", imgOutputFormat);
    ROS_INFO("%s: output image format is %s", _nodeName.c_str(), imgOutputFormat.c_str());

    _reconfigServer.setCallback(boost::bind(&RtspClients::toggle_video_stream, this, _1, _2));
    _subsStatus = _nodeHandle.subscribe("/Operator/Manager/status_msg",
                                        5, &RtspClients::callback_status_msg, this);

    // get cameras to stream from ros parameter server
    for (int i=0; i <= 10; ++i) {
        std::string name{""};
        if (_nodeHandle.getParam(std::string(_nodeName + "/camera" + std::to_string(i) + "/name"), name)) {
            auto stream = _streams.emplace_back(std::make_shared<RtspStream>());
            stream->name = name;
            stream->imageOutputFormat = imgOutputFormat;
            std::string name4topic(&name.at(1), name.size()-1); // name without '/'
            stream->pubImage = _nodeHandle.advertise<sensor_msgs::Image>(name4topic + "/image_raw", 1);
            stream->pubVideoInfo = _nodeHandle.advertise<tod_msgs::VideoInfo>(name4topic + "/video_info", 5);
            stream->pubPaketInfo = _nodeHandle.advertise<tod_msgs::PaketInfo>(name4topic + "/paket_info", 100);

            // support for camera names with (2) dots: ros topic names with 'DOT', uris with '.'
            std::string str2find = "DOT";
            std::size_t foundDOT1 = stream->name.find(str2find);
            if (foundDOT1 != std::string::npos)
                stream->name.replace(foundDOT1, str2find.length(), ".");
            std::size_t foundDOT2 = stream->name.find(str2find);
            if (foundDOT2 != std::string::npos)
                stream->name.replace(foundDOT2, str2find.length(), ".");
        }
    }
}

void RtspClients::run() {
    GMainLoop* gstLoop = g_main_loop_new(nullptr, FALSE);
    std::thread gstThread([gstLoop]() {
        g_main_loop_run(gstLoop); // blocking
    });
    ros::Rate r(10);
    while (ros::ok()) {
        r.sleep();
        ros::spinOnce();
        for (auto stream : _streams) {
            if (ros::Time::now() >= stream->lastVideoInfoCalc + ros::Duration(1.0)) {
                // calculate and set current video info
                std::lock_guard<std::mutex> lock(stream->mutex);
                stream->bitrate_kbit = stream->pktSizeSum_bit / 1024;
                stream->framerate = stream->frameCount;
                stream->rtpPaketCount = stream->pktSizeSum_bit = stream->frameCount = 0;
                stream->lastVideoInfoCalc = ros::Time::now();
            }
            // publish video info
            tod_msgs::VideoInfo msg;
            msg.header.stamp = ros::Time::now();
            msg.header.frame_id = stream->name;
            msg.kbitrate = stream->bitrate_kbit;
            msg.framerate = stream->framerate;
            msg.imageHeight = stream->imgHeight_px;
            msg.imageWidth = stream->imgWidth_px;
            msg.imageNofPixel = stream->imgHeight_px * stream->imgWidth_px;
            stream->pubVideoInfo.publish(msg);
        }
    }
    g_main_loop_quit(gstLoop);
    gstThread.join();
    g_main_loop_unref(gstLoop);
}

void RtspClients::callback_status_msg(const tod_msgs::Status &msg) {
    bool connected = (msg.tod_status != tod_msgs::Status::TOD_STATUS_IDLE);
    if (connected && !_connected) {
        // on connect
        for (auto stream : _streams) {
            connect_video_client(stream, msg.vehicle_vehicle_ip_address);
        }
    }
    if (!connected && _connected) {
        // on disconnect
        for (auto stream : _streams) {
            disconnect_video_client(stream);
        }
    }
    _connected = connected;
}

void RtspClients::connect_video_client(std::shared_ptr<RtspStream> stream, const std::string &vehicleIp) {
    if (vehicleIp == "" || stream->name == "") {
        ROS_ERROR("%s: vehicleIp \"%s\" or camera name \"%s\" are not set - abort connecting client",
                  _nodeName.c_str(), vehicleIp.c_str(), stream->name.c_str());
        return;
    }
    if (!gst_is_initialized()) gst_init(0, 0);

    // gst format
    std::string gstImageFormat{"RGB"};
    if (stream->imageOutputFormat == "i420") gstImageFormat = "I420";
    else if (stream->imageOutputFormat == sensor_msgs::image_encodings::RGB8) gstImageFormat = "RGB";
    else if (stream->imageOutputFormat == sensor_msgs::image_encodings::YUV422) gstImageFormat = "UYVY";
    else if (stream->imageOutputFormat == sensor_msgs::image_encodings::BGR8) gstImageFormat = "BGR";
    else if (stream->imageOutputFormat == sensor_msgs::image_encodings::BGRA8) gstImageFormat = "BGRA";
    else if (stream->imageOutputFormat == sensor_msgs::image_encodings::MONO8) gstImageFormat = "GRAY8";
    else ROS_WARN("%s: unknown image format %s - setting output format to %s",
                 _nodeName.c_str(), stream->imageOutputFormat.c_str(), gstImageFormat.c_str());

    std::string uri = "rtsp://" + vehicleIp + ":" + std::to_string(tod_network::VehiclePorts::RX_VIDEO_RTSP)
                      + stream->name;
    gchar *pipe_desc = g_strdup_printf("rtspsrc location=%s latency=%d drop-on-latency=true !"
                                       " identity name=myIdentity signal-handoffs=true !"
                                       " rtph264depay !"
                                       " h264parse !"
                                       " avdec_h264 output-corrupt=false !"
                                       " videoconvert !"
                                       " capsfilter caps=video/x-raw,format=%s !"
                                       " appsink name=mySink sync=false emit-signals=true",
                                       uri.c_str(), _latency, gstImageFormat.c_str());

    GError *error{NULL};
    stream->pipeline = gst_parse_launch(pipe_desc, &error);
    if (error) {
        ROS_ERROR("%s: Something went wrong parsing launch string - Abort!", _nodeName.c_str());
        return;
    }
    g_free(pipe_desc);

    GstElement *theIdentity = gst_bin_get_by_name(GST_BIN(stream->pipeline), "myIdentity");
    if (!theIdentity) {
        ROS_ERROR("%s: Could not get identity element from pipeline - Abort!", _nodeName.c_str());
        return;
    }
    g_signal_connect(theIdentity, "handoff", G_CALLBACK(new_rtp_packet), stream.get());

    GstElement *theAppSink = gst_bin_get_by_name(GST_BIN(stream->pipeline), "mySink");
    if (!theAppSink) {
        ROS_ERROR("%s: Could not get appsink from pipeline - Abort!", _nodeName.c_str());
        return;
    }
    g_signal_connect(theAppSink, "new-sample", G_CALLBACK(new_image_sample), stream.get());

    ROS_INFO("%s: Started streaming %s from uri %s", _nodeName.c_str(), stream->name.c_str(), uri.c_str());
    gst_element_set_state(stream->pipeline, GST_STATE_PLAYING);
}

void RtspClients::disconnect_video_client(std::shared_ptr<RtspStream> stream) {
    gst_element_send_event(stream->pipeline, gst_event_new_eos());
    gst_element_set_state(stream->pipeline, GST_STATE_NULL);
    gst_object_unref(stream->pipeline);
    ROS_INFO("%s: Stopped streaming %s", _nodeName.c_str(), stream->name.c_str());
}



void RtspClients::toggle_video_stream(tod_video::ClientConfig &config, uint32_t level) {
    if (!_connected) return;
    for (auto stream : _streams) {
        if (stream->name == config.camera_name) {
            if (!config.pause) {
                gst_element_set_state(stream->pipeline, GST_STATE_PLAYING);
                ROS_INFO("%s Client: Playing stream", config.camera_name.c_str());
            } else {
                gst_element_set_state(stream->pipeline, GST_STATE_PAUSED);
                ROS_INFO("%s Client: Pausing stream", config.camera_name.c_str());
                std::lock_guard<std::mutex> lock(stream->mutex);
                stream->imgHeight_px = 0;
                stream->imgWidth_px = 0;
            }
            return;
        }
    }

    // not processed reconfigure
    ROS_WARN("%s: Received request to play/pause unknown camera %s.",
             _nodeName.c_str(), config.camera_name.c_str());
    return;
}

void RtspClients::new_rtp_packet(GstElement *identity, GstBuffer *buffer, RtspStream *stream) {
    // store and publish packet info
    tod_msgs::PaketInfo pktInfoMsg;
    pktInfoMsg.header.stamp = ros::Time::now();
    pktInfoMsg.sizeBit = int32_t(8 * gst_buffer_peek_memory(buffer, 0)->size);
    gst_rtp_buffer_map(buffer, GST_MAP_READ, &stream->rtpPaket);
    pktInfoMsg.seqNum = gst_rtp_buffer_get_seq(&stream->rtpPaket);
    gst_rtp_buffer_unmap(&stream->rtpPaket);
    stream->pubPaketInfo.publish(pktInfoMsg);
    std::lock_guard<std::mutex> lock(stream->mutex);
    ++stream->rtpPaketCount;
    stream->pktSizeSum_bit += pktInfoMsg.sizeBit;
}

void RtspClients::new_image_sample(GstAppSink* appSink, RtspStream* stream) {
    GstSample* sample = gst_app_sink_pull_sample(appSink);
    GstCaps* caps = gst_sample_get_caps(sample);
    if (!caps) {
        ROS_ERROR("%s Client: could not get image info from filter caps",
                  stream->name.c_str());
        return;
    }

    GstStructure* s = gst_caps_get_structure(caps, 0);
    int width{0}, height{0};
    if (!(gst_structure_get_int(s, "width", &width)
          && gst_structure_get_int(s, "height", &height))) {
        ROS_ERROR("%s Client: Could not get image width and height from filter caps",
                  stream->name.c_str());
        return;
    }

    GstBuffer* buffer = gst_sample_get_buffer(sample);
    GstMemory* mem = gst_buffer_get_all_memory(buffer);
    GstMapInfo info;
    if (gst_memory_map(mem, &info, GST_MAP_READ)) {
        sensor_msgs::ImagePtr myMsg = boost::make_shared<sensor_msgs::Image>();
        myMsg->data = std::vector<u_char>(info.data, info.data + info.size);
        myMsg->width = width;
        myMsg->height = height;
        myMsg->step = uint(info.size / height);
        myMsg->encoding = stream->imageOutputFormat;
        stream->pubImage.publish(myMsg);
        stream->imgHeight_px = height;
        stream->imgWidth_px = width;
    } else {
        ROS_ERROR("%s Client: Could not gst_memory_map", stream->name.c_str());
    }
    gst_sample_unref(sample);
    gst_memory_unref(mem);
    gst_memory_unmap(mem, &info);
    std::lock_guard<std::mutex> lock(stream->mutex);
    ++stream->frameCount; // used to publish current frame rate from ros main loop
}
