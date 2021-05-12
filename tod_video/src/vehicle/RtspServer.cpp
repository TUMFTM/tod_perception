// Copyright 2020 Andreas Schimpe
#include "RtspServer.h"

RtspServer::RtspServer(ros::NodeHandle &n) : _nodeHandle(n) {
    _nodeName = ros::this_node::getName();
    bool debug = false;
    n.getParam(_nodeName + "/debug", debug);
    if (debug) // print ROS_DEBUG
        if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
            ros::console::notifyLoggerLevelsChanged();

    n.getParam(_nodeName + "/bitrate", _defaultBitrate);
    ROS_DEBUG("%s: Default bitrate for each stream is %d",
              _nodeName.c_str(), _defaultBitrate);

    // get topics and number of cameras from parameter server
    std::string topicNamespace{"/Vehicle/Video"}, imageName{"/image_raw"};
    n.getParam(_nodeName + "/camera_topics_namespace", topicNamespace);
    n.getParam(_nodeName + "/camera_image_name", imageName);
    ROS_DEBUG("%s: Using camera topic namespace: %s", _nodeName.c_str(), topicNamespace.c_str());
    ROS_DEBUG("%s: Using camera image name: %s", _nodeName.c_str(), imageName.c_str());
    // get cameras to stream from parameter server
    for (int i=0; i <= 10; ++i) {
        std::string name{""};
        if (n.getParam(std::string(_nodeName + "/camera" + std::to_string(i) + "/name"), name)) {
            // initialize an RtspStream for each camera name received
            auto stream = _streams.emplace_back(std::make_shared<RtspStream>(name, _defaultBitrate));
            std::string topic{topicNamespace + name + imageName};
            stream->subsImg =
                n.subscribe<sensor_msgs::Image>(
                    topic, 1, boost::bind(&RtspServer::callback_raw_image, this, _1, stream));
        }
    }
    ROS_DEBUG("%s: Got %d cameras to stream", _nodeName.c_str(), _streams.size());

    _reconfigServer.setCallback(boost::bind(&RtspServer::callback_stream_reconfig, this, _1, _2));
    _subsStatus = n.subscribe<tod_msgs::Status>(
        "/Vehicle/Manager/status_msg", 5, [&](const auto &msg) {
            _operatorConnected = (msg->tod_status != tod_msgs::Status::TOD_STATUS_IDLE);
        });

    gst_init(0, 0);
}

void RtspServer::run() {
    GstRTSPServer *gstServer = gst_rtsp_server_new();
    gst_rtsp_server_attach(gstServer, nullptr); // start serving
    GstRTSPMountPoints *gstMounts = gst_rtsp_server_get_mount_points(gstServer);
    GMainLoop *gstLoop = g_main_loop_new(nullptr, FALSE);
    std::thread gstThread([gstLoop]() {
        g_main_loop_run(gstLoop);
    });
    ros::Rate r(1000);
    while (ros::ok()) {
        r.sleep();
        ros::spinOnce();
        for (auto stream : _streams) {
            if (stream->newDataAvailable) {
                if (!stream->factory)
                    factory_gst_video_pipeline(stream, gstMounts);
                if (stream->needData) {
                    push_data(stream);
                } else if (ros::Time::now() >= stream->lastNeedDataStamp + ros::Duration(3.0)
                           && !stream->currentConfig.paused) {
                    ROS_INFO("%s: Stream %s is paused", _nodeName.c_str(), stream->name.c_str());
                    std::lock_guard lock(stream->mutex);
                    stream->currentConfig.paused = true;
                }
            }

            static bool operatorConnectedPrevious{false};
            if (operatorConnectedPrevious && !_operatorConnected) {
                for (auto stream : _streams) {
                    // reset stream settings on disconnect
                    stream->reset(_defaultBitrate);
                }
            }
            operatorConnectedPrevious = _operatorConnected;
        }
    }
    g_main_loop_quit(gstLoop);
    gstThread.join();
    g_object_unref(gstMounts);
    g_object_unref(gstServer);
    g_main_loop_unref(gstLoop);
}

void RtspServer::factory_gst_video_pipeline(std::shared_ptr<RtspStream> stream, GstRTSPMountPoints *_gstMounts) {
    std::string launchString =
        "( appsrc name=mysrc is-live=true ! videoconvert ! videocrop name=mycrop !"
        " videoscale ! capsfilter name=myscale !"
        " x264enc name=myenc "
        " tune=zerolatency speed-preset=superfast"
        " sliced-threads=true byte-stream=true threads=1"
        " key-int-max=15 intra-refresh=true !"
        " identity name=idH264 signal-handoffs=true !"
        " h264parse ! rtph264pay name=pay0 pt=96 )";

    stream->factory = gst_rtsp_media_factory_new();
    gst_rtsp_media_factory_set_launch(stream->factory, launchString.c_str());

    // callback when client connects
    g_signal_connect(stream->factory, "media-configure", (GCallback) media_configure,
                     stream.get());

    // attach factory to url
    gst_rtsp_mount_points_add_factory(_gstMounts, stream->name.c_str(), stream->factory);

    stream->reset(_defaultBitrate); // video settings to default

    std::string url{"rtsp://127.0.0.1:" + std::to_string(tod_network::VehiclePorts::RX_VIDEO_RTSP)
                    + stream->name};
    ROS_INFO("%s: Stream of size (%dx%d) and encoding %s available at %s", _nodeName.c_str(),
             stream->rawWidth, stream->rawHeight, stream->encoding.c_str(), url.c_str());
}

void RtspServer::push_data(std::shared_ptr<RtspStream> stream) {
    std::lock_guard lock(stream->mutex);
    // put image data to buffer and push to pipeline
    GstBuffer *buffer = gst_buffer_new_wrapped_full(
        (GstMemoryFlags) 0, (gpointer) &stream->imgData.at(0),
        stream->rawHeight * stream->rawStep, // size of image in Byte
        0, stream->rawHeight * stream->rawStep, NULL, NULL);
    GstFlowReturn ret;
    g_signal_emit_by_name(stream->appsrc, "push-buffer", buffer, &ret);
    gst_buffer_unref(buffer);
    stream->needData = stream->newDataAvailable = false;
}

void RtspServer::need_data(GstElement* appSrc, guint unused, RtspStream *stream) {
    std::lock_guard lock(stream->mutex);
    stream->needData = true;
    stream->lastNeedDataStamp = ros::Time::now();
    if (stream->currentConfig.paused) {
        ROS_INFO("%s: Stream %s is playing",
                 ros::this_node::getName().c_str(), stream->name.c_str());
        stream->currentConfig.paused = false;
    }
}

void RtspServer::media_configure(GstRTSPMediaFactory *factory, GstRTSPMedia *media,
                                 RtspStream* stream) {
    GstElement *element = gst_rtsp_media_get_element(media);
    stream->encoder = gst_bin_get_by_name_recurse_up(GST_BIN(element), "myenc");
    g_object_set(G_OBJECT(stream->encoder), "bitrate", stream->currentConfig.bitrate, NULL);
    stream->videocrop = gst_bin_get_by_name_recurse_up(GST_BIN(element), "mycrop");
    g_object_set(G_OBJECT(stream->videocrop), "top", 0, "bottom", 0, "left", 0, "right", 0, NULL);
    stream->scalingFilter = gst_bin_get_by_name_recurse_up(GST_BIN(element), "myscale");
    g_object_set(G_OBJECT(stream->scalingFilter), "caps", gst_caps_new_simple
                 ("video/x-raw",
                  "width", G_TYPE_INT, stream->rawWidth,
                  "height", G_TYPE_INT, stream->rawHeight,
                  NULL), NULL);

    // select image format as received in ros image callback
    std::string format{""};
    if (stream->encoding == sensor_msgs::image_encodings::MONO16) {
        format = "UYVY";
        ROS_WARN("%s: ROS image format for %s is %s, using %s for GStreamer",
                 ros::this_node::getName().c_str(), stream->name.c_str(),
                 stream->encoding.c_str(), format.c_str());
    } else if (stream->encoding == sensor_msgs::image_encodings::YUV422) {
        format = "UYVY";
    } else if (stream->encoding == sensor_msgs::image_encodings::BGR8) {
        format = "BGR";
    } else if (stream->encoding == sensor_msgs::image_encodings::BGRA8) {
        format = "BGRA";
    } else if (stream->encoding == sensor_msgs::image_encodings::MONO8) {
        format = "GRAY8";
    } else if (stream->encoding == sensor_msgs::image_encodings::RGB8) {
        format = "RGB";
    } else {
        ROS_WARN("%s: Unsupported image encoding %s for %s - Choosing MONO8!",
                 ros::this_node::getName().c_str(), stream->encoding.c_str(),
                 stream->name.c_str());
        format = "MONO8";
    }

    stream->appsrc = gst_bin_get_by_name_recurse_up(GST_BIN(element), "mysrc");
    g_object_set(G_OBJECT(stream->appsrc),
                 "stream-type", GST_APP_STREAM_TYPE_STREAM,
                 "format", GST_FORMAT_TIME,
                 "is-live", TRUE,
                 "do-timestamp", TRUE,
                 "caps", gst_caps_new_simple("video/x-raw",
                                     "format", G_TYPE_STRING, format.c_str(),
                                     "width", G_TYPE_INT, stream->rawWidth,
                                     "height", G_TYPE_INT, stream->rawHeight,
                                     NULL), NULL);

    // install the callback that will be called when a buffer is needed
    g_signal_connect(stream->appsrc, "need-data", (GCallback) need_data, stream);
    gst_object_unref(stream->appsrc);
    gst_object_unref(stream->encoder);
    gst_object_unref(element);

    ROS_INFO("%s: Cient connected for %s in GStreamer image format %s!",
             ros::this_node::getName().c_str(), stream->subsImg.getTopic().c_str(),
             format.c_str());
}

void RtspServer::callback_raw_image(const sensor_msgs::ImageConstPtr& msg,
                                    std::shared_ptr<RtspStream> stream) {
    std::lock_guard lock(stream->mutex);
    if (stream->newDataAvailable && stream->currentConfig.paused)
        return; // no unnecessary copies
    stream->rawHeight = msg->height;
    stream->rawWidth = msg->width;
    stream->encoding = msg->encoding;
    stream->rawStep = msg->step;
    stream->imgData.clear();
    stream->imgData = msg->data;
    stream->newDataAvailable = true;
}

void RtspServer::callback_stream_reconfig(tod_video::VideoConfig &config, uint32_t level) {
    static bool once{false};
    if (!once) { // rosservice gets called once on startup, do not process this
        once = true;
        return;
    }

    // get video stream to reconfigure
    std::shared_ptr<RtspStream> stream2reconfigure{NULL};
    for (auto stream : _streams) {
        if (stream->name == config.camera_name) {
            stream2reconfigure = stream;
            break;
        }
    }
    if (!stream2reconfigure) {
        ROS_ERROR("%s: ignoring request to reconfigure %s as no stream is available",
                  _nodeName.c_str(), config.camera_name.c_str());
        return;
    }

    if (config.width == -1 || config.height == -1) {
        // on -1, -1: reply with current confi
        config = stream2reconfigure->currentConfig;
        return;
    }

    if (config.bitrate != stream2reconfigure->currentConfig.bitrate)
        set_bitrate(config, stream2reconfigure);

    // it is not intended to change actual width and height via reconfig -> always reset
    config.actual_width = stream2reconfigure->currentConfig.actual_width;
    config.actual_height = stream2reconfigure->currentConfig.actual_height;

    if (config.scaling != stream2reconfigure->currentConfig.scaling ||
        config.width != stream2reconfigure->currentConfig.width ||
        config.height != stream2reconfigure->currentConfig.height ||
        config.offset_width != stream2reconfigure->currentConfig.offset_width ||
        config.offset_height != stream2reconfigure->currentConfig.offset_height) {
        set_cropping_and_scaling(config, stream2reconfigure);
    }

    stream2reconfigure->currentConfig = config;
}

void RtspServer::set_bitrate(tod_video::VideoConfig &config, std::shared_ptr<RtspStream> stream2reconfigure) {
    if (!stream2reconfigure->videocrop || !stream2reconfigure->scalingFilter) {
        ROS_WARN("%s: not cropping/scaling as no videocrop/videoscale for %s initialized",
                 _nodeName.c_str(), stream2reconfigure->name.c_str());
        return;
    }
    g_object_set(G_OBJECT(stream2reconfigure->encoder), "bitrate", config.bitrate, NULL);
    ROS_INFO("%s: set bitrate = %i for %s", _nodeName.c_str(),
             config.bitrate, stream2reconfigure->name.c_str());
}

void RtspServer::set_cropping_and_scaling(tod_video::VideoConfig &new_config,
                                          std::shared_ptr<RtspStream> stream2reconfigure) {
    if (!stream2reconfigure->encoder) {
        ROS_WARN("%s: not setting bitrate as no encoder for %s initialized",
                 _nodeName.c_str(), stream2reconfigure->name.c_str());
        return;
    }
    const int fullWidth = stream2reconfigure->rawWidth;
    const int fullHeight = stream2reconfigure->rawHeight;

    // saturate set values according to full width and height
    // also make sure there are only even numbers of pixels
    const int minPxSize = 16;
    new_config.width = std::min(fullWidth, new_config.width);
    new_config.width = std::max(minPxSize, new_config.width - new_config.width % 8);
    new_config.height = std::min(fullHeight, new_config.height);
    new_config.height = std::max(minPxSize, new_config.height - new_config.height % 8);
    new_config.offset_width = std::min(fullWidth-minPxSize, new_config.offset_width);
    new_config.offset_width = std::max(0, new_config.offset_width-new_config.offset_width % 8);
    new_config.offset_height = std::min(fullHeight-minPxSize, new_config.offset_height);
    new_config.offset_height = std::max(0, new_config.offset_height-new_config.offset_height % 8);

    // clamp width if through new width offset, image goes beyond full width
    if (new_config.offset_width != stream2reconfigure->currentConfig.offset_width) {
        new_config.width = std::min(new_config.width, fullWidth-new_config.offset_width);
    }
    // clamp height if through new height offset, image goes beyond full height
    if (new_config.offset_height != stream2reconfigure->currentConfig.offset_height) {
        new_config.height = std::min(new_config.height, fullHeight-new_config.offset_height);
    }
    // reduce offset width if through new width, image goes beyond full width
    if (new_config.width != stream2reconfigure->currentConfig.width) {
        new_config.offset_width = std::min(new_config.offset_width, fullWidth-new_config.width);
    }
    // reduce offset height if through new height, image goes beyond full height
    if (new_config.height != stream2reconfigure->currentConfig.height) {
        new_config.offset_height = std::min(new_config.offset_height, fullHeight-new_config.height);
    }

    // final cropping
    const int leftCrop = new_config.offset_width;
    const int rightCrop = fullWidth - new_config.width - new_config.offset_width;
    const int bottomCrop = new_config.offset_height;
    const int topCrop = fullHeight - new_config.height - new_config.offset_height;
    g_object_set(G_OBJECT(stream2reconfigure->videocrop),
                 "top", topCrop,
                 "bottom", bottomCrop,
                 "left", leftCrop,
                 "right", rightCrop, NULL);

    // final scaling - round to integer and even pixel numbers multiple of 8
    std::string scalingStr = new_config.scaling;
    // scaling[1] is 'p' in new_config, replace with '.' to cast as double
    scalingStr.at(1) = '.';
    const double scalingFactor = std::stod(scalingStr);
    new_config.actual_width = std::max(minPxSize, static_cast<int>(new_config.width * scalingFactor));
    new_config.actual_width -= new_config.actual_width % 8;
    new_config.actual_height = std::max(minPxSize, static_cast<int>(new_config.height * scalingFactor));
    new_config.actual_height -= new_config.actual_height % 8;

    g_object_set(G_OBJECT(stream2reconfigure->scalingFilter), "caps", gst_caps_new_simple
                 ("video/x-raw",
                  "width", G_TYPE_INT, new_config.actual_width,
                  "height", G_TYPE_INT, new_config.actual_height,
                  NULL), NULL);
    ROS_INFO("%s: set (aw,ah,w,h,ow,oh) = (%i,%i,%i,%i,%i,%i) at scaling factor = %f for %s",
             _nodeName.c_str(), new_config.actual_width, new_config.actual_height, new_config.width,
             new_config.height, new_config.offset_width, new_config.offset_height, scalingFactor,
             stream2reconfigure->name.c_str());
}
