// Copyright 2020 Andreas Schimpe
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include "tod_video/VideoConfig.h"
#include "tod_video/BitrateConfig.h"
#include "tod_network/mqtt_client.h"
#include "tod_network/connection_configs.h"
#include "tod_msgs/Status.h"

static std::unique_ptr<tod_network::MqttClient> _mqttClientVideo{nullptr};
static std::unique_ptr<tod_network::MqttClient> _mqttClientBitrate{nullptr};
static std::string _nodeName{""};
static bool _connected{false};
static uint8_t _controlMode{tod_msgs::Status::VIDEO_RATE_CONTROL_MODE_SINGLE};

void callback_status_msg(const tod_msgs::Status &msg);
void receive_desired_video_config(mqtt::const_message_ptr msg);
void receive_desired_bitrate_config(mqtt::const_message_ptr msg);

int main(int argc, char **argv) {
    ros::init(argc, argv, "VehicleVidStreamCfgReceive");
    ros::NodeHandle n;
    _nodeName = ros::this_node::getName();
    bool debug = false;
    n.getParam(_nodeName + "/debug", debug);
    if (debug) // print ROS_DEBUG
        if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
            ros::console::notifyLoggerLevelsChanged();

    ros::Subscriber subStatusMsg = n.subscribe("/Vehicle/Manager/status_msg", 5, callback_status_msg);

    ros::spin();

    return 0;
}

void callback_status_msg(const tod_msgs::Status &msg) {
    bool connected = (msg.tod_status != tod_msgs::Status::TOD_STATUS_IDLE);
    if (connected && !_connected) {
        // connected to vehicle
        _mqttClientVideo = std::make_unique<tod_network::MqttClient>(
            msg.operator_broker_ip_address, _nodeName + "1");
        _mqttClientVideo->subscribe(tod_network::MqttTopics::DesiredVideoConfig, 1,
                                    &receive_desired_video_config);
        _mqttClientBitrate = std::make_unique<tod_network::MqttClient>(
            msg.operator_broker_ip_address, _nodeName + "2");
        _mqttClientBitrate->subscribe(tod_network::MqttTopics::DesiredBitrateConfig, 1,
                                      &receive_desired_bitrate_config);
        ROS_DEBUG("%s: Subscribed to mqtt topics %s and %s!", _nodeName.c_str(),
                  tod_network::MqttTopics::DesiredVideoConfig.c_str(),
                  tod_network::MqttTopics::DesiredBitrateConfig.c_str());
    }
    if (!connected && _connected && _mqttClientVideo && _mqttClientBitrate) {
        // disconnected from operator
        if (_mqttClientVideo) _mqttClientVideo->disconnect();
        else ROS_WARN("%s: Disconnecting, although no mqtt client was initialized", _nodeName.c_str());
        if (_mqttClientBitrate) _mqttClientBitrate->disconnect();
        else ROS_WARN("%s: Disconnecting, although no mqtt client was initialized", _nodeName.c_str());
        _mqttClientVideo.release();
        _mqttClientBitrate.release();
        ROS_DEBUG("%s: Disconnected mqtt clients to receive reconfigure requests!", _nodeName.c_str());
    }
    _connected = connected;
    _controlMode = msg.operator_video_rate_mode;
}

void receive_desired_video_config(mqtt::const_message_ptr msg) {
    if (!_connected) {
        // do not process if not connected
        ROS_WARN("%s: Received reconfigure request via mqtt even though not connected - ignoring!",
                 _nodeName.c_str());
        return;
    }

    // get video config from mqtt message string
    std::string received = msg->to_string();
    ros::serialization::IStream stream((uint8_t*) &received.at(0), received.size());
    dynamic_reconfigure::Config msgRos;
    ros::serialization::Serializer<dynamic_reconfigure::Config>::read(stream, msgRos);
    tod_video::VideoConfig desiredConfig;
    desiredConfig.__fromMessage__(msgRos);
    ROS_DEBUG("%s: Received new desired config for camera %s on mqtt topic %s!",
              _nodeName.c_str(), desiredConfig.camera_name.c_str(), msg->get_topic().c_str());

    // request desired video stream config at rtsp server via ros service
    dynamic_reconfigure::ReconfigureRequest req;
    dynamic_reconfigure::ReconfigureResponse resp;
    desiredConfig.__toMessage__(req.config);
    if (!ros::service::call("/Vehicle/Video/RtspServer/set_parameters", req, resp))
        ROS_ERROR("%s: calling ros service to reconfigure video stream %s failed",
                  _nodeName.c_str(), desiredConfig.camera_name.c_str());

    // respond with actual config back to operator via mqtt
    ros::SerializedMessage serializedConfig = ros::serialization::serializeMessage(resp.config);
    _mqttClientVideo->publish(tod_network::MqttTopics::ActualVideoConfig, 1,
                              (char*) serializedConfig.message_start, serializedConfig.num_bytes);
    ROS_DEBUG("%s: Sent actual encoder config of camera %s back to operator on mqtt topic %s!",
              _nodeName.c_str(), desiredConfig.camera_name.c_str(),
              tod_network::MqttTopics::ActualVideoConfig.c_str());
}

void receive_desired_bitrate_config(mqtt::const_message_ptr msg) {
    if (!_connected || _controlMode != tod_msgs::Status::VIDEO_RATE_CONTROL_MODE_COLLECTIVE) {
        // do not process if not connected or not in mode collective
        ROS_WARN("%s: not connected or not in mode collective - ignoring request received via mqtt",
                 _nodeName.c_str());
        return;
    }

    // reconstruct config from mqtt message string
    std::string received = msg->to_string();
    ros::serialization::IStream stream((uint8_t*) &received.at(0), received.size());
    dynamic_reconfigure::Config msgRos;
    ros::serialization::Serializer<dynamic_reconfigure::Config>::read(stream, msgRos);
    tod_video::BitrateConfig desiredConfig;
    desiredConfig.__fromMessage__(msgRos);
    ROS_DEBUG("%s: Received new desired bitrate sum of %d on mqtt topic %s!",
              _nodeName.c_str(), desiredConfig.bitrate, msg->get_topic().c_str());

    // request desired qos from bandwidth manager via ros service
    dynamic_reconfigure::ReconfigureRequest req;
    dynamic_reconfigure::ReconfigureResponse resp;
    desiredConfig.__toMessage__(req.config);
    if (!ros::service::call("/Vehicle/Video/BandwidthManager/set_parameters", req, resp))
        ROS_ERROR("%s: calling ros service to set available bitrate of %d failed",
                  _nodeName.c_str(), desiredConfig.bitrate);

    // respond with actual config back to operator via mqtt
    ros::SerializedMessage serializedConfig = ros::serialization::serializeMessage(resp.config);
    _mqttClientBitrate->publish(tod_network::MqttTopics::ActualBitrateConfig, 1,
                                (char*) serializedConfig.message_start, serializedConfig.num_bytes);
    ROS_DEBUG("%s: Sent actual bitrate config of bitrate %d back to operator on mqtt topic %s!",
              _nodeName.c_str(), desiredConfig.bitrate, tod_network::MqttTopics::ActualBitrateConfig.c_str());
}
