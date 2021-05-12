// Copyright 2020 Andreas Schimpe
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <memory>
#include "tod_video/BitrateConfig.h"
#include "tod_network/mqtt_client.h"
#include "tod_network/connection_configs.h"
#include "tod_msgs/Status.h"

static std::unique_ptr<tod_network::MqttClient> _mqttClient{nullptr};
static bool _receivedActualConfigFromVehicle{false};
static tod_video::BitrateConfig _repliedConfig;
static std::string _nodeName{""};
static bool _connected{false};

void callback_status_msg(const tod_msgs::Status &msg);
void callback_video_reconfigure_request(tod_video::BitrateConfig &config, uint32_t level);
void send_desired_config_to_vehicle(const tod_video::BitrateConfig &config);
void wait_for_actual_config_from_vehicle();
void callback_actual_config_from_vehicle(mqtt::const_message_ptr msg);

int main(int argc, char **argv) {
    ros::init(argc, argv, "OperatorBitrateConfigSend");
    ros::NodeHandle n;
    _nodeName = ros::this_node::getName();
    bool debug = false;
    n.getParam(_nodeName + "/debug", debug);
    if (debug) // print ROS_DEBUG
        if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
            ros::console::notifyLoggerLevelsChanged();

    dynamic_reconfigure::Server<tod_video::BitrateConfig> server;
    server.setCallback(boost::bind(&callback_video_reconfigure_request, _1, _2));

    ros::Subscriber subStatusMsg = n.subscribe("/Operator/Manager/status_msg", 5, callback_status_msg);

    ros::Duration(2.0).sleep(); // give vehicle nodes time to start up
    ros::spin();

    return 0;
}

void callback_status_msg(const tod_msgs::Status &msg) {
    bool connected = (msg.tod_status != tod_msgs::Status::TOD_STATUS_IDLE);
    if (connected && !_connected) {
        // connected to vehicle
        _mqttClient = std::make_unique<tod_network::MqttClient>(
            msg.operator_broker_ip_address, _nodeName);
        ROS_DEBUG("%s: Created MQTT Client to send Encoder Config Requests at Broker Ip %s",
                  _nodeName.c_str(), msg.operator_broker_ip_address.c_str());
        ros::Duration(2.0).sleep(); // receiving mqtt clients need to start up
        _mqttClient->subscribe(tod_network::MqttTopics::ActualBitrateConfig, 1,
                               &callback_actual_config_from_vehicle);
        ROS_DEBUG("%s: Subscribed to mqtt topic %s!", _nodeName.c_str(),
                  tod_network::MqttTopics::ActualBitrateConfig.c_str());
    }
    if (!connected && _connected) {
        // disconnected from vehicle
        if (_mqttClient) _mqttClient->disconnect();
        else ROS_WARN("%s: Disconnecting, although no mqtt was initialized", _nodeName.c_str());
        _mqttClient.release();
        ROS_DEBUG("%s: Disconnected from MQTT broker in vehicle!", _nodeName.c_str());
    }
    _connected = connected;
}

void callback_video_reconfigure_request(tod_video::BitrateConfig &config, uint32_t level) {
    if (!_mqttClient) return;
    if (!_connected) {
        ROS_WARN("%s: Not connected - ignoring request!", ros::this_node::getName().c_str());
        return;
    }
    _receivedActualConfigFromVehicle = false;
    send_desired_config_to_vehicle(config);
    wait_for_actual_config_from_vehicle();
    if (_receivedActualConfigFromVehicle) {
        config = _repliedConfig;
    } else {
        // did not receive reply from vehicle - retry
        send_desired_config_to_vehicle(config);
        wait_for_actual_config_from_vehicle();
        if (_receivedActualConfigFromVehicle) {
            config = _repliedConfig;
        } else {
            ROS_ERROR("%s: did not get actual bitrate config",
                      _nodeName.c_str());
        }
    }
}

void send_desired_config_to_vehicle(const tod_video::BitrateConfig &config) {
    dynamic_reconfigure::Config msg;
    config.__toMessage__(msg);
    ros::SerializedMessage serCfg = ros::serialization::serializeMessage(msg);
    _mqttClient->publish(tod_network::MqttTopics::DesiredBitrateConfig, 1,
                         (char*) serCfg.message_start, serCfg.num_bytes);
    ROS_DEBUG("%s: Sent desired bitrate config bitrate %d on mqtt topic %s!", _nodeName.c_str(),
              config.bitrate, tod_network::MqttTopics::DesiredBitrateConfig.c_str());
}

void wait_for_actual_config_from_vehicle() {
    ros::Duration waitFor(2.0);
    ros::Time tStart = ros::Time::now();
    ros::Rate r{50};
    while (!_receivedActualConfigFromVehicle) {
        r.sleep();
        if (ros::Time::now() >= tStart + waitFor) break;
    }
}

void callback_actual_config_from_vehicle(mqtt::const_message_ptr msg) {
    // reconstruct cfg from mqtt message string
    std::string received = msg->to_string();
    dynamic_reconfigure::Config msgRos;
    ros::serialization::IStream stream((uint8_t*) &received.at(0), uint32_t(received.size()));
    ros::serialization::Serializer<dynamic_reconfigure::Config>::read(stream, msgRos);
    _repliedConfig.__fromMessage__(msgRos);
    _receivedActualConfigFromVehicle = true;
    ROS_DEBUG("%s: Received actual config on mqtt topic %s!",
              _nodeName.c_str(), msg->get_topic().c_str());
}
