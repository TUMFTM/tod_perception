// Copyright 2020 TUMFTM
#include "tod_network/tod_sender.h"
#include "geometry_msgs/PointStamped.h"
int main(int argc, char **argv) {
    ros::init(argc, argv, "VehicleBitratePredictionSender");
    ros::NodeHandle n;
    tod_network::Sender<geometry_msgs::PointStamped> sender(n, true);
    sender.add_processer("bitrate_prediction_on_gps", tod_network::OperatorPorts::RX_BITRATE_PREDICTIONS);
    sender.run();
    return 0;
}
