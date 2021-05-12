// Copyright 2020 TUMFTM
#include "tod_network/tod_receiver.h"
#include "geometry_msgs/PointStamped.h"
int main(int argc, char **argv) {
    ros::init(argc, argv, "OperatorBitratePredictionReceiver");
    ros::NodeHandle n;
    tod_network::Receiver<geometry_msgs::PointStamped> receiver(n);
    receiver.add_processer("bitrate_prediction_on_gps", tod_network::OperatorPorts::RX_BITRATE_PREDICTIONS);
    receiver.run();
    return 0;
}
