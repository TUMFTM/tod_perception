// Copyright 2020 Andreas Schimpe
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PointStamped.h>
#include <tod_msgs/VideoInfo.h>
#include <string>
#include <vector>

struct Stream2Integrate {
    ros::Subscriber subscriber;
    int kbitrate{0};
};

void callback_video_kpi(const tod_msgs::VideoInfoConstPtr &msg,
                        std::shared_ptr<Stream2Integrate> stream) {
    stream->kbitrate = msg->kbitrate;
}

void callback_gps(const sensor_msgs::NavSatFixConstPtr &msg,
                  sensor_msgs::NavSatFix &gpsMsg) {
    gpsMsg = *msg;
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "BitrateIntegrator");
    ros::NodeHandle nodeHandle;
    std::string nodeName = ros::this_node::getName();

    sensor_msgs::NavSatFix gpsMsg;
    ros::Subscriber subGps = nodeHandle.subscribe<sensor_msgs::NavSatFix>(
        "/Operator/VehicleConnection/gps/fix", 10, boost::bind(callback_gps, _1, std::ref(gpsMsg)));
    ros::Publisher pubSum = nodeHandle.advertise<geometry_msgs::PointStamped>("totalBitrateOnGps", 10);
    ros::Publisher pubAvg = nodeHandle.advertise<geometry_msgs::PointStamped>("averageFramerateOnGps", 10);

    // initialize streams to integrate from list of cameras on parameter server
    std::vector<std::shared_ptr<Stream2Integrate>> streams;
    for (int i=0; i <= 10; ++i) {
        std::string name{""};
        if (nodeHandle.getParam(std::string(nodeName + "/camera" + std::to_string(i) + "/name"), name)) {
            streams.emplace_back(std::make_shared<Stream2Integrate>());
            streams.back()->subscriber = nodeHandle.subscribe<tod_msgs::VideoInfo>
                                         ("/Operator/Video/" + name + "/video_info", 1,
                                          boost::bind(&callback_video_kpi, _1, streams.back()));
        }
    }

    // integrate and publish total bitrate periodically
    ros::Rate r(20);
    while (ros::ok()) {
        ros::spinOnce();
        int brSum{0};
        for (auto stream : streams)
            brSum += stream->kbitrate;
        geometry_msgs::PointStamped bitrateSumMsg;
        bitrateSumMsg.header.stamp = ros::Time::now();
        bitrateSumMsg.point.x = gpsMsg.longitude;
        bitrateSumMsg.point.y = gpsMsg.latitude;
        bitrateSumMsg.point.z = double(brSum);
        pubSum.publish(bitrateSumMsg);
        r.sleep();
    }

    return 0;
}
