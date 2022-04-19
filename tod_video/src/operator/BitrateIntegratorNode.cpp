// Copyright 2020 Andreas Schimpe
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PointStamped.h>
#include <tod_msgs/VideoInfo.h>
#include <tod_core/CameraParameters.h>
#include <string>
#include <vector>

struct Stream2Integrate {
    ros::Subscriber subscriber;
    int kbitrate{0};
};

std::unique_ptr<tod_core::CameraParameters> _camParams{nullptr};

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
    ros::NodeHandle nh;
    _camParams = std::make_unique<tod_core::CameraParameters>(nh);

    sensor_msgs::NavSatFix gpsMsg;
    ros::Subscriber subGps = nh.subscribe<sensor_msgs::NavSatFix>(
        "/Operator/VehicleBridge/gps/fix", 10, boost::bind(callback_gps, _1, std::ref(gpsMsg)));
    ros::Publisher pubSum = nh.advertise<geometry_msgs::PointStamped>("totalBitrateOnGps", 10);

    std::vector<std::shared_ptr<Stream2Integrate>> streams;
    for (const auto& cam : _camParams->get_sensors()) {
        auto stream = streams.emplace_back(std::make_shared<Stream2Integrate>());
        stream->subscriber = nh.subscribe<tod_msgs::VideoInfo>
                             ("/Operator/Video" + cam.operator_name + "/video_info", 1,
                              boost::bind(&callback_video_kpi, _1, streams.back()));
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
