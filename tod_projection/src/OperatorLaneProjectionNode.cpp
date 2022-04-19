// Copyright 2020 Andreas Schimpe
#include "ros/ros.h"
#include "tod_msgs/VehicleData.h"
#include <nav_msgs/Path.h>
#include <tod_helper/vehicle/Model.h>
#include "tod_msgs/VehicleEnums.h"
#include "tod_core/VehicleParameters.h"

static std::string _nn{""};
static ros::Publisher _pubVehicleLaneFL, _pubVehicleLaneFR, _pubVehicleLaneRL, _pubVehicleLaneRR;
static std::unique_ptr<tod_core::VehicleParameters> _vehicleParams{nullptr};

void callback_vehicle_data(const tod_msgs::VehicleData &msg);

int main(int argc, char **argv) {
    ros::init(argc, argv, "OperatorLaneProjection");
    ros::NodeHandle nh;
    _nn = ros::this_node::getName();
    _vehicleParams = std::make_unique<tod_core::VehicleParameters>(nh);

    ros::Subscriber subVehicleData = nh.subscribe("/Operator/VehicleBridge/vehicle_data", 5, callback_vehicle_data);
    _pubVehicleLaneFL = nh.advertise<nav_msgs::Path>("vehicle_lane_front_left", 5);
    _pubVehicleLaneFR = nh.advertise<nav_msgs::Path>("vehicle_lane_front_right", 5);
    _pubVehicleLaneRL = nh.advertise<nav_msgs::Path>("vehicle_lane_rear_left", 5);
    _pubVehicleLaneRR = nh.advertise<nav_msgs::Path>("vehicle_lane_rear_right", 5);

    ros::Rate r(100);
    while (ros::ok()) {
        if (_vehicleParams->vehicle_id_has_changed()) {
            _vehicleParams->load_parameters();
        }
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}

void callback_vehicle_data(const tod_msgs::VehicleData &msg) {
    // calc and publish vehicle lane as path
    int direction{1};
    nav_msgs::Path frontLeftLane, frontRightLane, rearLeftLane, rearRightLane;
    frontLeftLane.header.stamp = frontRightLane.header.stamp =
        rearLeftLane.header.stamp = rearRightLane.header.stamp = ros::Time::now();
    frontLeftLane.header.frame_id = frontRightLane.header.frame_id =
        rearLeftLane.header.frame_id = rearRightLane.header.frame_id = "base_footprint";

    // calc velocity to advance bicycle model in dependence of vehicle width
    const double dtStep_s = 0.050;
    const int nofSteps = 40;
    const double lengthPred_m = 5.0 * _vehicleParams->get_width();
    const double lengthStep_m = lengthPred_m / nofSteps;
    const double vel_mps = lengthStep_m / dtStep_s;

    if (msg.gearPosition == eGearPosition::GEARPOSITION_REVERSE) {
        direction = -1;
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.z = 0.0;
    pose.header = frontLeftLane.header;
    pose.pose.orientation.w = 1.0;

    const float rwa = tod_helper::Vehicle::Model::swa2rwa(
        msg.steeringWheelAngle, _vehicleParams->get_max_swa_rad(), _vehicleParams->get_max_rwa_rad());
    const double distRear = _vehicleParams->get_distance_rear_bumper();
    const double distFront = _vehicleParams->get_distance_front_bumper();
    const double beta = std::atan(distRear * std::tan(rwa) / (distFront + distRear));
    double xCoM{0.0}, yCoM{0.0}, yawCoM{0.0};
    for (int i=0; i < nofSteps; ++i) {
        // advance CoM position
        xCoM += direction * dtStep_s * vel_mps * std::cos(yawCoM + beta);
        yCoM += direction * dtStep_s * vel_mps * std::sin(yawCoM + beta);
        yawCoM += direction * dtStep_s * vel_mps * std::sin(beta) / distRear;

        double xfl, yfl, xfr, yfr, xrl, yrl, xrr, yrr;
        tod_helper::Vehicle::Model::calc_vehicle_front_edges(
            xCoM, yCoM, yawCoM, distFront, _vehicleParams->get_width(), xfl, yfl, xfr, yfr);
        tod_helper::Vehicle::Model::calc_vehicle_rear_edges(
            xCoM, yCoM, yawCoM, distRear, _vehicleParams->get_width(), xrl, yrl, xrr, yrr);

        pose.pose.position.x = xfl;
        pose.pose.position.y = yfl;
        frontLeftLane.poses.push_back(pose);

        pose.pose.position.x = xfr;
        pose.pose.position.y = yfr;
        frontRightLane.poses.push_back(pose);

        pose.pose.position.x = xrl;
        pose.pose.position.y = yrl;
        rearLeftLane.poses.push_back(pose);

        pose.pose.position.x = xrr;
        pose.pose.position.y = yrr;
        rearRightLane.poses.push_back(pose);
    }
    _pubVehicleLaneFL.publish(frontLeftLane);
    _pubVehicleLaneFR.publish(frontRightLane);
    _pubVehicleLaneRL.publish(rearLeftLane);
    _pubVehicleLaneRR.publish(rearRightLane);
    ROS_INFO_ONCE("%s: Published first set of vehicle lanes to ROS!", ros::this_node::getName().c_str());
}
