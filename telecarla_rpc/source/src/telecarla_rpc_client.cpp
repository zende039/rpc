#include "telecarla_rpc_client.h"

#include <std_msgs/Bool.h>
#include <ros/ros.h>

#include "bool.h"
#include "ros_msg_callback.h"
#include "vehicle_status.h"

using namespace lmt;

TeleCarlaRpcClient::TeleCarlaRpcClient(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : host_("10.131.222.209"),  // Set the server IP here
      port_(pnh.param("rpc_port", 2002)),
      client_(host_, port_)
{
    while (client_.get_connection_state() == rpc::client::connection_state::initial)
    {
        ROS_INFO_STREAM("Connecting to " << host_ << ":" << port_ << "...");
    }
    if (client_.get_connection_state() == rpc::client::connection_state::connected)
    {
        ROS_INFO_STREAM("Connected to " << host_ << ":" << port_);
    }
    else
    {
        ROS_ERROR_STREAM("Failed to connect to " << host_ << ":" << port_);
    }

    const auto prefix = "/carla/" + pnh.param("role_name", std::string("ego_vehicle"));

    // Timestamp and send enable_autopilot message
    subscribers_.push_back(nh.subscribe<std_msgs::Bool>(
        prefix + "/enable_autopilot",
        1,
        [this](const std_msgs::BoolConstPtr& msg) {
            ros::Time send_time = ros::Time::now();
            client_.call("set_enable_autopilot", msg->data, send_time.toSec());
            ROS_INFO_STREAM("Sent enable_autopilot with timestamp: " << send_time.toSec());
        }));
    ROS_INFO_STREAM("Subscribed to topic " << prefix + "/enable_autopilot -> set_enable_autopilot");

    // Timestamp and send vehicle_control_manual_override message
    subscribers_.push_back(nh.subscribe<std_msgs::Bool>(
        prefix + "/vehicle_control_manual_override",
        1,
        [this](const std_msgs::BoolConstPtr& msg) {
            ros::Time send_time = ros::Time::now();
            client_.call("set_vehicle_control_manual_override", msg->data, send_time.toSec());
            ROS_INFO_STREAM("Sent vehicle_control_manual_override with timestamp: " << send_time.toSec());
        }));
    ROS_INFO_STREAM("Subscribed to topic " << prefix + "/vehicle_control_manual_override -> set_vehicle_control_manual_override");

    // Timestamp and send vehicle_control_cmd_manual message
    subscribers_.push_back(nh.subscribe<carla_msgs::CarlaEgoVehicleControl>(
        prefix + "/vehicle_control_cmd_manual",
        1,
        [this](const carla_msgs::CarlaEgoVehicleControlConstPtr& msg) {
            ros::Time send_time = ros::Time::now();
            client_.call("set_vehicle_control_cmd_manual", msg->throttle, msg->steer, msg->brake, send_time.toSec());
            ROS_INFO_STREAM("Sent vehicle_control_cmd_manual with timestamp: " << send_time.toSec());
        }));
    ROS_INFO_STREAM("Subscribed to topic " << prefix + "/vehicle_control_cmd_manual -> set_vehicle_control_cmd_manual");

    vehicleStatusPublisher_ = nh.advertise<carla_msgs::CarlaEgoVehicleStatus>(prefix + "/vehicle_status", 1);
}

void TeleCarlaRpcClient::update()
{
    auto status = client_.call("get_vehicle_status").as<data::VehicleStatus>();
    vehicleStatusPublisher_.publish(status.toROSMessage());
}

