#include "telecarla_rpc_server.h"

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <fstream>  // For logging

#include "bool.h"
#include "rpc_msg_callback.h"
#include "vehicle_status.h"

using namespace lmt;

TeleCarlaRpcServer::TeleCarlaRpcServer(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : server_("10.131.222.209", pnh.param("rpc_port", 2002)),  // Bind to server IP and port
      egoVehicleStatus_(boost::make_shared<carla_msgs::CarlaEgoVehicleStatus>())
{
    ROS_INFO_STREAM("Opening RPC Server at IP 10.131.222.209 and port " << pnh.param("rpc_port", 2002));

    const auto prefix = "/carla/" + pnh.param("role_name", std::string("ego_vehicle"));

    server_.bind("set_enable_autopilot",
                 [this, &nh, prefix](const bool& data, double send_time_sec) {
                     ros::Time receive_time = ros::Time::now();
                     double latency_ms = (receive_time.toSec() - send_time_sec) * 1000.0;

                     // Log latency to console and file
                     ROS_INFO_STREAM("Received enable_autopilot with latency: " << latency_ms << " ms");
                     std::ofstream log_file("latency_log.csv", std::ios::app);
                     log_file << "enable_autopilot," << send_time_sec << "," << receive_time.toSec() << "," << latency_ms << "\n";
                     log_file.close();

                     std_msgs::Bool msg;
                     msg.data = data;
                     nh.advertise<std_msgs::Bool>(prefix + "/enable_autopilot", 1).publish(msg);
                 });
    ROS_INFO_STREAM("Provide RPC call set_enable_autopilot -> " << prefix << "/enable_autopilot");

    server_.bind("set_vehicle_control_manual_override",
                 [this, &nh, prefix](const bool& data, double send_time_sec) {
                     ros::Time receive_time = ros::Time::now();
                     double latency_ms = (receive_time.toSec() - send_time_sec) * 1000.0;

                     // Log latency to console and file
                     ROS_INFO_STREAM("Received vehicle_control_manual_override with latency: " << latency_ms << " ms");
                     std::ofstream log_file("latency_log.csv", std::ios::app);
                     log_file << "vehicle_control_manual_override," << send_time_sec << "," << receive_time.toSec() << "," << latency_ms << "\n";
                     log_file.close();

                     std_msgs::Bool msg;
                     msg.data = data;
                     nh.advertise<std_msgs::Bool>(prefix + "/vehicle_control_manual_override", 1).publish(msg);
                 });
    ROS_INFO_STREAM("Provide RPC call set_vehicle_control_manual_override -> " << prefix << "/vehicle_control_manual_override");

    server_.bind("set_vehicle_control_cmd_manual",
                 [this, &nh, prefix](float throttle, float steer, float brake, double send_time_sec) {
                     ros::Time receive_time = ros::Time::now();
                     double latency_ms = (receive_time.toSec() - send_time_sec) * 1000.0;

                     // Log latency to console and file
                     ROS_INFO_STREAM("Received vehicle_control_cmd_manual with latency: " << latency_ms << " ms");
                     std::ofstream log_file("latency_log.csv", std::ios::app);
                     log_file << "vehicle_control_cmd_manual," << send_time_sec << "," << receive_time.toSec() << "," << latency_ms << "\n";
                     log_file.close();

                     carla_msgs::CarlaEgoVehicleControl msg;
                     msg.throttle = throttle;
                     msg.steer = steer;
                     msg.brake = brake;
                     nh.advertise<carla_msgs::CarlaEgoVehicleControl>(prefix + "/vehicle_control_cmd_manual", 1).publish(msg);
                 });
    ROS_INFO_STREAM("Provide RPC call set_vehicle_control_cmd_manual -> " << prefix << "/vehicle_control_cmd_manual");

    server_.bind("get_vehicle_status", [this]() { return data::VehicleStatus(egoVehicleStatus_); });
    ROS_INFO("Provide RPC call get_vehicle_status");

    vehicleStatusSubscriber_ = nh.subscribe<carla_msgs::CarlaEgoVehicleStatus>(
        prefix + "/vehicle_status", 1, &TeleCarlaRpcServer::vehicleStatusCallback, this);
    ROS_INFO_STREAM("Subscribed to " << prefix << "/vehicle_status");
}

void TeleCarlaRpcServer::run()
{
    server_.async_run();
}

void TeleCarlaRpcServer::stop()
{
    server_.close_sessions();
    server_.stop();
}

void TeleCarlaRpcServer::vehicleStatusCallback(const carla_msgs::CarlaEgoVehicleStatusConstPtr& vehicleStatus)
{
    egoVehicleStatus_ = vehicleStatus;
}

