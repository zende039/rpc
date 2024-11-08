#ifndef TELECARLA_RPC_TELECARLA_RPC_CLIENT_H
#define TELECARLA_RPC_TELECARLA_RPC_CLIENT_H

#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <rpc/client.h>
#include <std_msgs/Bool.h>
#include <carla_msgs/CarlaEgoVehicleStatus.h>
#include <carla_msgs/CarlaEgoVehicleControl.h>

namespace lmt
{
class TeleCarlaRpcClient
{
  public:
    TeleCarlaRpcClient(ros::NodeHandle& nh, ros::NodeHandle& pnh);

    void update();

  private:
    std::string host_;
    int port_;
    rpc::client client_;
    std::vector<ros::Subscriber> subscribers_;

    ros::Publisher vehicleStatusPublisher_;
    ros::Publisher clientEnableAutopilotPublisher_;
    ros::Publisher clientManualOverridePublisher_;
    ros::Publisher clientVehicleControlCmdPublisher_;
};
}  // namespace lmt

#endif  // TELECARLA_RPC_TELECARLA_RPC_CLIENT_H

