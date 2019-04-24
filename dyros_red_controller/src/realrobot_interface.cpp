#include "dyros_red_controller/realrobot_interface.h"

RealRobotInterface::RealRobotInterface(DataContainer &dc_global) : dc(dc_global), StateManager(dc_global)
{
    imuSubscriber = dc.nh.subscribe("/imu/data", 1, &RealRobotInterface::ImuCallback, this);
}

void RealRobotInterface::updateState()
{
}

void RealRobotInterface::sendCommand(Eigen::VectorQd command, double sim_time)
{
}

bool RealRobotInterface::connect()
{
}

void RealRobotInterface::ImuCallback(const sensor_msgs::ImuConstPtr &msg)
{
    //msg->orientation;
}