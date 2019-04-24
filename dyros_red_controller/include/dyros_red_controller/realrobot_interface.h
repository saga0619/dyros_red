#ifndef MUJOCO_INTERFACE_H
#define MUJOCO_INTERFACE_H

#include "state_manager.h"

class RealRobotInterface : public StateManager
{
public:
    RealRobotInterface(DataContainer &dc_global);
    virtual ~RealRobotInterface() {}

    //update state of Robot from mujoco
    virtual void updateState() override;

    //Send command to Mujoco
    //virtual void sendCommand(Eigen::VectorQd command) override;
    virtual void sendCommand(Eigen::VectorQd command, double sim_time) override;

    //connect to Mujoco_ros
    virtual bool connect() override;

    //Toggle play
    //void playMujoco();

private:
    DataContainer &dc;

    void ImuCallback(const sensor_msgs::ImuConstPtr &msg);

    ros::Subscriber imuSubscriber;

    mujoco_ros_msgs::JointSet mujoco_joint_set_msg_;

    bool sim_runnung;

    std::string joint_name_mj[MODEL_DOF];
    //ros::Rate rate_;
    int dyn_hz;
};

#endif