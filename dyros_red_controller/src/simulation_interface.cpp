#include "dyros_red_controller/simulation_interface.h"

namespace dyros_red_controller
{

SimulationInterface::SimulationInterface(ros::NodeHandle &nh, double Hz):
  ControlBase(nh, Hz), rate_(Hz), simulation_step_done_(false)
{
  simulation_running_= true;
  simulation_time_ = 0.0f; // set initial simulation time

  vrep_sim_start_pub_ = nh.advertise<std_msgs::Bool>("/startSimulation", 5);
  vrep_sim_stop_pub_ = nh.advertise<std_msgs::Bool>("/stopSimulation", 5);
  vrep_sim_step_trigger_pub_ = nh.advertise<std_msgs::Bool>("/triggerNextStep", 100);
  vrep_sim_enable_syncmode_pub_ = nh.advertise<std_msgs::Bool>("/enableSyncMode", 5);


  vrep_sim_step_done_sub_ = nh.subscribe("/simulationStepDone", 100, &SimulationInterface::simulationStepDoneCallback, this);

  imu_sub_ = nh.subscribe("/vrep_ros_interface/imu", 100, &SimulationInterface::imuCallback, this);
  joint_sub_ = nh.subscribe("/vrep_ros_interface/joint_state", 100, &SimulationInterface::jointCallback, this);
  left_ft_sub_ = nh.subscribe("/vrep_ros_interface/left_foot_ft", 100, &SimulationInterface::leftFTCallback, this);
  right_ft_sub_ = nh.subscribe("/vrep_ros_interface/right_foot_ft", 100, &SimulationInterface::rightFTCallback, this);

  vrep_joint_set_pub_ = nh.advertise<sensor_msgs::JointState>("/vrep_ros_interface/joint_set", 1);

  joint_set_msg_.name.resize(total_dof_);
  joint_set_msg_.position.resize(total_dof_);
  joint_set_msg_.effort.resize(total_dof_);
  for(int i=0; i<total_dof_; i++)
  {
    joint_set_msg_.name[i] = DyrosRedModel::JOINT_NAME[i];
  }
  ros::Rate poll_rate(100);

  ROS_INFO("Waiting for connection of V-REP ROS Interface");

  while(vrep_sim_enable_syncmode_pub_.getNumSubscribers() == 0 && ros::ok())
    poll_rate.sleep();
  while(vrep_sim_start_pub_.getNumSubscribers() == 0 && ros::ok())
    poll_rate.sleep();

  ROS_INFO(" -- Connected -- ");
  vrepEnableSyncMode();
  vrepStart();

  ROS_INFO(" -- init end -- ");

}

void SimulationInterface::vrepStart()
{
  std_msgs::Bool msg;
  msg.data = true;
  vrep_sim_start_pub_.publish(msg);
}

void SimulationInterface::vrepStop()
{
  std_msgs::Bool msg;
  msg.data = true;
  vrep_sim_stop_pub_.publish(msg);
}

void SimulationInterface::vrepStepTrigger()
{
  std_msgs::Bool msg;
  msg.data = true;
  vrep_sim_step_trigger_pub_.publish(msg);
}

void SimulationInterface::vrepEnableSyncMode()
{
  std_msgs::Bool msg;
  msg.data = true;
  vrep_sim_enable_syncmode_pub_.publish(msg);
}

// Function implement
void SimulationInterface::update()
{
  ControlBase::update();

}
void SimulationInterface::compute()
{
  ControlBase::compute();
}

void SimulationInterface::writeDevice()
{


  for(int i=0;i<total_dof_;i++) {
    //std::cout<<"for loop"<<std::endl;
    //std::cout<<"done : "<<i<<" ::: effort : " <<torque_desired(i)<<std::endl;
    joint_set_msg_.position[i] = position_desired(i);
    joint_set_msg_.effort[i] = torque_desired(i);


  }
  vrep_joint_set_pub_.publish(joint_set_msg_);
  vrepStepTrigger();
}

void SimulationInterface::wait()
{
  // Wait for step done
  while(ros::ok() && !simulation_step_done_)
  {
    ros::spinOnce();
  }
  simulation_step_done_ = false;
  rate_.sleep();
}


// Callback functions



void SimulationInterface::simulationTimeCallback(const std_msgs::Float32ConstPtr& msg)
{
  ROS_INFO_ONCE("simtimeCB");
  simulation_time_ = msg->data;
}

void SimulationInterface::simulationStepDoneCallback(const std_msgs::BoolConstPtr &msg)
{
  ROS_INFO_ONCE("SimStepDoneCB");
  simulation_step_done_ = msg->data;
}

void SimulationInterface::jointCallback(const sensor_msgs::JointStateConstPtr& msg)
{
  ROS_INFO_ONCE("jointCB");
  for(int i=0; i<total_dof_; i++)
  {
    for (int j=0; j<msg->name.size(); j++)
    {
      if(DyrosRedModel::JOINT_NAME[i] == msg->name[j].data())
      {

        q_(i) = msg->position[j];
        q_virtual_(i+6) = msg->position[j];

        q_dot_(i) = msg->velocity[j];
        q_dot_virtual_(i+6)=msg->velocity[j];
        torque_(i) = msg->effort[j];
      }
    }
  }
}

void SimulationInterface::leftFTCallback(const geometry_msgs::WrenchStampedConstPtr& msg)
{

}

void SimulationInterface::rightFTCallback(const geometry_msgs::WrenchStampedConstPtr& msg)
{

}

void SimulationInterface::imuCallback(const sensor_msgs::ImuConstPtr &msg)
{

  ROS_INFO_ONCE("imuCB");

  Pelvis_quaternion.x() = msg->orientation.x;
  Pelvis_quaternion.y() = msg->orientation.y;
  Pelvis_quaternion.z() = msg->orientation.z;
  Pelvis_quaternion.w() = msg->orientation.w;


  pelvis_orientation_ = Pelvis_quaternion.normalized().toRotationMatrix();

  Eigen::Vector3d euler_ = Pelvis_quaternion.normalized().toRotationMatrix().eulerAngles(0,1,2);



  Pelvis_linear_velocity_(0) = msg->linear_acceleration.x;
  Pelvis_linear_velocity_(1) = msg->linear_acceleration.y;
  Pelvis_linear_velocity_(2) = msg->linear_acceleration.z;

  Pelvis_angular_velocity_(0) = msg->angular_velocity.x;
  Pelvis_angular_velocity_(1) = msg->angular_velocity.y;
  Pelvis_angular_velocity_(2) = msg->angular_velocity.z;
  for(int i=0;i<3;i++){
    q_dot_virtual_(i+3)=Pelvis_angular_velocity_(i);
  }


  //std::cout<<"q virtual "<<std::endl<<q_virtual_<<std::endl;
  //std::cout<<"pelv R " <<std::endl<<pelvis_orientation_<<std::endl;

  //model_.setquat(Pelvis_quaternion,q_virtual_);


  q_virtual_(3)=euler_(0);
  q_virtual_(4)=euler_(1);
  q_virtual_(5)=euler_(2);






  //std::cout<<"q virtual quat added " <<std::endl<<q_virtual_<<std::endl;


}

}

