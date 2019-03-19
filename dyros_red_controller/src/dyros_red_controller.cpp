/**
  @title DYROS RED Controller
  @authors JunHee Ahn
  */

#include <ros/ros.h>
#include "dyros_red_controller/simulation_interface.h"
#include "dyros_red_controller/mujoco_interface.h"

using namespace dyros_red_controller;

#include <math.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dyros_red_controller");
  ros::NodeHandle nh("~");

  std::string mode;
  nh.param<std::string>("run_mode", mode, "simulation");
  ControlBase *ctr_obj;

  bool debug_mode;
  nh.param<bool>("debug", debug_mode, false);

  bool rviz_publisher;
  nh.param<bool>("rviz_pub", rviz_publisher, false);

  bool force_reset;
  nh.param<bool>("force_reset", force_reset, false);

  ROS_WARN_COND(debug_mode, "//////////////////////////////////////////\n   DEBUG MODE IS ON \n//////////////////////////////////////////");
  ROS_WARN_COND(rviz_publisher, "//////////////////////////////////////////\n   rviz_publisher MODE IS ON \n//////////////////////////////////////////");
  double Hz;
  nh.param<double>("control_frequency", Hz, 400.0);

  //ctr_obj = new SimulationInterface(nh, Hz);
  ctr_obj = new mujoco_interface(nh, Hz);

  ctr_obj->debug = debug_mode;
  ctr_obj->rviz_pub = rviz_publisher;

  while (ros::ok())
  {
    ros::Time time_temp = ros::Time::now();
    ctr_obj->readDevice();
    double readDevice_time = ros::Time::now().toSec() - time_temp.toSec();

    time_temp = ros::Time::now();
    ctr_obj->update();
    double update_time = ros::Time::now().toSec() - time_temp.toSec();

    time_temp = ros::Time::now();
    ctr_obj->compute();
    double compute_time = ros::Time::now().toSec() - time_temp.toSec();

    time_temp = ros::Time::now();
    ctr_obj->reflect();
    double reflect_time = ros::Time::now().toSec() - time_temp.toSec();

    time_temp = ros::Time::now();
    ctr_obj->writeDevice();
    double writeDevice_time = ros::Time::now().toSec() - time_temp.toSec();

    time_temp = ros::Time::now();
    ctr_obj->wait();
    double wait_time = ros::Time::now().toSec() - time_temp.toSec();
    double total_time = readDevice_time + update_time + compute_time + reflect_time + writeDevice_time + wait_time;
    double simultation_real_Hz = 1 / total_time;
    ROS_INFO_COND(debug_mode, "\n:::::::::::::::::::: TIME INFO ::::::::::::::::::\n Current Simulation Frequency : %4.2f Hz \n Realtime Factor : %4.2f \n\n readDevice : %3.4f ms\n update : %3.4f ms\n compute : %3.4f ms\n reflect : %3.4f ms\n writeDevice : %3.4f ms\n wait : %3.4f ms\n", simultation_real_Hz, Hz / simultation_real_Hz, readDevice_time * 1000, update_time * 1000, compute_time * 1000, reflect_time * 1000, writeDevice_time * 1000, wait_time * 1000);
  }

  delete ctr_obj;

  return 0;
}
