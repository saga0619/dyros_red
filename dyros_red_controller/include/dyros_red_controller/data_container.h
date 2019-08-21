#ifndef DATA_CONTAINER_H
#define DATA_CONTAINER_H

#include <chrono>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <pthread.h>
#include <mutex>
#include <future>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <mujoco_ros_msgs/SensorState.h>
#include <mujoco_ros_msgs/JointSet.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <ncurses.h>
#include "dyros_red_controller/link.h"

////////////////////////////////////////////////////////////////////////////////////////////////////
//robot definition variables here

namespace RED
{
const std::string JOINT_NAME[MODEL_DOF] = {
    "HipRoll_L_Joint", "HipCenter_L_Joint", "Thigh_L_Joint",
    "Knee_L_Joint", "AnkleCenter_L_Joint", "AnkleRoll_L_Joint",
    "HipRoll_R_Joint", "HipCenter_R_Joint", "Thigh_R_Joint",
    "Knee_R_Joint", "AnkleCenter_R_Joint", "AnkleRoll_R_Joint"};

const std::string ACTUATOR_NAME[MODEL_DOF] = {
    "L_HipRoll_Motor", "L_HipCenter_Motor", "L_Thigh_Motor",
    "L_Knee_Motor", "L_AnkleCenter_Motor", "L_AnkleRoll_Motor",
    "R_HipRoll_Motor", "R_HipCenter_Motor", "R_Thigh_Motor",
    "R_Knee_Motor", "R_AnkleCenter_Motor", "R_AnkleRoll_Motor"};

static constexpr const char *LINK_NAME[32] = {
    "Pelvis_Link",
    "HipRoll_L_Link", "HipCenter_L_Link", "Thigh_L_Link", "Knee_L_Link", "AnkleCenter_L_Link", "AnkleRoll_L_Link",
    "HipRoll_R_Link", "HipCenter_R_Link", "Thigh_R_Link", "Knee_R_Link", "AnkleCenter_R_Link", "AnkleRoll_R_Link"};
} // namespace RED

const int Pelvis = 0;

const int Left_Foot = 6;
const int Right_Foot = 12;

const int COM_id = 13;
////////////////////////////////////////////////////////////////////////////////////////////////////
//
//Terminal Data Que
class TQue
{
public:
  bool update;
  bool clr_line;
  int x;
  int y;
  //std::string text;
  char text[256];
};

//Robot data Storage
class DataContainer
{
public:
  ros::NodeHandle nh;
  //Basic var
  bool simulation = true;
  bool shutdown = false;
  bool connected = false;
  bool firstcalcdone = false;

  bool ncurse_mode = false;

  bool statemanager_ready = false;

  std::string mode;

  //Tui Var..
  bool state_end;
  bool dynamics_end;
  TQue Tq_[100];

  double time;
  double com_time;
  double sim_time;

  int dym_hz;
  std::chrono::microseconds dym_timestep;

  int stm_hz;
  std::chrono::microseconds stm_timestep;
  bool check = false;

  Eigen::VectorQd q_;
  Eigen::VectorQVQd q_virtual_;
  Eigen::VectorQd q_dot_;
  Eigen::VectorVQd q_dot_virtual_;
  Eigen::VectorVQd q_ddot_virtual_;
  Eigen::VectorQd torque_;

  //Command Var
  Eigen::VectorQd torque_desired;

  //Kinematics Information :
  //MODEL RED;
  KinematicsData red_;

  //Kinematics Information :
  Link link_[LINK_NUMBER + 1];

  double yaw_radian;

  Eigen::MatrixVVd A_;
  Eigen::MatrixVVd A_inv;

  Com com_;

  //Model var

  //For real robot
  std::string ifname;
  int ctime;

  Eigen::VectorQd q_init_;
  Eigen::VectorQd q_elmo_;
  Eigen::VectorQd torqueElmo;
  Eigen::VectorQd torqueDemandElmo;
  Eigen::VectorQd positionDesired;
  Eigen::VectorQd accel_dif;
  Eigen::VectorQd accel_obsrvd;
  Eigen::VectorQd currentGain;

  int elmo_cnt;

  //Gui Command
  std::string command;

  double commandTime = 0.0;
  double commandTimeLock = -1.0;

  //Hardware switch

  bool torqueOn = false;
  bool torqueOff = false;
  bool emergencyoff = false;
  double t_gain = 0.0;
  double torqueOnTime = 0.0;
  double torqueOffTime = 0.0;

  //Simulation switch

  bool pubmode = false;       // Publish mode of mujoco, integrated mode(basic), detached mode.
  bool checkfreqency = false; // check running frequency of state thread and dynamics thread.

  bool testmode = false; // switch for controller test mode.

  //Controller switch

  bool positionControl = false;
  bool gravityMode = false;
  bool customGain = false;
  bool fixedgravity = false;
  bool torqueredis = false;

  bool spalarm = false; // support polygon alarm bool
  bool semode = false;  // state estimation running or not.

  bool initialize_request = false;
};

#endif