#ifndef STATE_MANAGER_H
#define STATE_MANAGER_H

#include "dyros_red_controller/data_container.h"
#include "dyros_red_controller/terminal.h"

extern std::mutex mtx;
extern std::mutex mtx_rbdl;
extern std::mutex mtx_dc;

class StateManager
{
public:
  StateManager(DataContainer &dc_global);
  virtual ~StateManager() {}
  DataContainer &dc;
  virtual void connect();
  virtual void stateThread(); //main thread managing state
  virtual void updateState();
  //virtual void sendCommand(Eigen::VectorQd command);
  virtual void sendCommand(Eigen::VectorQd command, double sim_time);

  //initialize variables
  virtual void initialize();
  //store data at container
  void storeState();

  //private functions

  //update kinematic information with RBDL
  void updateKinematics(const Eigen::VectorXd &q_virtual, const Eigen::VectorXd &q_dot_virtual, const Eigen::VectorXd &q_ddot_virtual);

  //testThread to test multithread
  void testThread();

  //private variables

  unsigned int link_id_[40];

  double control_time_;
  double sim_time_;

  Eigen::VectorQd q_;
  Eigen::VectorQd q_init_;
  Eigen::VectorQVQd q_virtual_;
  Eigen::VectorQd q_dot_;
  Eigen::VectorVQd q_dot_virtual_;
  Eigen::VectorVQd q_ddot_virtual_;
  Eigen::VectorQd torque_;
  Eigen::VectorQd torque_desired;

  double yaw_radian;

  Eigen::MatrixVVd A_;
  Eigen::MatrixVVd A_inv;
  Eigen::MatrixXd A_temp_;

  Eigen::Vector3d gravity_;

  RigidBodyDynamics::Model model_;

  Link link_[LINK_NUMBER + 1];
  Com com_;

  //Communication Subscriber!

  ros::Subscriber gui_command;
  ros::Publisher joint_states_pub;
  ros::Publisher time_pub;
  sensor_msgs::JointState joint_state_msg;
  std_msgs::Float32 time_msg;

  void CommandCallback(const std_msgs::StringConstPtr &msg);
};

#endif