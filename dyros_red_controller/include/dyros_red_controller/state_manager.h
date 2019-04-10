#include "dyros_red_controller/data_container.h"
#include "dyros_red_controller/terminal.h"

class StateManager
{
public:
  DataContainer &dc;
  StateManager(DataContainer &dc_global);

  virtual bool connect();
  virtual void stateThread(); //main thread managing state
  virtual void updateState();
  virtual void sendCommand(Eigen::VectorQd command);

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
  Eigen::VectorQVQd q_virtual_;
  Eigen::VectorQd q_dot_;
  Eigen::VectorVQd q_dot_virtual_;
  Eigen::VectorVQd q_ddot_virtual_;
  Eigen::VectorQd torque_;
  Eigen::VectorQd torque_desired;

  double yaw_radian;

  Eigen::Matrix37d A_;
  Eigen::MatrixXd A_temp_;

  Eigen::Vector3d gravity_;

  RigidBodyDynamics::Model model_;

  Link link_[LINK_NUMBER + 1];
  Com com_;
};
