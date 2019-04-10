#include "dyros_red_controller/dynamics_manager.h"
#include "dyros_red_controller/mujoco_interface.h"

class RedController
{
public:
  RedController(DataContainer &dc_global, StateManager &sm, DynamicsManager &dm);

  void stateThread();
  void dynamicsThreadLow();
  void dynamicsThreadHigh();
  void tuiThread();
  DataContainer &dc;

private:
  void getState();
  void initialize();
  StateManager &s_;
  DynamicsManager &d_;

  bool connected;

  //sim variables
  double time;
  double sim_time;
  int dym_hz, stm_hz;

  Eigen::VectorQd q_;
  Eigen::VectorQVQd q_virtual_;
  Eigen::VectorQd q_dot_;
  Eigen::VectorVQd q_dot_virtual_;
  Eigen::VectorVQd q_ddot_virtual_;
  Eigen::VectorQd torque_;
  //Command Var
  Eigen::VectorQd torque_desired;
  //Kinematics Information :
  Link link_[LINK_NUMBER + 1];
  double yaw_radian;
  Eigen::Matrix37d A_;
  Eigen::Matrix37d A_inv_;
  Com com_;
};