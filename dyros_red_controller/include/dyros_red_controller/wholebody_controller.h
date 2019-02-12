#ifndef WHOLEBODY_CONTROLLER_H
#define WHOLEBODY_CONTROLLER_H

#include "dyros_red_controller/dyros_red_model.h"
#include "math_type_define.h"

#include "dyros_red_controller/quadraticprogram.h"
#include <qpOASES.hpp>

using namespace Eigen;
using namespace std;
using namespace qpOASES;

namespace dyros_red_controller
{

class Wholebody_controller
{
public:
  //walking_controller();

  Wholebody_controller(DyrosRedModel &model, const VectorQd &current_q, const double hz, const double &control_time, const double &d_time);
  unsigned int total_dof_;
  DyrosRedModel &model_;

  // motion time
  const double hz_;
  const double &control_time_; // updated by control_base
  const double &d_time_;
  double start_time_[4];
  double end_time_[4];
  bool target_arrived_[4];
  bool debug;
  const VectorQd &current_q_; // updated by control_base

  //Main loop wholebody function

  //update mass matrix
  void update_dynamics();

  //set contact status of robot. true for contact false for not contact
  void contact_set_multi(bool right_foot, bool left_foot, bool right_hand, bool left_hand);

  //contact force redistribution by yisoolee method at 2 contact(both foot)
  VectorQd contact_force_redistribution_torque(double yaw_radian, VectorQd command_torque, Eigen::Vector12d &ForceRedistribution, double &eta);

  //set contact force to desired contact force
  VectorQd contact_force_custom(VectorQd command_torque, Eigen::VectorXd contact_force_now, Eigen::VectorXd contact_force_desired);

  //update gravity compensation torque
  VectorQd gravity_compensation_torque();

  //get contact redistribution torque with Quadratic programing
  VectorQd contact_torque_calc_from_QP(VectorQd command_torque);

  // Get Contact Redistribution Torque with QP. Wall contact mode.
  VectorQd contact_torque_calc_from_QP_wall(VectorQd command_torque, double wall_friction_ratio);

  //Get Contact Redistribution Torque with QP. Wall contact mode.

  VectorQd contact_torque_calc_from_QP_wall_mod2(VectorQd command_torque, double wall_friction_ratio);

  /*
  * Get Task Control Torque.
  * task jacobian and f_star must be defined. 
  */
  VectorQd task_control_torque(Eigen::MatrixXd J_task, Eigen::VectorXd f_star_);

  /*
  * Get Task Control Torque 
  * task jacobian and f_star must be defined. 
  */
  VectorQd task_control_torque_custom_force(MatrixXd J_task, VectorXd f_star_, MatrixXd selection_matrix, VectorXd desired_force);

  // Get Task Control Torque task jacobian and f_star must be defined.
  VectorQd task_control_torque_custom_force_feedback(MatrixXd J_task, VectorXd f_star_, MatrixXd selection_matrix, VectorXd desired_force, VectorXd ft_hand);

  //force control with selection matrix. selec 1 for control with fstar 0 for force control
  void set_force_control(MatrixXd selection_matrix, VectorXd desired_force);

  //force control selection matrix 1 for control with fstar 0 for force control
  void set_force_control_feedback(MatrixXd selection_matrix, VectorXd desired_force, VectorXd ft_hand);
  void set_zmp_control(Vector2d ZMP, double gain);

  MatrixXd task_selection_matrix;
  VectorXd task_desired_force;
  VectorXd task_feedback_reference;
  Vector2d ZMP_task;
  bool task_force_control = false;
  bool task_force_control_feedback = false;
  bool zmp_control = false;
  double zmp_gain;
  bool mpc_init = false;

  //Utility functions

  //Get contact force from command torque
  VectorXd get_contact_force(VectorQd command_torque);

  //Get ZMP position from contact forces and both foot position
  Vector3d GetZMPpos(Vector3d P_right, Vector3d P_left, Vector12d ContactForce);

  //Eigen::Vector6d Getfstar( );
  Vector3d getfstar(Vector3d kp, Vector3d kd, Vector3d p_desired, Vector3d p_now, Vector3d d_desired, Vector3d d_now);
  Vector3d getfstar(Vector3d kp, Vector3d kd, Matrix3d r_desired, Matrix3d r_now, Vector3d w_desired, Vector3d w_now);
  Vector3d getfstar_tra(int link_id, Vector3d kpt, Vector3d kdt);
  Vector3d getfstar_rot(int link_id, Vector3d kpa, Vector3d kda);
  Vector6d getfstar6d(int link_id, Vector3d kpt, Vector3d kdt, Vector3d kpa, Vector3d kda);

  //zmp controller
  VectorQd CP_control_init(double dT);
  VectorQd CP_controller();
  Vector6d zmp_controller(Vector2d ZMP, double height);
  Vector2d CP_ref[20];

  //Vector2d getcptraj(double time, Vector2d zmp);

  Vector2d getcpref(double task_time, double future_time);
  //Contact Mode
  const int DOUBLE_SUPPORT = 0;

  const int SINGLE_SUPPORT_LEFT = 1;
  const int SINGLE_SUPPORT_RIGHT = 2;
  const int TRIPPLE_SUPPORT = 3;
  const int QUAD_SUPPORT = 4;

  int contact_index;
  int contact_part[4];

  MatrixXd A_matrix;
  MatrixXd A_matrix_inverse;
  MatrixXd J_C, J_C_INV_T;
  MatrixXd J_COM;

  MatrixXd J_task;
  VectorXd f_star;

  MatrixXd Lambda_c;
  MatrixXd N_C;
  MatrixXd I37;

  VectorXd contact_force_predict;

  Vector3d Grav_ref;

  MatrixXd J_task_T, J_task_inv, J_task_inv_T;
  MatrixXd lambda_inv, lambda;
  MatrixXd W, W_inv;
  MatrixXd Q, Q_T_, Q_temp, Q_temp_inv, Jtemp, Jtemp_2;
  MatrixXd _F;

  VectorXd G;

  MatrixXd Slc_k, Slc_k_T;
  MatrixXd svd_U;

  int task_dof;

  Vector2d p_k_1;
  Vector3d ZMP_pos;

  double fc_redis;

  //QP solver setting
  void QPInitialize();
  void QPReset();
  int nIter;
  CQuadraticProgram QP_test;
  CQuadraticProgram QP_mpc;
  VectorXd result_temp;

private:
  //update contact space dynamics
  void contact_set(int contact_number, int link_id[]);

  void ForceRedistributionTwoContactMod2(double eta_cust, double footlength, double footwidth, double staticFrictionCoeff, double ratio_x, double ratio_y, Eigen::Vector3d P1, Eigen::Vector3d P2, Eigen::Vector12d &F12, Eigen::Vector6d &ResultantForce, Eigen::Vector12d &ForceRedistribution, double &eta);
  void ForceRedistributionTwoContactMod(double eta_cust, double footlength, double footwidth, double staticFrictionCoeff, double ratio_x, double ratio_y, Eigen::Vector3d P1, Eigen::Vector3d P2, Eigen::Vector12d &F12, Eigen::Vector6d &ResultantForce, Eigen::Vector12d &ForceRedistribution, double &eta);
};

} // namespace dyros_red_controller

#endif // WALKING_CONTROLLER_H
