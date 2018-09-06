#ifndef WHOLEBODY_CONTROLLER_H
#define WHOLEBODY_CONTROLLER_H

#include "dyros_red_controller/dyros_red_model.h"
#include "math_type_define.h"

using namespace Eigen;
using namespace std;


namespace dyros_red_controller
{


class Wholebody_controller
{
public:
  //walking_controller();

  Wholebody_controller(DyrosRedModel& model, const VectorQd& current_q, const double hz, const double& control_time);
  unsigned int total_dof_;
  DyrosRedModel &model_;


  // motion time
  const double hz_;
  const double &control_time_; // updated by control_base
  double start_time_[4];
  double end_time_[4];
  bool target_arrived_[4];

  const VectorQd &current_q_;  // updated by control_base



  VectorQd gravity_compensation_torque();
  VectorQd contact_force_redistribution_torque(VectorQd command_torque, Eigen::Vector12d& ForceRedistribution);
  VectorQd task_control_torque(Eigen::MatrixXd J_task, Eigen::Vector6d f_star_);
  void ForceRedistributionTwoContactMod2(double eta_cust, double footlength, double footwidth, double staticFrictionCoeff, double ratio_x, double ratio_y, Eigen::Vector3d P1, Eigen::Vector3d P2, Eigen::Vector12d &F12, Eigen::Vector6d& ResultantForce,  Eigen::Vector12d& ForceRedistribution);
  void ForceRedistributionTwoContactMod(double eta_cust, double footlength, double footwidth, double staticFrictionCoeff, double ratio_x, double ratio_y, Eigen::Vector3d P1, Eigen::Vector3d P2, Eigen::Vector12d &F12, Eigen::Vector6d& ResultantForce,  Eigen::Vector12d& ForceRedistribution, double& eta);
  void update_dynamics_mode(int mode);


  //Eigen::Vector6d Getfstar( );
  Vector3d getfstar(Vector3d kp, Vector3d kd, Vector3d p_desired, Vector3d p_now, Vector3d d_desired, Vector3d d_now);
  Vector3d getfstar(Vector3d kp, Vector3d kd, Matrix3d r_desired, Matrix3d r_now, Vector3d w_desired, Vector3d w_now);

  //Contact Mode
  const int DOUBLE_SUPPORT = 0;
  const int SINGLE_SUPPORT_LEFT = 1;
  const int SINGLE_SUPPORT_RIGHT = 2;

  MatrixXd A_matrix;
  MatrixXd A_matrix_inverse;
  MatrixXd J_C, J_C_INV_T;
  MatrixXd J_COM;

  MatrixXd J_task;
  VectorXd f_star;


  MatrixXd Lambda_c;
  MatrixXd N_C;
  MatrixXd I37;

  Vector3d Grav_ref;

  MatrixXd J_task_T, J_task_inv,J_task_inv_T;
  MatrixXd lambda_inv, lambda;
  MatrixXd W, W_inv;
  MatrixXd Q, Q_T_, Q_temp, Q_temp_inv, Jtemp, Jtemp_2;
  MatrixXd _F;

  VectorXd G;

  MatrixXd Slc_k,Slc_k_T;
  MatrixXd svd_U;

  int task_dof;

};


}


#endif // WALKING_CONTROLLER_H


