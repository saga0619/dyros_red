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
  VectorQd contact_force_redistribution_torque();
  VectorQd task_control_torque(Eigen::MatrixXd J_task, Eigen::Vector6d f_star_);
  void ForceRedistributionTwoContactMod2(double eta_cust, double footlength, double footwidth, double staticFrictionCoeff, double ratio_x, double ratio_y, Eigen::Vector3d P1, Eigen::Vector3d P2, Eigen::Vector12d &F12, Eigen::Vector6d& ResultantForce,  Eigen::Vector12d& ForceRedistribution);

  void update_dynamics_mode(int mode);


  //Eigen::Vector6d Getfstar( );
  Vector3d getfstar(Vector3d kp, Vector3d kd, Vector3d p_desired, Vector3d p_now, Vector3d d_desired, Vector3d d_now);
  Vector3d getfstar(Vector3d kp, Vector3d kd, Matrix3d r_desired, Matrix3d r_now, Vector3d w_desired, Vector3d w_now);

  //Contact Mode
  const int DOUBLE_SUPPORT = 0;
  const int SINGLE_SUPPORT_LEFT = 1;
  const int SINGLE_SUPPORT_RIGHT = 2;

  Eigen::MatrixXd A_matrix;
  Eigen::MatrixXd A_matrix_inverse;
  Eigen::MatrixXd J_C, J_C_INV_T;
  Eigen::MatrixXd J_COM;

  Eigen::MatrixXd J_task;
  Eigen::VectorXd f_star;


  Eigen::MatrixXd Lambda_c;
  Eigen::MatrixXd N_C;
  Eigen::MatrixXd I37;

  Eigen::Vector3d Grav_ref;


};


}


#endif // WALKING_CONTROLLER_H


