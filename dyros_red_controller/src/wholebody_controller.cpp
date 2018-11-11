#include "dyros_red_controller/wholebody_controller.h"

namespace dyros_red_controller
{

Wholebody_controller::Wholebody_controller(DyrosRedModel &model, const VectorQd &current_q, const double hz, const double &control_time, const double &d_time) : total_dof_(DyrosRedModel::MODEL_DOF), model_(model),
                                                                                                                                                                 current_q_(current_q), hz_(hz), control_time_(control_time), d_time_(d_time),
                                                                                                                                                                 start_time_{}, end_time_{}, target_arrived_{true, true, true, true}
{
  //debug_.open("/home/suhan/jet_test.txt");
}

void Wholebody_controller::update_dynamics()
{
  ROS_DEBUG_ONCE("dynamics update start ");
  A_matrix.setZero(total_dof_ + 6, total_dof_ + 6);
  A_matrix = model_.A_;
  A_matrix_inverse = A_matrix.inverse();
  Grav_ref.setZero(3);
  Grav_ref(2) = -9.81;

  //bool reset

  task_force_control = false;
  task_force_control_feedback = false;
}

void Wholebody_controller::contact_set(int contact_number, int link_id[], Vector3d contact_point[])
{

  J_C.setZero(contact_number * 6, total_dof_ + 6);

  for (int i = 0; i < contact_number; i++)
  {
    model_.Link_Set_Contact(link_id[i], contact_point[i]);

    J_C.block(i * 6, 0, 6, total_dof_ + 6) = model_.link_[link_id[i]].Jac_Contact;
  }

  Lambda_c = (J_C * A_matrix_inverse * (J_C.transpose())).inverse();
  J_C_INV_T = Lambda_c * J_C * A_matrix_inverse;

  N_C.setZero(total_dof_ + 6, total_dof_ + 6);
  I37.setIdentity(total_dof_ + 6, total_dof_ + 6);
  N_C = I37 - J_C.transpose() * J_C_INV_T;

  Slc_k.setZero(total_dof_, total_dof_ + 6);
  Slc_k.block(0, 6, total_dof_, total_dof_).setIdentity();
  Slc_k_T = Slc_k.transpose();

  W = Slc_k * N_C.transpose() * A_matrix_inverse * N_C * Slc_k_T;
  //W = Slc_k * A_matrix_inverse * N_C * Slc_k_T; //2 types for w matrix
  W_inv = DyrosMath::pinv_SVD(W);

  contact_force_predict.setZero();
}

VectorQd Wholebody_controller::gravity_compensation_torque()
{
  ROS_DEBUG_ONCE("gravity torque calc start ");
  G.setZero(total_dof_ + 6);

  for (int i = 0; i < total_dof_ + 1; i++)
  {
    G -= model_.link_[i].Jac_COM_p.transpose() * model_.link_[i].Mass * Grav_ref;
  }

  Eigen::MatrixXd J_g;
  J_g.setZero(total_dof_, total_dof_ + 6);
  J_g.block(0, 6, total_dof_, total_dof_).setIdentity();

  Eigen::VectorXd torque_grav(total_dof_);
  Eigen::MatrixXd aa = J_g * A_matrix_inverse * N_C * J_g.transpose();
  /*
  double epsilon = 1e-7;
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(aa ,Eigen::ComputeThinU | Eigen::ComputeThinV);
  double tolerance = epsilon * std::max(aa.cols(), aa.rows()) *svd.singularValues().array().abs()(0);
  Eigen::MatrixXd ppinv = svd.matrixV() *  (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
*/
  // Eigen::MatrixXd ppinv = aa.completeOrthogonalDecomposition().pseudoInverse();
  //torque_grav = (J_g*A_matrix.inverse()*N_C*J_g.transpose()).completeOrthogonalDecomposition().pseudoInverse()*J_g*A_matrix.inverse()*N_C*G;
  //torque_grav.setZero();
  //Eigen::MatrixXd ppinv = DyrosMath::pinv_QR(aa);
  Eigen::MatrixXd ppinv = DyrosMath::pinv_SVD(aa);

  Eigen::MatrixXd tg_temp = ppinv * J_g * A_matrix_inverse * N_C;
  torque_grav = tg_temp * G;

  ROS_DEBUG_ONCE("gravity torque calc end ");
  return torque_grav;
}

VectorQd Wholebody_controller::task_control_torque(MatrixXd J_task, VectorXd f_star_)
{

  ROS_DEBUG_ONCE("task torque calc start ");
  task_dof = J_task.rows();

  //Task Control Torque;
  J_task_T.resize(total_dof_ + 6, task_dof);
  J_task_T.setZero();
  lambda_inv.resize(task_dof, task_dof);
  lambda_inv.setZero();
  lambda.resize(task_dof, task_dof);
  lambda.setZero();

  J_task_T = J_task.transpose();

  lambda_inv = J_task * A_matrix_inverse * N_C * J_task_T;

  lambda = lambda_inv.inverse();
  J_task_inv_T = lambda * J_task * A_matrix_inverse * N_C;

  Q = J_task_inv_T * Slc_k_T;
  Q_T_ = Q.transpose();

  Q_temp = Q * W_inv * Q_T_;

  Q_temp_inv = DyrosMath::pinv_SVD(Q_temp);

  //_F=lambda*(f_star);
  //Jtemp=J_task_inv_T*Slc_k_T;
  //Jtemp_2 = DyrosMath::pinv_SVD(Jtemp);
  //Q.svd(s2,u2,v2);

  VectorQd torque_task;
  if (task_force_control)
  {
    VectorXd F_;
    F_.resize(task_dof);
    F_ = lambda * task_selection_matrix * f_star_;
    VectorXd F_2;
    F_2 = F_ + task_desired_force;
    torque_task = W_inv * Q_T_ * Q_temp_inv * F_2;
  }
  else if (task_force_control_feedback)
  {
    VectorXd F_;
    F_.resize(task_dof);

    static double right_i, left_i;

    double pd = 0.1;
    double pi = 4.0;

    double left_des = -50.0;
    double right_des = 50.0;

    double right_err = task_desired_force(10) + task_feedback_reference(1);
    double left_err = task_desired_force(16) + task_feedback_reference(7);

    right_i += right_err * d_time_;
    left_i += left_err * d_time_;

    VectorXd fc_fs;
    fc_fs = task_desired_force;
    fc_fs.setZero();

    fc_fs(10) = pd * right_err + pi * right_i;
    fc_fs(16) = pd * left_err + pi * left_i;
    F_ = lambda * (task_selection_matrix * f_star_ + fc_fs);

    VectorXd F_2;
    F_2 = F_ + task_desired_force;

    torque_task = W_inv * Q_T_ * Q_temp_inv * F_2;
  }
  else
  {
    torque_task = W_inv * Q_T_ * Q_temp_inv * (lambda * (f_star_));
  }

  //W.svd(s,u,v);
  //V2.resize(28,6);
  //V2.zero();

  ROS_DEBUG_ONCE("task torque calc end ");

  return torque_task;
}

VectorQd Wholebody_controller::task_control_torque_custom_force(MatrixXd J_task, VectorXd f_star_, MatrixXd selection_matrix, VectorXd desired_force)
{

  ROS_DEBUG_ONCE("task torque calc start ");
  task_dof = J_task.rows();

  //Task Control Torque;
  J_task_T.resize(total_dof_ + 6, task_dof);
  J_task_T.setZero();
  lambda_inv.resize(task_dof, task_dof);
  lambda_inv.setZero();
  lambda.resize(task_dof, task_dof);
  lambda.setZero();

  J_task_T = J_task.transpose();

  lambda_inv = J_task * A_matrix_inverse * N_C * J_task_T;

  lambda = lambda_inv.inverse();
  J_task_inv_T = lambda * J_task * A_matrix_inverse * N_C;

  Q = J_task_inv_T * Slc_k_T;
  Q_T_ = Q.transpose();

  Q_temp = Q * W_inv * Q_T_;

  Q_temp_inv = DyrosMath::pinv_SVD(Q_temp);

  //_F=lambda*(f_star);
  //Jtemp=J_task_inv_T*Slc_k_T;
  //Jtemp_2 = DyrosMath::pinv_SVD(Jtemp);
  //Q.svd(s2,u2,v2);

  VectorXd F_;
  F_.resize(task_dof);

  F_ = lambda * selection_matrix * f_star_;

  VectorXd F_2;

  F_2 = F_ + desired_force;

  VectorQd torque_task;
  torque_task = W_inv * Q_T_ * Q_temp_inv * F_2;

  //W.svd(s,u,v);
  //V2.resize(28,6);
  //V2.zero();

  ROS_DEBUG_ONCE("task torque calc end ");

  return torque_task;
}

VectorQd Wholebody_controller::task_control_torque_custom_force_feedback(MatrixXd J_task, VectorXd f_star_, MatrixXd selection_matrix, VectorXd desired_force, VectorXd ft_hand)
{

  ROS_DEBUG_ONCE("task torque calc start ");
  task_dof = J_task.rows();

  //Task Control Torque;
  J_task_T.resize(total_dof_ + 6, task_dof);
  J_task_T.setZero();
  lambda_inv.resize(task_dof, task_dof);
  lambda_inv.setZero();
  lambda.resize(task_dof, task_dof);
  lambda.setZero();

  J_task_T = J_task.transpose();

  lambda_inv = J_task * A_matrix_inverse * N_C * J_task_T;

  lambda = lambda_inv.inverse();
  J_task_inv_T = lambda * J_task * A_matrix_inverse * N_C;

  Q = J_task_inv_T * Slc_k_T;
  Q_T_ = Q.transpose();

  Q_temp = Q * W_inv * Q_T_;

  Q_temp_inv = DyrosMath::pinv_SVD(Q_temp);

  //_F=lambda*(f_star);
  //Jtemp=J_task_inv_T*Slc_k_T;
  //Jtemp_2 = DyrosMath::pinv_SVD(Jtemp);
  //Q.svd(s2,u2,v2);

  VectorXd F_;
  F_.resize(task_dof);

  static double right_i, left_i;

  double pd = 0.1;
  double pi = 4.0;

  double left_des = -50.0;
  double right_des = 50.0;

  double right_err = desired_force(10) + ft_hand(1);
  double left_err = desired_force(16) + ft_hand(7);

  right_i += right_err * d_time_;
  left_i += left_err * d_time_;

  VectorXd fc_fs; // = desired_force;
  fc_fs = desired_force;
  fc_fs.setZero();

  fc_fs(10) = pd * right_err + pi * right_i;
  fc_fs(16) = pd * left_err + pi * left_i;

  //std::cout << "right : " << fc_fs(10) << std::endl;
  //std::cout << "left : " << fc_fs(16) << std::endl;

  F_ = lambda * (selection_matrix * f_star_ + fc_fs);

  //F_ = selection_matrix * lambda * f_star_;

  VectorXd F_2;

  //desired_force(10) = desired_force(10) + right_des;
  //desired_force(16) = desired_force(16) + left_des;

  F_2 = F_ + desired_force;

  VectorQd torque_task;
  torque_task = W_inv * Q_T_ * Q_temp_inv * F_2;

  //W.svd(s,u,v);
  //V2.resize(28,6);
  //V2.zero();

  ROS_DEBUG_ONCE("task torque calc end ");

  return torque_task;
}

void Wholebody_controller::set_force_control(MatrixXd selection_matrix, VectorXd desired_force)
{
  task_force_control = true;
  task_selection_matrix = selection_matrix;
  task_desired_force = desired_force;
}
void Wholebody_controller::set_force_control_feedback(MatrixXd selection_matrix, VectorXd desired_force, VectorXd ft_hand)
{
  task_force_control_feedback = true;
  task_selection_matrix = selection_matrix;
  task_desired_force = desired_force;
  task_feedback_reference = ft_hand;
}

Vector3d Wholebody_controller::getfstar(Vector3d kp, Vector3d kd, Vector3d p_desired, Vector3d p_now, Vector3d d_desired, Vector3d d_now)
{

  ROS_DEBUG_ONCE("fstar calc");
  Vector3d fstar_;

  for (int i = 0; i < 3; i++)
  {
    fstar_(i) = kp(i) * (p_desired(i) - p_now(i)) + kd(i) * (d_desired(i) - d_now(i));
  }

  return fstar_;
}

Vector3d Wholebody_controller::getfstar(Vector3d kp, Vector3d kd, Matrix3d r_desired, Matrix3d r_now, Vector3d w_desired, Vector3d w_now)
{

  ROS_DEBUG_ONCE("fstar calc");
  Vector3d fstar_;

  Matrix3d Rotyaw = DyrosMath::rotateWithZ(model_.yaw_radian);
  Vector3d angle_d_global = Rotyaw * DyrosMath::getPhi(r_now, r_desired);

  for (int i = 0; i < 3; i++)
  {
    fstar_(i) = (kp(i) * angle_d_global(i) - kd(i) * w_now(i));
  }

  return fstar_;
}

Vector3d Wholebody_controller::getfstar_tra(int link_id, Vector3d kpt, Vector3d kdt)
{
  ROS_DEBUG_ONCE("fstar calc");
  Vector3d fstar_;

  for (int i = 0; i < 3; i++)
  {
    fstar_(i) = kpt(i) * (model_.link_[link_id].x_traj(i) - model_.link_[link_id].xpos(i)) + kdt(i) * (model_.link_[link_id].v_traj(i) - model_.link_[link_id].v(i));
  }

  return fstar_;
}

Vector3d Wholebody_controller::getfstar_rot(int link_id, Vector3d kpa, Vector3d kda)
{
  ROS_DEBUG_ONCE("fstar calc");
  Vector3d fstar_;

  Matrix3d Rotyaw = DyrosMath::rotateWithZ(model_.yaw_radian);

  Vector3d angle_d_global = -Rotyaw * DyrosMath::getPhi(model_.link_[link_id].Rotm, model_.link_[link_id].r_traj);

  //Matrix3d Rotyaw = DyrosMath::rotateWithZ(model_.yaw_radian);

  //Vector3d angle_d_global = Rotyaw * DyrosMath::getPhi(model_.link_[link_id].Rotm, model_.link_[link_id].r_traj);

  for (int i = 0; i < 3; i++)
  {
    fstar_(i) = (kpa(i) * angle_d_global(i) - kda(i) * model_.link_[link_id].w(i));
  }
  /*
  std::cout << "fstar check " << std::endl
            << model_.link_[link_id].name << std::endl
            << " rotation now " << std::endl
            << model_.link_[link_id].Rotm << std::endl
            << "desired rotation " << std::endl
            << model_.link_[link_id].r_traj << std::endl
            << "angle d " << std::endl
            << angle_d << std::endl
            << "global angle d " << std::endl
            << angle_d_global << std::endl
            << "fstar " << std::endl
            << fstar_ << std::endl
            << " ////////////////////////////////////////////////////////////////" << std::endl;
  */

  return fstar_;
}

Vector6d Wholebody_controller::getfstar6d(int link_id, Vector3d kpt, Vector3d kdt, Vector3d kpa, Vector3d kda)
{
  Vector6d f_star;
  f_star.segment(0, 3) = getfstar_tra(link_id, kpt, kdt);
  f_star.segment(3, 3) = getfstar_rot(link_id, kpa, kda);
  return f_star;
}
VectorQd Wholebody_controller::contact_force_custom(VectorQd command_torque, Eigen::VectorXd contact_force_now, Eigen::VectorXd contact_force_desired)
{
  JacobiSVD<MatrixXd> svd(W, ComputeThinU | ComputeThinV);
  svd_U = svd.matrixU();

  MatrixXd V2;

  int singular_dof = 6;
  int contact_dof = J_C.rows();

  V2.setZero(total_dof_, contact_dof - singular_dof);
  V2 = svd_U.block(0, total_dof_ - contact_dof + singular_dof, total_dof_, contact_dof - singular_dof);

  MatrixXd Scf_;
  Scf_.setZero(contact_dof - singular_dof, contact_dof);
  Scf_.block(0, 0, contact_dof - singular_dof, contact_dof - singular_dof).setIdentity();

  // std::cout << contact_force_desired << std::endl
  //           << std::endl
  //           << std::endl;
  // std::cout << contact_force_now << std::endl
  //           << std::endl
  //           << std::endl;
  // std::cout << std::endl;

  VectorXd desired_force = contact_force_desired - contact_force_now;

  MatrixXd temp = Scf_ * J_C_INV_T * Slc_k_T * V2;
  MatrixXd temp_inv = DyrosMath::pinv_SVD(temp);
  MatrixXd Vc_ = V2 * temp_inv;

  VectorXd reduced_desired_force = Scf_ * desired_force;
  VectorQd torque_contact_ = Vc_ * reduced_desired_force;

  return torque_contact_;
}

VectorXd Wholebody_controller::get_contact_force(VectorQd command_torque)
{
  VectorXd contactforce = J_C_INV_T * Slc_k_T * command_torque - Lambda_c * J_C * A_matrix_inverse * G;
  return contactforce;
}

VectorQd Wholebody_controller::contact_force_redistribution_torque(double yaw_radian, VectorQd command_torque, Eigen::Vector12d &ForceRedistribution, double &eta)
{
  //Contact Jacobian task : rightfoot to leftfoot
  ROS_INFO_ONCE("contact redistribution start");

  Vector12d ContactForce_ = J_C_INV_T * Slc_k_T * command_torque - Lambda_c * J_C * A_matrix_inverse * G;

  ROS_INFO_ONCE("contactForce calc");

  Vector3d P1_, P2_;

  P1_ = model_.link_[model_.Right_Foot].xpos;
  P2_ = model_.link_[model_.Left_Foot].xpos;

  Matrix3d Rotyaw = DyrosMath::rotateWithZ(-model_.yaw_radian);

  Vector3d P1_local, P2_local;
  P1_local = Rotyaw * P1_;
  P2_local = Rotyaw * P2_;

  Matrix12d force_rot_yaw;
  force_rot_yaw.setZero();
  for (int i = 0; i < 4; i++)
  {
    force_rot_yaw.block(i * 3, i * 3, 3, 3) = Rotyaw;
  }

  Vector6d ResultantForce_;
  ResultantForce_.setZero();

  Vector12d ResultRedistribution_;
  ResultRedistribution_.setZero();

  VectorQd torque_contact_;
  torque_contact_.setZero();

  double eta_cust = 0.95;
  double foot_length = 0.26;
  double foot_width = 0.1;

  Vector12d ContactForce_Local_yaw;
  ContactForce_Local_yaw = force_rot_yaw * ContactForce_;

  ZMP_pos = GetZMPpos(P1_local, P2_local, ContactForce_Local_yaw);

  ForceRedistributionTwoContactMod2(0.95, foot_length, foot_width, 1.0, 0.8, 0.8, P1_local, P2_local, ContactForce_Local_yaw, ResultantForce_, ResultRedistribution_, eta);

  ForceRedistribution = force_rot_yaw.transpose() * ResultRedistribution_;

  ROS_INFO_ONCE("contact redistribution end");

  JacobiSVD<MatrixXd> svd(W, ComputeThinU | ComputeThinV);
  svd_U = svd.matrixU();

  MatrixXd V2;

  int singular_dof = 6;
  int contact_dof = J_C.rows();

  V2.setZero(total_dof_, singular_dof);
  V2 = svd_U.block(0, total_dof_ - contact_dof + 6, total_dof_, contact_dof - 6);

  Vector12d desired_force;

  desired_force.setZero();
  MatrixXd Scf_;

  bool right_master = false;

  if (right_master)
  {
    Scf_.setZero(6, 12);
    Scf_.block(0, 0, 6, 6).setIdentity();

    for (int i = 0; i < 6; i++)
    {
      desired_force(i) = -ContactForce_(i) + ForceRedistribution(i);
    }
  }
  else
  {

    Scf_.setZero(6, 12);
    Scf_.block(0, 6, 6, 6).setIdentity();

    for (int i = 0; i < 6; i++)
    {
      desired_force(i + 6) = -ContactForce_(i + 6) + ForceRedistribution(i + 6);
    }
  }

  MatrixXd temp = Scf_ * J_C_INV_T * Slc_k_T * V2;
  MatrixXd temp_inv = DyrosMath::pinv_SVD(temp);
  MatrixXd Vc_ = V2 * temp_inv;

  Vector6d reduced_desired_force = Scf_ * desired_force;
  torque_contact_ = Vc_ * reduced_desired_force;

  return torque_contact_;
}

Vector3d Wholebody_controller::GetZMPpos(Vector3d P_right, Vector3d P_left, Vector12d ContactForce)
{

  Vector3d zmp_pos;
  Vector3d P_;
  zmp_pos.setZero();
  P_.setZero();

  //zmp_pos(0) = (-ContactForce(4) - P_right(2) * ContactForce(0) + P_right(0) * ContactForce(2) - ContactForce(10) - P_left(2) * ContactForce(6) + P_left(0) * ContactForce(8)) / (ContactForce(2)+ContactForce(8));
  //zmp_pos(1) = (ContactForce(3) - P_right(2) * ContactForce(1) + P_right(1) * ContactForce(2) + ContactForce(9) - P_left(2) * ContactForce(7) + P_left(1) * ContactForce(8)) / (ContactForce(2)+ContactForce(8));
  zmp_pos(0) = (-ContactForce(4) - (P_right(2) - P_(2)) * ContactForce(0) + P_right(0) * ContactForce(2) - ContactForce(10) - (P_left(2) - P_(2)) * ContactForce(6) + P_left(0) * ContactForce(8)) / (ContactForce(2) + ContactForce(8));
  zmp_pos(1) = (ContactForce(3) - (P_right(2) - P_(2)) * ContactForce(1) + P_right(1) * ContactForce(2) + ContactForce(9) - (P_left(2) - P_(2)) * ContactForce(7) + P_left(1) * ContactForce(8)) / (ContactForce(2) + ContactForce(8));

  return (zmp_pos);
}

void Wholebody_controller::ForceRedistributionTwoContactMod2(double eta_cust, double footlength, double footwidth, double staticFrictionCoeff, double ratio_x, double ratio_y, Eigen::Vector3d P1, Eigen::Vector3d P2, Eigen::Vector12d &F12, Eigen::Vector6d &ResultantForce, Eigen::Vector12d &ForceRedistribution, double &eta)
{
  ROS_DEBUG_ONCE("force redistribution start");
  Eigen::MatrixXd W;
  W.setZero(6, 12);

  Eigen::Matrix3d P1_hat, P2_hat;
  P1_hat = DyrosMath::skm(P1);
  P2_hat = DyrosMath::skm(P2);

  for (int i = 0; i < 3; i++)
  {
    W(i, i) = 1.0;
    W(i + 3, i + 3) = 1.0;
    W(i, i + 6) = 1.0;
    W(i + 3, i + 9) = 1.0;

    for (int j = 0; j < 3; j++)
    {
      W(i + 3, j) = P1_hat(i, j);
      W(i + 3, j + 6) = P2_hat(i, j);
    }
  }
  ResultantForce.resize(6);
  ResultantForce = W * F12; //F1F2;

  double eta_lb = 1.0 - eta_cust;
  double eta_ub = eta_cust;
  //printf("1 lb %f ub %f\n",eta_lb,eta_ub);
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////
  //boundary of eta Mx, A*eta + B < 0
  double A = (P1(2) - P2(2)) * ResultantForce(1) - (P1(1) - P2(1)) * ResultantForce(2);
  double B = ResultantForce(3) + P2(2) * ResultantForce(1) - P2(1) * ResultantForce(2);
  double C = ratio_y * footwidth / 2.0 * abs(ResultantForce(2));
  double a = A * A;
  double b = 2.0 * A * B;
  double c = B * B - C * C;
  double sol_eta1 = (-b + sqrt(b * b - 4.0 * a * c)) / 2.0 / a;
  double sol_eta2 = (-b - sqrt(b * b - 4.0 * a * c)) / 2.0 / a;
  if (sol_eta1 > sol_eta2) //sol_eta1 ÀÌ upper boundary
  {
    if (sol_eta1 < eta_ub)
    {
      eta_ub = sol_eta1;
    }

    if (sol_eta2 > eta_lb)
    {
      eta_lb = sol_eta2;
    }
  }
  else //sol_eta2 ÀÌ upper boundary
  {
    if (sol_eta2 < eta_ub)
    {
      eta_ub = sol_eta2;
    }

    if (sol_eta1 > eta_lb)
    {
      eta_lb = sol_eta1;
    }
  }

  //printf("3 lb %f ub %f A %f B %f\n",eta_lb,eta_ub, sol_eta1, sol_eta2);
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////
  //boundary of eta My, A*eta + B < 0
  A = -(P1(2) - P2(2)) * ResultantForce(0) + (P1(0) - P2(0)) * ResultantForce(2);
  B = ResultantForce(4) - P2(2) * ResultantForce(0) + P2(0) * ResultantForce(2);
  C = ratio_x * footlength / 2.0 * abs(ResultantForce(2));
  a = A * A;
  b = 2.0 * A * B;
  c = B * B - C * C;
  sol_eta1 = (-b + sqrt(b * b - 4.0 * a * c)) / 2.0 / a;
  sol_eta2 = (-b - sqrt(b * b - 4.0 * a * c)) / 2.0 / a;
  if (sol_eta1 > sol_eta2) //sol_eta1 ÀÌ upper boundary
  {
    if (sol_eta1 < eta_ub)
    {
      eta_ub = sol_eta1;
    }

    if (sol_eta2 > eta_lb)
    {
      eta_lb = sol_eta2;
    }
  }
  else //sol_eta2 ÀÌ upper boundary
  {
    if (sol_eta2 < eta_ub)
    {
      eta_ub = sol_eta2;
    }

    if (sol_eta1 > eta_lb)
    {
      eta_lb = sol_eta1;
    }
  }

  //printf("5 lb %f ub %f A %f B %f\n",eta_lb,eta_ub, sol_eta1, sol_eta2);
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////
  //boundary of eta Mz, (A^2-C^2)*eta^2 + 2*A*B*eta + B^2 < 0
  A = -(P1(0) - P2(0)) * ResultantForce(1) + (P1(1) - P2(1)) * ResultantForce(0);
  B = ResultantForce(5) + P2(1) * ResultantForce(0) - P2(0) * ResultantForce(1);
  C = staticFrictionCoeff * abs(ResultantForce(2));
  a = A * A;
  b = 2.0 * A * B;
  c = B * B - C * C;
  sol_eta1 = (-b + sqrt(b * b - 4.0 * a * c)) / 2.0 / a;
  sol_eta2 = (-b - sqrt(b * b - 4.0 * a * c)) / 2.0 / a;
  if (sol_eta1 > sol_eta2) //sol_eta1 ÀÌ upper boundary
  {
    if (sol_eta1 < eta_ub)
    {
      eta_ub = sol_eta1;
    }

    if (sol_eta2 > eta_lb)
    {
      eta_lb = sol_eta2;
    }
  }
  else //sol_eta2 ÀÌ upper boundary
  {
    if (sol_eta2 < eta_ub)
    {
      eta_ub = sol_eta2;
    }

    if (sol_eta1 > eta_lb)
    {
      eta_lb = sol_eta1;
    }
  }
  //printf("6 lb %f ub %f A %f B %f\n",eta_lb,eta_ub, sol_eta1, sol_eta2);

  double eta_s = (-ResultantForce(3) - P2(2) * ResultantForce(1) + P2(1) * ResultantForce(2)) / ((P1(2) - P2(2)) * ResultantForce(1) - (P1(1) - P2(1)) * ResultantForce(2));

  eta = eta_s;
  if (eta_s > eta_ub)
  {
    eta = eta_ub;
  }
  else if (eta_s < eta_lb)
  {
    eta = eta_lb;
  }

  if ((eta > eta_cust) || (eta < 1.0 - eta_cust))
  {
    eta = 0.5;
  }

  //std::cout<<"ETA :: "<<eta<<std::endl;

  //	printf("lb %f ub %f eta %f etas %f\n",eta_lb,eta_ub, eta, eta_s);

  //double Mx1Mx2 = ResultantForce(3) + ((P1(2)*eta*ResultantForce(1) + P2(2)*(1.0-eta)*ResultantForce(1)) - (P1(1)*eta*ResultantForce(2) + P2(1)*(1.0-eta)*ResultantForce(2)));
  //double etaMx = eta*Mx1Mx2;
  //printf("%f %f \n", Mx1Mx2,etaMx);
  //double My1My2 = ResultantForce(4) + ((P1(0)*eta*ResultantForce(2) + P2(0)*(1.0-eta)*ResultantForce(2)) - (P1(2)*eta*ResultantForce(0) + P2(2)*(1.0-eta)*ResultantForce(0)));
  //double Mz1Mz2 = ResultantForce(5) + ((P1(1)*eta*ResultantForce(0) + P2(1)*(1.0-eta)*ResultantForce(0)) - (P1(0)*eta*ResultantForce(1) + P2(0)*(1.0-eta)*ResultantForce(1)));
  //printf("sumMx %f sumMy %f sumMz %f\n",Mx1Mx2,My1My2,Mz1Mz2);

  ForceRedistribution(0) = eta * ResultantForce(0);
  ForceRedistribution(1) = eta * ResultantForce(1);
  ForceRedistribution(2) = eta * ResultantForce(2);
  ForceRedistribution(3) = ((P1(2) - P2(2)) * ResultantForce(1) - (P1(1) - P2(1)) * ResultantForce(2)) * eta * eta + (ResultantForce(3) + P2(2) * ResultantForce(1) - P2(1) * ResultantForce(2)) * eta;
  ForceRedistribution(4) = (-(P1(2) - P2(2)) * ResultantForce(0) + (P1(0) - P2(0)) * ResultantForce(2)) * eta * eta + (ResultantForce(4) - P2(2) * ResultantForce(0) + P2(0) * ResultantForce(2)) * eta;
  ForceRedistribution(5) = (-(P1(0) - P2(0)) * ResultantForce(1) + (P1(1) - P2(1)) * ResultantForce(0)) * eta * eta + (ResultantForce(5) + P2(1) * ResultantForce(0) - P2(0) * ResultantForce(1)) * eta;
  ForceRedistribution(6) = (1.0 - eta) * ResultantForce(0);
  ForceRedistribution(7) = (1.0 - eta) * ResultantForce(1);
  ForceRedistribution(8) = (1.0 - eta) * ResultantForce(2);
  ForceRedistribution(9) = (1.0 - eta) * (((P1(2) - P2(2)) * ResultantForce(1) - (P1(1) - P2(1)) * ResultantForce(2)) * eta + (ResultantForce(3) + P2(2) * ResultantForce(1) - P2(1) * ResultantForce(2)));
  ForceRedistribution(10) = (1.0 - eta) * ((-(P1(2) - P2(2)) * ResultantForce(0) + (P1(0) - P2(0)) * ResultantForce(2)) * eta + (ResultantForce(4) - P2(2) * ResultantForce(0) + P2(0) * ResultantForce(2)));
  ForceRedistribution(11) = (1.0 - eta) * ((-(P1(0) - P2(0)) * ResultantForce(1) + (P1(1) - P2(1)) * ResultantForce(0)) * eta + (ResultantForce(5) + P2(1) * ResultantForce(0) - P2(0) * ResultantForce(1)));
  //ForceRedistribution(9) = (1.0-eta)/eta*ForceRedistribution(3);
  //ForceRedistribution(10) = (1.0-eta)/eta*ForceRedistribution(4);
  //ForceRedistribution(11) = (1.0-eta)/eta*ForceRedistribution(5);
}

void Wholebody_controller::ForceRedistributionTwoContactMod(double eta_cust, double footlength, double footwidth, double staticFrictionCoeff, double ratio_x, double ratio_y, Eigen::Vector3d P1, Eigen::Vector3d P2, Eigen::Vector12d &F12, Eigen::Vector6d &ResultantForce, Eigen::Vector12d &ForceRedistribution, double &eta)
{

  Eigen::Matrix3d P1_hat, P2_hat;
  P1_hat = DyrosMath::skm(P1);
  P2_hat = DyrosMath::skm(P2);

  Eigen::MatrixXd W;
  W.setZero(6, 12);

  W.block(0, 0, 6, 6) = Eigen::Matrix6d::Identity();
  W.block(0, 6, 6, 6) = Eigen::Matrix6d::Identity();
  W.block(3, 0, 3, 3) = P1_hat;
  W.block(3, 6, 3, 3) = P2_hat;

  // model_.link_[model_.Right_Leg].Rotm;

  // for (int i = 0; i < 3; i++)
  // {
  //   W(i, i) = 1.0;
  //   W(i + 3, i + 3) = 1.0;
  //   W(i, i + 6) = 1.0;
  //   W(i + 3, i + 9) = 1.0;

  //   for (int j = 0; j < 3; j++)
  //   {
  //     W(i + 3, j) = P1_hat(i, j);
  //     W(i + 3, j + 6) = P2_hat(i, j);
  //   }
  // }

  ResultantForce.resize(6);
  ResultantForce = W * F12; //F1F2;

  double eta_lb = 1.0 - eta_cust;
  double eta_ub = eta_cust;
  double A_threshold = 0.001;
  ////printf("1 lb %f ub %f\n",eta_lb,eta_ub);
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ////boundary of eta Mx, A*eta + B < 0
  double A = (P1(2) - P2(2)) * ResultantForce(1) - (P1(1) - P2(1)) * ResultantForce(2);
  double B = ResultantForce(3) + P2(2) * ResultantForce(1) - P2(1) * ResultantForce(2);
  double C = ratio_y * footwidth / 2.0 * abs(ResultantForce(2));
  double a = A * A;
  double b = 2.0 * A * B;
  double c = B * B - C * C;

  if (abs(A) < A_threshold)
  {
    if (B * B - C * C < 0) //eta와 무관하게 항상 만족, boundary 수정하지 않음
    {
    }
    else // B*B-C*C >= 0이면 no solution, 추후 task 수정 과정을 넣어야 함
    {
      //printf("0.");
    }
  }
  else
  {
    double sol_eta1 = (-b + sqrt(b * b - 4.0 * a * c)) / 2.0 / a;
    double sol_eta2 = (-b - sqrt(b * b - 4.0 * a * c)) / 2.0 / a;
    if (sol_eta1 > sol_eta2) //sol_eta1 이 upper boundary
    {
      if (sol_eta1 < eta_ub && sol_eta1 > eta_lb)
      {
        eta_ub = sol_eta1;
      }
      else if (sol_eta1 > eta_ub) // 문제 없음, 기존 ub 유지
      {
      }
      else
      {
        //printf("1.");
      }

      if (sol_eta2 > eta_lb && sol_eta2 < eta_ub)
      {
        eta_lb = sol_eta2;
      }
      else if (sol_eta2 < eta_lb) // 문제 없음, 기존 lb 유지
      {
      }
      else
      {
        //printf("2.");
      }
    }
    else //sol_eta2 이 upper boundary
    {
      if (sol_eta2 < eta_ub && sol_eta2 > eta_lb)
      {
        eta_ub = sol_eta2;
      }
      else if (sol_eta2 > eta_ub) // 문제 없음, 기존 ub 유지
      {
      }
      else
      {
        //printf("3.");
      }

      if (sol_eta1 > eta_lb && sol_eta1 < eta_ub)
      {
        eta_lb = sol_eta1;
      }
      else if (sol_eta1 < eta_lb) // 문제 없음, 기존 lb 유지
      {
      }
      else
      {
        //printf("4.");
      }
    }
  }

  ////printf("3 lb %f ub %f A %f B %f\n",eta_lb,eta_ub, sol_eta1, sol_eta2);
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ////boundary of eta My, A*eta + B < 0
  A = -(P1(2) - P2(2)) * ResultantForce(0) + (P1(0) - P2(0)) * ResultantForce(2);
  B = ResultantForce(4) - P2(2) * ResultantForce(0) + P2(0) * ResultantForce(2);
  C = ratio_x * footlength / 2.0 * abs(ResultantForce(2));
  a = A * A;
  b = 2.0 * A * B;
  c = B * B - C * C;

  if (abs(A) < A_threshold)
  {
    if (B * B - C * C < 0) //eta와 무관하게 항상 만족, boundary 수정하지 않음
    {
    }
    else // B*B-C*C >= 0이면 no solution, 추후 task 수정 과정을 넣어야 함
    {
      //printf("0;");
    }
  }
  else
  {
    double sol_eta1 = (-b + sqrt(b * b - 4.0 * a * c)) / 2.0 / a;
    double sol_eta2 = (-b - sqrt(b * b - 4.0 * a * c)) / 2.0 / a;
    if (sol_eta1 > sol_eta2) //sol_eta1 이 upper boundary
    {
      if (sol_eta1 < eta_ub && sol_eta1 > eta_lb)
      {
        eta_ub = sol_eta1;
      }
      else if (sol_eta1 > eta_ub) // 문제 없음, 기존 ub 유지
      {
      }
      else
      {
        //printf("1;");
      }

      if (sol_eta2 > eta_lb && sol_eta2 < eta_ub)
      {
        eta_lb = sol_eta2;
      }
      else if (sol_eta2 < eta_lb) // 문제 없음, 기존 lb 유지
      {
      }
      else
      {
        //printf("2;");
      }
    }
    else //sol_eta2 이 upper boundary
    {
      if (sol_eta2 < eta_ub && sol_eta2 > eta_lb)
      {
        eta_ub = sol_eta2;
      }
      else if (sol_eta2 > eta_ub) // 문제 없음, 기존 ub 유지
      {
      }
      else
      {
        //printf("3;");
      }

      if (sol_eta1 > eta_lb && sol_eta1 < eta_ub)
      {
        eta_lb = sol_eta1;
      }
      else if (sol_eta1 < eta_lb) // 문제 없음, 기존 lb 유지
      {
      }
      else
      {
        //printf("4;");
      }
    }
  }

  //printf("5 lb %f ub %f A %f B %f\n",eta_lb,eta_ub, sol_eta1, sol_eta2);
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////
  ////boundary of eta Mz, (A^2-C^2)*eta^2 + 2*A*B*eta + B^2 < 0
  A = -(P1(0) - P2(0)) * ResultantForce(1) + (P1(1) - P2(1)) * ResultantForce(0);
  B = ResultantForce(5) + P2(1) * ResultantForce(0) - P2(0) * ResultantForce(1);
  C = staticFrictionCoeff * abs(ResultantForce(2));
  a = A * A;
  b = 2.0 * A * B;
  c = B * B - C * C;

  if (abs(A) < A_threshold)
  {
    if (B * B - C * C < 0) //eta와 무관하게 항상 만족, boundary 수정하지 않음
    {
    }
    else // B*B-C*C >= 0이면 no solution, 추후 task 수정 과정을 넣어야 함
    {
      //printf("0,");
    }
  }
  else
  {
    double sol_eta1 = (-b + sqrt(b * b - 4.0 * a * c)) / 2.0 / a;
    double sol_eta2 = (-b - sqrt(b * b - 4.0 * a * c)) / 2.0 / a;
    if (sol_eta1 > sol_eta2) //sol_eta1 이 upper boundary
    {
      if (sol_eta1 < eta_ub && sol_eta1 > eta_lb)
      {
        eta_ub = sol_eta1;
      }
      else if (sol_eta1 > eta_ub) // 문제 없음, 기존 ub 유지
      {
      }
      else
      {
        //printf("1,");
      }

      if (sol_eta2 > eta_lb && sol_eta2 < eta_ub)
      {
        eta_lb = sol_eta2;
      }
      else if (sol_eta2 < eta_lb) // 문제 없음, 기존 lb 유지
      {
      }
      else
      {
        //printf("2,");
      }
    }
    else //sol_eta2 이 upper boundary
    {
      if (sol_eta2 < eta_ub && sol_eta2 > eta_lb)
      {
        eta_ub = sol_eta2;
      }
      else if (sol_eta2 > eta_ub) // 문제 없음, 기존 ub 유지
      {
      }
      else
      {
        //printf("3,");
      }

      if (sol_eta1 > eta_lb && sol_eta1 < eta_ub)
      {
        eta_lb = sol_eta1;
      }
      else if (sol_eta1 < eta_lb) // 문제 없음, 기존 lb 유지
      {
      }
      else
      {
        //printf("4,");
      }
    }
  }
  //printf("6 lb %f ub %f A %f B %f\n",eta_lb,eta_ub, sol_eta1, sol_eta2);

  double eta_s = (-ResultantForce(3) - P2(2) * ResultantForce(1) + P2(1) * ResultantForce(2)) / ((P1(2) - P2(2)) * ResultantForce(1) - (P1(1) - P2(1)) * ResultantForce(2));

  if (eta_s > eta_ub)
  {
    eta = eta_ub;
  }
  else if (eta_s < eta_lb)
  {
    eta = eta_lb;
  }
  else
  {
    eta = eta_s;
  }

  if (eta_ub < eta_lb) //임시...roundoff error로 정확한 해가 안나올때
  {
    //printf("-");
  }
  else if (sqrt(eta_ub * eta_ub + eta_lb * eta_lb) > 1.0) //너무 큰 경계값이 섞여 있을 때
  {
    //printf("_");
  }

  //	printf("lb %f ub %f eta %f etas %f\n",eta_lb,eta_ub, eta, eta_s);

  //double Mx1Mx2 = ResultantForce(3) + ((P1(2)*eta*ResultantForce(1) + P2(2)*(1.0-eta)*ResultantForce(1)) - (P1(1)*eta*ResultantForce(2) + P2(1)*(1.0-eta)*ResultantForce(2)));
  //double etaMx = eta*Mx1Mx2;
  //printf("%f %f \n", Mx1Mx2,etaMx);
  //double My1My2 = ResultantForce(4) + ((P1(0)*eta*ResultantForce(2) + P2(0)*(1.0-eta)*ResultantForce(2)) - (P1(2)*eta*ResultantForce(0) + P2(2)*(1.0-eta)*ResultantForce(0)));
  //double Mz1Mz2 = ResultantForce(5) + ((P1(1)*eta*ResultantForce(0) + P2(1)*(1.0-eta)*ResultantForce(0)) - (P1(0)*eta*ResultantForce(1) + P2(0)*(1.0-eta)*ResultantForce(1)));
  //printf("sumMx %f sumMy %f sumMz %f\n",Mx1Mx2,My1My2,Mz1Mz2);

  ForceRedistribution(0) = eta * ResultantForce(0);
  ForceRedistribution(1) = eta * ResultantForce(1);
  ForceRedistribution(2) = eta * ResultantForce(2);
  ForceRedistribution(3) = ((P1(2) - P2(2)) * ResultantForce(1) - (P1(1) - P2(1)) * ResultantForce(2)) * eta * eta + (ResultantForce(3) + P2(2) * ResultantForce(1) - P2(1) * ResultantForce(2)) * eta;
  ForceRedistribution(4) = (-(P1(2) - P2(2)) * ResultantForce(0) + (P1(0) - P2(0)) * ResultantForce(2)) * eta * eta + (ResultantForce(4) - P2(2) * ResultantForce(0) + P2(0) * ResultantForce(2)) * eta;
  ForceRedistribution(5) = (-(P1(0) - P2(0)) * ResultantForce(1) + (P1(1) - P2(1)) * ResultantForce(0)) * eta * eta + (ResultantForce(5) + P2(1) * ResultantForce(0) - P2(0) * ResultantForce(1)) * eta;
  ForceRedistribution(6) = (1.0 - eta) * ResultantForce(0);
  ForceRedistribution(7) = (1.0 - eta) * ResultantForce(1);
  ForceRedistribution(8) = (1.0 - eta) * ResultantForce(2);
  ForceRedistribution(9) = (1.0 - eta) * (((P1(2) - P2(2)) * ResultantForce(1) - (P1(1) - P2(1)) * ResultantForce(2)) * eta + (ResultantForce(3) + P2(2) * ResultantForce(1) - P2(1) * ResultantForce(2)));
  ForceRedistribution(10) = (1.0 - eta) * ((-(P1(2) - P2(2)) * ResultantForce(0) + (P1(0) - P2(0)) * ResultantForce(2)) * eta + (ResultantForce(4) - P2(2) * ResultantForce(0) + P2(0) * ResultantForce(2)));
  ForceRedistribution(11) = (1.0 - eta) * ((-(P1(0) - P2(0)) * ResultantForce(1) + (P1(1) - P2(1)) * ResultantForce(0)) * eta + (ResultantForce(5) + P2(1) * ResultantForce(0) - P2(0) * ResultantForce(1)));
  //ForceRedistribution(9) = (1.0-eta)/eta*ForceRedistribution(3);
  //ForceRedistribution(10) = (1.0-eta)/eta*ForceRedistribution(4);
  //ForceRedistribution(11) = (1.0-eta)/eta*ForceRedistribution(5);
}

} // namespace dyros_red_controller
