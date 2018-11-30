#include "dyros_red_controller/control_base.h"

namespace dyros_red_controller
{

// Constructor
ControlBase::ControlBase(ros::NodeHandle &nh, double Hz) : ui_update_count_(0), is_first_boot_(true), Hz_(Hz), control_mask_{}, total_dof_(DyrosRedModel::MODEL_DOF),
                                                           wc_(model_, q_, Hz, control_time_, d_time_), tc_(model_, q_, Hz, control_time_)
{
  ROS_INFO_ONCE("BASE : control base initialize");

  point_pub = nh.advertise<geometry_msgs::PolygonStamped>("/dyros_red/point", 3);
  dynamics_pub = nh.advertise<dyros_red_msgs::Dynamicsinfo>("/dyros_red/dynamics", 10);

  command_sub = nh.subscribe("/dyros_red/command", 100, &ControlBase::command_cb, this);
  com_command_sub = nh.subscribe("/dyros_red/com_command", 10, &ControlBase::com_command_cb, this);

  dynamics_pub_msgs.force_redistribution.resize(12);
  dynamics_pub_msgs.contact_force_predict.resize(12);
  dynamics_pub_msgs.torque_contact.resize(total_dof_);
  dynamics_pub_msgs.torque_control.resize(total_dof_);
  dynamics_pub_msgs.torque_gravity.resize(total_dof_);
  dynamics_pub_msgs.torque_task.resize(total_dof_);
  dynamics_pub_msgs.f_star.resize(18);
  dynamics_pub_msgs.task_desired.resize(3);
  dynamics_pub_msgs.task_current.resize(3);

  if (rviz_pub)
  {
    //test
    joint_state_publisher_for_rviz = nh.advertise<sensor_msgs::JointState>("/joint_states", 1);
    joint_states_rviz.name.resize(total_dof_);
    joint_states_rviz.position.resize(total_dof_);
    for (int i = 0; i < total_dof_; i++)
    {
      joint_states_rviz.name[i] = model_.JOINT_NAME[i];
    }
  }

  model_.debug_mode_ = debug;
  wc_.debug = debug;

  point_pub_msgs.polygon.points.resize(4);

  parameterInitialize();
  ROS_INFO_ONCE("BASE : control base initialize end");
}

void ControlBase::command_cb(const std_msgs::StringConstPtr &msg)
{
  if (msg->data == "gravity")
  {
    gravity_switch = !gravity_switch;
    ROS_INFO_COND(gravity_switch, "GRAVITY SWITCH : ON");
    ROS_INFO_COND(!gravity_switch, "GRAVITY SWITCH : OFF");
  }
  if (msg->data == "task")
  {
    task_switch = !task_switch;
    ROS_INFO_COND(task_switch, "TASK SWITCH : ON");
    ROS_INFO_COND(!task_switch, "TASK SWITCH : OFF");
  }

  if (msg->data == "contact")
  {
    contact_switch = !contact_switch;
    ROS_INFO_COND(contact_switch, "CONTACT SWITCH : ON");
    ROS_INFO_COND(!contact_switch, "CONTACT SWITCH : OFF");
  }

  if (msg->data == "data")
  {
    data_switch = true;
    ROS_INFO("DATA SWITCH");

    std::cout << "//////////////////////////////////" << std::endl;
    std::cout << " q_ virtual is : " << std::endl;
    std::cout << q_virtual_ << std::endl;
    std::cout << " xpos" << std::endl;
    std::cout << model_.link_[0].xpos << std::endl;
    std::cout << " jac of " << model_.link_[0].name << std::endl;
    std::cout << model_.link_[0].Jac << std::endl;
    std::cout << " jac of " << model_.link_[1].name << std::endl;
    std::cout << model_.link_[1].Jac << std::endl;
    std::cout << " jac of " << model_.link_[model_.Right_Hand].name << std::endl;
    std::cout << model_.link_[model_.Right_Hand].Jac << std::endl;
    std::cout << " jac of " << model_.link_[model_.Right_Foot].name << std::endl;
    std::cout << model_.link_[model_.Right_Foot].Jac << std::endl;
    std::cout << "waist skm " << std::endl;
    std::cout << DyrosMath::skm(model_.link_[1].xpos - model_.link_[0].xpos);
    std::cout << "rhand skm " << std::endl;
    std::cout << DyrosMath::skm(model_.link_[model_.Right_Hand].xpos - model_.link_[0].xpos);
    std::cout << std::endl;
    std::cout << "A matrix" << std::endl;
    std::cout << model_.A_ << std::endl;
    std::cout << "G matrix" << std::endl
              << wc_.G << std::endl;
    std::cout << model_.link_[0].Rotm << std::endl
              << "Rhand rot" << std::endl
              << model_.link_[model_.Right_Hand].Rotm << std::endl
              << "Lhand rot" << std::endl
              << model_.link_[model_.Left_Hand].Rotm << std::endl;
    std::cout << "Contact Jacobian" << std::endl
              << wc_.J_C << std::endl;
    std::cout << "Weight Matrix " << std::endl
              << wc_.W << std::endl;
  }
}

void ControlBase::com_command_cb(const dyros_red_msgs::ComCommandConstPtr &msg)
{

  //rot_init = model_.link_[model_.Upper_Body].Rotm;
  tc_.taskcommand_.command_time = control_time_;
  tc_.taskcommand_.traj_time = msg->time;
  tc_.taskcommand_.f_ratio = msg->ratio;
  tc_.taskcommand_.height = msg->height;
  tc_.taskcommand_.mode_ = msg->mode;
  tc_.taskcommand_.angle = msg->angle;

  ROS_INFO("COM TRAJ MSG received at, %f \t to %f in %f seconds.", tc_.taskcommand_.command_time, tc_.taskcommand_.f_ratio, tc_.taskcommand_.traj_time);

  model_.Link_Set_initpos(model_.COM_id);
  model_.Link_Set_initpos(model_.Right_Hand);
  model_.Link_Set_initpos(model_.Left_Hand);
  model_.Link_Set_initpos(model_.Pelvis);
  model_.Link_Set_initpos(model_.Upper_Body);

  init_q_ = q_virtual_.segment(6, total_dof_);

  task_switch = true;
}

void ControlBase::update()
{
  model_.updateKinematics(q_virtual_, q_dot_virtual_); // Update end effector positions and Jacobian
}

void ControlBase::compute()
{
  ROS_INFO_ONCE("BASE : update dynamics");
  wc_.update_dynamics();

  int cp[4] = {model_.Right_Foot, model_.Left_Foot, model_.Right_Hand, model_.Left_Hand};

  torque_task_.setZero();
  torque_joint_control_.setZero();

  d_time_ = control_time_ - last_sim_time_;

  bool force_mode_ = false;
  QP_switch = false;
  QP_wall = false;

  Vector3d kp_, kd_, kpa_, kda_;
  for (int i = 0; i < 3; i++)
  {
    kp_(i) = 400;
    kd_(i) = 40;
    kpa_(i) = 400;
    kda_(i) = 40;
  }

  ROS_INFO_ONCE("BASE : compute 1");

  if (task_switch)
  {
    if (tc_.taskcommand_.mode_ == 0)
    {

      wc_.contact_set_multi(1, 1, 0, 0);
      torque_gravity_ = wc_.gravity_compensation_torque();
      // COM jacobian control
      task_number = 6;
      J_task.setZero(task_number, total_dof_ + 6);
      f_star.setZero(task_number);
      J_task = model_.link_[model_.COM_id].Jac;

      task_desired = (tc_.taskcommand_.f_ratio * model_.link_[model_.Left_Foot].xpos + (1.0 - tc_.taskcommand_.f_ratio) * model_.link_[model_.Right_Foot].xpos);
      task_desired(2) = tc_.taskcommand_.height;
      model_.Link_Set_Trajectory_from_quintic(model_.COM_id, control_time_, tc_.taskcommand_.command_time, tc_.taskcommand_.command_time + tc_.taskcommand_.traj_time, task_desired);
      f_star = wc_.getfstar6d(model_.COM_id, kp_, kd_, kpa_, kda_);
    }
    else if (tc_.taskcommand_.mode_ == 1)
    {

      wc_.contact_set_multi(1, 1, 0, 0);
      torque_gravity_ = wc_.gravity_compensation_torque();
      // Pelvis control with holding hand position
      task_number = 18;

      J_task.setZero(task_number, total_dof_ + 6);
      f_star.setZero(task_number);

      J_task.block(0, 0, 6, total_dof_ + 6) = model_.link_[model_.Pelvis].Jac_COM;
      J_task.block(6, 0, 6, total_dof_ + 6) = model_.link_[model_.Right_Hand].Jac_COM;
      J_task.block(12, 0, 6, total_dof_ + 6) = model_.link_[model_.Left_Hand].Jac_COM;

      task_desired = (tc_.taskcommand_.f_ratio * model_.link_[model_.Left_Foot].xpos + (1.0 - tc_.taskcommand_.f_ratio) * model_.link_[model_.Right_Foot].xpos);
      task_desired(2) = tc_.taskcommand_.height;

      model_.Link_Set_Trajectory_from_quintic(model_.Pelvis, control_time_, tc_.taskcommand_.command_time, tc_.taskcommand_.command_time + tc_.taskcommand_.traj_time, task_desired);
      model_.Link_Set_Trajectory(model_.Right_Hand, model_.link_[model_.Pelvis].xpos + model_.link_[model_.Right_Hand].x_init - model_.link_[model_.Pelvis].x_init, Eigen::Vector3d::Zero(), model_.link_[model_.Right_Hand].rot_init, Eigen::Vector3d::Zero());
      model_.Link_Set_Trajectory(model_.Left_Hand, model_.link_[model_.Pelvis].xpos + model_.link_[model_.Left_Hand].x_init - model_.link_[model_.Pelvis].x_init, Eigen::Vector3d::Zero(), model_.link_[model_.Left_Hand].rot_init, Eigen::Vector3d::Zero());

      f_star.segment(0, 6) = wc_.getfstar6d(model_.Pelvis, kp_, kd_, kpa_, kda_);
      f_star.segment(6, 6) = wc_.getfstar6d(model_.Right_Hand, kp_, kd_, kpa_, kda_);
      f_star.segment(12, 6) = wc_.getfstar6d(model_.Left_Hand, kp_, kd_, kpa_, kda_);
    }
    else if (tc_.taskcommand_.mode_ == 2)
    {

      wc_.contact_set(2, cp);
      torque_gravity_ = wc_.gravity_compensation_torque();
      // Pelvis control
      task_number = 6;
      J_task.setZero(task_number, total_dof_ + 6);
      f_star.setZero(task_number);

      J_task = model_.link_[model_.Pelvis].Jac;

      task_desired = (tc_.taskcommand_.f_ratio * model_.link_[model_.Left_Foot].xpos + (1.0 - tc_.taskcommand_.f_ratio) * model_.link_[model_.Right_Foot].xpos);
      task_desired(2) = tc_.taskcommand_.height;
      model_.Link_Set_Trajectory_from_quintic(model_.Pelvis, control_time_, tc_.taskcommand_.command_time, tc_.taskcommand_.command_time + tc_.taskcommand_.traj_time, task_desired);
      f_star = wc_.getfstar6d(model_.Pelvis, kp_, kd_, kpa_, kda_);
    }
    else if (tc_.taskcommand_.mode_ == 3)
    {

      wc_.contact_set(2, cp);
      torque_gravity_ = wc_.gravity_compensation_torque();
      // COM control with pelvis jacobian
      task_number = 6;
      J_task.setZero(task_number, total_dof_ + 6);
      f_star.setZero(task_number);

      J_task = model_.link_[model_.Pelvis].Jac;

      task_desired = (tc_.taskcommand_.f_ratio * model_.link_[model_.Left_Foot].xpos + (1.0 - tc_.taskcommand_.f_ratio) * model_.link_[model_.Right_Foot].xpos) - model_.link_[model_.COM_id].xpos + model_.link_[model_.Pelvis].xpos;
      task_desired(2) = tc_.taskcommand_.height;
      model_.Link_Set_Trajectory_from_quintic(model_.Pelvis, control_time_, tc_.taskcommand_.command_time, tc_.taskcommand_.command_time + tc_.taskcommand_.traj_time, task_desired);
      f_star = wc_.getfstar6d(model_.Pelvis, kp_, kd_, kpa_, kda_);
    }
    else if (tc_.taskcommand_.mode_ == 4)
    {

      wc_.contact_set(2, cp);
      torque_gravity_ = wc_.gravity_compensation_torque();
      task_number = 9;

      J_task.setZero(task_number, total_dof_ + 6);
      f_star.setZero(task_number);

      J_task.block(0, 0, 6, total_dof_ + 6) = model_.link_[model_.Pelvis].Jac;
      J_task.block(6, 0, 3, total_dof_ + 6) = model_.link_[model_.Upper_Body].Jac.block(3, 0, 3, total_dof_ + 6);

      task_desired = (tc_.taskcommand_.f_ratio * model_.link_[model_.Left_Foot].xpos + (1.0 - tc_.taskcommand_.f_ratio) * model_.link_[model_.Right_Foot].xpos) - model_.link_[model_.COM_id].xpos + model_.link_[model_.Pelvis].xpos;
      task_desired(2) = tc_.taskcommand_.height;

      model_.Link_Set_Trajectory_from_quintic(model_.Pelvis, control_time_, tc_.taskcommand_.command_time, tc_.taskcommand_.command_time + tc_.taskcommand_.traj_time, task_desired);
      model_.Link_Set_Trajectory_rotation(model_.Pelvis, control_time_, tc_.taskcommand_.command_time, tc_.taskcommand_.command_time + tc_.taskcommand_.traj_time, Eigen::Matrix3d::Identity(), true);
      model_.Link_Set_Trajectory_rotation(model_.Upper_Body, control_time_, tc_.taskcommand_.command_time, tc_.taskcommand_.command_time + tc_.taskcommand_.traj_time, DyrosMath::rotateWithY(3.141592 / 180.0 * tc_.taskcommand_.angle), true);

      f_star.segment(0, 6) = wc_.getfstar6d(model_.Pelvis, kp_, kd_, kpa_, kda_);
      f_star.segment(6, 3) = wc_.getfstar_rot(model_.Upper_Body, kpa_, kda_);
    }
    else if (tc_.taskcommand_.mode_ == 5)
    {

      wc_.contact_set(2, cp);
      torque_gravity_ = wc_.gravity_compensation_torque();
      task_number = 12;

      J_task.setZero(task_number, total_dof_ + 6);
      f_star.setZero(task_number);

      J_task.block(0, 0, 6, total_dof_ + 6) = model_.link_[model_.Pelvis].Jac;
      J_task.block(6, 0, 3, total_dof_ + 6) = model_.link_[model_.Upper_Body].Jac.block(3, 0, 3, total_dof_ + 6);
      J_task.block(9, 0, 3, total_dof_ + 6) = model_.link_[model_.Right_Hand].Jac.block(3, 0, 3, total_dof_ + 6);

      task_desired = (tc_.taskcommand_.f_ratio * model_.link_[model_.Left_Foot].xpos + (1.0 - tc_.taskcommand_.f_ratio) * model_.link_[model_.Right_Foot].xpos) - model_.link_[model_.COM_id].xpos + model_.link_[model_.Pelvis].xpos;
      task_desired(2) = tc_.taskcommand_.height;
      model_.Link_Set_Trajectory_from_quintic(model_.Pelvis, control_time_, tc_.taskcommand_.command_time, tc_.taskcommand_.command_time + tc_.taskcommand_.traj_time, task_desired);
      model_.Link_Set_Trajectory_rotation(model_.Pelvis, control_time_, tc_.taskcommand_.command_time, tc_.taskcommand_.command_time + tc_.taskcommand_.traj_time, Eigen::Matrix3d::Identity(), true);

      model_.Link_Set_Trajectory_rotation(model_.Upper_Body, control_time_, tc_.taskcommand_.command_time, tc_.taskcommand_.command_time + tc_.taskcommand_.traj_time, Eigen::Matrix3d::Identity(), true);

      //model_.link_[model_.Upper_Body].r_traj = DyrosMath::rotateWithX(3.141592 / 6.0) * DyrosMath::rotateWithY(3.141592 / 3.0);

      f_star.segment(0, 6) = wc_.getfstar6d(model_.Pelvis, kp_, kd_, kpa_, kda_);
      f_star.segment(6, 3) = wc_.getfstar_rot(model_.Upper_Body, kpa_, kda_);
    }
    else if (tc_.taskcommand_.mode_ == 6)
    {

      wc_.contact_set(2, cp);
      torque_gravity_ = wc_.gravity_compensation_torque();
      task_number = 15;

      J_task.setZero(task_number, total_dof_ + 6);
      f_star.setZero(task_number);
      J_task.block(0, 0, 6, total_dof_ + 6) = model_.link_[model_.Pelvis].Jac;
      J_task.block(6, 0, 3, total_dof_ + 6) = model_.link_[model_.Upper_Body].Jac.block(3, 0, 3, total_dof_ + 6);
      J_task.block(9, 0, 6, total_dof_ + 6) = model_.link_[model_.Right_Hand].Jac;

      task_desired = (tc_.taskcommand_.f_ratio * model_.link_[model_.Left_Foot].xpos + (1.0 - tc_.taskcommand_.f_ratio) * model_.link_[model_.Right_Foot].xpos) - model_.link_[model_.COM_id].xpos + model_.link_[model_.Pelvis].xpos;
      task_desired(2) = tc_.taskcommand_.height;
      model_.Link_Set_Trajectory_from_quintic(model_.Pelvis, control_time_, tc_.taskcommand_.command_time, tc_.taskcommand_.command_time + tc_.taskcommand_.traj_time, task_desired);
      model_.Link_Set_Trajectory_rotation(model_.Pelvis, control_time_, tc_.taskcommand_.command_time, tc_.taskcommand_.command_time + tc_.taskcommand_.traj_time, Eigen::Matrix3d::Identity(), true);

      model_.Link_Set_Trajectory_rotation(model_.Upper_Body, control_time_, tc_.taskcommand_.command_time, tc_.taskcommand_.command_time + tc_.taskcommand_.traj_time, Eigen::Matrix3d::Identity(), true);

      f_star.segment(0, 6) = wc_.getfstar6d(model_.Pelvis, kp_, kd_, kpa_, kda_);
      f_star.segment(6, 3) = wc_.getfstar_rot(model_.Upper_Body, kpa_, kda_);
      //wc_.getfstar(kpa_, kda_, model_.link_[model_.Upper_Body].r_traj, model_.link_[model_.Upper_Body].Rotm, model_.link_[model_.Upper_Body].w_traj, model_.link_[model_.Upper_Body].w);

      Vector3d rhand_desired;
      rhand_desired << 0.17, -0.2, 0.93;
      model_.Link_Set_Trajectory_from_quintic(model_.Right_Hand, control_time_, tc_.taskcommand_.command_time, tc_.taskcommand_.command_time + tc_.taskcommand_.traj_time, rhand_desired);
      model_.Link_Set_Trajectory_rotation(model_.Right_Hand, control_time_, tc_.taskcommand_.command_time, tc_.taskcommand_.command_time + tc_.taskcommand_.traj_time, DyrosMath::rotateWithZ(-3.141592 / 180.0 * tc_.taskcommand_.angle), false);

      f_star.segment(9, 6) = wc_.getfstar6d(model_.Pelvis, kp_, kd_, kpa_, kda_);
    }
    else if (tc_.taskcommand_.mode_ == 7)
    {

      wc_.contact_set(2, cp);
      torque_gravity_ = wc_.gravity_compensation_torque();
      task_number = 9;
      J_task.setZero(task_number, total_dof_ + 6);
      f_star.setZero(task_number);

      J_task.block(0, 0, 6, total_dof_ + 6) = model_.link_[model_.Pelvis].Jac;
      J_task.block(6, 0, 3, total_dof_ + 6) = model_.link_[model_.Upper_Body].Jac.block(3, 0, 3, total_dof_ + 6);

      task_desired = (tc_.taskcommand_.f_ratio * model_.link_[model_.Left_Foot].xpos + (1.0 - tc_.taskcommand_.f_ratio) * model_.link_[model_.Right_Foot].xpos) - model_.link_[model_.COM_id].xpos + model_.link_[model_.Pelvis].xpos;
      task_desired(2) = tc_.taskcommand_.height;
      model_.Link_Set_Trajectory_from_quintic(model_.Pelvis, control_time_, tc_.taskcommand_.command_time, tc_.taskcommand_.command_time + tc_.taskcommand_.traj_time, task_desired);
      model_.Link_Set_Trajectory_rotation(model_.Pelvis, control_time_, tc_.taskcommand_.command_time, tc_.taskcommand_.command_time + tc_.taskcommand_.traj_time, Eigen::Matrix3d::Identity(), true);
      model_.Link_Set_Trajectory_rotation(model_.Upper_Body, control_time_, tc_.taskcommand_.command_time, tc_.taskcommand_.command_time + tc_.taskcommand_.traj_time, DyrosMath::rotateWithY(3.141592 / 180.0 * tc_.taskcommand_.angle), true);

      f_star.segment(0, 6) = wc_.getfstar6d(model_.Pelvis, kp_, kd_, kpa_, kda_);
      f_star.segment(6, 3) = wc_.getfstar_rot(model_.Upper_Body, kpa_, kda_);

      VectorXd ls, rs;
      ls.setZero(8);
      rs.setZero(8);

      rs << 0.6679347644640007, 2.3066997216988754, -0.3786424777329119, -0.2596871465720641, 1.496633838360999, -1.2804412127743907, 0.47155860285827594, 0.5577709399612447;
      ls << -0.6679347553824808, -2.306699727386902, 0.37864248471482626, 0.25968714853102454, -1.4966338400186168, 1.2804412110532044, -0.47155861293173135, -0.557770929404626;

      VectorXd desired_q_ub, cubic_q_ub;
      desired_q_ub.setZero(16);
      cubic_q_ub.setZero(16);

      desired_q_ub.segment(0, 8) = ls;
      desired_q_ub.segment(8, 8) = rs;
      VectorVQd q_trac;
      q_trac.setZero();

      q_trac.segment(6, 15) = init_q_.segment(0, 15);

      for (int i = 0; i < 16; i++)
      {
        q_trac(21 + i) = DyrosMath::cubic(control_time_, tc_.taskcommand_.command_time, tc_.taskcommand_.command_time + tc_.taskcommand_.traj_time * 0.6, init_q_(15 + i), desired_q_ub(i), 0.0, 0.0);
        cubic_q_ub(i) = DyrosMath::cubic(control_time_, tc_.taskcommand_.command_time, tc_.taskcommand_.command_time + tc_.taskcommand_.traj_time * 0.6, init_q_(15 + i), desired_q_ub(i), 0, 0);
      }

      //torque_joint_control_ = ((100 * (q_trac - q_virtual_.segment(0, total_dof_ + 6)) + 20 * (-q_dot_virtual_))).segment(6, total_dof_);
      //torque_joint_control_.segment(0, 15).setZero();

      VectorXd q_star_ub, q_upper_body_, q_upper_dot_;
      q_star_ub.setZero(16);
      q_upper_body_.setZero(16);
      q_upper_dot_.setZero(16);
      q_upper_body_ = q_virtual_.segment(21, 16);
      q_upper_dot_ = q_dot_virtual_.segment(21, 16);
      VectorVQd q_star;
      q_star.setZero();
      q_star_ub = 625 * (cubic_q_ub - q_upper_body_) + 50 * (-q_upper_dot_);

      q_star.segment(21, 16) = q_star_ub;

      torque_joint_control_ = (model_.A_ * q_star).segment(6, total_dof_);
      //torque_joint_control_.segment(15, 16) = torque_upper_body;
    }
    else if (tc_.taskcommand_.mode_ == 8)
    {

      wc_.contact_set(2, cp);
      torque_gravity_ = wc_.gravity_compensation_torque();
      task_number = 21;

      J_task.setZero(task_number, total_dof_ + 6);
      f_star.setZero(task_number);
      J_task.block(0, 0, 6, total_dof_ + 6) = model_.link_[model_.Pelvis].Jac;
      J_task.block(6, 0, 3, total_dof_ + 6) = model_.link_[model_.Upper_Body].Jac.block(3, 0, 3, total_dof_ + 6);
      J_task.block(9, 0, 6, total_dof_ + 6) = model_.link_[model_.Right_Hand].Jac;
      J_task.block(15, 0, 6, total_dof_ + 6) = model_.link_[model_.Left_Hand].Jac;

      task_desired = (tc_.taskcommand_.f_ratio * model_.link_[model_.Left_Foot].xpos + (1.0 - tc_.taskcommand_.f_ratio) * model_.link_[model_.Right_Foot].xpos) - model_.link_[model_.COM_id].xpos + model_.link_[model_.Pelvis].xpos;
      task_desired(2) = tc_.taskcommand_.height;
      model_.Link_Set_Trajectory_from_quintic(model_.Pelvis, control_time_, tc_.taskcommand_.command_time, tc_.taskcommand_.command_time + tc_.taskcommand_.traj_time, task_desired);
      model_.Link_Set_Trajectory_rotation(model_.Pelvis, control_time_, tc_.taskcommand_.command_time, tc_.taskcommand_.command_time + tc_.taskcommand_.traj_time, Eigen::Matrix3d::Identity(), true);
      model_.Link_Set_Trajectory_rotation(model_.Upper_Body, control_time_, tc_.taskcommand_.command_time, tc_.taskcommand_.command_time + tc_.taskcommand_.traj_time, DyrosMath::rotateWithY(3.141592 / 180.0 * tc_.taskcommand_.angle), true);

      f_star.segment(0, 6) = wc_.getfstar6d(model_.Pelvis, kp_, kd_, kpa_, kda_);
      f_star.segment(6, 3) = wc_.getfstar_rot(model_.Upper_Body, kpa_, kda_);
      //wc_.getfstar(kpa_, kda_, model_.link_[model_.Upper_Body].r_traj, model_.link_[model_.Upper_Body].Rotm, model_.link_[model_.Upper_Body].w_traj, model_.link_[model_.Upper_Body].w);

      rdes = model_.link_[model_.Right_Hand].x_init;
      rdes(1) = -0.17;

      ldes = model_.link_[model_.Left_Hand].x_init;
      ldes(1) = 0.17;

      Matrix3d r_rot_des;
      r_rot_des << 0, 0, 1, 0, -1, 0, 1, 0, 0;

      model_.Link_Set_Trajectory_from_quintic(model_.Right_Hand, control_time_, tc_.taskcommand_.command_time, tc_.taskcommand_.command_time + tc_.taskcommand_.traj_time, rdes);
      model_.Link_Set_Trajectory_rotation(model_.Right_Hand, control_time_, tc_.taskcommand_.command_time, tc_.taskcommand_.command_time + tc_.taskcommand_.traj_time, r_rot_des, false);
      f_star.segment(9, 6) = wc_.getfstar6d(model_.Right_Hand, kp_, kd_, kpa_, kda_);

      model_.Link_Set_Trajectory_from_quintic(model_.Left_Hand, control_time_, tc_.taskcommand_.command_time, tc_.taskcommand_.command_time + tc_.taskcommand_.traj_time, ldes);
      model_.Link_Set_Trajectory_rotation(model_.Left_Hand, control_time_, tc_.taskcommand_.command_time, tc_.taskcommand_.command_time + tc_.taskcommand_.traj_time, r_rot_des, false);
      f_star.segment(15, 6) = wc_.getfstar6d(model_.Left_Hand, kp_, kd_, kpa_, kda_);

      if (control_time_ > tc_.taskcommand_.command_time + tc_.taskcommand_.traj_time)
      {
        f_slc_matrix.setIdentity(task_number, task_number);
        f_slc_matrix(10, 10) = 0;
        f_slc_matrix(16, 16) = 0;
        hand_f_desired.setZero(task_number);
        hand_f_desired(10) = 50.0;
        hand_f_desired(16) = -50.0;

        VectorXd ft_hand_;
        ft_hand_.resize(12);
        ft_hand_.segment(0, 6) = right_hand_ft_;
        ft_hand_.segment(6, 6) = left_hand_ft_;

        //torque_task_ = wc_.task_control_torque_custom_force(J_task, f_star, f_slc_matrix, hand_f_desired);
        //torque_task_ = wc_.task_control_torque_custom_force_feedback(J_task, f_star, f_slc_matrix, hand_f_desired, ft_hand_);
        wc_.set_force_control_feedback(f_slc_matrix, hand_f_desired, ft_hand_);
      }
    }
    else if (tc_.taskcommand_.mode_ == 9)
    {

      wc_.contact_set(2, cp);
      torque_gravity_ = wc_.gravity_compensation_torque();
      task_number = 21;

      J_task.setZero(task_number, total_dof_ + 6);
      f_star.setZero(task_number);
      J_task.block(0, 0, 6, total_dof_ + 6) = model_.link_[model_.Pelvis].Jac;
      J_task.block(6, 0, 3, total_dof_ + 6) = model_.link_[model_.Upper_Body].Jac.block(3, 0, 3, total_dof_ + 6);
      J_task.block(9, 0, 6, total_dof_ + 6) = model_.link_[model_.Right_Hand].Jac;
      J_task.block(15, 0, 6, total_dof_ + 6) = model_.link_[model_.Left_Hand].Jac;

      task_desired = (tc_.taskcommand_.f_ratio * model_.link_[model_.Left_Foot].xpos + (1.0 - tc_.taskcommand_.f_ratio) * model_.link_[model_.Right_Foot].xpos) - model_.link_[model_.COM_id].xpos + model_.link_[model_.Pelvis].xpos;
      task_desired(2) = tc_.taskcommand_.height;
      model_.Link_Set_Trajectory_from_quintic(model_.Pelvis, control_time_, tc_.taskcommand_.command_time, tc_.taskcommand_.command_time + tc_.taskcommand_.traj_time, task_desired);
      model_.Link_Set_Trajectory_rotation(model_.Pelvis, control_time_, tc_.taskcommand_.command_time, tc_.taskcommand_.command_time + tc_.taskcommand_.traj_time, Eigen::Matrix3d::Identity(), true);
      model_.Link_Set_Trajectory_rotation(model_.Upper_Body, control_time_, tc_.taskcommand_.command_time, tc_.taskcommand_.command_time + tc_.taskcommand_.traj_time, DyrosMath::rotateWithY(3.141592 / 180.0 * tc_.taskcommand_.angle), true);

      f_star.segment(0, 6) = wc_.getfstar6d(model_.Pelvis, kp_, kd_, kpa_, kda_);
      f_star.segment(6, 3) = wc_.getfstar_rot(model_.Upper_Body, kpa_, kda_);
      //wc_.getfstar(kpa_, kda_, model_.link_[model_.Upper_Body].r_traj, model_.link_[model_.Upper_Body].Rotm, model_.link_[model_.Upper_Body].w_traj, model_.link_[model_.Upper_Body].w);

      model_.link_[model_.Right_Hand].x_init(1) = -0.17;
      model_.link_[model_.Left_Hand].x_init(1) = 0.17;

      rdes << 0.375, -0.14, tc_.taskcommand_.height + 0.4;
      ldes << 0.375, 0.14, tc_.taskcommand_.height + 0.4;

      rdes << model_.link_[model_.Right_Hand].x_init(0), -0.14, model_.link_[model_.Right_Hand].x_init(2) + 0.2;
      ldes << model_.link_[model_.Left_Hand].x_init(0), 0.14, model_.link_[model_.Left_Hand].x_init(2) + 0.2;

      Matrix3d r_rot_des;
      //r_rot_des << -1, 0, 0, 0, -1, 0, 0, 0, 1;

      r_rot_des << 0, 0, 1, 0, -1, 0, 1, 0, 0;

      //rdes = model_.link_[model_.Upper_Body].rot_init.transpose() * (rdes - model_.link_[model_.Upper_Body].x_init);
      //ldes = model_.link_[model_.Upper_Body].rot_init.transpose() * (ldes - model_.link_[model_.Upper_Body].x_init);

      //model_.Link_Set_Trajectory_from_quintic(model_.Right_Hand, control_time_, tc_.taskcommand_.command_time, tc_.taskcommand_.command_time + tc_.taskcommand_.traj_time, model_.link_[model_.Upper_Body].Rotm * rdes + model_.link_[model_.Upper_Body].xpos);
      model_.Link_Set_Trajectory_from_quintic(model_.Right_Hand, control_time_, tc_.taskcommand_.command_time, tc_.taskcommand_.command_time + tc_.taskcommand_.traj_time, rdes);
      model_.Link_Set_Trajectory_rotation(model_.Right_Hand, control_time_, tc_.taskcommand_.command_time, tc_.taskcommand_.command_time + tc_.taskcommand_.traj_time, r_rot_des, false);
      model_.link_[model_.Right_Hand].v_traj += model_.link_[model_.Pelvis].v_traj + model_.link_[model_.Upper_Body].w_traj.cross(model_.link_[model_.Right_Hand].xpos - model_.link_[model_.Upper_Body].xpos);
      f_star.segment(9, 6) = wc_.getfstar6d(model_.Right_Hand, kp_, kd_, kpa_, kda_);

      //model_.Link_Set_Trajectory_from_quintic(model_.Left_Hand, control_time_, tc_.taskcommand_.command_time, tc_.taskcommand_.command_time + tc_.taskcommand_.traj_time, model_.link_[model_.Upper_Body].Rotm * ldes + model_.link_[model_.Upper_Body].xpos);
      model_.Link_Set_Trajectory_from_quintic(model_.Left_Hand, control_time_, tc_.taskcommand_.command_time, tc_.taskcommand_.command_time + tc_.taskcommand_.traj_time, ldes);
      model_.Link_Set_Trajectory_rotation(model_.Left_Hand, control_time_, tc_.taskcommand_.command_time, tc_.taskcommand_.command_time + tc_.taskcommand_.traj_time, r_rot_des, false);
      model_.link_[model_.Left_Hand].v_traj += model_.link_[model_.Pelvis].v_traj + model_.link_[model_.Upper_Body].w_traj.cross(model_.link_[model_.Left_Hand].xpos - model_.link_[model_.Upper_Body].xpos);
      f_star.segment(15, 6) = wc_.getfstar6d(model_.Left_Hand, kp_, kd_, kpa_, kda_);

      f_slc_matrix.setIdentity(task_number, task_number);
      f_slc_matrix(10, 10) = 0;
      f_slc_matrix(16, 16) = 0;
      hand_f_desired.setZero(task_number);
      hand_f_desired(10) = 50.0;
      hand_f_desired(16) = -50.0;

      VectorXd ft_hand_;
      ft_hand_.resize(12);
      ft_hand_.segment(0, 6) = right_hand_ft_;
      ft_hand_.segment(6, 6) = left_hand_ft_;

      //torque_task_ = wc_.task_control_torque_custom_force(J_task, f_star, f_slc_matrix, hand_f_desired);
      //torque_task_ = wc_.task_control_torque_custom_force_feedback(J_task, f_star, f_slc_matrix, hand_f_desired, ft_hand_);
      wc_.set_force_control_feedback(f_slc_matrix, hand_f_desired, ft_hand_);
    }
    else if (tc_.taskcommand_.mode_ == 10)
    { //Contact Force Distribution with QP
      QP_switch = true;
      task_number = 6;
      J_task.setZero(task_number, total_dof_ + 6);
      f_star.setZero(task_number);
      J_task.block(0, 0, 6, total_dof_ + 6) = model_.link_[model_.Pelvis].Jac;

      task_desired = (tc_.taskcommand_.f_ratio * model_.link_[model_.Left_Foot].xpos + (1.0 - tc_.taskcommand_.f_ratio) * model_.link_[model_.Right_Foot].xpos) - model_.link_[model_.COM_id].xpos + model_.link_[model_.Pelvis].xpos;

      task_desired(0) = (tc_.taskcommand_.f_ratio * model_.link_[model_.Right_Foot].xpos(0) + (1.0 - tc_.taskcommand_.f_ratio) * model_.link_[model_.Right_Hand].xpos(0));
      task_desired(1) = 0.5 * model_.link_[model_.Left_Foot].xpos(1) + 0.5 * model_.link_[model_.Right_Foot].xpos(1);
      task_desired(2) = tc_.taskcommand_.height;

      model_.Link_Set_Trajectory_from_quintic(model_.Pelvis, control_time_, tc_.taskcommand_.command_time, tc_.taskcommand_.command_time + tc_.taskcommand_.traj_time, task_desired);
      model_.Link_Set_Trajectory_rotation(model_.Pelvis, control_time_, tc_.taskcommand_.command_time, tc_.taskcommand_.command_time + tc_.taskcommand_.traj_time, Eigen::Matrix3d::Identity(), true);
      if (tc_.taskcommand_.angle > 1)
        f_star.segment(0, 6) = wc_.getfstar6d(model_.Pelvis, kp_, kd_, kpa_, kda_);

      f_star.segment(3, 3).setZero();

      wc_.contact_set_multi(1, 1, 1, 1);
      torque_gravity_ = wc_.gravity_compensation_torque();
    }
    else if (tc_.taskcommand_.mode_ == 11)
    {
      QP_wall = true;
      task_number = 6;
      J_task.setZero(task_number, total_dof_ + 6);
      f_star.setZero(task_number);
      J_task.block(0, 0, 6, total_dof_ + 6) = model_.link_[model_.Pelvis].Jac;
      wc_.contact_set_multi(1, 1, 0, 0);
      torque_gravity_ = wc_.gravity_compensation_torque();
    }
    else if (tc_.taskcommand_.mode_ == 12)
    {
      QP_wall = true;
      task_number = 6;
      J_task.setZero(task_number, total_dof_ + 6);
      f_star.setZero(task_number);
      J_task.block(0, 0, 6, total_dof_ + 6) = model_.link_[model_.Pelvis].Jac;
      wc_.contact_set_multi(0, 0, 1, 1);
      torque_gravity_ = wc_.gravity_compensation_torque();
    }
    else if (tc_.taskcommand_.mode_ == 13)
    {
      QP_wall = true;
      task_number = 6;
      J_task.setZero(task_number, total_dof_ + 6);
      f_star.setZero(task_number);
      J_task.block(0, 0, 6, total_dof_ + 6) = model_.link_[model_.Pelvis].Jac;
      wc_.contact_set_multi(1, 1, 1, 1);
      torque_gravity_ = wc_.gravity_compensation_torque();
    }

    ROS_INFO_ONCE("BASE : compute - Task torque");
    torque_task_ = wc_.task_control_torque(J_task, f_star);
  }
  else
  {
    //IF there is no task
    wc_.contact_set_multi(1, 1, 0, 0);
    torque_gravity_ = wc_.gravity_compensation_torque();
  }

  ROS_INFO_ONCE("BASE : compute 2");
  torque_desired = torque_gravity_ + torque_task_ + torque_joint_control_;

  ROS_INFO_ONCE("BASE : compute 3");

  if (!contact_switch)
  {
    torque_contact_.setZero();
  }
  else if (QP_switch)
  {
    double t_qp = ros::Time::now().toSec();
    torque_contact_ = wc_.contact_torque_calc_from_QP(torque_desired);
    double t_qp_a = ros::Time::now().toSec();

    if (debug)
    {
      std::cout << "QP compute time : " << (t_qp_a - t_qp) * 1000 << std::endl;
    }
  }
  else if (QP_wall)
  {
    torque_contact_ = wc_.contact_torque_calc_from_QP_wall_mod2(torque_desired, 2.0);
  }
  else
    torque_contact_ = wc_.contact_force_redistribution_torque(model_.yaw_radian, torque_desired, fc_redis, fc_ratio);

  if (!task_switch)
    torque_task_.setZero();

  ROS_INFO_ONCE("BASE : compute 5");
  torque_desired = torque_gravity_ + torque_task_ + torque_contact_ + torque_joint_control_;

  if (!gravity_switch)
    torque_desired = torque_gravity_;

  VectorXd cf_temp = wc_.get_contact_force(torque_desired);
  for (int i = 0; i < 12; i++)
    dynamics_pub_msgs.contact_force_predict[i] = cf_temp(i);
}

void ControlBase::reflect()
{
  last_sim_time_ = control_time_;

  dynamics_pub_msgs.header.stamp = ros::Time::now();
  dynamics_pub_msgs.control_time = control_time_;
  for (int i = 0; i < total_dof_; i++)
  {
    dynamics_pub_msgs.torque_task[i] = torque_task_(i);
    dynamics_pub_msgs.torque_contact[i] = torque_contact_(i);
    dynamics_pub_msgs.torque_gravity[i] = torque_gravity_(i);
    dynamics_pub_msgs.torque_control[i] = torque_(i);
  }
  for (int i = 0; i < 12; i++)
  {
    dynamics_pub_msgs.force_redistribution[i] = fc_redis(i);
  }
  for (int i = 0; i < task_number; i++)
  {
    dynamics_pub_msgs.f_star[i] = f_star(i);
  }
  for (int i = 0; i < 3; i++)
  {
    dynamics_pub_msgs.task_current[i] = model_.com_(i);
    dynamics_pub_msgs.zmp_position[i] = wc_.ZMP_pos(i);
  }
  dynamics_pub_msgs.redistribution_ratio = fc_ratio;
  if (rviz_pub)
  {
    joint_states_rviz.header.stamp = ros::Time::now();
    for (int i = 0; i < total_dof_; i++)
    {
      joint_states_rviz.position[i] = q_virtual_(i + 6);
    }
    joint_state_publisher_for_rviz.publish(joint_states_rviz);
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(q_virtual_(0), q_virtual_(1), q_virtual_(2)));
    tf::Quaternion q(q_virtual_(3), q_virtual_(4), q_virtual_(5), q_virtual_(total_dof_ + 6));
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "Pelvis_Link"));
  }

  point_pub_msgs.polygon.points[0].x = model_.com_(0);
  point_pub_msgs.polygon.points[0].y = model_.com_(1);
  point_pub_msgs.polygon.points[0].z = model_.com_(2);

  point_pub_msgs.polygon.points[1].x = model_.link_[model_.Right_Foot].xpos(0);
  point_pub_msgs.polygon.points[1].y = model_.link_[model_.Right_Foot].xpos(1);
  point_pub_msgs.polygon.points[1].z = model_.link_[model_.Right_Foot].xpos(2);

  point_pub_msgs.polygon.points[2].x = model_.link_[model_.Left_Foot].xpos(0);
  point_pub_msgs.polygon.points[2].y = model_.link_[model_.Left_Foot].xpos(1);
  point_pub_msgs.polygon.points[2].z = model_.link_[model_.Left_Foot].xpos(2);

  point_pub_msgs.polygon.points[3].x = model_.link_[model_.Pelvis].xpos(0);
  point_pub_msgs.polygon.points[3].y = model_.link_[model_.Pelvis].xpos(1);
  point_pub_msgs.polygon.points[3].z = model_.link_[model_.Pelvis].xpos(2);

  point_pub_msgs.header.stamp = ros::Time::now();

  point_pub.publish(point_pub_msgs);
  dynamics_pub.publish(dynamics_pub_msgs);
}

void ControlBase::parameterInitialize()
{
  ROS_INFO_ONCE("BASE : parameter initialize");
  q_.setZero();
  q_dot_.setZero();
  q_virtual_.setZero(total_dof_ + 7);
  q_dot_virtual_.setZero();
  torque_.setZero();
  left_foot_ft_.setZero();
  left_foot_ft_.setZero();
  desired_q_.setZero();
  torque_desired.setZero();
  position_desired.setZero();
  com_init.setZero();
  rot_init.setZero();
  task_desired.setZero();
  rot_desired.setZero();

  torque_task_.setZero();
  torque_contact_.setZero();

  compute_init = 0;
  gravity_switch = true;
  task_switch = false;
  contact_switch = true;

  last_sim_time_ = 0.0;
  control_time_ = 0.0;

  ROS_INFO_ONCE("BASE : parameter initialize end");
}
void ControlBase::readDevice()
{
  ros::spinOnce();
}

} // namespace dyros_red_controller
