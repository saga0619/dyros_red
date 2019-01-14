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

  point_pub_msgs.polygon.points.resize(11); //

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
  model_.Link_Set_initpos(model_.Right_Foot);
  model_.Link_Set_initpos(model_.Left_Foot);

  init_q_ = q_virtual_.segment(6, total_dof_);

  task_switch = true;
  cgen_init = true;
}

void ControlBase::update()
{
  model_.updateKinematics(q_virtual_, q_dot_virtual_, q_ddot_virtaul_); // Update end effector positions and Jacobian
}

void ControlBase::compute()
{
  ROS_INFO_ONCE("BASE : update dynamics");
  wc_.update_dynamics();

  int cp[4] = {model_.Right_Foot, model_.Left_Foot, model_.Right_Hand, model_.Left_Hand};

  torque_task_.setZero();
  torque_joint_control_.setZero();
  torque_dc_.setZero();

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

  model_.link_[model_.Upper_Body].Jac_COM;
  ROS_INFO_ONCE("BASE : compute 1");

  if (task_switch)
  {
    if (tc_.taskcommand_.mode_ == 0) //com jacobian control
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
    else if (tc_.taskcommand_.mode_ == 1) //Pelvis with Arms
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
    else if (tc_.taskcommand_.mode_ == 2) //Pelvis only
    {

      wc_.contact_set_multi(1, 1, 0, 0);
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
    else if (tc_.taskcommand_.mode_ == 3) //Pelvis COM combined
    {

      wc_.contact_set_multi(1, 1, 0, 0);
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
    else if (tc_.taskcommand_.mode_ == 4) //Pelvis COM combined + Upperbody rotate 90
    {

      wc_.contact_set_multi(1, 1, 0, 0);
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
    else if (tc_.taskcommand_.mode_ == 5) //Pelvis COM combined + Upperbody rotation hold
    {
      //QP_switch = true;
      wc_.contact_set_multi(1, 1, 0, 0);
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
    else if (tc_.taskcommand_.mode_ == 6) //right hand rotation hold with z axis test
    {

      wc_.contact_set_multi(1, 1, 0, 0);
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
    else if (tc_.taskcommand_.mode_ == 7) //pelv_COM control + arms joint control
    {

      wc_.contact_set_multi(1, 1, 0, 0);
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
    else if (tc_.taskcommand_.mode_ == 8) //pelv_COM control + arms task control
    {

      wc_.contact_set_multi(1, 1, 0, 0);
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
    else if (tc_.taskcommand_.mode_ == 9) //pelv_COM control + Upperbody rotation + arms hold control
    {

      wc_.contact_set_multi(1, 1, 0, 0);
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
    else if (tc_.taskcommand_.mode_ == 10) //multi contact
    {                                      //Contact Force Distribution with QP
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

      QP_switch = true;
    }
    else if (tc_.taskcommand_.mode_ == 11) //wall_holding_test 1
    {
      QP_wall = true;
      task_number = 6;
      J_task.setZero(task_number, total_dof_ + 6);
      f_star.setZero(task_number);
      J_task.block(0, 0, 6, total_dof_ + 6) = model_.link_[model_.Pelvis].Jac;
      wc_.contact_set_multi(1, 1, 0, 0);
      torque_gravity_ = wc_.gravity_compensation_torque();
    }
    else if (tc_.taskcommand_.mode_ == 12) //wall_holding_test 2
    {
      QP_wall = true;
      task_number = 6;
      J_task.setZero(task_number, total_dof_ + 6);
      f_star.setZero(task_number);
      J_task.block(0, 0, 6, total_dof_ + 6) = model_.link_[model_.Pelvis].Jac;
      wc_.contact_set_multi(0, 0, 1, 1);
      torque_gravity_ = wc_.gravity_compensation_torque();
    }
    else if (tc_.taskcommand_.mode_ == 13) //wall_holding_test 3
    {
      QP_wall = true;
      task_number = 6;
      J_task.setZero(task_number, total_dof_ + 6);
      f_star.setZero(task_number);
      J_task.block(0, 0, 6, total_dof_ + 6) = model_.link_[model_.Pelvis].Jac;
      wc_.contact_set_multi(1, 1, 1, 1);
      torque_gravity_ = wc_.gravity_compensation_torque();
    }
    else if (tc_.taskcommand_.mode_ == 14) //single support right
    {
      task_number = 6;
      J_task.setZero(task_number, total_dof_ + 6);
      f_star.setZero(task_number);
      J_task.block(0, 0, 6, total_dof_ + 6) = model_.link_[model_.Pelvis].Jac;
      wc_.contact_set_multi(1, 0, 0, 0);
      torque_gravity_ = wc_.gravity_compensation_torque();

      task_desired = (tc_.taskcommand_.f_ratio * model_.link_[model_.Left_Foot].xpos + (1.0 - tc_.taskcommand_.f_ratio) * model_.link_[model_.Right_Foot].xpos) - model_.link_[model_.COM_id].xpos + model_.link_[model_.Pelvis].xpos;
      task_desired(2) = tc_.taskcommand_.height;
      model_.Link_Set_Trajectory_from_quintic(model_.Pelvis, control_time_, tc_.taskcommand_.command_time, tc_.taskcommand_.command_time + tc_.taskcommand_.traj_time, task_desired);
      model_.Link_Set_Trajectory_rotation(model_.Pelvis, control_time_, tc_.taskcommand_.command_time, tc_.taskcommand_.command_time + tc_.taskcommand_.traj_time, Eigen::Matrix3d::Identity(), true);

      f_star = wc_.getfstar6d(model_.Pelvis, kp_, kd_, kpa_, kda_);
    }
    else if (tc_.taskcommand_.mode_ == 15) //ZMP control test
    {
      //QP_switch = true;

      right_foot_contact_ = true;
      left_foot_contact_ = true;

      double time_segment = 1.0;
      double step_length = 0.1;

      double task_time = control_time_ - tc_.taskcommand_.command_time;
      Vector2d cp_current = model_.com_.CP;
      double w_ = sqrt(9.81 / model_.com_.pos(2));
      loop_temp = loop_;
      loop_ = (int)(task_time / time_segment);

      double loop_time = task_time - (double)loop_ * time_segment;

      double b_ = exp(w_ * (time_segment - loop_time));

      Vector2d desired_cp;
      Vector2d right_cp, left_cp, cp_mod;
      cp_mod << -0.02, -0.00747;
      right_cp << -0.04, -0.095;
      left_cp << -0.04, 0.095;

      if (loop_ - loop_temp)
      {
        loop_cnged = true;
        cgen_init = true;
      }
      double st_temp;

      if (loop_ % 2)
      {
        st_temp = step_length;
        if (loop_ == 1)
          st_temp = step_length / 2.0;

        desired_cp = right_cp;
        desired_cp(0) = right_cp(0) + ((double)loop_) * st_temp;
      }
      else
      {
        st_temp = step_length;
        if (loop_ == 1)
          st_temp = step_length / 2.0;
        desired_cp = left_cp;
        desired_cp(0) = left_cp(0) + ((double)loop_) * st_temp;
      }

      Vector2d zmp = 1 / (1 - b_) * desired_cp - b_ / (1 - b_) * model_.com_.CP;

      if (cgen_init)
      {
        std::cout << "loop : " << loop_ << " loop time : " << loop_time << std::endl;
        //cx_init = model_.com_.pos.segment(0, 2);
        //cv_init = model_.com_.vel.segment(0, 2);
        std::cout << "desired cp   x : " << desired_cp(0) << "  y : " << desired_cp(1) << std::endl;
        std::cout << "zmp gen   x : " << zmp(0) << "  y : " << zmp(1) << std::endl;
        //std::cout << "c CP" << std::endl;
        //std::cout << model_.com_.CP << std::endl;

        //zmp = 1 / (1 - b_) * desired_cp - b_ / (1 - b_) * model_.com_.CP;
        //cgen_init = false;
      }

      if (zmp(1) > 0.12)
      {
        zmp(1) = 0.12;
      }
      if (zmp(1) < -0.12)
      {
        zmp(1) = -0.12;
      }

      // est_cp_ = b_ * (model_.com_.pos.segment(0, 2) + model_.com_.vel.segment(0, 2) / w_) + (1 - b_) * zmp;
      // std::cout << "estimated_cp" << std::endl;
      // std::cout << est_cp_ << std::endl;
      wc_.set_zmp_control(zmp, 1.05);
      //MatrixXd damping_matrix;
      //damping_matrix.setIdentity(total_dof_, total_dof_);

      // //torque_dc_.segment(0, 12) = q_dot_.segment(0, 12) * 2.0;
      // std::cout << "zmp_des" << std::endl;
      // std::cout << zmp << std::endl;
      //std::cout << "zmp_cur" << std::endl;
      //std::cout << model_.com_.ZMP << std::endl;
      //std::cout << "Sensor ZMP" << std::endl;
      //std::cout << body_zmp_ << std::endl;

      //single support test at tc_
      // single support , 0.1 ~ 0.9s foot up -> down just for 4cm?
      // at loop_ = 0( first phase)
      // at loop_ = 1, zmp at right foot.
      // at each loop, 0~0.1 double support 0.1~ 0.9 singlesupport 0.9~1.0 double support

      //
      double lr_st, lr_mt, lr_et;
      lr_st = time_segment / 10.0;
      lr_mt = time_segment / 10.0 * 5.0;
      lr_et = time_segment / 10.0 * 9.0;

      if ((double)loop_ > 0.1)
      {
        if (loop_ % 2)
        {
          if ((loop_time < lr_et))
          {
            right_foot_contact_ = false;
            left_foot_contact_ = true;
          }
          else
          {
            right_foot_contact_ = true;
            left_foot_contact_ = true;
          }
        }
        else
        {
          if ((loop_time < lr_et))
          {
            right_foot_contact_ = true;
            left_foot_contact_ = false;
          }
          else
          {
            right_foot_contact_ = true;
            left_foot_contact_ = true;
          }
        }
      }
      //(model_.link_[model_.COM_id].x_init - zmp)*cosh(loop_time/time_segment)+time_segment * model_.link_[model_.COM_id]
      task_desired.setZero();
      task_desired(2) = tc_.taskcommand_.height;
      model_.Link_Set_Trajectory_from_quintic(model_.COM_id, control_time_, tc_.taskcommand_.command_time, tc_.taskcommand_.command_time + tc_.taskcommand_.traj_time, task_desired);

      // model_.link_[model_.COM_id].x_traj.segment(0, 2) = (cx_init - zmp) * cosh(loop_time * w_) + cv_init * sinh(loop_time * w_) / w_ + zmp;

      // model_.link_[model_.COM_id].v_traj.segment(0, 2) = (cx_init - zmp) * w_ * sinh(loop_time * w_) + cv_init * cosh(loop_time * w_);

      // std::cout << " xtraj : " << std::endl;
      //std::cout << model_.link_[model_.COM_id].x_traj.segment(0, 2) << std::endl;
      //std::cout << "vtrah : " << std::endl;
      //std::cout << model_.link_[model_.COM_id].v_traj.segment(0, 2) << std::endl;

      if (right_foot_contact_ && left_foot_contact_)
      {
        walking_init = true;
        task_number = 9;
        J_task.setZero(task_number, total_dof_ + 6);
        f_star.setZero(task_number);
        J_task.block(0, 0, 6, total_dof_ + 6) = model_.link_[model_.COM_id].Jac;
        J_task.block(6, 0, 3, total_dof_ + 6) = model_.link_[model_.Upper_Body].Jac.block(3, 0, 3, total_dof_ + 6);
        wc_.contact_set_multi(1, 1, 0, 0);
        torque_gravity_ = wc_.gravity_compensation_torque();

        model_.Link_Set_Trajectory_rotation(model_.Pelvis, control_time_, tc_.taskcommand_.command_time, tc_.taskcommand_.command_time + tc_.taskcommand_.traj_time, Eigen::Matrix3d::Identity(), false);
        model_.Link_Set_Trajectory_rotation(model_.Upper_Body, control_time_, tc_.taskcommand_.command_time, tc_.taskcommand_.command_time + tc_.taskcommand_.traj_time, Eigen::Matrix3d::Identity(), false);

        f_star.segment(0, 3) = wc_.getfstar_tra(model_.COM_id, kp_, kd_);
        f_star.segment(3, 3) = wc_.getfstar_rot(model_.Pelvis, kpa_, kda_);
        f_star.segment(6, 3) = wc_.getfstar_rot(model_.Upper_Body, kpa_, kda_);
      }
      else if (left_foot_contact_)
      {
        if (walking_init)
        {
          model_.link_[model_.Right_Foot].x_init = model_.link_[model_.Right_Foot].xpos;
          walking_init = false;
        }
        task_number = 15;

        wc_.contact_set_multi(0, 1, 0, 0);
        J_task.setZero(task_number, total_dof_ + 6);
        f_star.setZero(task_number);

        J_task.block(0, 0, 6, total_dof_ + 6) = model_.link_[model_.COM_id].Jac;
        J_task.block(6, 0, 3, total_dof_ + 6) = model_.link_[model_.Upper_Body].Jac.block(3, 0, 3, total_dof_ + 6);
        J_task.block(9, 0, 6, total_dof_ + 6) = model_.link_[model_.Right_Foot].Jac;
        torque_gravity_ = wc_.gravity_compensation_torque();
        model_.Link_Set_Trajectory_rotation(model_.Pelvis, control_time_, tc_.taskcommand_.command_time, tc_.taskcommand_.command_time + tc_.taskcommand_.traj_time, Eigen::Matrix3d::Identity(), false);
        model_.Link_Set_Trajectory_rotation(model_.Upper_Body, control_time_, tc_.taskcommand_.command_time, tc_.taskcommand_.command_time + tc_.taskcommand_.traj_time, Eigen::Matrix3d::Identity(), false);

        Vector3d lf_desired;
        lf_desired = model_.link_[model_.Right_Foot].x_init;
        lf_desired(1) = -0.1024;
        lf_desired(2) = lf_desired(2) + 0.04;

        model_.Link_Set_Trajectory_from_quintic(model_.Right_Foot, control_time_, tc_.taskcommand_.command_time + (double)loop_ * time_segment + lr_st, tc_.taskcommand_.command_time + (double)loop_ * time_segment + lr_mt, lf_desired);
        Vector3d lf_init = lf_desired;

        lf_desired(2) = lf_desired(2) - 0.04;
        if (loop_time > lr_mt)
          model_.Link_Set_Trajectory_from_quintic(model_.Right_Foot, control_time_, tc_.taskcommand_.command_time + (double)loop_ * time_segment + lr_mt, tc_.taskcommand_.command_time + (double)loop_ * time_segment + lr_et, lf_init, lf_desired);

        Eigen::Vector3d quintic = DyrosMath::QuinticSpline(control_time_, tc_.taskcommand_.command_time + (double)loop_ * time_segment + lr_st, tc_.taskcommand_.command_time + (double)loop_ * time_segment + lr_et - 0.05, model_.link_[model_.Right_Foot].x_init(0), 0, 0, model_.link_[model_.Left_Foot].xpos(0) + step_length, 0, 0);
        model_.link_[model_.Right_Foot].x_traj(0) = quintic(0);
        model_.link_[model_.Right_Foot].v_traj(0) = quintic(1);

        model_.Link_Set_Trajectory_rotation(model_.Right_Foot, control_time_, tc_.taskcommand_.command_time, tc_.taskcommand_.command_time + tc_.taskcommand_.traj_time, Eigen::Matrix3d::Identity(), false);

        f_star.segment(0, 3) = wc_.getfstar_tra(model_.COM_id, kp_, kd_);
        f_star.segment(3, 3) = wc_.getfstar_rot(model_.Pelvis, kpa_, kda_);
        f_star.segment(6, 3) = wc_.getfstar_rot(model_.Upper_Body, kpa_, kda_);
        f_star.segment(9, 3) = wc_.getfstar_tra(model_.Right_Foot, kp_, kd_);
        f_star.segment(12, 3) = wc_.getfstar_rot(model_.Right_Foot, kpa_, kda_);
      }
      else if (right_foot_contact_)
      {

        Vector3d lf_desired;
        if (walking_init)
        {
          model_.link_[model_.Left_Foot].x_init = model_.link_[model_.Left_Foot].xpos;
          walking_init = false;
        }
        task_number = 15;
        wc_.contact_set_multi(1, 0, 0, 0);
        J_task.setZero(task_number, total_dof_ + 6);
        f_star.setZero(task_number);

        J_task.block(0, 0, 6, total_dof_ + 6) = model_.link_[model_.COM_id].Jac;
        J_task.block(6, 0, 3, total_dof_ + 6) = model_.link_[model_.Upper_Body].Jac.block(3, 0, 3, total_dof_ + 6);
        J_task.block(9, 0, 6, total_dof_ + 6) = model_.link_[model_.Left_Foot].Jac;

        torque_gravity_ = wc_.gravity_compensation_torque();

        model_.Link_Set_Trajectory_rotation(model_.Pelvis, control_time_, tc_.taskcommand_.command_time, tc_.taskcommand_.command_time + tc_.taskcommand_.traj_time, Eigen::Matrix3d::Identity(), false);
        model_.Link_Set_Trajectory_rotation(model_.Upper_Body, control_time_, tc_.taskcommand_.command_time, tc_.taskcommand_.command_time + tc_.taskcommand_.traj_time, Eigen::Matrix3d::Identity(), false);

        lf_desired = model_.link_[model_.Left_Foot].x_init;

        lf_desired(1) = 0.1024;
        lf_desired(2) = lf_desired(2) + 0.04;

        model_.Link_Set_Trajectory_from_quintic(model_.Left_Foot, control_time_, tc_.taskcommand_.command_time + (double)loop_ * time_segment + lr_st, tc_.taskcommand_.command_time + (double)loop_ * time_segment + lr_mt, lf_desired);
        Vector3d lf_init = lf_desired;

        lf_desired(2) = lf_desired(2) - 0.04;
        if (loop_time > lr_mt)
          model_.Link_Set_Trajectory_from_quintic(model_.Left_Foot, control_time_, tc_.taskcommand_.command_time + (double)loop_ * time_segment + lr_mt, tc_.taskcommand_.command_time + (double)loop_ * time_segment + lr_et, lf_init, lf_desired);

        Eigen::Vector3d quintic = DyrosMath::QuinticSpline(control_time_, tc_.taskcommand_.command_time + (double)loop_ * time_segment + lr_st, tc_.taskcommand_.command_time + (double)loop_ * time_segment + lr_et - 0.05, model_.link_[model_.Left_Foot].x_init(0), 0, 0, model_.link_[model_.Right_Foot].xpos(0) + step_length, 0, 0);
        model_.link_[model_.Left_Foot].x_traj(0) = quintic(0);
        model_.link_[model_.Left_Foot].v_traj(0) = quintic(1);

        model_.Link_Set_Trajectory_rotation(model_.Left_Foot, control_time_, tc_.taskcommand_.command_time, tc_.taskcommand_.command_time + tc_.taskcommand_.traj_time, Eigen::Matrix3d::Identity(), false);

        f_star.segment(0, 3) = wc_.getfstar_tra(model_.COM_id, kp_, kd_);
        f_star.segment(3, 3) = wc_.getfstar_rot(model_.Pelvis, kpa_, kda_);
        f_star.segment(6, 3) = wc_.getfstar_rot(model_.Upper_Body, kpa_, kda_);
        f_star.segment(9, 3) = wc_.getfstar_tra(model_.Left_Foot, kp_, kd_);
        f_star.segment(12, 3) = wc_.getfstar_rot(model_.Left_Foot, kpa_, kda_);
      }
    }
    else if (tc_.taskcommand_.mode_ == 16)
    {
      right_foot_contact_ = true;
      left_foot_contact_ = true;

      double time_segment = 1.0;
      double step_length = 0.1;

      double task_time = control_time_ - tc_.taskcommand_.command_time;
      Vector2d cp_current = model_.com_.CP;
      double w_ = sqrt(9.81 / model_.com_.pos(2));
      loop_temp = loop_;
      loop_ = (int)(task_time / time_segment);

      if (loop_ > 7)
        loop_ = 7;

      double loop_time = task_time - (double)loop_ * time_segment;

      double b_ = exp(w_ * (time_segment - loop_time));

      Vector2d desired_cp;
      Vector2d right_cp, left_cp, cp_mod;
      cp_mod << -0.02, -0.00747;
      right_cp << -0.04, -0.095;
      left_cp << -0.04, 0.095;

      Vector3d R_init, L_init;
      R_init << -0.04, -0.1024, 0.13637;
      L_init << -0.04, 0.1024, 0.13637;
      Vector3d support_position[8];
      for (int i = 0; i < 8; i++)
      {
        support_position[i];
      }

      Vector3d des_foot_position[9];
      for (int i = 0; i < 8; i++)
      {
        des_foot_position[i] = R_init * (i % 2) + L_init * ((i + 1) % 2);
        des_foot_position[i](0) = des_foot_position[i](0) + i * step_length;
      }

      des_foot_position[6](0) = des_foot_position[6](0) - step_length;

      Vector3d cp_ref[9];
      for (int i = 0; i < 7; i++)
      {
        cp_ref[i] = des_foot_position[i];
        cp_ref[i](1) = des_foot_position[i](1) * 0.94;
      }
      cp_ref[6] = (des_foot_position[5] + des_foot_position[6]) * 0.5;
      cp_ref[7] = cp_ref[6];
      Vector2d zmp = 1 / (1 - b_) * cp_ref[loop_].segment(0, 2) - b_ / (1 - b_) * model_.com_.CP;

      if (zmp(1) > 0.12)
      {
        zmp(1) = 0.12;
      }
      if (zmp(1) < -0.12)
      {
        zmp(1) = -0.12;
      }

      static bool init = true;
      if (init)
      {
        for (int i = 0; i < 8; i++)
        {
          cout << i << " cpref    x : " << cp_ref[i](0) << "  y : " << cp_ref[i](1) << std::endl;
          cout << i << " cpref    x : " << des_foot_position[i](0) << "  y : " << des_foot_position[i](1) << std::endl;
        }

        init = false;
      }

      wc_.set_zmp_control(zmp, 1.05);

      double lr_st, lr_mt, lr_et;
      lr_st = time_segment / 10.0;
      lr_mt = time_segment / 10.0 * 5.0;
      lr_et = time_segment / 10.0 * 9.0;
      if ((double)loop_ > 0.1)
      {
        if (loop_ % 2)
        {
          if ((loop_time > lr_st) && (loop_time < lr_et))
          {
            right_foot_contact_ = false;
            left_foot_contact_ = true;
          }
          else
          {
            right_foot_contact_ = true;
            left_foot_contact_ = true;
          }
        }
        else
        {
          if ((loop_time > lr_st) && (loop_time < lr_et))
          {
            right_foot_contact_ = true;
            left_foot_contact_ = false;
          }
          else
          {
            right_foot_contact_ = true;
            left_foot_contact_ = true;
          }
        }

        if (loop_ == 7)
        {
          std::cout << "walk end" << std::endl;
          right_foot_contact_ = true;
          left_foot_contact_ = true;
        }
      }

      task_desired.setZero();
      task_desired(2) = tc_.taskcommand_.height;
      model_.Link_Set_Trajectory_from_quintic(model_.COM_id, control_time_, tc_.taskcommand_.command_time, tc_.taskcommand_.command_time + tc_.taskcommand_.traj_time, task_desired);

      //

      point_pub_msgs.polygon.points[8].x = wc_.getcpref(task_time, 0.5)(0);
      point_pub_msgs.polygon.points[8].y = wc_.getcpref(task_time, 0.5)(1);
      point_pub_msgs.polygon.points[8].z = 0.0;

      if (right_foot_contact_ && left_foot_contact_)
      {
        walking_init = true;
        task_number = 9;
        J_task.setZero(task_number, total_dof_ + 6);
        f_star.setZero(task_number);
        J_task.block(0, 0, 6, total_dof_ + 6) = model_.link_[model_.COM_id].Jac;
        J_task.block(6, 0, 3, total_dof_ + 6) = model_.link_[model_.Upper_Body].Jac.block(3, 0, 3, total_dof_ + 6);
        wc_.contact_set_multi(1, 1, 0, 0);
        torque_gravity_ = wc_.gravity_compensation_torque();

        model_.Link_Set_Trajectory_rotation(model_.Pelvis, control_time_, tc_.taskcommand_.command_time, tc_.taskcommand_.command_time + tc_.taskcommand_.traj_time, Eigen::Matrix3d::Identity(), false);
        model_.Link_Set_Trajectory_rotation(model_.Upper_Body, control_time_, tc_.taskcommand_.command_time, tc_.taskcommand_.command_time + tc_.taskcommand_.traj_time, Eigen::Matrix3d::Identity(), false);

        f_star.segment(0, 3) = wc_.getfstar_tra(model_.COM_id, kp_, kd_);
        f_star.segment(3, 3) = wc_.getfstar_rot(model_.Pelvis, kpa_, kda_);
        f_star.segment(6, 3) = wc_.getfstar_rot(model_.Upper_Body, kpa_, kda_);
      }
      else if (left_foot_contact_)
      {
        std::cout << "SS_ left_support" << std::endl;
        Vector3d lf_desired;
        if (walking_init)
        {
          model_.link_[model_.Right_Foot].x_init = model_.link_[model_.Right_Foot].xpos;
          walking_init = false;
        }
        task_number = 15;

        wc_.contact_set_multi(0, 1, 0, 0);
        J_task.setZero(task_number, total_dof_ + 6);
        f_star.setZero(task_number);

        J_task.block(0, 0, 6, total_dof_ + 6) = model_.link_[model_.COM_id].Jac;
        J_task.block(6, 0, 3, total_dof_ + 6) = model_.link_[model_.Upper_Body].Jac.block(3, 0, 3, total_dof_ + 6);
        J_task.block(9, 0, 6, total_dof_ + 6) = model_.link_[model_.Right_Foot].Jac;
        torque_gravity_ = wc_.gravity_compensation_torque();
        model_.Link_Set_Trajectory_rotation(model_.Pelvis, control_time_, tc_.taskcommand_.command_time, tc_.taskcommand_.command_time + tc_.taskcommand_.traj_time, Eigen::Matrix3d::Identity(), false);
        model_.Link_Set_Trajectory_rotation(model_.Upper_Body, control_time_, tc_.taskcommand_.command_time, tc_.taskcommand_.command_time + tc_.taskcommand_.traj_time, Eigen::Matrix3d::Identity(), false);

        lf_desired = des_foot_position[loop_];
        lf_desired(2) = 0.1363727 + 0.04;
        model_.Link_Set_Trajectory_from_quintic(model_.Right_Foot, control_time_, tc_.taskcommand_.command_time + (double)loop_ * time_segment + lr_st, tc_.taskcommand_.command_time + (double)loop_ * time_segment + lr_mt, lf_desired);
        Vector3d lf_init = lf_desired;

        lf_desired(2) = 0.1363727;
        if (loop_time > lr_mt)
          model_.Link_Set_Trajectory_from_quintic(model_.Right_Foot, control_time_, tc_.taskcommand_.command_time + (double)loop_ * time_segment + lr_mt, tc_.taskcommand_.command_time + (double)loop_ * time_segment + lr_et, lf_init, lf_desired);
        std::cout << loop_ << " landing pos : " << des_foot_position[loop_] << std::endl;
        Eigen::Vector3d quintic = DyrosMath::QuinticSpline(control_time_, tc_.taskcommand_.command_time + (double)loop_ * time_segment + lr_st, tc_.taskcommand_.command_time + (double)loop_ * time_segment + lr_et - 0.05, model_.link_[model_.Right_Foot].x_init(0), 0, 0, des_foot_position[loop_](0), 0, 0);
        model_.link_[model_.Right_Foot].x_traj(0) = quintic(0);
        model_.link_[model_.Right_Foot].v_traj(0) = quintic(1);

        model_.Link_Set_Trajectory_rotation(model_.Right_Foot, control_time_, tc_.taskcommand_.command_time, tc_.taskcommand_.command_time + tc_.taskcommand_.traj_time, Eigen::Matrix3d::Identity(), false);

        f_star.segment(0, 3) = wc_.getfstar_tra(model_.COM_id, kp_, kd_);
        f_star.segment(3, 3) = wc_.getfstar_rot(model_.Pelvis, kpa_, kda_);
        f_star.segment(6, 3) = wc_.getfstar_rot(model_.Upper_Body, kpa_, kda_);
        f_star.segment(9, 3) = wc_.getfstar_tra(model_.Right_Foot, kp_, kd_);
        f_star.segment(12, 3) = wc_.getfstar_rot(model_.Right_Foot, kpa_, kda_);
      }
      else if (right_foot_contact_)
      {
        std::cout << "SS_ right_support" << std::endl;

        Vector3d lf_desired;
        if (walking_init)
        {
          model_.link_[model_.Left_Foot].x_init = model_.link_[model_.Left_Foot].xpos;
          walking_init = false;
        }
        task_number = 15;
        wc_.contact_set_multi(1, 0, 0, 0);
        J_task.setZero(task_number, total_dof_ + 6);
        f_star.setZero(task_number);

        J_task.block(0, 0, 6, total_dof_ + 6) = model_.link_[model_.COM_id].Jac;
        J_task.block(6, 0, 3, total_dof_ + 6) = model_.link_[model_.Upper_Body].Jac.block(3, 0, 3, total_dof_ + 6);
        J_task.block(9, 0, 6, total_dof_ + 6) = model_.link_[model_.Left_Foot].Jac;

        torque_gravity_ = wc_.gravity_compensation_torque();

        model_.Link_Set_Trajectory_rotation(model_.Pelvis, control_time_, tc_.taskcommand_.command_time, tc_.taskcommand_.command_time + tc_.taskcommand_.traj_time, Eigen::Matrix3d::Identity(), false);
        model_.Link_Set_Trajectory_rotation(model_.Upper_Body, control_time_, tc_.taskcommand_.command_time, tc_.taskcommand_.command_time + tc_.taskcommand_.traj_time, Eigen::Matrix3d::Identity(), false);
        lf_desired = des_foot_position[loop_];
        lf_desired(2) = 0.1363727 + 0.04;

        model_.Link_Set_Trajectory_from_quintic(model_.Left_Foot, control_time_, tc_.taskcommand_.command_time + (double)loop_ * time_segment + lr_st, tc_.taskcommand_.command_time + (double)loop_ * time_segment + lr_mt, lf_desired);
        Vector3d lf_init = lf_desired;

        lf_desired(2) = 0.1363727;
        if (loop_time > lr_mt)
          model_.Link_Set_Trajectory_from_quintic(model_.Left_Foot, control_time_, tc_.taskcommand_.command_time + (double)loop_ * time_segment + lr_mt, tc_.taskcommand_.command_time + (double)loop_ * time_segment + lr_et, lf_init, lf_desired);
        std::cout << loop_ << " landing pos : " << des_foot_position[loop_] << std::endl;
        Eigen::Vector3d quintic = DyrosMath::QuinticSpline(control_time_, tc_.taskcommand_.command_time + (double)loop_ * time_segment + lr_st, tc_.taskcommand_.command_time + (double)loop_ * time_segment + lr_et - 0.05, model_.link_[model_.Left_Foot].x_init(0), 0, 0, des_foot_position[loop_](0), 0, 0);
        model_.link_[model_.Left_Foot].x_traj(0) = quintic(0);
        model_.link_[model_.Left_Foot].v_traj(0) = quintic(1);

        model_.Link_Set_Trajectory_rotation(model_.Left_Foot, control_time_, tc_.taskcommand_.command_time, tc_.taskcommand_.command_time + tc_.taskcommand_.traj_time, Eigen::Matrix3d::Identity(), false);

        f_star.segment(0, 3) = wc_.getfstar_tra(model_.COM_id, kp_, kd_);
        f_star.segment(3, 3) = wc_.getfstar_rot(model_.Pelvis, kpa_, kda_);
        f_star.segment(6, 3) = wc_.getfstar_rot(model_.Upper_Body, kpa_, kda_);
        f_star.segment(9, 3) = wc_.getfstar_tra(model_.Left_Foot, kp_, kd_);
        f_star.segment(12, 3) = wc_.getfstar_rot(model_.Left_Foot, kpa_, kda_);
      }
    }
    else if (tc_.taskcommand_.mode_ == 17)
    {
      //MPC application
      //MPC settings

      int n_sample = 30;
      double t_sample = 10 / 1000;

      double q_par = 1.0;
      double r_par = 0.4;

      MatrixXd H(2 * n_sample, 2 * n_sample);
      VectorXd g(2 * n_sample);
      //MatrixXd A;
      double f = 0.4;

      // MPC setting ends

      right_foot_contact_ = true;
      left_foot_contact_ = true;

      double time_segment = 1.0;
      double step_length = 0.1;

      double task_time = control_time_ - tc_.taskcommand_.command_time;
      Vector2d cp_current = model_.com_.CP;
      double w_ = sqrt(9.81 / model_.com_.pos(2));
      loop_temp = loop_;
      loop_ = (int)(task_time / time_segment);
      if (loop_ > 7)
        loop_ = 7;
      double loop_time = task_time - (double)loop_ * time_segment;
      double b_ = exp(w_ * (time_segment - loop_time));

      Vector2d desired_cp;
      Vector3d R_init, L_init;
      R_init << -0.04, -0.1024, 0.13637;
      L_init << -0.04, 0.1024, 0.13637;

      Vector3d des_foot_position[9];
      for (int i = 0; i < 8; i++)
      {
        des_foot_position[i] = R_init * (i % 2) + L_init * ((i + 1) % 2);
        des_foot_position[i](0) = des_foot_position[i](0) + i * step_length;
      }

      des_foot_position[6](0) = des_foot_position[6](0) - step_length;

      Vector3d cp_ref[9];
      for (int i = 0; i < 7; i++)
      {
        cp_ref[i] = des_foot_position[i];
        cp_ref[i](1) = des_foot_position[i](1) * 0.94;
      }
      cp_ref[6] = (des_foot_position[5] + des_foot_position[6]) * 0.5;
      cp_ref[7] = cp_ref[6];
      Vector2d zmp = 1 / (1 - b_) * cp_ref[loop_].segment(0, 2) - b_ / (1 - b_) * model_.com_.CP;

      std::cout << "current control is : " << zmp << std::endl;
      Vector2d MPC_zmp;

      //MPC_zmp = wc_.getcptraj(task_time, zmp);

      Vector3d cp_ref2[9];

      for (int i = 0; i < 8; i++)
        cp_ref2[i + 1] = cp_ref[i];
      cp_ref2[0] = (R_init + L_init) / 2;
      Vector2d CP_mpc;
      CP_mpc = exp(w_ * loop_time) * cp_ref2[loop_].segment(0, 2) + (1 - exp(w_ * loop_time)) * MPC_zmp;

      point_pub_msgs.polygon.points[9].x = MPC_zmp(0);
      point_pub_msgs.polygon.points[9].y = MPC_zmp(1);
      point_pub_msgs.polygon.points[9].z = 0.0;

      point_pub_msgs.polygon.points[10].x = CP_mpc(0);
      point_pub_msgs.polygon.points[10].y = CP_mpc(1);
      point_pub_msgs.polygon.points[10].z = 0.0;

      if (zmp(1) > 0.12)
      {
        zmp(1) = 0.12;
      }
      if (zmp(1) < -0.12)
      {
        zmp(1) = -0.12;
      }

      wc_.set_zmp_control(zmp, 1.0);

      double lr_st, lr_mt, lr_et;
      lr_st = time_segment / 10.0;
      lr_mt = time_segment / 10.0 * 5.0;
      lr_et = time_segment / 10.0 * 9.0;

      if ((double)loop_ > 0.1)
      {
        if (loop_ % 2)
        {
          if ((loop_time > lr_st) && (loop_time < lr_et))
          {
            right_foot_contact_ = false;
            left_foot_contact_ = true;
          }
          else
          {
            right_foot_contact_ = true;
            left_foot_contact_ = true;
          }
        }
        else
        {
          if ((loop_time > lr_st) && (loop_time < lr_et))
          {
            right_foot_contact_ = true;
            left_foot_contact_ = false;
          }
          else
          {
            right_foot_contact_ = true;
            left_foot_contact_ = true;
          }
        }

        if (loop_ == 7)
        {
          std::cout << "END" << std::endl;
          right_foot_contact_ = true;
          left_foot_contact_ = true;
        }
      }

      task_desired.setZero();
      task_desired(2) = tc_.taskcommand_.height;
      model_.Link_Set_Trajectory_from_quintic(model_.COM_id, control_time_, tc_.taskcommand_.command_time, tc_.taskcommand_.command_time + tc_.taskcommand_.traj_time, task_desired);

      //

      if (right_foot_contact_ && left_foot_contact_)
      {
        walking_init = true;
        task_number = 9;
        J_task.setZero(task_number, total_dof_ + 6);
        f_star.setZero(task_number);
        J_task.block(0, 0, 6, total_dof_ + 6) = model_.link_[model_.COM_id].Jac;
        J_task.block(6, 0, 3, total_dof_ + 6) = model_.link_[model_.Upper_Body].Jac.block(3, 0, 3, total_dof_ + 6);
        wc_.contact_set_multi(1, 1, 0, 0);
        torque_gravity_ = wc_.gravity_compensation_torque();

        model_.Link_Set_Trajectory_rotation(model_.Pelvis, control_time_, tc_.taskcommand_.command_time, tc_.taskcommand_.command_time + tc_.taskcommand_.traj_time, Eigen::Matrix3d::Identity(), false);
        model_.Link_Set_Trajectory_rotation(model_.Upper_Body, control_time_, tc_.taskcommand_.command_time, tc_.taskcommand_.command_time + tc_.taskcommand_.traj_time, Eigen::Matrix3d::Identity(), false);

        f_star.segment(0, 3) = wc_.getfstar_tra(model_.COM_id, kp_, kd_);
        f_star.segment(3, 3) = wc_.getfstar_rot(model_.Pelvis, kpa_, kda_);
        f_star.segment(6, 3) = wc_.getfstar_rot(model_.Upper_Body, kpa_, kda_);
      }
      else if (left_foot_contact_)
      {
        Vector3d lf_desired;
        if (walking_init)
        {
          model_.link_[model_.Right_Foot].x_init = model_.link_[model_.Right_Foot].xpos;
          walking_init = false;
        }
        task_number = 15;

        wc_.contact_set_multi(0, 1, 0, 0);
        J_task.setZero(task_number, total_dof_ + 6);
        f_star.setZero(task_number);

        J_task.block(0, 0, 6, total_dof_ + 6) = model_.link_[model_.COM_id].Jac;
        J_task.block(6, 0, 3, total_dof_ + 6) = model_.link_[model_.Upper_Body].Jac.block(3, 0, 3, total_dof_ + 6);
        J_task.block(9, 0, 6, total_dof_ + 6) = model_.link_[model_.Right_Foot].Jac;
        torque_gravity_ = wc_.gravity_compensation_torque();
        model_.Link_Set_Trajectory_rotation(model_.Pelvis, control_time_, tc_.taskcommand_.command_time, tc_.taskcommand_.command_time + tc_.taskcommand_.traj_time, Eigen::Matrix3d::Identity(), false);
        model_.Link_Set_Trajectory_rotation(model_.Upper_Body, control_time_, tc_.taskcommand_.command_time, tc_.taskcommand_.command_time + tc_.taskcommand_.traj_time, Eigen::Matrix3d::Identity(), false);

        lf_desired = des_foot_position[loop_];
        lf_desired(2) = 0.1363727 + 0.04;
        model_.Link_Set_Trajectory_from_quintic(model_.Right_Foot, control_time_, tc_.taskcommand_.command_time + (double)loop_ * time_segment + lr_st, tc_.taskcommand_.command_time + (double)loop_ * time_segment + lr_mt, lf_desired);
        Vector3d lf_init = lf_desired;

        lf_desired(2) = 0.1363727;
        if (loop_time > lr_mt)
          model_.Link_Set_Trajectory_from_quintic(model_.Right_Foot, control_time_, tc_.taskcommand_.command_time + (double)loop_ * time_segment + lr_mt, tc_.taskcommand_.command_time + (double)loop_ * time_segment + lr_et, lf_init, lf_desired);

        Eigen::Vector3d quintic = DyrosMath::QuinticSpline(control_time_, tc_.taskcommand_.command_time + (double)loop_ * time_segment + lr_st, tc_.taskcommand_.command_time + (double)loop_ * time_segment + lr_et - 0.05, model_.link_[model_.Right_Foot].x_init(0), 0, 0, des_foot_position[loop_](0), 0, 0);
        model_.link_[model_.Right_Foot].x_traj(0) = quintic(0);
        model_.link_[model_.Right_Foot].v_traj(0) = quintic(1);

        model_.Link_Set_Trajectory_rotation(model_.Right_Foot, control_time_, tc_.taskcommand_.command_time, tc_.taskcommand_.command_time + tc_.taskcommand_.traj_time, Eigen::Matrix3d::Identity(), false);

        f_star.segment(0, 3) = wc_.getfstar_tra(model_.COM_id, kp_, kd_);
        f_star.segment(3, 3) = wc_.getfstar_rot(model_.Pelvis, kpa_, kda_);
        f_star.segment(6, 3) = wc_.getfstar_rot(model_.Upper_Body, kpa_, kda_);
        f_star.segment(9, 3) = wc_.getfstar_tra(model_.Right_Foot, kp_, kd_);
        f_star.segment(12, 3) = wc_.getfstar_rot(model_.Right_Foot, kpa_, kda_);
      }
      else if (right_foot_contact_)
      {

        Vector3d lf_desired;
        if (walking_init)
        {
          model_.link_[model_.Left_Foot].x_init = model_.link_[model_.Left_Foot].xpos;
          walking_init = false;
        }
        task_number = 15;
        wc_.contact_set_multi(1, 0, 0, 0);
        J_task.setZero(task_number, total_dof_ + 6);
        f_star.setZero(task_number);

        J_task.block(0, 0, 6, total_dof_ + 6) = model_.link_[model_.COM_id].Jac;
        J_task.block(6, 0, 3, total_dof_ + 6) = model_.link_[model_.Upper_Body].Jac.block(3, 0, 3, total_dof_ + 6);
        J_task.block(9, 0, 6, total_dof_ + 6) = model_.link_[model_.Left_Foot].Jac;

        torque_gravity_ = wc_.gravity_compensation_torque();

        model_.Link_Set_Trajectory_rotation(model_.Pelvis, control_time_, tc_.taskcommand_.command_time, tc_.taskcommand_.command_time + tc_.taskcommand_.traj_time, Eigen::Matrix3d::Identity(), false);
        model_.Link_Set_Trajectory_rotation(model_.Upper_Body, control_time_, tc_.taskcommand_.command_time, tc_.taskcommand_.command_time + tc_.taskcommand_.traj_time, Eigen::Matrix3d::Identity(), false);
        lf_desired = des_foot_position[loop_];
        lf_desired(2) = 0.1363727 + 0.04;

        model_.Link_Set_Trajectory_from_quintic(model_.Left_Foot, control_time_, tc_.taskcommand_.command_time + (double)loop_ * time_segment + lr_st, tc_.taskcommand_.command_time + (double)loop_ * time_segment + lr_mt, lf_desired);
        Vector3d lf_init = lf_desired;

        lf_desired(2) = 0.1363727;
        if (loop_time > lr_mt)
          model_.Link_Set_Trajectory_from_quintic(model_.Left_Foot, control_time_, tc_.taskcommand_.command_time + (double)loop_ * time_segment + lr_mt, tc_.taskcommand_.command_time + (double)loop_ * time_segment + lr_et, lf_init, lf_desired);

        Eigen::Vector3d quintic = DyrosMath::QuinticSpline(control_time_, tc_.taskcommand_.command_time + (double)loop_ * time_segment + lr_st, tc_.taskcommand_.command_time + (double)loop_ * time_segment + lr_et - 0.05, model_.link_[model_.Left_Foot].x_init(0), 0, 0, des_foot_position[loop_](0), 0, 0);
        model_.link_[model_.Left_Foot].x_traj(0) = quintic(0);
        model_.link_[model_.Left_Foot].v_traj(0) = quintic(1);

        model_.Link_Set_Trajectory_rotation(model_.Left_Foot, control_time_, tc_.taskcommand_.command_time, tc_.taskcommand_.command_time + tc_.taskcommand_.traj_time, Eigen::Matrix3d::Identity(), false);

        f_star.segment(0, 3) = wc_.getfstar_tra(model_.COM_id, kp_, kd_);
        f_star.segment(3, 3) = wc_.getfstar_rot(model_.Pelvis, kpa_, kda_);
        f_star.segment(6, 3) = wc_.getfstar_rot(model_.Upper_Body, kpa_, kda_);
        f_star.segment(9, 3) = wc_.getfstar_tra(model_.Left_Foot, kp_, kd_);
        f_star.segment(12, 3) = wc_.getfstar_rot(model_.Left_Foot, kpa_, kda_);
      }
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
  {
    if (wc_.contact_index > 1)
    {
      torque_contact_ = wc_.contact_force_redistribution_torque(model_.yaw_radian, torque_desired, fc_redis, fc_ratio);
      //torque_contact_ = wc_.contact_torque_calc_from_QP(torque_desired);
    }
    else
    {
      torque_contact_.setZero();
    }
  }

  if (!task_switch)
    torque_task_.setZero();

  ROS_INFO_ONCE("BASE : compute 5");
  torque_desired = torque_gravity_ + torque_task_ + torque_contact_ + torque_joint_control_ + torque_dc_;

  left_foot_zmp_(0) = model_.link_[model_.Left_Foot].xpos(0) + (-left_foot_ft_(4) - left_foot_ft_(0) * 0.027) / left_foot_ft_(2);
  left_foot_zmp_(1) = model_.link_[model_.Left_Foot].xpos(1) + (left_foot_ft_(3) - left_foot_ft_(1) * 0.027) / left_foot_ft_(2);

  right_foot_zmp_(0) = model_.link_[model_.Right_Foot].xpos(1) + (-right_foot_ft_(4) - right_foot_ft_(0) * 0.027) / right_foot_ft_(2);
  right_foot_zmp_(1) = model_.link_[model_.Right_Foot].xpos(1) + (right_foot_ft_(3) - right_foot_ft_(1) * 0.027) / right_foot_ft_(2);

  Vector2d body_zmp_;
  body_zmp_(0) = (right_foot_zmp_(0) * right_foot_ft_(2) + left_foot_zmp_(0) * left_foot_ft_(2)) / (right_foot_ft_(2) + left_foot_ft_(2));
  body_zmp_(1) = (right_foot_zmp_(1) * right_foot_ft_(2) + left_foot_zmp_(1) * left_foot_ft_(2)) / (right_foot_ft_(2) + left_foot_ft_(2));

  Vector2d est_cp_;
  point_pub_msgs.polygon.points[4].x = body_zmp_(0);
  point_pub_msgs.polygon.points[4].y = body_zmp_(1);

  if (!gravity_switch)
    torque_desired = torque_gravity_;

  VectorXd cf_temp = wc_.get_contact_force(torque_desired);

  for (int i = 0; i < cf_temp.size(); i++)
    dynamics_pub_msgs.contact_force_predict[i] = cf_temp(i);
} // namespace dyros_red_controller

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
    dynamics_pub_msgs.task_current[i] = model_.com_.pos(i);
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

  point_pub_msgs.polygon.points[0].x = model_.com_.pos(0);
  point_pub_msgs.polygon.points[0].y = model_.com_.pos(1);
  point_pub_msgs.polygon.points[0].z = model_.com_.pos(2);

  point_pub_msgs.polygon.points[1].x = model_.link_[model_.Right_Foot].xpos(0);
  point_pub_msgs.polygon.points[1].y = model_.link_[model_.Right_Foot].xpos(1);
  point_pub_msgs.polygon.points[1].z = model_.link_[model_.Right_Foot].xpos(2);

  point_pub_msgs.polygon.points[2].x = model_.link_[model_.Left_Foot].xpos(0);
  point_pub_msgs.polygon.points[2].y = model_.link_[model_.Left_Foot].xpos(1);
  point_pub_msgs.polygon.points[2].z = model_.link_[model_.Left_Foot].xpos(2);

  point_pub_msgs.polygon.points[3].x = model_.link_[model_.Pelvis].xpos(0);
  point_pub_msgs.polygon.points[3].y = model_.link_[model_.Pelvis].xpos(1);
  point_pub_msgs.polygon.points[3].z = model_.link_[model_.Pelvis].xpos(2);

  //point_pub_msgs.polygon.points[4].x = model_.com_.ZMP(0);
  //point_pub_msgs.polygon.points[4].y = model_.com_.ZMP(1);
  point_pub_msgs.polygon.points[4].z = 0.0;

  point_pub_msgs.polygon.points[5].x = model_.com_.CP(0);
  point_pub_msgs.polygon.points[5].y = model_.com_.CP(1);
  point_pub_msgs.polygon.points[5].z = 0.0;

  point_pub_msgs.polygon.points[6].x = model_.com_.accel(0);
  point_pub_msgs.polygon.points[6].y = model_.com_.accel(1);
  point_pub_msgs.polygon.points[6].z = 0.0;

  point_pub_msgs.polygon.points[7].x = model_.com_.vel(0);
  point_pub_msgs.polygon.points[7].y = model_.com_.vel(1);
  point_pub_msgs.polygon.points[7].z = 0.0;

  point_pub_msgs.header.stamp = ros::Time::now();

  point_pub.publish(point_pub_msgs);
  dynamics_pub.publish(dynamics_pub_msgs);
}

void ControlBase::parameterInitialize()
{
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
}
void ControlBase::readDevice()
{
  ros::spinOnce();
}

} // namespace dyros_red_controller
