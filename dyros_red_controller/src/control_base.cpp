
#include "dyros_red_controller/control_base.h"

namespace dyros_red_controller
{

// Constructor
ControlBase::ControlBase(ros::NodeHandle &nh, double Hz) : ui_update_count_(0), is_first_boot_(true), Hz_(Hz), control_mask_{}, total_dof_(DyrosRedModel::MODEL_DOF),
                                                           wholebody_controller_(model_, q_, Hz, control_time_), task_controller_(model_, q_, Hz, control_time_)
{
  ROS_INFO_ONCE("BASE : control base initialize");

  point_pub = nh.advertise<geometry_msgs::PolygonStamped>("/dyros_red/point", 3);
  dynamics_pub = nh.advertise<dyros_red_msgs::Dynamicsinfo>("/dyros_red/dynamics", 10);

  command_sub = nh.subscribe("/dyros_red/command", 100, &ControlBase::command_cb, this);
  com_command_sub = nh.subscribe("/dyros_red/com_command", 10, &ControlBase::com_command_cb, this);

  dynamics_pub_msgs.force_redistribution.resize(12);
  dynamics_pub_msgs.torque_contact.resize(total_dof_);
  dynamics_pub_msgs.torque_control.resize(total_dof_);
  dynamics_pub_msgs.torque_gravity.resize(total_dof_);
  dynamics_pub_msgs.torque_task.resize(total_dof_);
  dynamics_pub_msgs.f_star.resize(18);
  dynamics_pub_msgs.task_desired.resize(3);
  dynamics_pub_msgs.task_current.resize(3);

  q_virtual_quaternion.setZero(total_dof_ + 7);

  if (rviz_pub)
  {
    joint_state_publisher_for_rviz = nh.advertise<sensor_msgs::JointState>("/joint_states", 1);
    joint_states_rviz.name.resize(total_dof_);
    joint_states_rviz.position.resize(total_dof_);
    for (int i = 0; i < total_dof_; i++)
    {
      joint_states_rviz.name[i] = model_.JOINT_NAME[i];
    }
  }

  model_.debug_mode_ = debug;
  wholebody_controller_.debug = debug;

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
              << wholebody_controller_.G << std::endl;
    std::cout << model_.link_[0].Rotm << std::endl
              << "Rhand rot" << std::endl
              << model_.link_[model_.Right_Hand].Rotm << std::endl
              << "Lhand rot" << std::endl
              << model_.link_[model_.Left_Hand].Rotm << std::endl;
  }
}

void ControlBase::com_command_cb(const dyros_red_msgs::ComCommandConstPtr &msg)
{

  //rot_init = model_.link_[model_.Upper_Body].Rotm;
  task_controller_.taskcommand_.command_time = control_time_;
  task_controller_.taskcommand_.traj_time = msg->time;
  task_controller_.taskcommand_.f_ratio = msg->ratio;
  task_controller_.taskcommand_.height = msg->height;
  task_controller_.taskcommand_.mode_ = msg->mode;
  task_controller_.taskcommand_.angle = msg->angle;

  ROS_INFO("COM TRAJ MSG received at, %f \t to %f in %f seconds.", task_controller_.taskcommand_.command_time, task_controller_.taskcommand_.f_ratio, task_controller_.taskcommand_.traj_time);

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
  int ci = 0;
  ROS_INFO_ONCE("BASE : update dynamics");
  wholebody_controller_.update_dynamics_mode(wholebody_controller_.DOUBLE_SUPPORT);

  ROS_INFO_ONCE("BASE : gravity torque calc");
  torque_gravity_ = wholebody_controller_.gravity_compensation_torque();
  torque_task_.setZero();
  ROS_INFO_ONCE("BASE : compute 0");

  VectorQd torque_joint_control_;
  torque_joint_control_.setZero();

  Vector3d kp_, kd_, kpa_, kda_;
  for (int i = 0; i < 3; i++)
  {
    kp_(i) = 400;
    kd_(i) = 40;
    kpa_(i) = 400;
    kda_(i) = 40;
  }
  MatrixXd J_task;

  ROS_INFO_ONCE("BASE : compute 1");
  if (task_switch)
  {
    if (task_controller_.taskcommand_.mode_ == 0)
    {
      // COM jacobian control
      task_number = 6;
      J_task.setZero(task_number, total_dof_ + 6);
      f_star.setZero(task_number);
      J_task = model_.link_[model_.COM_id].Jac;

      task_desired = (task_controller_.taskcommand_.f_ratio * model_.link_[model_.Left_Foot].xpos + (1.0 - task_controller_.taskcommand_.f_ratio) * model_.link_[model_.Right_Foot].xpos);
      task_desired(2) = task_controller_.taskcommand_.height;
      model_.Link_Set_Trajectory_from_quintic(model_.COM_id, control_time_, task_controller_.taskcommand_.command_time, task_controller_.taskcommand_.command_time + task_controller_.taskcommand_.traj_time, task_desired);
      f_star = wholebody_controller_.getfstar6d(model_.COM_id, kp_, kd_, kpa_, kda_);
    }
    else if (task_controller_.taskcommand_.mode_ == 1)
    {
      // Pelvis control with holding hand position
      task_number = 18;

      J_task.setZero(task_number, total_dof_ + 6);
      f_star.setZero(task_number);

      J_task.block(0, 0, 6, total_dof_ + 6) = model_.link_[model_.Pelvis].Jac_COM;
      J_task.block(6, 0, 6, total_dof_ + 6) = model_.link_[model_.Right_Hand].Jac_COM;
      J_task.block(12, 0, 6, total_dof_ + 6) = model_.link_[model_.Left_Hand].Jac_COM;

      task_desired = (task_controller_.taskcommand_.f_ratio * model_.link_[model_.Left_Foot].xpos + (1.0 - task_controller_.taskcommand_.f_ratio) * model_.link_[model_.Right_Foot].xpos);
      task_desired(2) = task_controller_.taskcommand_.height;

      model_.Link_Set_Trajectory_from_quintic(model_.Pelvis, control_time_, task_controller_.taskcommand_.command_time, task_controller_.taskcommand_.command_time + task_controller_.taskcommand_.traj_time, task_desired);
      model_.Link_Set_Trajectory(model_.Right_Hand, model_.link_[model_.Pelvis].xpos + model_.link_[model_.Right_Hand].x_init - model_.link_[model_.Pelvis].x_init, Eigen::Vector3d::Zero(), model_.link_[model_.Right_Hand].rot_init, Eigen::Vector3d::Zero());
      model_.Link_Set_Trajectory(model_.Left_Hand, model_.link_[model_.Pelvis].xpos + model_.link_[model_.Left_Hand].x_init - model_.link_[model_.Pelvis].x_init, Eigen::Vector3d::Zero(), model_.link_[model_.Left_Hand].rot_init, Eigen::Vector3d::Zero());

      f_star.segment(0, 6) = wholebody_controller_.getfstar6d(model_.Pelvis, kp_, kd_, kpa_, kda_);
      f_star.segment(6, 6) = wholebody_controller_.getfstar6d(model_.Right_Hand, kp_, kd_, kpa_, kda_);
      f_star.segment(12, 6) = wholebody_controller_.getfstar6d(model_.Left_Hand, kp_, kd_, kpa_, kda_);
    }
    else if (task_controller_.taskcommand_.mode_ == 2)
    {
      // Pelvis control
      task_number = 6;
      J_task.setZero(task_number, total_dof_ + 6);
      f_star.setZero(task_number);

      J_task = model_.link_[model_.Pelvis].Jac;

      task_desired = (task_controller_.taskcommand_.f_ratio * model_.link_[model_.Left_Foot].xpos + (1.0 - task_controller_.taskcommand_.f_ratio) * model_.link_[model_.Right_Foot].xpos);
      task_desired(2) = task_controller_.taskcommand_.height;
      model_.Link_Set_Trajectory_from_quintic(model_.Pelvis, control_time_, task_controller_.taskcommand_.command_time, task_controller_.taskcommand_.command_time + task_controller_.taskcommand_.traj_time, task_desired);
      f_star = wholebody_controller_.getfstar6d(model_.Pelvis, kp_, kd_, kpa_, kda_);
    }
    else if (task_controller_.taskcommand_.mode_ == 3)
    {
      // COM control with pelvis jacobian
      task_number = 6;
      J_task.setZero(task_number, total_dof_ + 6);
      f_star.setZero(task_number);

      J_task = model_.link_[model_.Pelvis].Jac;

      task_desired = (task_controller_.taskcommand_.f_ratio * model_.link_[model_.Left_Foot].xpos + (1.0 - task_controller_.taskcommand_.f_ratio) * model_.link_[model_.Right_Foot].xpos) - model_.link_[model_.COM_id].xpos + model_.link_[model_.Pelvis].xpos;
      task_desired(2) = task_controller_.taskcommand_.height;
      model_.Link_Set_Trajectory_from_quintic(model_.Pelvis, control_time_, task_controller_.taskcommand_.command_time, task_controller_.taskcommand_.command_time + task_controller_.taskcommand_.traj_time, task_desired);
      f_star = wholebody_controller_.getfstar6d(model_.Pelvis, kp_, kd_, kpa_, kda_);
    }
    else if (task_controller_.taskcommand_.mode_ == 4)
    {
      task_number = 9;

      J_task.setZero(task_number, total_dof_ + 6);
      f_star.setZero(task_number);

      J_task.block(0, 0, 6, total_dof_ + 6) = model_.link_[model_.Pelvis].Jac;
      J_task.block(6, 0, 3, total_dof_ + 6) = model_.link_[model_.Upper_Body].Jac.block(3, 0, 3, total_dof_ + 6);

      task_desired = (task_controller_.taskcommand_.f_ratio * model_.link_[model_.Left_Foot].xpos + (1.0 - task_controller_.taskcommand_.f_ratio) * model_.link_[model_.Right_Foot].xpos) - model_.link_[model_.COM_id].xpos + model_.link_[model_.Pelvis].xpos;
      task_desired(2) = task_controller_.taskcommand_.height;

      model_.Link_Set_Trajectory_from_quintic(model_.Pelvis, control_time_, task_controller_.taskcommand_.command_time, task_controller_.taskcommand_.command_time + task_controller_.taskcommand_.traj_time, task_desired);
      model_.Link_Set_Trajectory_rotation(model_.Pelvis, control_time_, task_controller_.taskcommand_.command_time, task_controller_.taskcommand_.command_time + task_controller_.taskcommand_.traj_time, Eigen::Matrix3d::Identity(), true);
      model_.Link_Set_Trajectory_rotation(model_.Upper_Body, control_time_, task_controller_.taskcommand_.command_time, task_controller_.taskcommand_.command_time + task_controller_.taskcommand_.traj_time, DyrosMath::rotateWithY(3.141592 / 180.0 * task_controller_.taskcommand_.angle), true);

      f_star.segment(0, 6) = wholebody_controller_.getfstar6d(model_.Pelvis, kp_, kd_, kpa_, kda_);
      f_star.segment(6, 3) = wholebody_controller_.getfstar_rot(model_.Upper_Body, kpa_, kda_);
    }
    else if (task_controller_.taskcommand_.mode_ == 5)
    {
      task_number = 12;

      J_task.setZero(task_number, total_dof_ + 6);
      f_star.setZero(task_number);

      J_task.block(0, 0, 6, total_dof_ + 6) = model_.link_[model_.Pelvis].Jac;
      J_task.block(6, 0, 3, total_dof_ + 6) = model_.link_[model_.Upper_Body].Jac.block(3, 0, 3, total_dof_ + 6);
      J_task.block(9, 0, 3, total_dof_ + 6) = model_.link_[model_.Right_Hand].Jac.block(3, 0, 3, total_dof_ + 6);

      task_desired = (task_controller_.taskcommand_.f_ratio * model_.link_[model_.Left_Foot].xpos + (1.0 - task_controller_.taskcommand_.f_ratio) * model_.link_[model_.Right_Foot].xpos) - model_.link_[model_.COM_id].xpos + model_.link_[model_.Pelvis].xpos;
      task_desired(2) = task_controller_.taskcommand_.height;
      model_.Link_Set_Trajectory_from_quintic(model_.Pelvis, control_time_, task_controller_.taskcommand_.command_time, task_controller_.taskcommand_.command_time + task_controller_.taskcommand_.traj_time, task_desired);
      model_.Link_Set_Trajectory_rotation(model_.Pelvis, control_time_, task_controller_.taskcommand_.command_time, task_controller_.taskcommand_.command_time + task_controller_.taskcommand_.traj_time, Eigen::Matrix3d::Identity(), true);

      model_.Link_Set_Trajectory_rotation(model_.Upper_Body, control_time_, task_controller_.taskcommand_.command_time, task_controller_.taskcommand_.command_time + task_controller_.taskcommand_.traj_time, Eigen::Matrix3d::Identity(), true);

      //model_.link_[model_.Upper_Body].r_traj = DyrosMath::rotateWithX(3.141592 / 6.0) * DyrosMath::rotateWithY(3.141592 / 3.0);

      f_star.segment(0, 6) = wholebody_controller_.getfstar6d(model_.Pelvis, kp_, kd_, kpa_, kda_);
      f_star.segment(6, 3) = wholebody_controller_.getfstar_rot(model_.Upper_Body, kpa_, kda_);
    }
    else if (task_controller_.taskcommand_.mode_ == 6)
    {
      task_number = 15;

      J_task.setZero(task_number, total_dof_ + 6);
      f_star.setZero(task_number);
      J_task.block(0, 0, 6, total_dof_ + 6) = model_.link_[model_.Pelvis].Jac;
      J_task.block(6, 0, 3, total_dof_ + 6) = model_.link_[model_.Upper_Body].Jac.block(3, 0, 3, total_dof_ + 6);
      J_task.block(9, 0, 6, total_dof_ + 6) = model_.link_[model_.Right_Hand].Jac;

      task_desired = (task_controller_.taskcommand_.f_ratio * model_.link_[model_.Left_Foot].xpos + (1.0 - task_controller_.taskcommand_.f_ratio) * model_.link_[model_.Right_Foot].xpos) - model_.link_[model_.COM_id].xpos + model_.link_[model_.Pelvis].xpos;
      task_desired(2) = task_controller_.taskcommand_.height;
      model_.Link_Set_Trajectory_from_quintic(model_.Pelvis, control_time_, task_controller_.taskcommand_.command_time, task_controller_.taskcommand_.command_time + task_controller_.taskcommand_.traj_time, task_desired);
      model_.Link_Set_Trajectory_rotation(model_.Pelvis, control_time_, task_controller_.taskcommand_.command_time, task_controller_.taskcommand_.command_time + task_controller_.taskcommand_.traj_time, Eigen::Matrix3d::Identity(), true);

      model_.Link_Set_Trajectory_rotation(model_.Upper_Body, control_time_, task_controller_.taskcommand_.command_time, task_controller_.taskcommand_.command_time + task_controller_.taskcommand_.traj_time, Eigen::Matrix3d::Identity(), true);

      f_star.segment(0, 6) = wholebody_controller_.getfstar6d(model_.Pelvis, kp_, kd_, kpa_, kda_);
      f_star.segment(6, 3) = wholebody_controller_.getfstar_rot(model_.Upper_Body, kpa_, kda_);
      //wholebody_controller_.getfstar(kpa_, kda_, model_.link_[model_.Upper_Body].r_traj, model_.link_[model_.Upper_Body].Rotm, model_.link_[model_.Upper_Body].w_traj, model_.link_[model_.Upper_Body].w);

      Vector3d rhand_desired;
      rhand_desired << 0.17, -0.2, 0.93;
      model_.Link_Set_Trajectory_from_quintic(model_.Right_Hand, control_time_, task_controller_.taskcommand_.command_time, task_controller_.taskcommand_.command_time + task_controller_.taskcommand_.traj_time, rhand_desired);
      model_.Link_Set_Trajectory_rotation(model_.Right_Hand, control_time_, task_controller_.taskcommand_.command_time, task_controller_.taskcommand_.command_time + task_controller_.taskcommand_.traj_time, DyrosMath::rotateWithZ(-3.141592 / 180.0 * task_controller_.taskcommand_.angle), false);

      f_star.segment(9, 6) = wholebody_controller_.getfstar6d(model_.Pelvis, kp_, kd_, kpa_, kda_);
    }
    else if (task_controller_.taskcommand_.mode_ == 7)
    {
      task_number = 9;
      J_task.setZero(task_number, total_dof_ + 6);
      f_star.setZero(task_number);

      J_task.block(0, 0, 6, total_dof_ + 6) = model_.link_[model_.Pelvis].Jac;
      J_task.block(6, 0, 3, total_dof_ + 6) = model_.link_[model_.Upper_Body].Jac.block(3, 0, 3, total_dof_ + 6);

      task_desired = (task_controller_.taskcommand_.f_ratio * model_.link_[model_.Left_Foot].xpos + (1.0 - task_controller_.taskcommand_.f_ratio) * model_.link_[model_.Right_Foot].xpos) - model_.link_[model_.COM_id].xpos + model_.link_[model_.Pelvis].xpos;
      task_desired(2) = task_controller_.taskcommand_.height;
      model_.Link_Set_Trajectory_from_quintic(model_.Pelvis, control_time_, task_controller_.taskcommand_.command_time, task_controller_.taskcommand_.command_time + task_controller_.taskcommand_.traj_time, task_desired);
      model_.Link_Set_Trajectory_rotation(model_.Pelvis, control_time_, task_controller_.taskcommand_.command_time, task_controller_.taskcommand_.command_time + task_controller_.taskcommand_.traj_time, Eigen::Matrix3d::Identity(), true);
      model_.Link_Set_Trajectory_rotation(model_.Upper_Body, control_time_, task_controller_.taskcommand_.command_time, task_controller_.taskcommand_.command_time + task_controller_.taskcommand_.traj_time, DyrosMath::rotateWithY(3.141592 / 180.0 * task_controller_.taskcommand_.angle), true);

      f_star.segment(0, 6) = wholebody_controller_.getfstar6d(model_.Pelvis, kp_, kd_, kpa_, kda_);
      f_star.segment(6, 3) = wholebody_controller_.getfstar_rot(model_.Upper_Body, kpa_, kda_);

      //r_arm_desired : 0.5512798123847757, 2.164994013118243, -0.31764867041525985, -0.3103130262398612, 1.8743844462393051, -1.3159605294206944, 0.2769620816042189, 0.40181840209960534
      //l_arm_desired : -0.5512798123847757, -2.164994013118243, 0.31764867041525985, 0.3103130262398612, -1.8743844462393051, 1.3159605294206944, -0.2769620816042189, -0.40181840209960534

      VectorXd desired_q_ub;
      desired_q_ub.setZero(16);
      desired_q_ub << -0.5512798123847757, -2.164994013118243, 0.31764867041525985, 0.3103130262398612, -1.8743844462393051, 1.3159605294206944, -0.2769620816042189, -0.40181840209960534, 0.5512798123847757, 2.164994013118243, -0.31764867041525985, -0.3103130262398612, 1.8743844462393051, -1.3159605294206944, 0.2769620816042189, 0.40181840209960534;

      VectorVQd q_trac;
      q_trac.setZero();

      q_trac.segment(6, 15) = init_q_.segment(6, 15);

      for (int i = 0; i < 16; i++)
      {
        q_trac(21 + i) = DyrosMath::cubic(control_time_, task_controller_.taskcommand_.command_time, task_controller_.taskcommand_.command_time + task_controller_.taskcommand_.traj_time, init_q_(15 + i), desired_q_ub(i), 0.0, 0.0);
      }

      torque_joint_control_ = (20 * (q_trac - q_virtual_.segment(0, total_dof_ + 6)) + 4 * (-q_dot_virtual_)).segment(6, total_dof_);

      torque_joint_control_.segment(0, 15).setZero();
    }
    else if (task_controller_.taskcommand_.mode_ == 8)
    {
      task_number = 21;

      J_task.setZero(task_number, total_dof_ + 6);
      f_star.setZero(task_number);
      J_task.block(0, 0, 6, total_dof_ + 6) = model_.link_[model_.Pelvis].Jac;
      J_task.block(6, 0, 3, total_dof_ + 6) = model_.link_[model_.Upper_Body].Jac.block(3, 0, 3, total_dof_ + 6);
      J_task.block(9, 0, 6, total_dof_ + 6) = model_.link_[model_.Right_Hand].Jac;
      J_task.block(15, 0, 6, total_dof_ + 6) = model_.link_[model_.Left_Hand].Jac;

      task_desired = (task_controller_.taskcommand_.f_ratio * model_.link_[model_.Left_Foot].xpos + (1.0 - task_controller_.taskcommand_.f_ratio) * model_.link_[model_.Right_Foot].xpos) - model_.link_[model_.COM_id].xpos + model_.link_[model_.Pelvis].xpos;
      task_desired(2) = task_controller_.taskcommand_.height;
      model_.Link_Set_Trajectory_from_quintic(model_.Pelvis, control_time_, task_controller_.taskcommand_.command_time, task_controller_.taskcommand_.command_time + task_controller_.taskcommand_.traj_time, task_desired);
      model_.Link_Set_Trajectory_rotation(model_.Pelvis, control_time_, task_controller_.taskcommand_.command_time, task_controller_.taskcommand_.command_time + task_controller_.taskcommand_.traj_time, Eigen::Matrix3d::Identity(), true);
      model_.Link_Set_Trajectory_rotation(model_.Upper_Body, control_time_, task_controller_.taskcommand_.command_time, task_controller_.taskcommand_.command_time + task_controller_.taskcommand_.traj_time, DyrosMath::rotateWithY(3.141592 / 180.0 * task_controller_.taskcommand_.angle), true);

      f_star.segment(0, 6) = wholebody_controller_.getfstar6d(model_.Pelvis, kp_, kd_, kpa_, kda_);
      f_star.segment(6, 3) = wholebody_controller_.getfstar_rot(model_.Upper_Body, kpa_, kda_);
      //wholebody_controller_.getfstar(kpa_, kda_, model_.link_[model_.Upper_Body].r_traj, model_.link_[model_.Upper_Body].Rotm, model_.link_[model_.Upper_Body].w_traj, model_.link_[model_.Upper_Body].w);

      Vector3d rdes, ldes;
      rdes << 0.23, -0.09, 0.1684;
      ldes << 0.23, 0.09, 0.1684;

      Matrix3d r_rot_des;
      r_rot_des << 0, 0, 1, 0, -1, 0, 1, 0, 0;

      model_.Link_Set_Trajectory_from_quintic(model_.Right_Hand, control_time_, task_controller_.taskcommand_.command_time, task_controller_.taskcommand_.command_time + task_controller_.taskcommand_.traj_time, rdes);
      model_.Link_Set_Trajectory_rotation(model_.Right_Hand, control_time_, task_controller_.taskcommand_.command_time, task_controller_.taskcommand_.command_time + task_controller_.taskcommand_.traj_time, r_rot_des, false);
      f_star.segment(9, 6) = wholebody_controller_.getfstar6d(model_.Right_Hand, kp_, kd_, kpa_, kda_);

      model_.Link_Set_Trajectory_from_quintic(model_.Left_Hand, control_time_, task_controller_.taskcommand_.command_time, task_controller_.taskcommand_.command_time + task_controller_.taskcommand_.traj_time, ldes);
      model_.Link_Set_Trajectory_rotation(model_.Left_Hand, control_time_, task_controller_.taskcommand_.command_time, task_controller_.taskcommand_.command_time + task_controller_.taskcommand_.traj_time, r_rot_des, false);
      f_star.segment(15, 6) = wholebody_controller_.getfstar6d(model_.Left_Hand, kp_, kd_, kpa_, kda_);
    }
    else if (task_controller_.taskcommand_.mode_ == 9)
    {
      task_number = 21;

      J_task.setZero(task_number, total_dof_ + 6);
      f_star.setZero(task_number);
      J_task.block(0, 0, 6, total_dof_ + 6) = model_.link_[model_.Pelvis].Jac;
      J_task.block(6, 0, 3, total_dof_ + 6) = model_.link_[model_.Upper_Body].Jac.block(3, 0, 3, total_dof_ + 6);
      J_task.block(9, 0, 6, total_dof_ + 6) = model_.link_[model_.Right_Hand].Jac;
      J_task.block(15, 0, 6, total_dof_ + 6) = model_.link_[model_.Left_Hand].Jac;

      task_desired = (task_controller_.taskcommand_.f_ratio * model_.link_[model_.Left_Foot].xpos + (1.0 - task_controller_.taskcommand_.f_ratio) * model_.link_[model_.Right_Foot].xpos) - model_.link_[model_.COM_id].xpos + model_.link_[model_.Pelvis].xpos;
      task_desired(2) = task_controller_.taskcommand_.height;
      model_.Link_Set_Trajectory_from_quintic(model_.Pelvis, control_time_, task_controller_.taskcommand_.command_time, task_controller_.taskcommand_.command_time + task_controller_.taskcommand_.traj_time, task_desired);
      model_.Link_Set_Trajectory_rotation(model_.Pelvis, control_time_, task_controller_.taskcommand_.command_time, task_controller_.taskcommand_.command_time + task_controller_.taskcommand_.traj_time, Eigen::Matrix3d::Identity(), true);
      model_.Link_Set_Trajectory_rotation(model_.Upper_Body, control_time_, task_controller_.taskcommand_.command_time, task_controller_.taskcommand_.command_time + task_controller_.taskcommand_.traj_time, DyrosMath::rotateWithY(3.141592 / 180.0 * task_controller_.taskcommand_.angle), true);

      f_star.segment(0, 6) = wholebody_controller_.getfstar6d(model_.Pelvis, kp_, kd_, kpa_, kda_);
      f_star.segment(6, 3) = wholebody_controller_.getfstar_rot(model_.Upper_Body, kpa_, kda_);
      //wholebody_controller_.getfstar(kpa_, kda_, model_.link_[model_.Upper_Body].r_traj, model_.link_[model_.Upper_Body].Rotm, model_.link_[model_.Upper_Body].w_traj, model_.link_[model_.Upper_Body].w);

      Vector3d rdes, ldes;
      rdes << 0.23, -0.08, 0.1684;
      ldes << 0.23, 0.08, 0.1684;

      Matrix3d r_rot_des;
      r_rot_des << -1, 0, 0, 0, -1, 0, 0, 0, 1;
      rdes = model_.link_[model_.Upper_Body].rot_init.transpose() * (rdes - model_.link_[model_.Upper_Body].x_init);
      ldes = model_.link_[model_.Upper_Body].rot_init.transpose() * (ldes - model_.link_[model_.Upper_Body].x_init);

      model_.Link_Set_Trajectory_from_quintic(model_.Right_Hand, control_time_, task_controller_.taskcommand_.command_time, task_controller_.taskcommand_.command_time + task_controller_.taskcommand_.traj_time, model_.link_[model_.Upper_Body].Rotm * rdes + model_.link_[model_.Upper_Body].xpos);
      model_.Link_Set_Trajectory_rotation(model_.Right_Hand, control_time_, task_controller_.taskcommand_.command_time, task_controller_.taskcommand_.command_time + task_controller_.taskcommand_.traj_time, r_rot_des, false);
      f_star.segment(9, 6) = wholebody_controller_.getfstar6d(model_.Right_Hand, kp_, kd_, kpa_ * 4, kda_ * 2);

      model_.Link_Set_Trajectory_from_quintic(model_.Left_Hand, control_time_, task_controller_.taskcommand_.command_time, task_controller_.taskcommand_.command_time + task_controller_.taskcommand_.traj_time, model_.link_[model_.Upper_Body].Rotm * ldes + model_.link_[model_.Upper_Body].xpos);
      model_.Link_Set_Trajectory_rotation(model_.Left_Hand, control_time_, task_controller_.taskcommand_.command_time, task_controller_.taskcommand_.command_time + task_controller_.taskcommand_.traj_time, r_rot_des, false);
      f_star.segment(15, 6) = wholebody_controller_.getfstar6d(model_.Left_Hand, kp_, kd_, kpa_ * 4, kda_ * 2);
    }

    torque_task_ = wholebody_controller_.task_control_torque(J_task, f_star);
  }

  ROS_INFO_ONCE("BASE : compute 2");
  torque_desired = torque_gravity_ + torque_task_ + torque_joint_control_;

  ROS_INFO_ONCE("BASE : compute 3");
  torque_contact_ = wholebody_controller_.contact_force_redistribution_torque(model_.yaw_radian, torque_desired, fc_redis, fc_ratio);

  ROS_INFO_ONCE("BASE : compute 4");
  if (!contact_switch)
    torque_contact_.setZero();
  if (!task_switch)
    torque_task_.setZero();

  ROS_INFO_ONCE("BASE : compute 5");
  torque_desired = torque_gravity_ + torque_task_ + torque_contact_ + torque_joint_control_;

  if (!gravity_switch)
    torque_desired = torque_gravity_;
}

void ControlBase::reflect()
{
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
    dynamics_pub_msgs.zmp_position[i] = wholebody_controller_.ZMP_pos(i);
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

  ROS_INFO_ONCE("BASE : parameter initialize end");
}
void ControlBase::readDevice()
{
  ros::spinOnce();
}

} // namespace dyros_red_controller
