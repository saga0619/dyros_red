
#include "dyros_red_controller/control_base.h"

namespace dyros_red_controller
{

// Constructor
ControlBase::ControlBase(ros::NodeHandle &nh, double Hz) : ui_update_count_(0), is_first_boot_(true), Hz_(Hz), control_mask_{}, total_dof_(DyrosRedModel::MODEL_DOF),
                                                           wholebody_controller_(model_, q_, Hz, control_time_), task_controller_(model_, q_, Hz, control_time_)
{

  ROS_INFO_ONCE("BASE : control base initialize");
  /*walking_cmd_sub_ = nh.subscribe
  makeIDInverseList();
  joint_state_pub_.init(nh, "/torque_jet/joint_state", 3);
  joint_state_pub_.msg_.name.resize(DyrosRedModel::MODEL_DOF);
  joint_state_pub_.msg_.angle.resize(DyrosRedModel::MODEL_DOF);
  joint_state_pub_.msg_.velocity.resize(DyrosRedModel::MODEL_DOF);
  joint_state_pub_.msg_.current.resize(DyrosRedModel::MODEL_DOF);
  joint_state_pub_.msg_.error.resize(DyrosRedModel::MODEL_DOF);
  joint_state_pub_.msg_.effort.resize(DyrosRedModel::MODEL_DOF);
  for (int i=0; i< DyrosRedModel::MODEL_DOF; i++)
  {
    joint_state_pub_.msg_.name[i] = DyrosRedModel::JOINT_NAME[i];
    joint_state_pub_.msg_.id[i] = DyrosRedModel::JOINT_ID[i];
  }

  smach_pub_.init(nh, "/transition", 1);
  smach_sub_ = nh.subscribe("/Jimin_machine/smach/container_status", 3, &ControlBase::smachCallback, this);
  smach_sub_ = nh.subscribe("/torque_jet/smach/container_status", 3, &ControlBase::smachCallback, this);
  task_comamnd_sub_ = nh.subscribe("/torque_jet/task_command", 3, &ControlBase::taskCommandCallback, this);
*/

  com_pos_pub = nh.advertise<geometry_msgs::PolygonStamped>("/dyros_red/point", 3);
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

  com_pos_pub_msgs.polygon.points.resize(4);

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
    std::cout << " Eri " << std::endl;
    std::cout << model_.Eri << std::endl;
    std::cout << "waist skm " << std::endl;
    std::cout << DyrosMath::skm(model_.link_[1].xpos - model_.link_[0].xpos);
    std::cout << "rhand skm " << std::endl;
    std::cout << DyrosMath::skm(model_.link_[model_.Right_Hand].xpos - model_.link_[0].xpos);
    std::cout << std::endl;
    std::cout << "A matrix" << std::endl;
    std::cout << model_.A_ << std::endl;
    std::cout << "G matrix" << std::endl
              << wholebody_controller_.G << std::endl;
    std::cout << model_.link_[0].Rotm << std::endl;
    std::cout << std::endl;
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

  ROS_INFO("COM TRAJ MSG received at, %f \t to %f in %f seconds.", task_controller_.taskcommand_.command_time, task_controller_.taskcommand_.f_ratio, task_controller_.taskcommand_.traj_time);

  model_.Link_Set_initpos(model_.COM_id);
  model_.Link_Set_initpos(model_.Right_Hand);
  model_.Link_Set_initpos(model_.Left_Hand);
  model_.Link_Set_initpos(model_.Pelvis);

  task_switch = true;
}

void ControlBase::update()
{
  model_.updateKinematics(q_virtual_); // Update end effector positions and Jacobian
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
      model_.Link_vw_Update(model_.COM_id, q_dot_virtual_);
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

      model_.Link_vw_Update(model_.Pelvis, q_dot_virtual_);
      model_.Link_vw_Update(model_.Right_Hand, q_dot_virtual_);
      model_.Link_vw_Update(model_.Left_Hand, q_dot_virtual_);

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
      model_.Link_vw_Update(model_.Pelvis, q_dot_virtual_);
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
      model_.Link_vw_Update(model_.Pelvis, q_dot_virtual_);
      model_.Link_Set_Trajectory_from_quintic(model_.Pelvis, control_time_, task_controller_.taskcommand_.command_time, task_controller_.taskcommand_.command_time + task_controller_.taskcommand_.traj_time, task_desired);
      f_star = wholebody_controller_.getfstar6d(model_.Pelvis, kp_, kd_, kpa_, kda_);
    }
    torque_task_ = wholebody_controller_.task_control_torque(J_task, f_star);
  }

  ROS_INFO_ONCE("BASE : compute 2");
  torque_desired = torque_gravity_ + torque_task_;

  ROS_INFO_ONCE("BASE : compute 3");
  torque_contact_ = wholebody_controller_.contact_force_redistribution_torque(model_.yaw_radian, torque_desired, fc_redis, fc_ratio);

  ROS_INFO_ONCE("BASE : compute 4");
  if (!contact_switch)
    torque_contact_.setZero();
  if (!task_switch)
    torque_task_.setZero();

  ROS_INFO_ONCE("BASE : compute 5");
  torque_desired = torque_gravity_ + torque_task_ + torque_contact_;

  if (!gravity_switch)
    torque_desired = torque_gravity_;
}

void ControlBase::reflect()
{
  /*
  data_pub_msg_.header.stamp = ros::Time::now();
  for(int i=0;i<total_dof_;i++)data_pub_msg_.effort[i] = torque_task_(i);
  for(int i=0;i<6;i++)data_pub_msg_.position[i] = f_star(i);
  for(int i=0;i<12;i++){
    data_pub_msg_.velocity[i] = fc_redis(i);
  }
  */
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
      joint_states_rviz.position[i] = q_(i);
    }
    joint_state_publisher_for_rviz.publish(joint_states_rviz);
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(q_virtual_(0), q_virtual_(1), q_virtual_(2)));
    tf::Quaternion q;
    q.setRPY(q_virtual_(3), q_virtual_(4), q_virtual_(5));
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "Pelvis_Link"));
  }

  com_pos_pub_msgs.polygon.points[0].x = model_.com_(0);
  com_pos_pub_msgs.polygon.points[0].y = model_.com_(1);
  com_pos_pub_msgs.polygon.points[0].z = model_.com_(2);

  com_pos_pub_msgs.polygon.points[1].x = model_.link_[model_.Right_Foot].xpos(0);
  com_pos_pub_msgs.polygon.points[1].y = model_.link_[model_.Right_Foot].xpos(1);
  com_pos_pub_msgs.polygon.points[1].z = model_.link_[model_.Right_Foot].xpos(2);

  com_pos_pub_msgs.polygon.points[2].x = model_.link_[model_.Left_Foot].xpos(0);
  com_pos_pub_msgs.polygon.points[2].y = model_.link_[model_.Left_Foot].xpos(1);
  com_pos_pub_msgs.polygon.points[2].z = model_.link_[model_.Left_Foot].xpos(2);

  com_pos_pub_msgs.polygon.points[3].x = model_.link_[model_.Pelvis].xpos(0);
  com_pos_pub_msgs.polygon.points[3].y = model_.link_[model_.Pelvis].xpos(1);
  com_pos_pub_msgs.polygon.points[3].z = model_.link_[model_.Pelvis].xpos(2);

  com_pos_pub_msgs.header.stamp = ros::Time::now();

  com_pos_pub.publish(com_pos_pub_msgs);
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

/*
void ControlBase::taskCommandCallback(const dyros_red_msgs::TaskCommandConstPtr& msg)
{
  for(unsigned int i=0; i<4; i++)
  {
    if(msg->end_effector[i])
    {
      Eigen::Isometry3d target;
      tf::poseMsgToEigen(msg->pose[i], target);

      if(msg->mode[i] == dyros_red_msgs::TaskCommand::RELATIVE)
      {
        const auto &current =  model_.getCurrentTrasmfrom((DyrosRedModel::EndEffector)i);
        target.translation() = target.translation() + current.translation();
        target.linear() = current.linear() * target.linear();
      }
      task_controller_.setTarget((DyrosRedModel::EndEffector)i, target, msg->duration[i]);
      task_controller_.setEnable((DyrosRedModel::EndEffector)i, true);
    }
  }
}
*/

} // namespace dyros_red_controller
