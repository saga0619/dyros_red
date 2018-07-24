
#include "dyros_red_controller/control_base.h"

namespace dyros_red_controller
{

// Constructor
ControlBase::ControlBase(ros::NodeHandle &nh, double Hz) :
  ui_update_count_(0), is_first_boot_(true), Hz_(Hz), control_mask_{}, total_dof_(DyrosRedModel::MODEL_DOF)
{
  //walking_cmd_sub_ = nh.subscribe
  //makeIDInverseList();

  joint_state_pub_.init(nh, "/torque_jet/joint_state", 3);
  joint_state_pub_.msg_.name.resize(DyrosRedModel::MODEL_DOF);
  joint_state_pub_.msg_.angle.resize(DyrosRedModel::MODEL_DOF);
  joint_state_pub_.msg_.velocity.resize(DyrosRedModel::MODEL_DOF);
  joint_state_pub_.msg_.current.resize(DyrosRedModel::MODEL_DOF);
  joint_state_pub_.msg_.error.resize(DyrosRedModel::MODEL_DOF);
  //joint_state_pub_.msg_.effort.resize(DyrosRedModel::MODEL_DOF);

  for (int i=0; i< DyrosRedModel::MODEL_DOF; i++)
  {
    joint_state_pub_.msg_.name[i] = DyrosRedModel::JOINT_NAME[i];
    //joint_state_pub_.msg_.id[i] = DyrosRedModel::JOINT_ID[i];
  }

  smach_pub_.init(nh, "/transition", 1);
  smach_sub_ = nh.subscribe("/Jimin_machine/smach/container_status", 3, &ControlBase::smachCallback, this);
  //smach_sub_ = nh.subscribe("/torque_jet/smach/container_status", 3, &ControlBase::smachCallback, this);
  //task_comamnd_sub_ = nh.subscribe("/torque_jet/task_command", 3, &ControlBase::taskCommandCallback, this);
  parameterInitialize();
  // model_.test();
}

bool ControlBase::checkStateChanged()
{
  if(previous_state_ != current_state_)
  {
    previous_state_ = current_state_;
    return true;
  }
  return false;
}


/*
void ControlBase::makeIDInverseList()
{
  for(int i=0;i<DyrosRedModel::MODEL_DOF; i++)
  {
    joint_id_[i] = DyrosRedModel::JOINT_ID[i];
    joint_id_inversed_[DyrosRedModel::JOINT_ID[i]] = i;
  }
}*/

void ControlBase::update()
{
  model_.updateKinematics(q_virtual_.head<DyrosRedModel::MODEL_DOF+6>());  // Update end effector positions and Jacobian
  //model_.test();
}

void ControlBase::stateChangeEvent()
{
  if(checkStateChanged())
  {
    if(current_state_ == "move1")
    {
      /*
      task_controller_.setEnable(DyrosRedModel::EE_LEFT_HAND, true);
      task_controller_.setEnable(DyrosRedModel::EE_RIGHT_HAND, false);
      task_controller_.setEnable(DyrosRedModel::EE_LEFT_FOOT, false);
      task_controller_.setEnable(DyrosRedModel::EE_RIGHT_FOOT, false);

      Eigen::Isometry3d target;
      target.linear() = Eigen::Matrix3d::Identity();
      target.translation() << 1.0, 0.0, 1.0;
      task_controller_.setTarget(DyrosRedModel::EE_LEFT_HAND, target, 5.0);
      */
    }
  }
}
void ControlBase::compute()
{
  if(control_time_<10.0)
  {
    torque_control_mode = false ;

  }



  if(control_time_>=5.0 )
  {
    torque_control_mode = true;
    torque_desired= model_.getGravityCompensation();

  }
  torque_control_mode = true;
  torque_desired= model_.getGravityCompensation();








}

void ControlBase::reflect()
{
  for (int i=0; i<DyrosRedModel::MODEL_DOF; i++)
  {
    joint_state_pub_.msg_.angle[i] = q_(i);
    joint_state_pub_.msg_.velocity[i] = q_dot_(i);
    joint_state_pub_.msg_.current[i] = torque_(i);
  }

  if(joint_state_pub_.trylock())
  {
    joint_state_pub_.unlockAndPublish();
  }
}

void ControlBase::parameterInitialize()
{
  ROS_INFO("parameter initialize");
  q_.setZero();
  q_dot_.setZero();
  q_virtual_.setZero();
  q_dot_virtual_.setZero();
  torque_.setZero();
  left_foot_ft_.setZero();
  left_foot_ft_.setZero();
  desired_q_.setZero();
  torque_desired.setZero();
  position_desired.setZero();
  compute_init=0;
}
void ControlBase::readDevice()
{
  ros::spinOnce();
}

void ControlBase::smachCallback(const smach_msgs::SmachContainerStatusConstPtr& msg)
{
  current_state_ = msg->active_states[0];
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
}
