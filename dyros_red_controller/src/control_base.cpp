
#include "dyros_red_controller/control_base.h"

namespace dyros_red_controller
{

// Constructor
ControlBase::ControlBase(ros::NodeHandle &nh, double Hz) :
  ui_update_count_(0), is_first_boot_(true), Hz_(Hz), control_mask_{}, total_dof_(DyrosRedModel::MODEL_DOF),
  wholebody_controller_(model_, q_, Hz, control_time_)
{
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

  data_pub_ = nh.advertise<sensor_msgs::JointState>("/dyros_red/data",3);
  data_pub_msg_.position.resize(6);
  data_pub_msg_.velocity.resize(12);
  data_pub_msg_.effort.resize(total_dof_);

  parameterInitialize();
  // model_.test();
}

void ControlBase::update()
{
  model_.updateKinematics(q_virtual_.head<DyrosRedModel::MODEL_DOF+6>());  // Update end effector positions and Jacobian

}

void ControlBase::compute()
{
  //ROS_DEBUG("compute enter ");
  wholebody_controller_.update_dynamics_mode(wholebody_controller_.DOUBLE_SUPPORT);
  VectorQd torque_gravity_ = wholebody_controller_.gravity_compensation_torque();

  MatrixXd J_task;
  J_task.setZero(6,total_dof_+6);
  J_task.block(0,0,3,total_dof_+6) = wholebody_controller_.J_COM;
  J_task.block(3,0,3,total_dof_+6) = model_.link_[model_.Upper_Body].Jac_COM_r;
      //model_.link_[model_.Upper_Body].Jac.block(3,0,3,total_dof_+6);



  static Vector3d com_init;
  static Matrix3d rot_init;
  Vector3d com_desired;
  Matrix3d rot_desired;

  if(control_time_ < 1){
    com_init = model_.com_;
    rot_init = model_.link_[17].Rotm;
  }


  com_desired =( model_.link_[model_.Left_Foot].xpos+ model_.link_[model_.Right_Foot].xpos)/2.0;
  com_desired(1) = 0.08;
  com_desired(2) = com_init(2);
  rot_desired = Matrix3d::Identity(3,3);

  Vector3d kp_;
  Vector3d kd_;
  Vector3d kpa_;
  Vector3d kda_;
  for(int i=0;i<3;i++){
    kp_(i) = 400;
    kd_(i) = 40;
    kpa_(i) = 1600;
    kda_(i) = 80;

  }

  std::cout<<" left leg? "<<model_.link_[model_.Left_Foot].id<<"  pos x : "<<model_.link_[model_.Left_Foot].xpos<< std::endl;
  std::cout<<" com  x : "<<model_.com_(0)<<"  y : "<<model_.com_(1)<<"  z : "<<model_.com_(2)<<"   desired com : x : " <<com_desired(0)<<"  y: "<<com_desired(1)<<"  z: "<<com_desired(2)<<std::endl;

  Vector3d com_velocity;

  com_velocity = wholebody_controller_.J_COM*q_dot_virtual_;


  Vector3d task_pos_desired;
  Vector3d task_dot_desired;

  Vector6d f_star;
  VectorQd torque_task_;
  VectorQd torque_contact_;
  torque_task_.setZero();
  torque_contact_.setZero();




  if(control_time_>=1.0){
    for(int i=0;i<3;i++){
      Vector3d quintic = DyrosMath::QuinticSpline(control_time_,1.0,5.0,com_init(i),0,0, com_desired(i),0,0);
      task_pos_desired(i) = quintic(0);
      task_dot_desired(i) = quintic(1);
    }

    f_star.segment(0,3) = wholebody_controller_.getfstar(kp_,kd_,task_pos_desired,model_.com_,task_dot_desired,com_velocity);
    Vector3d w = model_.link_[model_.Upper_Body].Jac_COM_r*q_dot_virtual_;
    f_star.segment(3,3) = wholebody_controller_.getfstar(kpa_,kda_,Matrix3d::Identity(3,3),model_.link_[model_.Upper_Body].Rotm,w,w);
    torque_task_ = wholebody_controller_.task_control_torque(J_task,f_star);



    data_pub_msg_.header.stamp = ros::Time::now();
    for(int i=0;i<total_dof_;i++)data_pub_msg_.effort[i] = torque_task_(i);
    for(int i=0;i<6;i++)data_pub_msg_.position[i] = f_star(i);


    Vector12d fc_redis;
    torque_desired = torque_gravity_ + torque_task_;
    torque_contact_ = wholebody_controller_.contact_force_redistribution_torque(torque_desired,fc_redis);

    for(int i=0;i<12;i++){

      data_pub_msg_.velocity[i] = fc_redis(i);
    }
  }


  torque_desired = torque_gravity_ + torque_task_ + torque_contact_;



  data_pub_.publish(data_pub_msg_);
  //torque_desired = torque_gravity_ + torque_task_;

}

void ControlBase::reflect()
{
  /*
  for (int i=0; i<total_dof_; i++)
  {
    joint_state_pub_.msg_.angle[i] = q_(i);
    joint_state_pub_.msg_.velocity[i] = q_dot_(i);
    joint_state_pub_.msg_.current[i] = torque_(i);
  }

  if(joint_state_pub_.trylock())
  {
    joint_state_pub_.unlockAndPublish();
  }*/
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

/*
void ControlBase::smachCallback(const smach_msgs::SmachContainerStatusConstPtr& msg)
{
  current_state_ = msg->active_states[0];
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



void ControlBase::makeIDInverseList()
{
  for(int i=0;i<DyrosRedModel::MODEL_DOF; i++)
  {
    joint_id_[i] = DyrosRedModel::JOINT_ID[i];
    joint_id_inversed_[DyrosRedModel::JOINT_ID[i]] = i;
  }
}



void ControlBase::stateChangeEvent()
{
  if(checkStateChanged())
  {
    if(current_state_ == "move1")
    {
      task_controller_.setEnable(DyrosRedModel::EE_LEFT_HAND, true);
      task_controller_.setEnable(DyrosRedModel::EE_RIGHT_HAND, false);
      task_controller_.setEnable(DyrosRedModel::EE_LEFT_FOOT, false);
      task_controller_.setEnable(DyrosRedModel::EE_RIGHT_FOOT, false);

      Eigen::Isometry3d target;
      target.linear() = Eigen::Matrix3d::Identity();
      target.translation() << 1.0, 0.0, 1.0;
      task_controller_.setTarget(DyrosRedModel::EE_LEFT_HAND, target, 5.0);

    }
  }
}


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
