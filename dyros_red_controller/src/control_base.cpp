
#include "dyros_red_controller/control_base.h"

namespace dyros_red_controller
{

// Constructor
ControlBase::ControlBase(ros::NodeHandle &nh, double Hz) :
  ui_update_count_(0), is_first_boot_(true), Hz_(Hz), control_mask_{}, total_dof_(DyrosRedModel::MODEL_DOF),
  wholebody_controller_(model_, q_, Hz, control_time_)
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
  q_virtual_quaternion.setZero(total_dof_+7);

  data_pub_ = nh.advertise<sensor_msgs::JointState>("/dyros_red/data",3);
  com_pos_pub = nh.advertise<geometry_msgs::PolygonStamped>("/dyros_red/point",3);
  command_sub = nh.subscribe("/dyros_red/command",100,&ControlBase::command_cb,this);
  com_command_sub = nh.subscribe("/dyros_red/com_command",10,&ControlBase::com_command_cb,this);


  data_pub_msg_.position.resize(6);
  data_pub_msg_.velocity.resize(12);
  data_pub_msg_.effort.resize(total_dof_);

  joint_state_publisher_for_rviz = nh.advertise<sensor_msgs::JointState>("/joint_states",1);
  joint_states_rviz.name.resize(total_dof_);
  joint_states_rviz.position.resize(total_dof_);


  for(int i=0;i<total_dof_;i++){
    joint_states_rviz.name[i]=model_.JOINT_NAME_2[i];
  }
  com_pos_pub_msgs.polygon.points.resize(3);

  parameterInitialize();
  // model_.test();


  ROS_INFO_ONCE("BASE : control base initialize end");

}
void ControlBase::com_command_cb(const dyros_red_msgs::ComCommandConstPtr &msg){

  com_command_time = control_time_;
  com_command_position = msg->ratio;
  com_command_traj_time = msg->time;

  ROS_INFO("COM TRAJ MSG received at, %f \t to %f in %f seconds.",com_command_time,com_command_position,com_command_traj_time);

  com_init = model_.com_;
  rot_init = model_.link_[model_.Upper_Body].Rotm;

  task_switch = true;

}



void ControlBase::command_cb(const std_msgs::StringConstPtr &msg){
  if(msg->data=="gravity"){
    gravity_switch = !gravity_switch;
    ROS_INFO_COND(gravity_switch,"GRAVITY SWITCH : ON");
    ROS_INFO_COND(!gravity_switch,"GRAVITY SWITCH : OFF");
  }
  if(msg->data=="task"){
    task_switch = !task_switch;
    ROS_INFO_COND(task_switch,"TASK SWITCH : ON");
    ROS_INFO_COND(!task_switch,"TASK SWITCH : OFF");
  }

  if(msg->data=="contact"){
    contact_switch = !contact_switch;
    ROS_INFO_COND(contact_switch,"CONTACT SWITCH : ON");
    ROS_INFO_COND(!contact_switch,"CONTACT SWITCH : OFF");
  }

  if(msg->data=="data"){
    data_switch = true;
    ROS_INFO("DATA SWITCH");





    std::cout <<"//////////////////////////////////"<<std::endl;
    std::cout <<" q_ virtual is : "<<std::endl;
    std::cout <<q_virtual_<<std::endl;
    std::cout <<" xpos"<<std::endl;
    std::cout <<model_.link_[0].xpos<<std::endl;
    std::cout <<" jac of "<<model_.link_[0].name<<std::endl;
    std::cout <<model_.link_[0].Jac << std::endl;
    std::cout <<" jac of "<<model_.link_[1].name<<std::endl;
    std::cout <<model_.link_[1].Jac << std::endl;
    std::cout <<" jac of "<<model_.link_[model_.Right_Hand].name<<std::endl;
    std::cout <<model_.link_[model_.Right_Hand].Jac << std::endl;

    std::cout <<" jac of "<<model_.link_[model_.Right_Foot].name<<std::endl;
    std::cout <<model_.link_[model_.Right_Foot].Jac << std::endl;

    std::cout <<" Eri "<< std::endl;
    std::cout <<model_.Eri << std::endl;
    std::cout <<"waist skm " <<std::endl;
    std::cout << DyrosMath::skm(model_.link_[1].xpos-model_.link_[0].xpos);
    std::cout <<"rhand skm " <<std::endl;
    std::cout << DyrosMath::skm(model_.link_[model_.Right_Hand].xpos-model_.link_[0].xpos);

    std::cout <<std::endl;
    std::cout << "A matrix" <<std::endl;
    std::cout << model_.A_<<std::endl;
    std::cout << "G matrix" <<std::endl<<wholebody_controller_.G<<std::endl;

    std::cout << model_.link_[0].Rotm <<std::endl;
    std::cout <<std::endl;

  }
}

void ControlBase::update()
{
  model_.updateKinematics(q_virtual_quaternion);  // Update end effector positions and Jacobian

}

void ControlBase::compute()
{
  int ci=0;
  ROS_INFO_ONCE("BASE : compute %d", ci++);
  wholebody_controller_.update_dynamics_mode(wholebody_controller_.DOUBLE_SUPPORT);

  ROS_INFO_ONCE("BASE : compute %d", ci++);
  VectorQd torque_gravity_ = wholebody_controller_.gravity_compensation_torque();

  ROS_INFO_ONCE("BASE : compute %d", ci++);

  MatrixXd J_task;
  J_task.setZero(6,total_dof_+6);
  J_task.block(0,0,3,total_dof_+6) = wholebody_controller_.J_COM;
  J_task.block(3,0,3,total_dof_+6) = model_.link_[model_.Upper_Body].Jac.block(3,0,3,total_dof_+6);
      //model_.link_[model_.Upper_Body].Jac.block(3,0,3,total_dof_+6);


  ROS_INFO_ONCE("BASE : compute %d", ci++);


  Vector3d kp_;
  Vector3d kd_;
  Vector3d kpa_;
  Vector3d kda_;
  for(int i=0;i<3;i++){
    kp_(i) = 400;
    kd_(i) = 40;
    kpa_(i) = 400;
    kda_(i) = 40;

  }

  //std::cout<<" left leg? "<<model_.link_[model_.Left_Foot].id<<"  pos x : "<<model_.link_[model_.Left_Foot].xpos<< std::endl;
  //std::cout<<" com  x : "<<model_.com_(0)<<"  y : "<<model_.com_(1)<<"  z : "<<model_.com_(2)<<"   desired com : x : " <<com_desired(0)<<"  y: "<<com_desired(1)<<"  z: "<<com_desired(2)<<std::endl;


  ROS_INFO_ONCE("BASE : compute %d", ci++);
  Vector3d com_velocity;

  com_velocity = wholebody_controller_.J_COM*q_dot_virtual_;


  Vector3d task_pos_desired;
  Vector3d task_dot_desired;


  ROS_INFO_ONCE("BASE : compute %d", ci++);
  f_star.setZero();
  torque_task_.setZero();
  torque_contact_.setZero();



  ROS_INFO_ONCE("BASE : compute %d", ci++);


  if(task_switch){
    com_desired =( com_command_position*model_.link_[model_.Left_Foot].xpos+ (1.0-com_command_position)*model_.link_[model_.Right_Foot].xpos);
    com_desired(2) = com_init(2);
    rot_desired = Matrix3d::Identity(3,3);


    for(int i=0;i<3;i++){
      Vector3d quintic = DyrosMath::QuinticSpline(control_time_,com_command_time,com_command_time + com_command_traj_time ,com_init(i),0,0, com_desired(i),0,0);
      task_pos_desired(i) = quintic(0);
      task_dot_desired(i) = quintic(1);
    }


    f_star.segment(0,3) = wholebody_controller_.getfstar(kp_,kd_,task_pos_desired,model_.com_,task_dot_desired,com_velocity);
    Vector3d w = model_.link_[model_.Upper_Body].Jac.block(3,0,3,total_dof_+6)*q_dot_virtual_;
    f_star.segment(3,3) = wholebody_controller_.getfstar(kpa_,kda_,rot_init,model_.link_[model_.Upper_Body].Rotm,Eigen::Vector3d::Zero(),w);


  }
  else{
    f_star.setZero();
  }


  ROS_INFO_ONCE("BASE : compute %d", ci++);
    torque_task_ = wholebody_controller_.task_control_torque(J_task,f_star);
    torque_desired = torque_gravity_ + torque_task_;
    torque_contact_ = wholebody_controller_.contact_force_redistribution_torque(q_virtual_(5),torque_desired,fc_redis);


    ROS_INFO_ONCE("BASE : compute %d", ci++);
  if(!contact_switch)torque_contact_.setZero();
  if(!task_switch)torque_task_.setZero();

  torque_desired = torque_gravity_ + torque_task_ + torque_contact_;

  if(!gravity_switch)torque_desired = torque_gravity_;

  //torque_desired = torque_gravity_ + torque_task_;

}

void ControlBase::reflect()
{
  data_pub_msg_.header.stamp = ros::Time::now();
  for(int i=0;i<total_dof_;i++)data_pub_msg_.effort[i] = torque_task_(i);
  for(int i=0;i<6;i++)data_pub_msg_.position[i] = f_star(i);
  for(int i=0;i<12;i++){
    data_pub_msg_.velocity[i] = fc_redis(i);
  }



  joint_states_rviz.header.stamp = ros::Time::now();
  for(int i=0;i<total_dof_;i++){

    joint_states_rviz.position[i]=q_(i);
  }
  joint_state_publisher_for_rviz.publish(joint_states_rviz);

  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(q_virtual_(0),q_virtual_(1),q_virtual_(2)));
  tf::Quaternion q;
  q.setRPY(q_virtual_(3),q_virtual_(4),q_virtual_(5));
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),"base_link","Pelvis_Link"));

  com_pos_pub_msgs.polygon.points[0].x=model_.com_(0);
  com_pos_pub_msgs.polygon.points[0].y=model_.com_(1);
  com_pos_pub_msgs.polygon.points[0].z=model_.com_(2);

  com_pos_pub_msgs.polygon.points[1].x=model_.link_[model_.Right_Foot].xpos(0);
  com_pos_pub_msgs.polygon.points[1].y=model_.link_[model_.Right_Foot].xpos(1);
  com_pos_pub_msgs.polygon.points[1].z=model_.link_[model_.Right_Foot].xpos(2);

  com_pos_pub_msgs.polygon.points[2].x=model_.link_[model_.Left_Foot].xpos(0);
  com_pos_pub_msgs.polygon.points[2].y=model_.link_[model_.Left_Foot].xpos(1);
  com_pos_pub_msgs.polygon.points[2].z=model_.link_[model_.Left_Foot].xpos(2);

  com_pos_pub_msgs.header.stamp=ros::Time::now();


  data_pub_.publish(data_pub_msg_);
  com_pos_pub.publish(com_pos_pub_msgs);
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
  ROS_INFO_ONCE("BASE : parameter initialize");
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

  com_init.setZero();
  rot_init.setZero();
  com_desired.setZero();
  rot_desired.setZero();



  compute_init=0;

  gravity_switch = true;
  task_switch = false;
  contact_switch = true;



  com_command_time = 0.0;
  com_command_position = 0.0;
  com_command_traj_time = 0.0;

  ROS_INFO_ONCE("BASE : parameter initialize end");
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
