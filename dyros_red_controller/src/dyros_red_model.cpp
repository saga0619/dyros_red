#include "dyros_red_controller/dyros_red_model.h"



namespace dyros_red_controller
{

// https://stackoverflow.com/questions/8016780/undefined-reference-to-static-constexpr-char
constexpr const char* DyrosRedModel::EE_NAME[4];

constexpr const char* DyrosRedModel::LINK_NAME[32];

constexpr const size_t DyrosRedModel::MODEL_DOF;

/*
// These should be replaced by YAML or URDF or something
const std::string DyrosRedModel::JOINT_NAME[DyrosRedModel::MODEL_DOF] = {
      "L_HipRoll_Joint", "L_HipCenter_Joint", "L_Thigh_Joint", "L_Knee_Joint", "L_AnkleCenter_Joint", "L_AnkleRoll_Joint",
      "R_HipRoll_Joint", "R_HipCenter_Joint", "R_Thigh_Joint", "R_Knee_Joint", "R_AnkleCenter_Joint", "R_AnkleRoll_Joint",
      "Waist1_Joint","Waist2_Joint","Upperbody_Joint",
      "L_Shoulder1_Joint","L_Shoulder2_Joint","L_Shoulder3_Joint","L_Armlink_Joint","L_Elbow_Joint","L_Forearm_Joint","L_Wrist1_Joint","L_Wrist2_Joint",
      "R_Shoulder1_Joint","R_Shoulder2_Joint","R_Shoulder3_Joint","R_Armlink_Joint","R_Elbow_Joint","R_Forearm_Joint","R_Wrist1_Joint","R_Wrist2_Joint"};

*/

const std::string DyrosRedModel::JOINT_NAME[DyrosRedModel::MODEL_DOF] = {
      "L_HipRoll_Motor", "L_HipCenter_Motor", "L_Thigh_Motor", "L_Knee_Motor", "L_AnkleCenter_Motor", "L_AnkleRoll_Motor",
      "R_HipRoll_Motor", "R_HipCenter_Motor", "R_Thigh_Motor", "R_Knee_Motor", "R_AnkleCenter_Motor", "R_AnkleRoll_Motor",
      "Waist1_Motor","Waist2_Motor","Upperbody_Motor",
      "L_Shoulder1_Motor","L_Shoulder2_Motor","L_Shoulder3_Motor","L_Armlink_Motor","L_Elbow_Motor","L_Forearm_Motor","L_Wrist1_Motor","L_Wrist2_Motor",
      "R_Shoulder1_Motor","R_Shoulder2_Motor","R_Shoulder3_Motor","R_Armlink_Motor","R_Elbow_Motor","R_Forearm_Motor","R_Wrist1_Motor","R_Wrist2_Motor"};





// 0~6 Left leg
// 7~11 Right leg
// 12~14 Waist
// 15~22 Left arm
// 23~30 Right arm

/*
//DyrosRedModel::part::part(DyrosRedModel& RM):RM_(RM){}
DyrosRedModel::part::part(const DyrosRedModel &RM) : RM_(RM){}



void DyrosRedModel::part::initialize(int id_, double mass_, Eigen::Matrix3d inertia_, Eigen::Vector3d COMpos_){
  id= id_;
  Mass=mass_;
  inertia = inertia_;
  xipos = COMpos_;

  Jac.resize(6,MODEL_DOF+6); Jac.setZero();
  Jac_COM.resize(6,MODEL_DOF+6);  Jac_COM.setZero();
  Jac_COM_p.resize(3,MODEL_DOF+6);  Jac_COM_p.setZero();
  Jac_COM_r.resize(3,MODEL_DOF+6);  Jac_COM_r.setZero();
  Jac_Contact.resize(6,MODEL_DOF+6);  Jac_Contact.setZero();
  Jac_point.resize(6,MODEL_DOF+6);  Jac_point.setZero();
  Rotm.resize(3,3);  Rotm.setZero();
  inertia.setZero();  xpos.setZero();  xipos.setZero();


}

void DyrosRedModel::part::Update(Eigen::Vector3d xpos_, Eigen::MatrixXd fj1_, Eigen::MatrixXd fj2_, Eigen::Matrix3d pel_rot){


  xpos = xpos_;

  Jac = RBDLJac2GlobalJac(fj1_,pel_rot);
  Jac_COM = RBDLJac2GlobalJac(fj2_,pel_rot);
  Jac_COM_p = Jac_COM.block<3,MODEL_DOF+6>(0,0);
  Jac_COM_r = Jac_COM.block<3,MODEL_DOF+6>(3,0);


}
*/

void DyrosRedModel::Link_initialize(int i, int id, double mass, Eigen::Vector3d xipos){
  link_[i].id = id;
  link_[i].Mass = mass;
  link_[i].COM_position = xipos;

  link_[i].Rotm.setZero();
  link_[i].inertia.setZero();


  link_[i].contact_point.setZero();

  link_[i].Jac.resize(6,MODEL_DOF+6); link_[i].Jac.setZero();
  link_[i].Jac_COM.resize(6,MODEL_DOF+6); link_[i].Jac_COM.setZero();
  link_[i].Jac_COM_p.resize(3,MODEL_DOF+6); link_[i].Jac_COM_p.setZero();
  link_[i].Jac_COM_r.resize(3,MODEL_DOF+6); link_[i].Jac_COM_r.setZero();
  link_[i].Jac_Contact.resize(6,MODEL_DOF+6); link_[i].Jac_Contact.setZero();
  link_[i].Jac_point.resize(6,MODEL_DOF+6); link_[i].Jac_point.setZero();



}

void DyrosRedModel::Link_pos_Update(int i){


  link_[i].xpos =RigidBodyDynamics::CalcBodyToBaseCoordinates(model_,q_virtual_,link_[i].id,Eigen::Vector3d::Zero(),false);
  link_[i].xipos = RigidBodyDynamics::CalcBodyToBaseCoordinates(model_,q_virtual_,link_[i].id,link_[i].COM_position,false);
  //link_[i].COM_position = RigidBodyDynamics::CalcBaseToBodyCoordinates(model_,q_virtual_,link_[i])



}

void DyrosRedModel::Link_Jac_Update(int i){

  Eigen::MatrixXd j_p_(3,MODEL_DOF+6), j_r_(3,MODEL_DOF+6);
  Eigen::MatrixXd j_(6,MODEL_DOF+6);
  Eigen::MatrixXd fj_(6,MODEL_DOF+6);
  fj_.setZero();




  //RigidBodyDynamics::Calc


  ros::Time t_temp = ros::Time::now();
  RigidBodyDynamics::CalcPointJacobian6D(model_, q_virtual_, link_[i].id, link_[i].COM_position, fj_, false);

  double j_calc_time = ros::Time::now().toSec() - t_temp.toSec();



  j_p_=fj_.block<3,MODEL_DOF+6>(3,0);
  j_r_=fj_.block<3,MODEL_DOF+6>(0,0);


  //Eigen::MatrixXd jcp, jcr;




  //t_temp = ros::Time::now();

  //jcp = j_p_;


  //jcp.block<3,3>(0,3)=j_p_.block<3,3>(0,3)*Eri;

  //jcr = j_r_;

  //jcr.block<3,3>(0,3)=j_r_.block<3,3>(0,3)*Eri;


 // double ar_time0 = ros::Time::now().toSec() - t_temp.toSec();


  t_temp = ros::Time::now();


  link_[i].Jac_COM_p = j_p_;
  link_[i].Jac_COM_r = j_r_;

  j_.block<3,MODEL_DOF+6>(0,0) = link_[i].Jac_COM_p;
  j_.block<3,MODEL_DOF+6>(3,0) = link_[i].Jac_COM_r;






  link_[i].Jac_COM = j_;

  double ar_time = ros::Time::now().toSec() - t_temp.toSec();
  //ROS_INFO(" ::: %d Link calc time : %8.4f,%8.4f", i, j_calc_time*1000, ar_time*1000);


}

void DyrosRedModel::Link_Set_Jacobian(int i, Eigen::Vector3d Jacobian_position){
  Eigen::MatrixXd fj_(6,MODEL_DOF+6);
  fj_.setZero();

  RigidBodyDynamics::CalcPointJacobian6D(model_,q_virtual_,link_[i].id,Jacobian_position,fj_,false);

  //link_[i].Jac.block<3,MODEL_DOF+6>(0,0)=fj_.block<3,MODEL_DOF+6>(3,0)*E_T_;
  //link_[i].Jac.block<3,MODEL_DOF+6>(3,0)=fj_.block<3,MODEL_DOF+6>(0,0)*E_T_;
  link_[i].Jac.block<3,MODEL_DOF+6>(0,0)=fj_.block<3,MODEL_DOF+6>(3,0);
  link_[i].Jac.block<3,MODEL_DOF+6>(3,0)=fj_.block<3,MODEL_DOF+6>(0,0);



}


void DyrosRedModel::Link_Set_Contact(int i, Eigen::Vector3d Contact_position){


  Eigen::MatrixXd fj_(6,MODEL_DOF+6);
  fj_.setZero();

  RigidBodyDynamics::CalcPointJacobian6D(model_,q_virtual_,link_[i].id,Contact_position,fj_,false);


  //link_[i].Jac_Contact.block<3,MODEL_DOF+6>(0,0)=fj_.block<3,MODEL_DOF+6>(3,0)*E_T_;
  //link_[i].Jac_Contact.block<3,MODEL_DOF+6>(3,0)=fj_.block<3,MODEL_DOF+6>(0,0)*E_T_;
  link_[i].Jac_Contact.block<3,MODEL_DOF+6>(0,0)=fj_.block<3,MODEL_DOF+6>(3,0);
  link_[i].Jac_Contact.block<3,MODEL_DOF+6>(3,0)=fj_.block<3,MODEL_DOF+6>(0,0);


}



DyrosRedModel::DyrosRedModel()
{
  A_.resize(MODEL_DOF+6,MODEL_DOF+6);
  A_.setZero();
  q_.resize(MODEL_DOF);
  q_.setZero();

  q_virtual_.resize(MODEL_DOF+6);
  q_virtual_.setZero();
  A_temp_.resize(MODEL_DOF+6, MODEL_DOF+6);
  A_temp_.setZero();

  R_temp_.resize(MODEL_DOF+6, MODEL_DOF+6);
  R_temp_.setZero();

  Gravity_.setZero();

  Gravity_(2) = GRAVITY;

  test_run=0;




  E_T_.resize(MODEL_DOF+6,MODEL_DOF+6);
  E_T_.setZero();


  std::string desc_package_path = ros::package::getPath("dyros_red_description");
  std::string urdf_path = desc_package_path + "/robots/dyros_red_robot.urdf";

  ROS_INFO("Loading DYROS JET description from = %s",urdf_path.c_str());
  RigidBodyDynamics::Addons::URDFReadFromFile(urdf_path.c_str(), &model_, true, true);

  ROS_INFO("Successfully loaded.");


  ROS_INFO("MODEL DOF COUNT = %d", model_.dof_count);
  ROS_INFO("MODEL Q SIZE = %d", model_.q_size);
  //model_.mJoints[0].)
  if(model_.dof_count != MODEL_DOF+6)
  {
    ROS_WARN("The DoF in the model file and the code do not match.");
    ROS_WARN("Model file = %d, Code = %d", model_.dof_count, (int)MODEL_DOF+6);
  }
  else{
    //ROS_INFO("id:0 name is : %s",model_.GetBodyName(0));
    for (int i=0; i<MODEL_DOF+1; i++)
    {
      link_id_[i] = model_.GetBodyId(LINK_NAME[i]);
      //ROS_INFO("%s: \t\t id = %d \t parent link = %d",LINK_NAME[i], link_id_[i],model_.GetParentBodyId(link_id_[i]));

      //ROS_INFO("%dth parent %d",link_id_[i],model_.GetParentBodyId(link_id_[i]));
      //std::cout << model_.mBodies[link_id_[i]].mCenterOfMass << std::endl;
      //joint_name_map_[JOINT_NAME[i]] = i;
    }
    for(int i=0; i<MODEL_DOF+1;i++)
    {
      Link_initialize(i,link_id_[i],model_.mBodies[link_id_[i]].mMass,model_.mBodies[link_id_[i]].mCenterOfMass);
    }
    total_mass = 0;
    for(int i=0;i<MODEL_DOF+1;i++)
    {
      total_mass = total_mass + link_[i].Mass;

    }




    //RigidBodyDynamics::Joint J_temp;
    //J_temp=RigidBodyDynamics::Joint(RigidBodyDynamics::JointTypeEulerXYZ);


    //model_.mJoints[2] = J_temp;


    for(int i=0;i<MODEL_DOF+6;i++){

      //ROS_INFO("Joint type %d : %d", i, model_.mJoints[i].mJointType);

    }

  }


}


void DyrosRedModel::test()
{
  //updateKinematics(Eigen::Vector28d::Zero());
  //std::cout << "test" << std::endl;


  if(test_run == 0 ){

    for(int i=0;i<MODEL_DOF;i++){


    //std::cout <<model_.GetBodyName(link_[i].id) << "position is :::::  "<<std::endl<< RigidBodyDynamics::CalcBodyToBaseCoordinates(model_,q_,link_[i].id,Eigen::Vector3d::Zero(),false) << std::endl;

    }

    //std::cout << "link0 Jac ::" << std::endl;

    //std::cout << link_[0].Jac<<std::endl;

    //std::cout << " link 0 COM POS :: " <<std::endl << link_[0].COM_position << std::endl;

    //std::cout << "link0 Jac COM ::" <<std::endl;

    //std::cout << link_[0].Jac_COM<<std::endl;



    test_run++;
  }


  //std::cout << "Left_Arm Wrist2 Body to Base position ::::: "<<std::endl<< RigidBodyDynamics::CalcBodyToBaseCoordinates(model_,q_,link_[Left_Arm+7].id,Eigen::Vector3d::Zero(),false)<<std::endl;
  //std::cout << "Left_Arm Wrist2 Base to Body position ::::: "<<std::endl<< RigidBodyDynamics::CalcBaseToBodyCoordinates(model_,q_,link_[Left_Arm+7].id,Eigen::Vector3d::Zero(),false)<<std::endl;


}

void DyrosRedModel::updateKinematics(const Eigen::VectorXd& q_virtual)
{
  //orientation representation of virtual joint is derivatives of euler angle XYZ

  q_virtual_ = q_virtual;

  Eigen::Matrix3d Er1,Er2,Er3,Er;
  Er.setZero();
  Er1 = Eigen::AngleAxisd(q_virtual_(3), Eigen::Vector3d::UnitX());
  Er2 = Eigen::AngleAxisd(q_virtual_(3), Eigen::Vector3d::UnitX())*Eigen::AngleAxisd(q_virtual_(4), Eigen::Vector3d::UnitY());
  Er3 = Eigen::AngleAxisd(q_virtual_(3), Eigen::Vector3d::UnitX())*Eigen::AngleAxisd(q_virtual_(4), Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(q_virtual_(5), Eigen::Vector3d::UnitZ());

  Er.block<3,1>(0,0) = Er1.block<3,1>(0,0);
  Er.block<3,1>(0,1) = Er2.block<3,1>(0,1);
  Er.block<3,1>(0,2) = Er3.block<3,1>(0,2);

  Eri=Er.inverse();
  E_T_ = Eigen::MatrixXd::Identity(MODEL_DOF+6,MODEL_DOF+6);
  Eri = Eigen::MatrixXd::Identity(3,3);
  //E_T_.block<3,3>(3,3)=Eri;





  ros::Time t_temp = ros::Time::now();
  RigidBodyDynamics::UpdateKinematicsCustom(model_, &q_virtual, NULL, NULL);
  double uk_time = ros::Time::now().toSec() - t_temp.toSec();

  t_temp = ros::Time::now();
  RigidBodyDynamics::CompositeRigidBodyAlgorithm(model_, q_virtual_, A_temp_, true);
  double cr_time = ros::Time::now().toSec() - t_temp.toSec();



  //A_=E_T_.transpose()*A_temp_*E_T_;

  A_ = A_temp_;

  com_ = getCenterOfMassPosition();

  //R_temp_.block<3,3>(0,0) = -base_rotation_;
  //R_temp_.block<3,3>(3,3) = -base_rotation_;
  //R_temp_.block<MODEL_DOF,MODEL_DOF>(6,6) = Eigen::MatrixXd::Identity(MODEL_DOF,MODEL_DOF);
  //A_= R_temp_.transpose()*A_temp_*R_temp_;

  for(int i=0;i<MODEL_DOF+1;i++){
    Link_pos_Update(i);
  }



  t_temp = ros::Time::now();
  for(int i=0;i<MODEL_DOF+1;i++){
    Link_Jac_Update(i);
  }
  double ju_time = ros::Time::now().toSec() - t_temp.toSec();


  /*
  std::cout<<"LINK xpos :::::::::: x : "<<std::endl<<link_[Left_Arm+7].xpos(0)<<" y : "<<link_[Left_Arm+7].xpos(1) <<" z : "<<link_[Left_Arm+7].xpos(2)<<std::endl<<std::endl;
  std::cout<<"euler:::::::::: x : "<<std::endl<<q_virtual_(3)<<" y : "<<q_virtual_(4) <<" z : "<<q_virtual_(5)<<std::endl<<std::endl;
  std::cout<<"Er : " <<std::endl<<Er<<std::endl<<std::endl;
  std::cout<<"Eri : " <<std::endl<<Eri<<std::endl<<std::endl;
  std::cout<<"Jacobian ::: " <<std::endl <<link_[Left_Arm+7].Jac<<std::endl;
  std::cout<<"Jacobian*E_T_ ::: " <<std::endl <<link_[Left_Arm+7].Jac_COM<<std::endl;
  */


  ROS_DEBUG("Update time - detail \n updatekinematicsCustum Time : % 3.4f ms\n compositeRigid time : %3.4f ms\n jac_update time : %3.4f ms",uk_time*1000,cr_time*1000,ju_time*1000);


}

void DyrosRedModel::setquat(Eigen::Quaterniond& quat,const Eigen::VectorXd& q)
{
  Eigen::VectorXd q_virtual;
  q_virtual.resize(MODEL_DOF+6);
  q_virtual = q;
  //std::cout<<"q is : "<<std::endl;
  //std::cout<<q<<std::endl;
  //ROS_INFO_ONCE("t1");
  //std::cout<<"quaterniond : " <<quat.x() <<"  "<< quat.y() <<"  "<<quat.z() <<"  "<< quat.w()<<std::endl;
  RigidBodyDynamics::Math::Quaternion quat_temp(quat.x(),quat.y(),quat.z(),quat.w());
  //std::cout<<"quaternionv : " <<quat_temp<<std::endl;
  //ROS_INFO_ONCE("t2");
  model_.SetQuaternion(link_[0].id, quat_temp, q_virtual);
  //ROS_INFO_ONCE("t4");
  //std::cout<<"after q is : "<<std::endl;
  //std::cout<<q<<std::endl;


}

Eigen::Vector3d DyrosRedModel::getCenterOfMassPosition()
{
  RigidBodyDynamics::Math::Vector3d position_temp;
  position_temp.setZero();

  for(int i=0;i<MODEL_DOF+1;i++){
   position_temp = position_temp +link_[i].Mass*link_[i].xipos;
  }
   position_temp = position_temp / total_mass;


  return position_temp;

}




/*

void DyrosRedModel::getTransformEndEffector // must call updateKinematics before calling this function
(EndEffector ee, Eigen::Isometry3d* transform_matrix)
{
  Eigen::Vector3d gghg = RigidBodyDynamics::CalcBodyToBaseCoordinates(model_, q_,end_effector_id_[ee], base_position_, false);
  transform_matrix->translation() = RigidBodyDynamics::CalcBodyToBaseCoordinates
      (model_, q_,end_effector_id_[ee], base_position_, false);
  transform_matrix->linear() = RigidBodyDynamics::CalcBodyWorldOrientation(
        model_, q_, end_effector_id_[ee], false).transpose();
}

void DyrosRedModel::getTransformEndEffector // must call updateKinematics before calling this function
(EndEffector ee, Eigen::Vector3d* position, Eigen::Matrix3d* rotation)
{
  *position = RigidBodyDynamics::CalcBodyToBaseCoordinates
      (model_, q_,end_effector_id_[ee], base_position_, false);
  *rotation = RigidBodyDynamics::CalcBodyWorldOrientation(
        model_, q_, end_effector_id_[ee], false).transpose();
}

void DyrosRedModel::getTransformEndEffector
(EndEffector ee, const Eigen::VectorXd& q, bool update_kinematics,
 Eigen::Vector3d* position, Eigen::Matrix3d* rotation)
{
  Eigen::Vector28d q_new;
  q_new = q_;
  switch (ee)
  {
  case EE_LEFT_FOOT:
  case EE_RIGHT_FOOT:
    q_new.segment<6>(joint_start_index_[ee]) = q;
    break;
  case EE_LEFT_HAND:
  case EE_RIGHT_HAND:
    q_new.segment<8>(joint_start_index_[ee]) = q;
    break;
  }
  if (update_kinematics)
  {
    q_ = q_new;
  }
  *position = RigidBodyDynamics::CalcBodyToBaseCoordinates
      (model_,q_new,end_effector_id_[ee], base_position_, update_kinematics);
  *rotation = RigidBodyDynamics::CalcBodyWorldOrientation(
        model_, q_new, end_effector_id_[ee], update_kinematics).transpose();
  // RigidBodyDynamics::Calcpo
  // model_.mBodies[0].mCenterOfMass
}


void DyrosRedModel::getJacobianMatrix6DoF
(EndEffector ee, Eigen::Matrix<double, 6, 6> *jacobian)
{
  Eigen::MatrixXd full_jacobian(6,MODEL_DOF);
  full_jacobian.setZero();
  RigidBodyDynamics::CalcPointJacobian6D(model_, q_, end_effector_id_[ee], Eigen::Vector3d::Zero(), full_jacobian, false);

  switch (ee)
  {
  case EE_LEFT_FOOT:
  case EE_RIGHT_FOOT:
    // swap
    jacobian->block<3, 6>(0, 0) = full_jacobian.block<3, 6>(3, joint_start_index_[ee]);
    jacobian->block<3, 6>(3, 0) = full_jacobian.block<3, 6>(0, joint_start_index_[ee]);
    break;
  case EE_LEFT_HAND:
  case EE_RIGHT_HAND:
  //*jacobian = full_jacobian.block<6, 7>(0, joint_start_index_[ee]);
    ROS_ERROR("Arm is 7 DoF. Please call getJacobianMatrix7DoF");
    break;
  }
}

void DyrosRedModel::getJacobianMatrix8DoF
(EndEffector ee, Eigen::Matrix<double, 6, 8> *jacobian)
{
  Eigen::MatrixXd full_jacobian(6,MODEL_DOF);
  full_jacobian.setZero();
  RigidBodyDynamics::CalcPointJacobian6D(model_, q_, end_effector_id_[ee],
                                         Eigen::Vector3d::Zero(), full_jacobian, false);

  switch (ee)
  {
  case EE_LEFT_FOOT:
  case EE_RIGHT_FOOT:
  // swap
  ROS_ERROR("Leg is 6 DoF. Please call getJacobianMatrix7DoF");
  break;
  case EE_LEFT_HAND:
  case EE_RIGHT_HAND:
  //*jacobian = full_jacobian.block<6, 7>(0, joint_start_index_[ee]);
  jacobian->block<3, 8>(0, 0) = full_jacobian.block<3, 8>(3, joint_start_index_[ee]);
  jacobian->block<3, 8>(3, 0) = full_jacobian.block<3, 8>(0, joint_start_index_[ee]);
  break;
  }
}

*/



}
