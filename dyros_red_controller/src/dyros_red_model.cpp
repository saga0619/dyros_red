#include "dyros_red_controller/dyros_red_model.h"

namespace dyros_red_controller
{
// https://stackoverflow.com/questions/8016780/undefined-reference-to-static-constexpr-char
constexpr const char *DyrosRedModel::EE_NAME[4];
constexpr const char *DyrosRedModel::LINK_NAME[32];
constexpr const size_t DyrosRedModel::MODEL_DOF;

// These should be replaced by YAML or URDF or something
const std::string DyrosRedModel::JOINT_NAME[DyrosRedModel::MODEL_DOF] = {
    "L_HipRoll_Joint", "L_HipCenter_Joint", "L_Thigh_Joint",
    "L_Knee_Joint", "L_AnkleCenter_Joint", "L_AnkleRoll_Joint",
    "R_HipRoll_Joint", "R_HipCenter_Joint", "R_Thigh_Joint",
    "R_Knee_Joint", "R_AnkleCenter_Joint", "R_AnkleRoll_Joint",
    "Waist1_Joint", "Waist2_Joint", "Upperbody_Joint",
    "L_Shoulder1_Joint", "L_Shoulder2_Joint", "L_Shoulder3_Joint",
    "L_Armlink_Joint", "L_Elbow_Joint", "L_Forearm_Joint",
    "L_Wrist1_Joint", "L_Wrist2_Joint", "R_Shoulder1_Joint",
    "R_Shoulder2_Joint", "R_Shoulder3_Joint", "R_Armlink_Joint",
    "R_Elbow_Joint", "R_Forearm_Joint", "R_Wrist1_Joint",
    "R_Wrist2_Joint"};

const std::string DyrosRedModel::ACTUATOR_NAME[DyrosRedModel::MODEL_DOF] = {
    "L_HipRoll_Motor", "L_HipCenter_Motor", "L_Thigh_Motor",
    "L_Knee_Motor", "L_AnkleCenter_Motor", "L_AnkleRoll_Motor",
    "R_HipRoll_Motor", "R_HipCenter_Motor", "R_Thigh_Motor",
    "R_Knee_Motor", "R_AnkleCenter_Motor", "R_AnkleRoll_Motor",
    "Waist1_Motor", "Waist2_Motor", "Upperbody_Motor",
    "L_Shoulder1_Motor", "L_Shoulder2_Motor", "L_Shoulder3_Motor",
    "L_Armlink_Motor", "L_Elbow_Motor", "L_Forearm_Motor",
    "L_Wrist1_Motor", "L_Wrist2_Motor", "R_Shoulder1_Motor",
    "R_Shoulder2_Motor", "R_Shoulder3_Motor", "R_Armlink_Motor",
    "R_Elbow_Motor", "R_Forearm_Motor", "R_Wrist1_Motor",
    "R_Wrist2_Motor"};

// 0~6 Left leg
// 7~11 Right leg
// 12~14 Waist
// 15~22 Left arm
// 23~30 Right arm

void DyrosRedModel::Link_initialize(int i, int id, std::string name, double mass, Eigen::Vector3d local_com_position)
{
  link_[i].id = id;
  link_[i].Mass = mass;
  link_[i].COM_position = local_com_position;
  link_[i].name = name;
  link_[i].Rotm.setZero();
  link_[i].inertia.setZero();
  link_[i].contact_point.setZero();

  link_[i].Jac.setZero(6, MODEL_DOF + 6);
  link_[i].Jac_COM.setZero(6, MODEL_DOF + 6);
  link_[i].Jac_COM_p.setZero(3, MODEL_DOF + 6);
  link_[i].Jac_COM_r.setZero(3, MODEL_DOF + 6);
  link_[i].Jac_Contact.setZero(6, MODEL_DOF + 6);
  link_[i].Jac_point.setZero(6, MODEL_DOF + 6);
}

void DyrosRedModel::Link_pos_Update(int i)
{
  link_[i].xpos = RigidBodyDynamics::CalcBodyToBaseCoordinates(model_, q_virtual_, link_[i].id, Eigen::Vector3d::Zero(), false);
  link_[i].xipos = RigidBodyDynamics::CalcBodyToBaseCoordinates(model_, q_virtual_, link_[i].id, link_[i].COM_position, false);
  link_[i].Rotm = RigidBodyDynamics::CalcBodyWorldOrientation(model_, q_virtual_, link_[i].id, false);
  // link_[i].COM_position =
  // RigidBodyDynamics::CalcBaseToBodyCoordinates(model_,q_virtual_,link_[i])
}

void DyrosRedModel::Link_COM_Jac_Update(int i)
{
  Eigen::MatrixXd j_p_(3, MODEL_DOF + 6), j_r_(3, MODEL_DOF + 6);
  Eigen::MatrixXd j_(6, MODEL_DOF + 6);
  Eigen::MatrixXd fj_(6, MODEL_DOF + 6);
  fj_.setZero();
  ros::Time t_temp = ros::Time::now();
  RigidBodyDynamics::CalcPointJacobian6D(model_, q_virtual_, link_[i].id, link_[i].COM_position, fj_, false);
  j_p_ = fj_.block<3, MODEL_DOF + 6>(3, 0);
  j_r_ = fj_.block<3, MODEL_DOF + 6>(0, 0);
  t_temp = ros::Time::now();
  link_[i].Jac_COM_p = j_p_; //*E_T_;
  link_[i].Jac_COM_r = j_r_; //*E_T_;
  // link_[i].Jac_COM_p.block<3,3>(0,3)=- DyrosMath::skm(link_[i].xipos -
  // link_[0].xpos);
  j_.block<3, MODEL_DOF + 6>(0, 0) = link_[i].Jac_COM_p;
  j_.block<3, MODEL_DOF + 6>(3, 0) = link_[i].Jac_COM_r;

  link_[i].Jac_COM = j_;
  double ar_time = ros::Time::now().toSec() - t_temp.toSec();
  // ROS_INFO(" ::: %d Link calc time : %8.4f,%8.4f", i, j_calc_time*1000,
  // ar_time*1000);
}

void DyrosRedModel::Link_Set_Jacobian(int i, Eigen::Vector3d Jacobian_position)
{
  Eigen::MatrixXd fj_(6, MODEL_DOF + 6);
  fj_.setZero();
  RigidBodyDynamics::CalcPointJacobian6D(model_, q_virtual_, link_[i].id, Jacobian_position, fj_, false);
  // RigidBodyDynamics::CalcBodySpatialJacobian(model_,q_virtual_,link_[i].id,fj_,false);
  // link_[i].Jac.block<3,MODEL_DOF+6>(0,0)=fj_.block<3,MODEL_DOF+6>(3,0)*E_T_;
  // link_[i].Jac.block<3,MODEL_DOF+6>(3,0)=fj_.block<3,MODEL_DOF+6>(0,0)*E_T_;
  link_[i].Jac.block<3, MODEL_DOF + 6>(0, 0) = fj_.block<3, MODEL_DOF + 6>(3, 0);
  link_[i].Jac.block<3, MODEL_DOF + 6>(3, 0) = fj_.block<3, MODEL_DOF + 6>(0, 0);

  // link_[i].Jac.block<3,3>(0,3)= - DyrosMath::skm(link_[i].xpos -
  // link_[0].xpos);
}

void DyrosRedModel::Link_Set_Contact(int i, Eigen::Vector3d Contact_position)
{

  Eigen::MatrixXd fj_(6, MODEL_DOF + 6);
  fj_.setZero();

  RigidBodyDynamics::CalcPointJacobian6D(model_, q_virtual_, link_[i].id, Contact_position, fj_, false);

  // link_[i].Jac_Contact.block<3,MODEL_DOF+6>(0,0)=fj_.block<3,MODEL_DOF+6>(3,0)*E_T_;
  // link_[i].Jac_Contact.block<3,MODEL_DOF+6>(3,0)=fj_.block<3,MODEL_DOF+6>(0,0)*E_T_;
  link_[i].Jac_Contact.block<3, MODEL_DOF + 6>(0, 0) = fj_.block<3, MODEL_DOF + 6>(3, 0);
  link_[i].Jac_Contact.block<3, MODEL_DOF + 6>(3, 0) = fj_.block<3, MODEL_DOF + 6>(0, 0);

  // link_[i].Jac_Contact.block<3,3>(0,3)= -
  // DyrosMath::skm(RigidBodyDynamics::CalcBodyToBaseCoordinates(model_,q_virtual_,link_[i].id,Contact_position,false)
  // - link_[0].xpos);
}

void DyrosRedModel::Link_vw_Update(int i, Eigen::VectorXd q_dot_virtual)
{
  Eigen::Vector6d vw;
  vw = link_[i].Jac * q_dot_virtual;
  link_[i].v = vw.segment(0, 3);
  link_[i].w = vw.segment(3, 3);
}

void DyrosRedModel::Link_Set_Trajectory(int i, Eigen::Vector3d position_desired, Eigen::Vector3d velocity_desired, Eigen::Matrix3d rotation_desired, Eigen::Vector3d rotational_velocity_desired)
{
  link_[i].x_traj = position_desired;
  link_[i].v_traj = velocity_desired;
  link_[i].r_traj = rotation_desired;
  link_[i].w_traj = rotational_velocity_desired;
}

void DyrosRedModel::Link_Set_Trajectory_from_quintic(int i, double current_time, double start_time, double end_time, Eigen::Vector3d pos_desired)
{
  for (int j = 0; j < 3; j++)
  {
    Eigen::Vector3d quintic = DyrosMath::QuinticSpline(current_time, start_time, end_time, link_[i].x_init(j), 0, 0, pos_desired(j), 0, 0);
    link_[i].x_traj(j) = quintic(0);
    link_[i].v_traj(j) = quintic(1);
  }

  link_[i].r_traj = link_[i].rot_init;
  link_[i].w_traj = Eigen::Vector3d::Zero();
}

void DyrosRedModel::Link_Set_initpos(int i)
{
  link_[i].x_init = link_[i].xpos;
  link_[i].rot_init = link_[i].Rotm;
}

DyrosRedModel::DyrosRedModel()
{
  A_.resize(MODEL_DOF + 6, MODEL_DOF + 6);
  A_.setZero();
  q_.resize(MODEL_DOF);
  q_.setZero();
  q_virtual_.resize(MODEL_DOF + 6);
  q_virtual_.setZero();
  q_virtual_quaternion_.resize(MODEL_DOF + 7);
  q_virtual_quaternion_.setZero();
  A_temp_.resize(MODEL_DOF + 6, MODEL_DOF + 6);
  A_temp_.setZero();
  R_temp_.resize(MODEL_DOF + 6, MODEL_DOF + 6);
  R_temp_.setZero();
  Gravity_.setZero();
  Gravity_(2) = GRAVITY;
  test_run = 0;

  E_T_.resize(MODEL_DOF + 6, MODEL_DOF + 6);
  E_T_.setZero();

  std::string desc_package_path = ros::package::getPath("dyros_red_description");
  std::string urdf_path = desc_package_path + "/robots/dyros_red_robot.urdf";
  ROS_INFO("Loading DYROS JET description from = %s", urdf_path.c_str());
  RigidBodyDynamics::Addons::URDFReadFromFile(urdf_path.c_str(), &model_, true, true);
  ROS_INFO("Successfully loaded.");
  ROS_INFO("MODEL DOF COUNT = %d", model_.dof_count);
  ROS_INFO("MODEL Q SIZE = %d", model_.q_size);
  // model_.mJoints[0].)
  if (model_.dof_count != MODEL_DOF + 6)
  {
    ROS_WARN("The DoF in the model file and the code do not match.");
    ROS_WARN("Model file = %d, Code = %d", model_.dof_count, (int)MODEL_DOF + 6);
  }
  else
  {
    // ROS_INFO("id:0 name is : %s",model_.GetBodyName(0));
    for (int i = 0; i < MODEL_DOF + 1; i++)
    {
      link_id_[i] = model_.GetBodyId(LINK_NAME[i]);
      // ROS_INFO("%s: \t\t id = %d \t parent link = %d",LINK_NAME[i],
      // link_id_[i],model_.GetParentBodyId(link_id_[i]));

      // ROS_INFO("%dth parent
      // %d",link_id_[i],model_.GetParentBodyId(link_id_[i]));
      // std::cout << model_.mBodies[link_id_[i]].mCenterOfMass << std::endl;
      // joint_name_map_[JOINT_NAME[i]] = i;
    }
    for (int i = 0; i < MODEL_DOF + 1; i++)
    {
      Link_initialize(i, link_id_[i], LINK_NAME[i], model_.mBodies[link_id_[i]].mMass, model_.mBodies[link_id_[i]].mCenterOfMass);
    }
    total_mass = 0;
    for (int i = 0; i < MODEL_DOF + 1; i++)
    {
      total_mass += link_[i].Mass;
    }

    // RigidBodyDynamics::Joint J_temp;
    // J_temp=RigidBodyDynamics::Joint(RigidBodyDynamics::JointTypeEulerXYZ);

    // model_.mJoints[2] = J_temp;

    for (int i = 0; i < MODEL_DOF + 6; i++)
    {

      // ROS_INFO("Joint type %d : %d", i, model_.mJoints[i].mJointType);
    }
  }

  ROS_INFO("MODEL initialize END");
}

void DyrosRedModel::test()
{
  // updateKinematics(Eigen::Vector28d::Zero());
  // std::cout << "test" << std::endl;

  if (test_run == 0)
  {
    /*
        for(int i=0;i<MODEL_DOF;i++){
          VectorQd q_virtual;<<0,0,0.92611,0,0, 0,
          0, 0, -0.24, 0.6, -0.36, 0,
          0, 0, -0.24, 0.6, -0.36, 0,
          0, 0, 0,
          -0.3, 0, -1.5, -1.87, -0.7, 0, -1, 0,
          0.3, 0, 1.5, 1.87, 0.7, 0, 1, 0;


          RigidBodyDynamics::UpdateKinematicsCustom(model_, &q_virtual, NULL,
       NULL);
          RigidBodyDynamics::CompositeRigidBodyAlgorithm(model_, q_virtual,
       A_temp_, true);

          for(int i=0;i<MODEL_DOF+1;i++){
            Link_pos_Update(i);
          }

          for(int i=0;i<MODEL_DOF+1;i++){
            Link_Set_Jacobian(i,zero);
            Link_Jac_Update(i);
          }




        //std::cout <<model_.GetBodyName(link_[i].id) << "position is :::::
       "<<std::endl<<
       RigidBodyDynamics::CalcBodyToBaseCoordinates(model_,q_,link_[i].id,Eigen::Vector3d::Zero(),false)
       << std::endl;

        }
    */
    // std::cout << "link0 Jac ::" << std::endl;

    // std::cout << link_[0].Jac<<std::endl;

    // std::cout << " link 0 COM POS :: " <<std::endl << link_[0].COM_position
    // << std::endl;

    // std::cout << "link0 Jac COM ::" <<std::endl;

    // std::cout << link_[0].Jac_COM<<std::endl;

    test_run++;
  }

  // std::cout << "Left_Arm Wrist2 Body to Base position ::::: "<<std::endl<<
  // RigidBodyDynamics::CalcBodyToBaseCoordinates(model_,q_,link_[Left_Arm+7].id,Eigen::Vector3d::Zero(),false)<<std::endl;
  // std::cout << "Left_Arm Wrist2 Base to Body position ::::: "<<std::endl<<
  // RigidBodyDynamics::CalcBaseToBodyCoordinates(model_,q_,link_[Left_Arm+7].id,Eigen::Vector3d::Zero(),false)<<std::endl;
}

void DyrosRedModel::updateKinematics(const Eigen::VectorXd &q_virtual)
{
  ROS_INFO_ONCE("CONTROLLER : MODEL : updatekinematics enter ");
  /* q_virtual description
   * 0 ~ 2 : XYZ cartesian coordinates
   * 3 ~ 5 : XYZ Quaternion
   * 6 ~ MODEL_DOF + 5 : joint position
   * model dof + 6 ( last component of q_virtual) : w of Quaternion
   * */

  /*
   Eigen::Matrix3d Er1,Er2,Er3,Er;
   Er.setZero();
   Er1 = Eigen::AngleAxisd(q_virtual_(3), Eigen::Vector3d::UnitX());
   Er2 = Eigen::AngleAxisd(q_virtual_(3),
   Eigen::Vector3d::UnitX())*Eigen::AngleAxisd(q_virtual_(4),
   Eigen::Vector3d::UnitY());
   Er3 = Eigen::AngleAxisd(q_virtual_(3),
   Eigen::Vector3d::UnitX())*Eigen::AngleAxisd(q_virtual_(4),
   Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(q_virtual_(5),
   Eigen::Vector3d::UnitZ());

   Er.block<3,1>(0,0) = Er1.block<3,1>(0,0);
   Er.block<3,1>(0,1) = Er2.block<3,1>(0,1);
   Er.block<3,1>(0,2) = Er3.block<3,1>(0,2);

   Eri=Er.inverse();
   E_T_ = Eigen::MatrixXd::Identity(MODEL_DOF+6,MODEL_DOF+6);
   Eri = Eigen::MatrixXd::Identity(3,3);
   E_T_.block<3,3>(3,3)=Eri;
 */
  q_virtual_ = q_virtual;
  ros::Time t_temp = ros::Time::now();
  RigidBodyDynamics::UpdateKinematicsCustom(model_, &q_virtual, NULL, NULL);
  double uk_time = ros::Time::now().toSec() - t_temp.toSec();

  t_temp = ros::Time::now();
  RigidBodyDynamics::CompositeRigidBodyAlgorithm(model_, q_virtual_, A_temp_, true);
  double cr_time = ros::Time::now().toSec() - t_temp.toSec();

  tf::Quaternion q(q_virtual_(3), q_virtual_(4), q_virtual_(5), q_virtual_(MODEL_DOF + 6));

  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  yaw_radian = yaw;

  /*
    Eigen::Matrix3d temp = link_[0].Rotm.transpose()*link_[Right_Foot].Rotm;
    tf::Matrix3x3
    m_(temp(0,0),temp(1,0),temp(2,0),temp(0,1),temp(1,1),temp(2,1),temp(0,2),temp(1,2),temp(2,2));

    Eigen::Vector3d euler;
    m_.getRPY(euler(0),euler(1),euler(2));

    std::cout <<" Pelvis euler from foot "<<std::endl;
    std::cout <<euler <<std::endl;

    for(int i=0;i<3;i++)q_virtual_(3+i)=euler(i);

    RigidBodyDynamics::CompositeRigidBodyAlgorithm(model_, q_virtual_, A_temp_,
    true);
  */

  // A_=E_T_.transpose()*A_temp_*E_T_;
  A_ = A_temp_;

  com_ = getCenterOfMassPosition();

  for (int i = 0; i < MODEL_DOF + 1; i++)
  {
    Link_pos_Update(i);
  }

  Eigen::Vector3d zero;
  zero.setZero();
  t_temp = ros::Time::now();
  for (int i = 0; i < MODEL_DOF + 1; i++)
  {
    Link_Set_Jacobian(i, zero);
    Link_COM_Jac_Update(i);
  }

  //COM link information update ::

  Eigen::MatrixXd jacobian_com;
  jacobian_com.setZero(3, MODEL_DOF + 6);

  for (int i = 0; i < MODEL_DOF + 1; i++)
  {
    jacobian_com += link_[i].Jac_COM_p * link_[i].Mass;
  }
  jacobian_com = jacobian_com / total_mass;

  link_[COM_id].Jac.setZero(6, MODEL_DOF + 6);

  link_[COM_id].Jac.block(0, 0, 3, MODEL_DOF + 6) = jacobian_com;
  link_[COM_id].Jac.block(3, 0, 3, MODEL_DOF + 6) = link_[Pelvis].Jac_COM_r;
  link_[COM_id].xpos = com_;
  link_[COM_id].Rotm = link_[Pelvis].Rotm;

  double ju_time = ros::Time::now().toSec() - t_temp.toSec();

  ROS_DEBUG("Update time - detail \n updatekinematicsCustum Time : % 3.4f ms\n compositeRigid time : %3.4f ms\n jac_update time : %3.4f ms", uk_time * 1000, cr_time * 1000, ju_time * 1000);
  ROS_INFO_ONCE("CONTROLLER : MODEL : updatekinematics end ");
}

Eigen::Vector3d DyrosRedModel::getCenterOfMassPosition()
{
  RigidBodyDynamics::Math::Vector3d position_temp;
  position_temp.setZero();

  for (int i = 0; i < MODEL_DOF + 1; i++)
  {
    position_temp = position_temp + link_[i].Mass * link_[i].xipos;
  }
  position_temp = position_temp / total_mass;

  return position_temp;
}

/*
void DyrosRedModel::getTransformEndEffector // must call updateKinematics before
    calling this function(EndEffector ee, Eigen::Isometry3d *transform_matrix)
{
  Eigen::Vector3d gghg = RigidBodyDynamics::CalcBodyToBaseCoordinates(model_, q_, end_effector_id_[ee], base_position_, false);
  transform_matrix->translation() = RigidBodyDynamics::CalcBodyToBaseCoordinates(model_, q_, end_effector_id_[ee], base_position_, false);
  transform_matrix->linear() = RigidBodyDynamics::CalcBodyWorldOrientation(model_, q_, end_effector_id_[ee], false).transpose();
}

void DyrosRedModel::getTransformEndEffector // must call updateKinematics before
    calling this function(EndEffector ee, Eigen::Vector3d *position, Eigen::Matrix3d *rotation)
{
  *position = RigidBodyDynamics::CalcBodyToBaseCoordinates(model_, q_, end_effector_id_[ee], base_position_, false);
  *rotation = RigidBodyDynamics::CalcBodyWorldOrientation(model_, q_, end_effector_id_[ee], false).transpose();
}

void DyrosRedModel::getTransformEndEffector(EndEffector ee, const Eigen::VectorXd &q, bool update_kinematics, Eigen::Vector3d *position, Eigen::Matrix3d *rotation)
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
  *position = RigidBodyDynamics::CalcBodyToBaseCoordinates(model_, q_new, end_effector_id_[ee], base_position_, update_kinematics);
  *rotation = RigidBodyDynamics::CalcBodyWorldOrientation(model_, q_new, end_effector_id_[ee], update_kinematics).transpose();
  // RigidBodyDynamics::Calcpo
  // model_.mBodies[0].mCenterOfMass
}
*/
} // namespace dyros_red_controller
