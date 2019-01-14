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
    "L_Shoulder1_Joint", "L_Shoulder2_Joint", "L_Shoulder3_Joint", "L_Armlink_Joint",
    "L_Elbow_Joint", "L_Forearm_Joint", "L_Wrist1_Joint", "L_Wrist2_Joint",
    "R_Shoulder1_Joint", "R_Shoulder2_Joint", "R_Shoulder3_Joint", "R_Armlink_Joint",
    "R_Elbow_Joint", "R_Forearm_Joint", "R_Wrist1_Joint", "R_Wrist2_Joint"};

const std::string DyrosRedModel::ACTUATOR_NAME[DyrosRedModel::MODEL_DOF] = {
    "L_HipRoll_Motor", "L_HipCenter_Motor", "L_Thigh_Motor",
    "L_Knee_Motor", "L_AnkleCenter_Motor", "L_AnkleRoll_Motor",
    "R_HipRoll_Motor", "R_HipCenter_Motor", "R_Thigh_Motor",
    "R_Knee_Motor", "R_AnkleCenter_Motor", "R_AnkleRoll_Motor",
    "Waist1_Motor", "Waist2_Motor", "Upperbody_Motor",
    "L_Shoulder1_Motor", "L_Shoulder2_Motor", "L_Shoulder3_Motor", "L_Armlink_Motor",
    "L_Elbow_Motor", "L_Forearm_Motor", "L_Wrist1_Motor", "L_Wrist2_Motor",
    "R_Shoulder1_Motor", "R_Shoulder2_Motor", "R_Shoulder3_Motor", "R_Armlink_Motor",
    "R_Elbow_Motor", "R_Forearm_Motor", "R_Wrist1_Motor", "R_Wrist2_Motor"};

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
  link_[i].Rotm = (RigidBodyDynamics::CalcBodyWorldOrientation(model_, q_virtual_, link_[i].id, false)).transpose();
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

  link_[i].xpos_contact = RigidBodyDynamics::CalcBodyToBaseCoordinates(model_, q_virtual_, link_[i].id, Contact_position, false);

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

void DyrosRedModel::Link_Set_Trajectory_from_quintic(int i, double current_time, double start_time, double end_time, Eigen::Vector3d pos_init, Eigen::Vector3d pos_desired)
{
  for (int j = 0; j < 3; j++)
  {
    Eigen::Vector3d quintic = DyrosMath::QuinticSpline(current_time, start_time, end_time, pos_init(j), 0, 0, pos_desired(j), 0, 0);
    link_[i].x_traj(j) = quintic(0);
    link_[i].v_traj(j) = quintic(1);
  }

  link_[i].r_traj = link_[i].rot_init;
  link_[i].w_traj = Eigen::Vector3d::Zero();
}

void DyrosRedModel::Link_Set_Trajectory_rotation(int i, double current_time, double start_time, double end_time, Eigen::Matrix3d rot_desired, bool local_)
{
  Eigen::Vector3d axis;
  double angle;
  if (local_)
  {
    Eigen::AngleAxisd aa(rot_desired);
    axis = aa.axis();
    angle = aa.angle();
  }
  else
  {
    Eigen::AngleAxisd aa(link_[i].rot_init.transpose() * rot_desired);
    axis = aa.axis();
    angle = aa.angle();
  }
  double c_a = DyrosMath::cubic(current_time, start_time, end_time, 0.0, angle, 0.0, 0.0);
  Eigen::Matrix3d rmat;
  rmat = Eigen::AngleAxisd(c_a, axis);

  link_[i].r_traj = link_[i].rot_init * rmat;

  double dtime = 0.0001;
  double c_a_dtime = DyrosMath::cubic(current_time + dtime, start_time, end_time, 0.0, angle, 0.0, 0.0);

  Eigen::Vector3d ea = link_[i].r_traj.eulerAngles(0, 1, 2);

  Eigen::Vector3d ea_dtime = (link_[i].rot_init * Eigen::AngleAxisd(c_a_dtime, axis)).eulerAngles(0, 1, 2);

  link_[i].w_traj = (ea_dtime - ea) / dtime;
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
      // //joint_name_map_[JOINT_NAME[i]] = i;
    }

    for (int i = 0; i < MODEL_DOF + 1; i++)
    {
      Link_initialize(i, link_id_[i], LINK_NAME[i], model_.mBodies[link_id_[i]].mMass, model_.mBodies[link_id_[i]].mCenterOfMass);
    }

    Eigen::Vector3d lf_c, rf_c, lh_c, rh_c;
    lf_c << 0.0317, 0, -0.1368;
    rf_c << 0.0317, 0, -0.1368;
    rh_c << 0, -0.092, 0;
    lh_c << 0, 0.092, 0;
    link_[Right_Foot].contact_point = rf_c;
    link_[Left_Foot].contact_point = lf_c;
    link_[Right_Hand].contact_point = rh_c;
    link_[Left_Hand].contact_point = lh_c;

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
  if (test_run == 0)
  {
    test_run++;
  }
}

void DyrosRedModel::updateKinematics(const Eigen::VectorXd &q_virtual, const Eigen::VectorXd &q_dot_virtual, const Eigen::VectorXd &q_ddot_virtual)
{
  ROS_INFO_ONCE("CONTROLLER : MODEL : updatekinematics enter ");
  /* q_virtual description
   * 0 ~ 2 : XYZ cartesian coordinates
   * 3 ~ 5 : XYZ Quaternion
   * 6 ~ MODEL_DOF + 5 : joint position
   * model dof + 6 ( last component of q_virtual) : w of Quaternion
   * */

  q_virtual_ = q_virtual;
  q_dot_virtual_ = q_dot_virtual;
  q_ddot_virtual_ = q_ddot_virtual;

  ros::Time t_temp = ros::Time::now();
  RigidBodyDynamics::UpdateKinematicsCustom(model_, &q_virtual, &q_dot_virtual, &q_ddot_virtual);
  double uk_time = ros::Time::now().toSec() - t_temp.toSec();

  t_temp = ros::Time::now();
  RigidBodyDynamics::CompositeRigidBodyAlgorithm(model_, q_virtual_, A_temp_, true);
  double cr_time = ros::Time::now().toSec() - t_temp.toSec();

  tf::Quaternion q(q_virtual_(3), q_virtual_(4), q_virtual_(5), q_virtual_(MODEL_DOF + 6));

  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  yaw_radian = yaw;

  A_ = A_temp_;

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
  double com_mass;
  RigidBodyDynamics::Math::Vector3d com_pos;
  RigidBodyDynamics::Math::Vector3d com_vel, com_accel, com_ang_momentum;

  RigidBodyDynamics::Utils::CalcCenterOfMass(model_, q_virtual_, q_dot_virtual_, &q_ddot_virtual, com_mass, com_pos, &com_vel, &com_accel, &com_ang_momentum, NULL, true);
  com_.mass = com_mass;
  com_.pos = com_pos;

  Eigen::Vector3d vel_temp;
  vel_temp = com_.vel;
  com_.vel = com_vel;

  com_.accel = -com_accel;
  com_.angular_momentum = com_ang_momentum;

  ROS_INFO_ONCE("TOTAL MASS : %f", com_.mass);
  double w_ = sqrt(9.81 / com_.pos(2));

  com_.ZMP(0) = com_.pos(0) - com_.accel(0) / pow(w_, 2);
  com_.ZMP(1) = com_.pos(1) - com_.accel(1) / pow(w_, 2);

  //com_.ZMP(0) = (com_.pos(0) * (com_.accel(2) + 9.81) - com_pos(2) * com_accel(0)) / (com_.accel(2) + 9.81) - com_.angular_momentum(2) / com_.mass / (com_.accel(2) + 9.81);

  //com_.ZMP(1) = (com_.pos(1) * (com_.accel(2) + 9.81) - com_pos(2) * com_accel(1)) / (com_.accel(2) + 9.81) - com_.angular_momentum(1) / com_.mass / (com_.accel(2) + 9.81);

  com_.CP(0) = com_.pos(0) + com_.vel(0) / w_;
  com_.CP(1) = com_.pos(1) + com_.vel(1) / w_;

  Eigen::MatrixXd jacobian_com;
  jacobian_com.setZero(3, MODEL_DOF + 6);

  for (int i = 0; i < MODEL_DOF + 1; i++)
  {
    jacobian_com += link_[i].Jac_COM_p * link_[i].Mass;
  }
  jacobian_com = jacobian_com / com_.mass;

  link_[COM_id].Jac.setZero(6, MODEL_DOF + 6);

  link_[COM_id].Jac.block(0, 0, 2, MODEL_DOF + 6) = jacobian_com.block(0, 0, 2, MODEL_DOF + 6);
  link_[COM_id].Jac.block(2, 0, 4, MODEL_DOF + 6) = link_[Pelvis].Jac.block(2, 0, 4, MODEL_DOF + 6);
  link_[COM_id].xpos = com_.pos;
  link_[COM_id].xpos(2) = link_[Pelvis].xpos(2);
  link_[COM_id].Rotm = link_[Pelvis].Rotm;

  for (int i = 0; i < MODEL_DOF + 2; i++)
  {
    Link_vw_Update(i, q_dot_virtual_);
  }

  R_ARM_A_.setZero(8, 8);
  R_ARM_A_.setZero(8, 8);

  for (int i = 0; i < 8; i++)
  {
  }

  double ju_time = ros::Time::now().toSec() - t_temp.toSec();

  ROS_DEBUG("Update time - detail \n updatekinematicsCustum Time : % 3.4f ms\n compositeRigid time : %3.4f ms\n jac_update time : %3.4f ms", uk_time * 1000, cr_time * 1000, ju_time * 1000);
  ROS_INFO_ONCE("CONTROLLER : MODEL : updatekinematics end ");
}

} // namespace dyros_red_controller
