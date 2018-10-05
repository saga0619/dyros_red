#ifndef dyros_red_MODEL_H
#define dyros_red_MODEL_H

#include <string>
#include <map>

#include <ros/ros.h>
#include <ros/package.h>
#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h>
#include "math_type_define.h"
#include <tf/transform_datatypes.h>

namespace dyros_red_controller
{

class DyrosRedModel
{
public:
  DyrosRedModel();

  struct Link
  {
    int id;
    double Mass;
    std::string name;

    //local COM position of body
    Eigen::Vector3d COM_position;

    //inertial matrix
    Eigen::Matrix3d inertia;

    //rotation matrix
    Eigen::Matrix3d Rotm;

    //global position of body
    Eigen::Vector3d xpos;

    //global COM position of body
    Eigen::Vector3d xipos;

    //local contact point
    Eigen::Vector3d contact_point;

    //cartesian velocity of body
    Eigen::Vector3d v;

    //rotational velocity of body
    Eigen::Vector3d w;

    Eigen::MatrixXd Jac_point;
    Eigen::MatrixXd Jac;
    Eigen::MatrixXd Jac_COM;
    Eigen::MatrixXd Jac_COM_p;
    Eigen::MatrixXd Jac_COM_r;
    Eigen::MatrixXd Jac_Contact;

    //realtime traj of cartesian & orientation.
    //)) traj is outcome of cubic or quintic function, which will be used to make fstar!
    // x : cartesian coordinate traj (3x1)
    // v : cartesian velocity (3x1)
    // r : rotational matrix of current orientation (3x3)
    // w : rotational speed of current orientation (3x1)

    Eigen::Vector3d x_traj;
    Eigen::Vector3d v_traj;

    Eigen::Matrix3d r_traj;
    Eigen::Vector3d w_traj;

    Eigen::Vector3d x_init;
    Eigen::Matrix3d rot_init;
  };

  // Update link i of rbdl link id. name : link name, mass : link mass, xipos : local center of mass position
  void Link_initialize(int i, int id, std::string name, double mass, Eigen::Vector3d local_com_position);

  // Update COM jacobian
  void Link_COM_Jac_Update(int i);

  // Update xpos, xipos, rotm.
  void Link_pos_Update(int i);

  // Set Contact point, Contact jacobian
  void Link_Set_Contact(int i, Eigen::Vector3d Contact_position);

  // update Jacobian matrix of local position at link.
  void Link_Set_Jacobian(int i, Eigen::Vector3d Jacobian_position);

  // update link velocity(6D, translation and rotation) from jacobian matrix Jac.
  void Link_vw_Update(int i, Eigen::VectorXd q_dot_virtual);

  // set link Trajectory of id i.
  void Link_Set_Trajectory(int i, Eigen::Vector3d position_desired, Eigen::Vector3d velocity_desired, Eigen::Matrix3d rotation_desired, Eigen::Vector3d rotational_velocity_desired);

  // set realtime trajectory of link from quintic spline.
  void Link_Set_Trajectory_from_quintic(int i, double current_time, double start_time, double end_time, Eigen::Vector3d pos_desired);

  // set realtime trajectory of rotation of link
  void Link_Set_Trajectory_rotation(int i, double current_time, double start_time, double end_time, Eigen::Matrix3d rot_desired, bool local_);

  // set link initial position and rotation. initial position for task control.
  void Link_Set_initpos(int i);

  // test funtion for link.
  void Link_Set_test(int i);

  int test_run;

  static constexpr size_t MODEL_DOF = 31;
  static constexpr size_t LINK_NUMBER = 32;
  static constexpr size_t MODEL_DOF_VIRTUAL = 37;
  static constexpr size_t ARM_DOF = 8;
  static constexpr size_t LEG_DOF = 6;
  static constexpr size_t WAIST_DOF = 3;

  static constexpr size_t Pelvis = 0;
  static constexpr size_t Upper_Body = 3;
  static constexpr size_t Left_Leg = 4;
  static constexpr size_t Left_Foot = 9;
  static constexpr size_t Right_Leg = 10;
  static constexpr size_t Right_Foot = 15;
  static constexpr size_t Left_Arm = 16;
  static constexpr size_t Left_Hand = 23;
  static constexpr size_t Right_Arm = 24;
  static constexpr size_t Right_Hand = 31;
  static constexpr size_t COM_id = 32;

  //part part_[MODEL_DOF+1];

  Link link_[40];

  static const std::string ACTUATOR_NAME[MODEL_DOF];
  static const std::string JOINT_NAME[MODEL_DOF];
  static constexpr const char *EE_NAME[4] =
      {"L_AnkleRoll_Link", "R_AnkleRoll_Link",
       "L_HandYaw_Link", "R_HandYaw_Link"};

  /*static constexpr const char* LINK_NAME[32] =
      {"L0_Link", "L1_Link", "L2_Link", "L3_Link", "L4_Link", "L5_Link",
       "R0_Link", "R1_Link", "R2_Link", "R3_Link", "R4_Link", "R5_Link",
        "Waist1_Link","Waist2_Link","Upperbody_Link",
       "UL0_Link","UL1_Link","UL2_Link","UL3_Link","UL4_Link","UL5_Link","UL6_Link","UL7_Link",
       "UR0_Link","UR1_Link","UR2_Link","UR3_Link","UR4_Link","UR5_Link","UR6_Link","UR7_Link"};
  */

  static constexpr const char *LINK_NAME[32] = {
      "Pelvis_Link", "Waist1_Link", "Waist2_Link", "Upperbody_Link",
      "L_HipRoll_Link", "L_HipCenter_Link", "L_Thigh_Link", "L_Knee_Link", "L_AnkleCenter_Link", "L_AnkleRoll_Link",
      "R_HipRoll_Link", "R_HipCenter_Link", "R_Thigh_Link", "R_Knee_Link", "R_AnkleCenter_Link", "R_AnkleRoll_Link",
      "L_Shoulder1_Link", "L_Shoulder2_Link", "L_Shoulder3_Link", "L_Armlink_Link", "L_Elbow_Link", "L_Forearm_Link", "L_Wrist1_Link", "L_Wrist2_Link",
      "R_Shoulder1_Link", "R_Shoulder2_Link", "R_Shoulder3_Link", "R_Armlink_Link", "R_Elbow_Link", "R_Forearm_Link", "R_Wrist1_Link", "R_Wrist2_Link"};

  unsigned int end_effector_id_[4];
  unsigned int link_id_[40];

  void test();

  std::map<std::string, size_t> joint_name_map_;
  size_t getIndex(const std::string &joint_name)
  {
    return joint_name_map_[joint_name];
  }
  // Calc Jacobian, Transformation
  void updateKinematics(const Eigen::VectorXd &q, const Eigen::VectorXd &q_dot);

  Eigen::Vector3d getCenterOfMassPosition();

  const Eigen::Vector3d getCurrentCom() { return com_; }

  //Eigen::VectorXd getGravityCompensation();
  //Eigen::VectorXd GetDampingTorque(Eigen::VectorXd qdot, double damp_);

private:
  RigidBodyDynamics::Model model_;
  Eigen::VectorXd q_;
  Eigen::VectorXd q_virtual_;
  Eigen::VectorXd q_virtual_quaternion_;
  Eigen::Vector3d base_position_;
  Eigen::VectorXd q_dot_virtual_;
  Eigen::Isometry3d currnet_transform_[4];

public:
  Eigen::MatrixXd A_;
  Eigen::MatrixXd A_temp_;
  Eigen::MatrixXd R_temp_;
  Eigen::Matrix3d Eri;
  Eigen::MatrixXd E_T_;
  Eigen::Vector3d com_;
  Eigen::Vector3d Gravity_;
  double total_mass;
  double yaw_radian;

  bool debug_mode_;
};

typedef Eigen::Matrix<double, DyrosRedModel::MODEL_DOF, 1> VectorQd;
typedef Eigen::Matrix<double, DyrosRedModel::MODEL_DOF_VIRTUAL, 1> VectorVQd;
typedef Eigen::Matrix<double, DyrosRedModel::MODEL_DOF_VIRTUAL + 1, 1> VectorVquatQd;

} // namespace dyros_red_controller
#endif // dyros_red_MODEL_H
