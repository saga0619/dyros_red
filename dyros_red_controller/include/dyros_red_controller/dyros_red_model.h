#ifndef dyros_red_MODEL_H
#define dyros_red_MODEL_H

#include <string>
#include <map>

#include <ros/ros.h>
#include <ros/package.h>
#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h>
#include "math_type_define.h"


namespace dyros_red_controller
{

class DyrosRedModel
{
public:
  DyrosRedModel();

  struct Link{
    int id;
    double Mass;
    Eigen::Vector3d COM_position;
    Eigen::Matrix3d inertia;

    Eigen::Matrix3d Rotm;
    Eigen::Vector3d xpos;
    Eigen::Vector3d contact_point;

    Eigen::MatrixXd Jac_point;
    Eigen::MatrixXd Jac;
    Eigen::MatrixXd Jac_COM;
    Eigen::MatrixXd Jac_COM_p;
    Eigen::MatrixXd Jac_COM_r;
    Eigen::MatrixXd Jac_Contact;

  /*
    void Set_Jac_point(Eigen::MatrixXd J);
    void Set_Jac(Eigen::MatrixXd J);
    void Set_Jac_COM(Eigen::MatrixXd J);
    void Set_Jac_Contact(Eigen::MatrixXd J);


    Eigen::MatrixXd Get_Jac_point();
    Eigen::MatrixXd Get_Jac();
    Eigen::MatrixXd Get_Jac_COM();
    Eigen::MatrixXd Get_Jac_COM_p();
    Eigen::MatrixXd Get_Jac_COM_r();
    Eigen::MatrixXd Get_Jac_Contact();*/

  };

  void Link_initialize(int i, int id, double mass, Eigen::Vector3d xipos);
  void Link_Jac_Update(int i);
  void Link_pos_Update(int i);
  void Link_Set_Contact(int i, Eigen::Vector3d Contact_position);
  void Link_Set_Jacobian(int i, Eigen::Vector3d Jacobian_position);
  void Link_Set_test(int i);
  int test_run;



  /*
  class part{
    part(DyrosRedModel &RM);

  public:
    DyrosRedModel &RM_;
    void initialize(int id_, double mass_, Eigen::Matrix3d inertia_, Eigen::Vector3d COMpos_);
    void Update(Eigen::Vector3d xpos_, Eigen::MatrixXd fj1_, Eigen::MatrixXd fj2_, Eigen::Matrix3d pel_rot);
    void SetContact(Eigen::Vector3d contact_point);
    void SetContact(double x, double y, double z);
    void SetPoint(Eigen::Vector3d point);
    void SetPoint(double x, double y, double z);
    void test();
    Eigen::MatrixXd RBDLJac2GlobalJac(Eigen::MatrixXd rbdljac, Eigen::Matrix3d pel_rot);


    int id;
    double Mass;
    Eigen::Matrix3d Rotm;
    Eigen::MatrixXd Jac_point;
    Eigen::MatrixXd Jac;
    Eigen::MatrixXd Jac_COM;
    Eigen::MatrixXd Jac_COM_p;
    Eigen::MatrixXd Jac_COM_r;
    Eigen::MatrixXd Jac_Contact;
    Eigen::Matrix3d inertia;
    Eigen::Vector3d xpos;
    Eigen::Vector3d xipos;


  };
*/




  static constexpr size_t MODEL_DOF = 31;
  static constexpr size_t LINK_NUMBER = 32;
  static constexpr size_t MODEL_DOF_VIRTUAL = 37;
  static constexpr size_t ARM_DOF = 8;
  static constexpr size_t LEG_DOF = 6;
  static constexpr size_t WAIST_DOF = 3;



  static constexpr size_t Left_Leg = 4;
  static constexpr size_t Right_Leg = 10;
  static constexpr size_t Left_Arm = 16;
  static constexpr size_t Right_Arm = 24;

  static constexpr size_t joint_left_leg = 0;
  static constexpr size_t joint_right_leg = 6;
  static constexpr size_t joint_body = 12;
  static constexpr size_t joint_left_arm = 15;
  static constexpr size_t joint_right_arm = 23;


  //part part_[MODEL_DOF+1];

  Link link_[40];

  static const std::string JOINT_NAME[MODEL_DOF];
  static constexpr const char* EE_NAME[4] =
      {"L_AnkleRoll_Link", "R_AnkleRoll_Link",
       "L_HandYaw_Link", "R_HandYaw_Link" };

  /*static constexpr const char* LINK_NAME[32] =
      {"L0_Link", "L1_Link", "L2_Link", "L3_Link", "L4_Link", "L5_Link",
       "R0_Link", "R1_Link", "R2_Link", "R3_Link", "R4_Link", "R5_Link",
        "Waist1_Link","Waist2_Link","Upperbody_Link",
       "UL0_Link","UL1_Link","UL2_Link","UL3_Link","UL4_Link","UL5_Link","UL6_Link","UL7_Link",
       "UR0_Link","UR1_Link","UR2_Link","UR3_Link","UR4_Link","UR5_Link","UR6_Link","UR7_Link"};
  */


  static constexpr const char* LINK_NAME[32] =  {
    "Pelvis_Link","Waist1_Link","Waist2_Link","Upperbody_Link",
    "L_HipRoll_Link", "L_HipCenter_Link", "L_Thigh_Link", "L_Knee_Link", "L_AnkleCenter_Link", "L_AnkleRoll_Link",
    "R_HipRoll_Link", "R_HipCenter_Link", "R_Thigh_Link", "R_Knee_Link", "R_AnkleCenter_Link", "R_AnkleRoll_Link",
    "L_Shoulder1_Link","L_Shoulder2_Link","L_Shoulder3_Link","L_Armlink_Link","L_Elbow_Link","L_Forearm_Link","L_Wrist1_Link","L_Wrist2_Link",
    "R_Shoulder1_Link","R_Shoulder2_Link","R_Shoulder3_Link","R_Armlink_Link","R_Elbow_Link","R_Forearm_Link","R_Wrist1_Link","R_Wrist2_Link"
    };


  unsigned int end_effector_id_[4];
  unsigned int link_id_[40];

  void test();

  std::map<std::string, size_t> joint_name_map_;
  size_t getIndex(const std::string& joint_name)
  {
    return joint_name_map_[joint_name];
  }
  // Calc Jacobian, Transformation
  void updateKinematics(const Eigen::VectorXd &q);


  void getCenterOfMassPosition(Eigen::Vector3d* position);

  void setquat(Eigen::Quaterniond& quat, const Eigen::VectorXd& q);

  const Eigen::Vector3d getCurrentCom(){ return com_;}


  Eigen::VectorXd getGravityCompensation();
  Eigen::VectorXd GetDampingTorque(Eigen::VectorXd qdot, double damp_);

private:


  RigidBodyDynamics::Model model_;
  Eigen::VectorXd q_;
  Eigen::VectorXd q_virtual_;
  Eigen::Vector3d base_position_;

  Eigen::Isometry3d currnet_transform_[4];


  Eigen::MatrixXd A_;
  Eigen::MatrixXd A_temp_;
  Eigen::MatrixXd R_temp_;
  Eigen::Matrix3d Eri;
  Eigen::MatrixXd E_T_;
  Eigen::Vector3d com_;

  Eigen::Vector3d Gravity_;


};

typedef Eigen::Matrix<double, DyrosRedModel::MODEL_DOF, 1> VectorQd;
typedef Eigen::Matrix<double, DyrosRedModel::MODEL_DOF_VIRTUAL, 1> VectorVQd;

}
#endif // dyros_red_MODEL_H
