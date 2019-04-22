#ifndef LINK_H
#define LINK_H

#include "Eigen/Dense"
#include "math_type_define.h"
#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h>
#include <mutex>

extern std::mutex mtx_rbdl;

struct Com
{
  double mass;
  Eigen::Vector3d pos;
  Eigen::Vector3d vel;
  Eigen::Vector3d accel;
  Eigen::Vector3d angular_momentum;
  Eigen::Vector2d ZMP;
  Eigen::Vector2d CP;
};

class Link
{
public:
  // Update link i of rbdl link id. name : link name, mass : link mass, xipos : local center of mass position
  void initialize(RigidBodyDynamics::Model &model_, int id_, std::string name_, double mass, Eigen::Vector3d &local_com_position);

  // Update COM jacobian
  void COM_Jac_Update(RigidBodyDynamics::Model &model_, Eigen::VectorQVQd &q_virtual_);

  // Update xpos, xipos, rotm.
  void pos_Update(RigidBodyDynamics::Model &model_, Eigen::VectorQVQd &q_virtual_);

  // Set Contact point, Contact jacobian
  void Set_Contact(RigidBodyDynamics::Model &model_, Eigen::VectorQVQd &q_virtual_, Eigen::Vector3d &Contact_position);

  // Set Contact point, Contact jacobian
  void Set_Contact(Eigen::VectorQVQd &q_virtual_, Eigen::Vector3d &Contact_position);

  // update Jacobian matrix of local position at link.
  void Set_Jacobian(RigidBodyDynamics::Model &model_, Eigen::VectorQVQd &q_virtual_, Eigen::Vector3d &Jacobian_position);

  // update link velocity(6D, translation and rotation) from jacobian matrix Jac.
  void vw_Update(Eigen::VectorVQd &q_dot_virtual);

  // set link Trajectory of id i.
  void Set_Trajectory(Eigen::Vector3d position_desired, Eigen::Vector3d velocity_desired, Eigen::Matrix3d rotation_desired, Eigen::Vector3d rotational_velocity_desired);

  // set realtime trajectory of link from quintic spline.
  void Set_Trajectory_from_quintic(double current_time, double start_time, double end_time, Eigen::Vector3d pos_desired);

  // set realtime trajectory of link from quintic spline.
  void Set_Trajectory_from_quintic(double current_time, double start_time, double end_time, Eigen::Vector3d pos_init, Eigen::Vector3d pos_desired);

  // set realtime trajectory of rotation of link
  void Set_Trajectory_rotation(double current_time, double start_time, double end_time, Eigen::Matrix3d rot_desired, bool local_);

  // set link initial position and rotation. initial position for task control.
  void Set_initpos();

  //constant variables
  int id;
  double Mass;
  std::string name;
  //local COM position of body
  Eigen::Vector3d COM_position;
  //inertial matrix
  Eigen::Matrix3d inertia;
  //local contact point
  Eigen::Vector3d contact_point;

  //changing variables
  //rotation matrix
  Eigen::Matrix3d Rotm;

  //global position of body
  Eigen::Vector3d xpos;

  //global COM position of body
  Eigen::Vector3d xipos;

  //global position of contact point at body
  Eigen::Vector3d xpos_contact;

  //cartesian velocity of body
  Eigen::Vector3d v;

  //rotational velocity of body
  Eigen::Vector3d w;

  Eigen::Matrix6Vd Jac_point;
  Eigen::Matrix6Vd Jac;
  Eigen::Matrix6Vd Jac_COM;
  Eigen::Matrix3Vd Jac_COM_p;
  Eigen::Matrix3Vd Jac_COM_r;
  Eigen::Matrix6Vd Jac_Contact;

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
  Eigen::Vector3d v_init;
  Eigen::Matrix3d rot_init;

private:
  Eigen::MatrixXd j_temp;
  Eigen::MatrixXd j_temp2;
  RigidBodyDynamics::Model *model;
};

std::ostream &operator<<(std::ostream &out, const Link &link);
#endif