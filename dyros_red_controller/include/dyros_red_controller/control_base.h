#ifndef _CONTROL_BASE_H
#define _CONTROL_BASE_H

// STD Library
#include <array>
#include <vector>

// System Library
#include <termios.h>

// ROS Library
#include <ros/ros.h>
#include <realtime_tools/realtime_publisher.h>
#include <tf/transform_datatypes.h>
#include <eigen_conversions/eigen_msg.h>

// ROS Messages
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <smach_msgs/SmachContainerStatus.h>

#include "dyros_red_msgs/JointSet.h"
#include "dyros_red_msgs/JointState.h"
#include "dyros_red_msgs/TaskCommand.h"
#include "dyros_red_msgs/JointCommand.h"
#include "dyros_red_msgs/Dynamicsinfo.h"
#include "dyros_red_msgs/ComCommand.h"
#include "dyros_red_msgs/WalkingCommand.h"
//#include "dyros_red_msgs/RecogCmd.h"
//#include "dyros_red_msgs/TaskCmdboth.h"

// User Library
#include "math_type_define.h"
#include "dyros_red_controller/dyros_red_model.h"
#include "dyros_red_controller/wholebody_controller.h"
#include "dyros_red_controller/task_controller.h"

// #include "Upperbody_Controller.h"

#include <tf/transform_broadcaster.h>

namespace dyros_red_controller
{

using namespace Eigen;
using namespace std;

class ControlBase
{

public:
  ControlBase(ros::NodeHandle &nh, double Hz);
  virtual ~ControlBase() {}
  // Default User Call function
  void parameterInitialize();     // initialize all parameter function(q,qdot,force else...)
  virtual void readDevice();      // read device means update all subscribed sensor data and user command
  virtual void update();          // update controller based on readdevice
  virtual void compute();         // compute algorithm and update all class object
  virtual void reflect();         // reflect next step actuation such as motor angle else
  virtual void writeDevice() = 0; // publish to actuate devices
  virtual void wait() = 0;        // wait

  //bool checkStateChanged();
  //void stateChangeEvent();
  const double getHz() { return Hz_; }
  double control_time_;
  double d_time_;
  double last_sim_time_;

  bool debug;
  bool rviz_pub;

protected:
  //unsigned int joint_id_[DyrosRedModel::MODEL_DOF];
  //unsigned int joint_id_inversed_[DyrosRedModel::MODEL_DOF];
  unsigned int control_mask_[DyrosRedModel::MODEL_DOF];

  int ui_update_count_;
  bool is_first_boot_;

  VectorQd q_;      // current q
  VectorQd q_dot_;  // current qdot
  VectorQd torque_; // current joint toruqe

  /* q_virtual
   * 0~2 global x,y,z translation of base body.
   * 3~5 x,y,z of Quaternion
   * final element : w of Quaternion
  **/
  VectorVquatQd q_virtual_;
  VectorVQd q_dot_virtual_;
  VectorVQd q_ddot_virtaul_;

  Vector6d left_foot_ft_;  // current left ft sensor values
  Vector6d right_foot_ft_; // current right ft sensor values
  Vector6d left_hand_ft_;  // current left hand ft sensor values
  Vector6d right_hand_ft_; // current right hand ft sensor values

  Vector2d left_foot_zmp_;  //
  Vector2d right_foot_zmp_; //

  tf::Quaternion imu_data_; ///< IMU data with filter

  Vector3d gyro_;        // current gyro sensor values
  Vector3d accelometer_; // current accelometer values

  Matrix3d pelvis_orientation_;
  Vector3d pelvis_position_;
  Vector3d Pelvis_linear_velocity_;
  Vector3d Pelvis_angular_velocity_;
  Quaterniond Pelvis_quaternion;

  double fc_ratio;
  Vector12d fc_redis;

  int task_number;
  VectorQd torque_contact_;
  VectorQd torque_task_;
  VectorQd torque_gravity_;
  VectorQd torque_joint_control_;
  VectorQd torque_dc_;

  VectorQd desired_q_; // current desired joint values
  VectorQd init_q_;
  VectorQd torque_desired;
  VectorQd torque_damping;
  VectorQd position_desired;

  MatrixXd J_task;
  VectorXd f_star, hand_f_desired;
  MatrixXd f_slc_matrix;

  Vector3d rdes, ldes;

  bool torque_control_mode = true;
  int compute_init = true;

  int task_mode;

  int total_dof_;

  DyrosRedModel model_;
  TaskController tc_;
  Wholebody_controller wc_;

  //ros::Publisher data_pub_;
  //sensor_msgs::JointState data_pub_msg_;

  // ROS
  // Subscriber
  ros::Subscriber com_command_sub;
  ros::Subscriber command_sub;

  // Publisher
  ros::Publisher point_pub;
  geometry_msgs::PolygonStamped point_pub_msgs;

  ros::Publisher dynamics_pub;
  dyros_red_msgs::Dynamicsinfo dynamics_pub_msgs;

  ros::Publisher joint_state_publisher_for_rviz;
  sensor_msgs::JointState joint_states_rviz;

  void command_cb(const std_msgs::StringConstPtr &msg);
  void com_command_cb(const dyros_red_msgs::ComCommandConstPtr &msg);

protected:
  string current_state_;
  //realtime_tools::RealtimePublisher<dyros_red_msgs::JointState> joint_state_pub_;

private:
  double Hz_; ///< control
  unsigned long tick_;

  string previous_state_;

  bool gravity_switch = false;
  bool task_switch = false;
  bool contact_switch = false;
  bool data_switch = false;
  bool QP_switch = false;
  bool QP_wall = false;
  bool left_foot_contact_ = true;
  bool right_foot_contact_ = true;
  bool walking_init = true;
  bool cgen_init = true;
  bool loop_cnged = false;

  int loop_temp = 0;
  int loop_ = 0;

  Vector2d zmp;

  Vector2d cx_init;
  Vector2d cv_init;

  Vector3d com_init;
  Matrix3d rot_init;

  Vector3d task_desired;
  Matrix3d rot_desired;

  //ros::Subscriber recog_point_sub_;
  // ros::Subscriber recog_cmd_sub_;

  // State Machine (SMACH)
  //realtime_tools::RealtimePublisher<std_msgs::String> smach_pub_;
  //ros::Subscriber smach_sub_;

  //void smachCallback(const smach_msgs::SmachContainerStatusConstPtr& msg);
  //void taskCommandCallback(const dyros_red_msgs::TaskCommandConstPtr& msg);
private:
  //void makeIDInverseList();
};

} // namespace dyros_red_controller

#endif
