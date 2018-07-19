#ifndef TASK_CONTROLLER_H
#define TASK_CONTROLLER_H

#include <fstream>
#include <array>
#include <Eigen/Geometry>
#include "dyros_red_controller/dyros_red_model.h"
#include "math_type_define.h"

namespace dyros_red_controller
{

class TaskController
{
public:

  static constexpr unsigned int PRIORITY = 16;  ///< Joint priority

  TaskController(DyrosRedModel& model, const VectorQd& current_q, const double hz, const double& control_time) :
    total_dof_(DyrosRedModel::MODEL_DOF), model_(model),
    current_q_(current_q), hz_(hz), control_time_(control_time),
    start_time_{}, end_time_{}, target_arrived_{true,true,true,true} {
    //debug_.open("/home/suhan/jet_test.txt");
  }
  void compute();
  //void setTarget(DyrosRedModel::EndEffector ee, Eigen::Isometry3d target, double start_time, double end_time);
  //void setTarget(DyrosRedModel::EndEffector ee, Eigen::Isometry3d target, double duration);
  //void setEnable(DyrosRedModel::EndEffector ee, bool enable);
  //void updateControlMask(unsigned int *mask);
  void writeDesired(const unsigned int *mask, VectorQd& desired_q);
  //void setTarget(DyrosRedModel::EndEffector ee, )
private:

  //void taskCLIKControl(DyrosRedModel::EndEffector ee);

  unsigned int total_dof_;

  bool ee_enabled_[4];

  // Eigen::Isometry3d current_transform_[4]; // --> Use model_.getCurrentTransform()

  // motion time
  const double hz_;
  const double &control_time_; // updated by control_base
  double start_time_[4];
  double end_time_[4];
  bool target_arrived_[4];

  Eigen::Isometry3d start_transform_[4];
  Eigen::Isometry3d previous_transform_[4];
  Eigen::Isometry3d desired_transform_[4];
  Eigen::Isometry3d target_transform_[4];

  Eigen::Vector3d start_x_dot_[4];
  Eigen::Vector3d x_prev_[4];  //< Previous x

  DyrosRedModel &model_;

  VectorQd desired_q_;
  const VectorQd &current_q_;  // updated by control_base


  //std::ofstream debug_;

};


}
#endif // TASK_CONTROLLER_H
