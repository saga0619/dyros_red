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
  static constexpr unsigned int PRIORITY = 16; ///< Joint priority

  TaskController(DyrosRedModel &model, const VectorQd &current_q, const double hz, const double &control_time) : total_dof_(DyrosRedModel::MODEL_DOF), model_(model),
                                                                                                                 current_q_(current_q), hz_(hz), control_time_(control_time)
  {
  }
  void compute();
  void initTask();
  void addTask();

  struct task_command
  {
    double command_time;
    int mode_;
    double f_ratio;
    double height;
    double traj_time;
  };

  task_command taskcommand_;

private:
  //void taskCLIKControl(DyrosRedModel::EndEffector ee);

  unsigned int total_dof_;

  bool ee_enabled_[4];

  // Eigen::Isometry3d current_transform_[4]; // --> Use model_.getCurrentTransform()

  // motion time
  const double hz_;
  const double &control_time_; // updated by control_base
  DyrosRedModel &model_;
  const VectorQd &current_q_; // updated by control_base

  //std::ofstream debug_;
};

} // namespace dyros_red_controller
#endif // TASK_CONTROLLER_H
