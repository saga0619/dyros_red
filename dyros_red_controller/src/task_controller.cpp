#include "dyros_red_controller/task_controller.h"
#include "dyros_red_controller/dyros_red_model.h"

namespace dyros_red_controller
{

constexpr unsigned int TaskController::PRIORITY;

void TaskController::compute()
{
  // update state
  //
}

/*
void TaskController::setTarget(DyrosRedModel::EndEffector ee, Eigen::Isometry3d target, double start_time, double end_time)
{
  start_transform_[ee] = model_.getCurrentTrasmfrom(ee);
  target_transform_[ee] = target;
  start_time_[ee] = start_time;
  end_time_[ee] = end_time;
  x_prev_[ee] = start_transform_[ee].translation();
  target_arrived_[ee] = false;
}
void TaskController::setTarget(DyrosRedModel::EndEffector ee, Eigen::Isometry3d target, double duration)
{
  setTarget(ee, target, control_time_, control_time_ + duration);
}
void TaskController::setEnable(DyrosRedModel::EndEffector ee, bool enable)
{
  ee_enabled_[ee] = enable;
}



void TaskController::updateControlMask(unsigned int *mask)
{
  unsigned int index = 0;
  for(int i=0; i<total_dof_; i++)
  {
    if(i < 6)
    {
      index = 0;
    }
    else if (i < 6 + 6)
    {
      index = 1;
    }
    else if (i < 6 + 6 + 2)
    {
      continue; // waist
    }
    else if (i < 6 + 6 + 2 + 7)
    {
      index = 2;
    }
    else if (i < 6 + 6 + 2 + 7 + 7)
    {
      index = 3;
    }

    if(ee_enabled_[index])
    {
      if (mask[i] >= PRIORITY * 2)
      {
        // Higher priority task detected
        ee_enabled_[index] = false;
        target_transform_[index] = model_.getCurrentTrasmfrom((DyrosRedModel::EndEffector)index);
        end_time_[index] = control_time_;
        if (index < 2)  // Legs
        {
          desired_q_.segment<6>(model_.joint_start_index_[index]) = current_q_.segment<6>(model_.joint_start_index_[index]);
        }
        else
        {
          desired_q_.segment<7>(model_.joint_start_index_[index]) = current_q_.segment<7>(model_.joint_start_index_[index]);
        }
        //setTarget((DyrosRedModel::EndEffector)index, model_.getCurrentTrasmfrom((DyrosRedModel::EndEffector)index), 0); // Stop moving
        target_arrived_[index] = true;
      }
      mask[i] = (mask[i] | PRIORITY);
    }
    else
    {
      mask[i] = (mask[i] & ~PRIORITY);
      //setTarget((DyrosRedModel::EndEffector)index, model_.getCurrentTrasmfrom((DyrosRedModel::EndEffector)index), 0); // Stop moving
      target_arrived_[index] = true;
    }
  }
}


*/


void TaskController::writeDesired(const unsigned int *mask, VectorQd& desired_q)
{
  for(unsigned int i=0; i<total_dof_; i++)
  {
    if( mask[i] >= PRIORITY && mask[i] < PRIORITY * 2 )
    {
      desired_q(i) = desired_q_(i);
    }
  }
}

}
