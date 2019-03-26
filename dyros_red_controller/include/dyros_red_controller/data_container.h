#ifndef DATA_CONTAINER_H
#define DATA_CONTAINER_H

#include <chrono>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <thread>
#include <mutex>
#include <future>

class DataContainer
{
public:
  bool simulation = true;

  double controltime;
  double hz;
  std::chrono::microseconds timeStep;

  double state_hz;
  std::chrono::microseconds state_timestep;
  bool check = false;

  ros::NodeHandle nh;
};

#endif