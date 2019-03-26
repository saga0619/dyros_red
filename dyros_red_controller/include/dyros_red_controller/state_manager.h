#include "dyros_red_controller/data_container.h"

std::mutex mtx;

class StateManager
{
public:
  StateManager(DataContainer &dc_global);
  void StateUpdateThread();

  ros::Subscriber sub_test;
  ros::Publisher pub_test;

  DataContainer *dc;
};
