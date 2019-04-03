#include "dyros_red_controller/data_container.h"

class DynamicsManager
{
private:
  DataContainer &dc;

public:
  DynamicsManager(DataContainer &dc_global);
  void dynamicsThread();
  void testThread();
};
