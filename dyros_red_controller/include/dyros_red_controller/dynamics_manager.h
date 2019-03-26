#include "dyros_red_controller/data_container.h"

class DynamicsManager
{
public:
  DynamicsManager(DataContainer &dc_global);
  void DynamicsUpdateThread();
  DataContainer *dc;
};
