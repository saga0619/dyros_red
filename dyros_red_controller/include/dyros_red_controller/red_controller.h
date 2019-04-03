#include "dyros_red_controller/dynamics_manager.h"
#include "dyros_red_controller/mujoco_interface.h"

class RedController
{
  public:
    RedController(DataContainer& datacontainer, StateManager& sm, DynamicsManager& dm);

    void stateThread();
    void dynamicsThreadLow();
    void dynamicsThreadHigh();

    DataContainer &dc;

    StateManager &s;
    DynamicsManager &r;
};