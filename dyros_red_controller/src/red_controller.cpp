#include "dyros_red_controller/red_controller.h"

RedController::RedController(DataContainer &datacontainer, StateManager &sm, DynamicsManager &dm) : dc(datacontainer), s(sm), r(dm)
{
}

void RedController::stateThread()
{
    s.testThread();
}

void RedController::dynamicsThreadHigh()
{
}

void RedController::dynamicsThreadLow()
{
}