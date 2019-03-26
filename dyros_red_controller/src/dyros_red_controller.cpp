#include <ros/ros.h>
#include "dyros_red_controller/state_manager.h"
#include "dyros_red_controller/dynamics_manager.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dyros_red_controller");
    ros::NodeHandle nh;

    DataContainer dc; //robot data container

    dc.nh = nh;

    dc.hz = 500; //frequency should be divisor of a million (timestep must be integer)
    dc.state_hz = 2000;

    dc.timeStep = std::chrono::microseconds((int)(1000000 / dc.hz));
    dc.state_timestep = std::chrono::microseconds((int)(1000000 / dc.state_hz));

    StateManager stm(dc);
    DynamicsManager dym(dc);

    std::thread state_thread(&StateManager::StateUpdateThread, &stm);
    std::thread dynamics_thread(&DynamicsManager::DynamicsUpdateThread, &dym);

    state_thread.join();
    dynamics_thread.join();

    return 0;
}
