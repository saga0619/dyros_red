#include "dyros_red_controller/state_manager.h"

StateManager::StateManager(DataContainer &dc_global)
    : dc(&dc_global)
{
    if (dc->simulation)
    {
    }
    else
    {
    }
}

void StateManager::StateUpdateThread(void)
{
    static std::chrono::high_resolution_clock::time_point StartTime = std::chrono::high_resolution_clock::now();
    static int ThreadCount = 0;

    while (ros::ok())
    {
        mtx.lock();
        ros::spinOnce();

        mtx.unlock();
        ThreadCount++;

        std::this_thread::sleep_until(StartTime + ThreadCount * dc->state_timestep);

        std::chrono::duration<double> e_time = std::chrono::high_resolution_clock::now() - StartTime;
        dc->controltime = e_time.count();
        std::cout << ThreadCount << " : time : " << dc->controltime << " s" << std::endl;
    }
}