#include <ros/ros.h>
#include "dyros_red_controller/red_controller.h"
#include "dyros_red_controller/terminal.h"
#include <osal.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dyros_red_controller");
    DataContainer dc;
    system("beep");
    std::string mode;

    dc.nh.param<std::string>("/dyros_red_controller/run_mode", mode, "default");
    dc.nh.param("/dyros_red_controller/ncurse", dc.ncurse_mode, true);
    dc.nh.param<std::string>("/dyros_red_controller/ifname", dc.ifname, "enp0s31f6");
    dc.nh.param("/dyros_red_controller/ctime", dc.ctime, 250);

    dc.mode = mode;

    Tui tui(dc);
    std::string cs[10][10];
    tui.ReadAndPrint(3, 0, "ascii0");

    cs[0][0] = "SIMULATION";
    cs[1][0] = "REALROBOT";
    cs[2][0] = "TEST";
    cs[3][0] = "EXIT";
    std::string menu_slcc;

    if (mode == "simulation")
    {
        //mvprintw(20, 30, " :: SIMULATION MODE :: ");
        //refresh();
        //wait_for_ms(1000);
    }
    else if (mode == "realrobot")
    {
        //mvprintw(20, 30, " :: REAL ROBOT MODE :: ");
        //refresh();
        //wait_for_ms(1000);
    }
    else if (mode == "default")
    {
        wait_for_keypress();
        erase();
        tui.ReadAndPrint(0, 0, "red");
        refresh();

        menu_slcc = tui.menu(3, 35, 2, 0, 4, 1, cs);

        if (menu_slcc == "SIMULATION")
        {
            mvprintw(16, 10, "SIMULATION MODE! ");
            mode = "simulation";
        }
        else if (menu_slcc == "REALROBOT")
        {
            mvprintw(16, 10, "REAL ROBOT IS NOT READY");
            mode = "realrobot";
        }
        else if (menu_slcc == "TEST")
        {
            mvprintw(16, 10, "TEST MODE !");
            mode = "testmode";
        }
        else if (menu_slcc == "EXIT")
        {
            erase();
            endwin();
            return 0;
        }
    }
    else
    {
        rprint_sol(dc.ncurse_mode, 20, 10, " !! SOMETHING WRONG :: Unidentified ROS Param! Press Any Key to End");
        wait_for_keypress();
        endwin();
        return 0;
    }

    erase();
    tui.ReadAndPrint(0, 0, "red");
    refresh();

    if (!dc.ncurse_mode)
        endwin();

    bool simulation = true;
    dc.dym_hz = 500; //frequency should be divisor of a million (timestep must be integer)
    dc.stm_hz = 4000;
    dc.dym_timestep = std::chrono::microseconds((int)(1000000 / dc.dym_hz));
    dc.stm_timestep = std::chrono::microseconds((int)(1000000 / dc.stm_hz));

    std::thread thread[4];

    pthread_t pthr[4];

    if (mode == "simulation")
    {
        MujocoInterface stm(dc);
        DynamicsManager dym(dc);
        RedController rc(dc, stm, dym);

        thread[0] = std::thread(&RedController::stateThread, &rc);
        thread[1] = std::thread(&RedController::dynamicsThreadHigh, &rc);
        thread[2] = std::thread(&RedController::dynamicsThreadLow, &rc);
        thread[3] = std::thread(&RedController::tuiThread, &rc);

        for (int i = 0; i < 3; i++)
        {
            thread[i].join();
            rprint_sol(dc.ncurse_mode, 3 + 2 * i, 35, "Thread %d End", i);
        }
    }
    else if (mode == "realrobot")
    {
        RealRobotInterface rtm(dc);
        DynamicsManager dym(dc);
        RedController rc(dc, rtm, dym);

        //EthercatElmo Management Thread
        osal_thread_create(&pthr[0], NULL, (void *)&RealRobotInterface::ethercatCheck, &rtm);
        osal_thread_create_rt(&pthr[1], NULL, (void *)&RealRobotInterface::ethercatThread, &rtm);

        //Robot Controller Thread
        thread[0] = std::thread(&RedController::stateThread, &rc);
        thread[1] = std::thread(&RedController::dynamicsThreadHigh, &rc);
        thread[2] = std::thread(&RedController::dynamicsThreadLow, &rc);

        //For Additional functions ..
        thread[3] = std::thread(&RedController::tuiThread, &rc);

        pthread_join(pthr[1], NULL);
        for (int i = 1; i < 3; i++)
        {
            thread[i].join();
            rprint_sol(dc.ncurse_mode, 3 + 2 * i, 35, "Thread %d End", i);
        }
    }
    else if (mode == "testmode")
    {
        MujocoInterface stm(dc);
        DynamicsManager dym(dc);
        RedController rc(dc, stm, dym);
        thread[0] = std::thread(&StateManager::testThread, &stm);
        thread[1] = std::thread(&DynamicsManager::testThread, &dym);
        thread[2] = std::thread(&RedController::tuiThread, &rc);

        for (int i = 0; i < 3; i++)
        {
            thread[i].join();
            rprint_sol(dc.ncurse_mode, 3 + 2 * i, 35, "Thread %d End", i);
        }
    }

    if (dc.ncurse_mode)
        mvprintw(22, 10, "PRESS ANY KEY TO EXIT ...");

    while (ros::ok())
    {
        if (!(getch() == -1))
        {
            endwin();
            std::cout << "PROGRAM ENDED ! \n";
            return 0;
        }
    }
}
