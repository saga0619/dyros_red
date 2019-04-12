#include <ros/ros.h>
#include "dyros_red_controller/red_controller.h"
#include "dyros_red_controller/terminal.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dyros_red_controller");
    DataContainer dc;

    std::string mode;

    dc.nh.param<std::string>("run_mode", mode, "default");
    Tui tui(dc);
    tui.ReadAndPrint(3, 0, "ascii0");

    if (mode == "simulation")
    {
        mvprintw(20, 30, " :: SIMULATION MODE :: ");
        refresh();
        wait_for_ms(1000);
    }
    else if (mode == "realrobot")
    {
        mvprintw(20, 30, " :: REAL ROBOT MODE :: ");
        refresh();
        wait_for_ms(1000);
    }
    else if (mode == "default")
    {
        wait_for_keypress();
        erase();
        tui.ReadAndPrint(0, 0, "red");
        refresh();
    }
    else
    {
        mvprintw(20, 10, " !! SOMETHING WRONG :: Unidentified ROS Param! Press Any Key to End");
        refresh();
        wait_for_keypress();
        endwin();
        return 0;
    }

    int menu_slc = 0;
    while (mode == "default")
    {
        if (menu_slc == 0)
        {
            attron(COLOR_PAIR(2));
            mvprintw(3, 35, "SIMULATION");
            attroff(COLOR_PAIR(2));
            mvprintw(5, 35, "REALROBOT");
            mvprintw(7, 35, "TEST");
            mvprintw(9, 35, "EXIT");
        }
        else if (menu_slc == 1)
        {
            mvprintw(3, 35, "SIMULATION");
            attron(COLOR_PAIR(2));
            mvprintw(5, 35, "REALROBOT");
            attroff(COLOR_PAIR(2));
            mvprintw(7, 35, "TEST");
            mvprintw(9, 35, "EXIT");
        }
        else if (menu_slc == 2)
        {
            mvprintw(3, 35, "SIMULATION");
            mvprintw(5, 35, "REALROBOT");
            attron(COLOR_PAIR(2));
            mvprintw(7, 35, "TEST");
            attroff(COLOR_PAIR(2));
            mvprintw(9, 35, "EXIT");
        }
        else if (menu_slc == 3)
        {
            mvprintw(3, 35, "SIMULATION");
            mvprintw(5, 35, "REALROBOT");
            mvprintw(7, 35, "TEST");
            attron(COLOR_PAIR(2));
            mvprintw(9, 35, "EXIT");
            attroff(COLOR_PAIR(2));
        }

        int ch = getch();
        if (ch == 10)
        {
            if (menu_slc == 0)
            {
                mvprintw(16, 10, "SIMULATION MODE! ");
                mode = "simulation";
            }
            else if (menu_slc == 1)
            {
                mvprintw(16, 10, "REAL ROBOT IS NOT READY");
                mode = "realrobot";
            }
            else if (menu_slc == 2)
            {
                mvprintw(16, 10, "TEST MODE !");
                mode = "testmode";
            }
            else if (menu_slc == 3)
            {
                erase();
                endwin();
                return 0;
            }
            break;
        }
        else if (ch == KEY_DOWN)
        {
            menu_slc++;
            if (menu_slc > 3)
                menu_slc = 0;
        }
        else if (ch == KEY_UP)
        {
            menu_slc--;
            if (menu_slc < 0)
                menu_slc = 3;
        }

        wait_for_ms(10);
        refresh();
    }
    erase();
    tui.ReadAndPrint(0, 0, "red");
    refresh();

    dc.ncurse_mode = false;

    if (!dc.ncurse_mode)
        endwin();

    bool simulation = true;
    dc.dym_hz = 500; //frequency should be divisor of a million (timestep must be integer)
    dc.stm_hz = 4000;
    dc.dym_timestep = std::chrono::microseconds((int)(1000000 / dc.dym_hz));
    dc.stm_timestep = std::chrono::microseconds((int)(1000000 / dc.stm_hz));

    MujocoInterface stm(dc);
    DynamicsManager dym(dc);
    RedController rc(dc, stm, dym);

    std::thread thread[4];
    if (mode == "simulation")
    {
        thread[0] = std::thread(&RedController::stateThread, &rc);
        thread[1] = std::thread(&RedController::dynamicsThreadHigh, &rc);
        thread[2] = std::thread(&RedController::dynamicsThreadLow, &rc);
        thread[3] = std::thread(&RedController::tuiThread, &rc);
        bool tj[4];
        while (ros::ok())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            for (int i = 0; i < 4; i++)
            {
                if (thread[i].joinable())
                {
                    if (dc.ncurse_mode)
                    {
                        mvprintw(3 + 2 * i, 35, "Thread %d End", i);
                        refresh();
                    }
                    else
                    {
                        std::cout << "Thread " << i << " End!" << std::endl;
                    }
                }
            }
            if (thread[0].joinable() && thread[1].joinable() && thread[2].joinable() && thread[3].joinable())
            {
                break;
            }
        }

        for (int i = 0; i < 4; i++)
        {
            thread[i].join();
        }
    }
    else if (mode == "realrobot")
    {
    }
    else if (mode == "testmode")
    {
        thread[0] = std::thread(&StateManager::testThread, &stm);
        thread[1] = std::thread(&DynamicsManager::testThread, &dym);
        thread[2] = std::thread(&RedController::tuiThread, &rc);

        for (int i = 0; i < 3; i++)
        {
            thread[i].join();
            if (dc.ncurse_mode)
            {
                mvprintw(3 + 2 * i, 35, "Thread %d End", i);
                refresh();
            }
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
