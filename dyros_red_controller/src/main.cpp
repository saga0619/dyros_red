#include <ros/ros.h>
#include "dyros_red_controller/red_controller.h"
#include "dyros_red_controller/terminal.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dyros_red_controller");

    DataContainer dc;
    Tui tui(dc);

    //mvprintw(3, 0, wc.c_str());

    tui.ReadAndPrint(3, 0, "ascii0");

    refresh();
    wait_for_keypress();

    bool s3;

    erase();
    //mvprintw(0, 0, red.c_str());

    tui.ReadAndPrint(0, 0, "red");
    refresh();
    s3 = true;
    init_pair(1, -1, -1);
    init_pair(2, COLOR_BLACK, COLOR_WHITE);

    int menu_slc = 0;
    while (s3)
    {
        int ch = getch();
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
        if (ch == 10)
        {
            s3 = false;
            if (menu_slc == 0)
            {
                mvprintw(16, 10, "SIMULATION MODE! ");
            }
            else if (menu_slc == 1)
            {
                mvprintw(16, 10, "REAL ROBOT IS NOT READY");
            }
            else if (menu_slc == 2)
            {
                mvprintw(16, 10, "TEST MODE !");
            }
            else if (menu_slc == 3)
            {
                erase();
                endwin();
                return 0;
            }
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

    bool simulation = true;
    dc.dym_hz = 500; //frequency should be divisor of a million (timestep must be integer)
    dc.stm_hz = 4000;
    dc.dym_timestep = std::chrono::microseconds((int)(1000000 / dc.dym_hz));
    dc.stm_timestep = std::chrono::microseconds((int)(1000000 / dc.stm_hz));

    MujocoInterface stm(dc);
    DynamicsManager dym(dc);
    RedController rc(dc, stm, dym);

    if (menu_slc == 0)
    {
        std::thread thread0(&RedController::stateThread, &rc);
        std::thread thread1(&RedController::dynamicsThreadHigh, &rc);
        std::thread thread2(&RedController::dynamicsThreadLow, &rc);
        std::thread thread3(&RedController::tuiThread, &rc);

        thread0.join();
        mvprintw(0, 10, "t1");
        refresh();

        thread1.join();
        mvprintw(0, 14, "t2");
        refresh();

        thread2.join();
        mvprintw(0, 18, "t3");
        refresh();

        thread3.join();
        mvprintw(0, 22, "t4");
        refresh();
    }
    else if (menu_slc == 1)
    {
    }
    else if (menu_slc == 2)
    {
        std::thread state_thread(&StateManager::testThread, &stm);
        std::thread dynamics_thread(&DynamicsManager::testThread, &dym);
        state_thread.join();
        dynamics_thread.join();
    }
    mvprintw(22, 10, "PRESS ANY KEY TO EXIT ...");
    while (1)
    {
        if (!(getch() == -1))
        {
            endwin();
            std::cout << "PROGRAM ENDED ! \n";
            return 0;
        }
    }
}
