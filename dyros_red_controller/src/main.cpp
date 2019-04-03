#include <ros/ros.h>
#include <dyros_red_controller/red_controller.h>

#include "dyros_red_controller/terminal.h"

void waitfor(int ch)
{
}

int main(int argc, char **argv)
{
    initscr();
    nodelay(stdscr, TRUE);
    noecho();
    curs_set(0);
    keypad(stdscr, TRUE);
    printw(welcome.c_str());

    while (1)
    {
        if (!(getch() == -1))
            break;
    }

    bool s1, s2, s3;
    s1 = false;
    s2 = false;
    s3 = true;

    int cnt = 0;
    erase();
    mvprintw(0, 0, red.c_str());
    refresh();
    s3 = true;
    start_color();
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

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        cnt++;
        refresh();
    }

    ros::init(argc, argv, "dyros_red_controller");
    DataContainer dc;
    //std::cout << std::endl  << welcome << std::endl;

    std::this_thread::sleep_for(std::chrono::seconds(2));
    //td::cout << red0 ;

    bool simulation = true;
    dc.dym_hz = 500; //frequency should be divisor of a million (timestep must be integer)
    dc.stm_hz = 4000;
    dc.dym_timestep = std::chrono::microseconds((int)(1000000 / dc.dym_hz));
    dc.stm_timestep = std::chrono::microseconds((int)(1000000 / dc.stm_hz));
    /*
    if (simulation)
    {
        std::cout << red0 << " simulation. " << std::endl;
    }
    std::cout << red1 << dc.stm_hz << "Hz and " << std::endl << red2 << dc.dym_hz << "Hz!" << std::endl;

    if ((!(1000000 % dc.dym_hz)) && (!(1000000 % dc.stm_hz)))
    {
        std::cout << red3 << "     Frequency settings are OK !" << std::endl;
    }
    else
    {
        std::cout << "Oops! something wrong with frequency setting. stoping controller" << std::endl;
        return 0;
    }*/
    //std::cout << red35 << "    Managers initialzation complete! " << std::endl;
    //std::cout << red4 << "    Here We Go! " << std::endl
    //          << red5 << std::endl;
    //std::cout << "\n\n";
    //StateManager stm(dc);
    /*
    if (menu_slc == 0)
    {
        MujocoInterface stm(dc);
        DynamicsManager dym(dc);
        std::thread state_thread(&StateManager::stateThread, &stm);
        std::thread dynamics_thread(&DynamicsManager::dynamicsThread, &dym);
        state_thread.join();
        dynamics_thread.join();
    }
    else if (menu_slc == 1)
    {
    }
    else if (menu_slc == 2)
    {
        MujocoInterface stm(dc);
        DynamicsManager dym(dc);
        erase();
        std::thread state_thread(&StateManager::testThread, &stm);
        std::thread dynamics_thread(&DynamicsManager::testThread, &dym);
        state_thread.join();
        dynamics_thread.join();
    }
    */
    //std::cout << "red controller exit \n";

    MujocoInterface stm(dc);
    DynamicsManager dym(dc);
    RedController rc(dc, stm, dym);

    std::thread thread0(&RedController::stateThread, &rc);
    std::thread thread1(&RedController::dynamicsThreadHigh, &rc);
    std::thread thread2(&RedController::dynamicsThreadLow, &rc);
    thread0.join();
    thread1.join();
    thread2.join();

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
