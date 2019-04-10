#include "dyros_red_controller/terminal.h"
#include <ros/package.h>
#include <fstream>

Tui::Tui(DataContainer &dc_global) : dc(dc_global)
{
    initscr();
    nodelay(stdscr, TRUE);
    noecho();
    curs_set(0);
    keypad(stdscr, TRUE);
    start_color();

    ncurse_ = true;
    dc.ncurse_mode = true;
}

void Tui::tuiThread()
{ //100hz
    while (!dc.shutdown)
    {
        for (int i = 0; i < 40; i++)
        {
            if (dc.Tq_[i].update && ncurse_)
            {
                mtx.lock();
                mvprintw(dc.Tq_[i].x, dc.Tq_[i].y, dc.Tq_[i].text);
                dc.Tq_[i].update = false;
                mtx.unlock();
            }
            else if (dc.Tq_[i].update && (!ncurse_))
            {
                std::cout << dc.Tq_[i].text << std::endl;
                dc.Tq_[i].update = false;
            }
        }
        if (getch() == 'q')
        {
            dc.shutdown = true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void tui_addQue(DataContainer &dc_q, int y, int x, const char *str, ...)
{
    mtx.lock();
    va_list lst;
    va_start(lst, str);
    //char buff[1024];
    for (int i = 0; i < 50; i++)
    {

        if (!dc_q.Tq_[i].update)
        {
            dc_q.Tq_[i].update = true;
            dc_q.Tq_[i].y = y;
            dc_q.Tq_[i].x = x;
            sprintf(dc_q.Tq_[i].text, str, lst);
            break;
        }
    }
    va_end(lst);
    mtx.unlock();
}

void rprint(DataContainer &dc_q, int y, int x, const char *str, ...)
{
    mtx.lock();
    va_list lst;
    va_start(lst, str);
    //char buff[1024];
    for (int i = 0; i < 50; i++)
    {

        if (!dc_q.Tq_[i].update)
        {
            dc_q.Tq_[i].update = true;
            dc_q.Tq_[i].y = y;
            dc_q.Tq_[i].x = x;
            sprintf(dc_q.Tq_[i].text, str, lst);
            break;
        }
    }
    va_end(lst);
    mtx.unlock();
}

/*
void tui_addQue(DataContainer &dc_g, int y, int x, std::string text)
{
    mtx.lock();
    for (int i = 0; i < 100; i++)
    {
        if (!dc_g.Tq_[i].update)
        {
            dc_g.Tq_[i].update = true;
            dc_g.Tq_[i].y = y;
            dc_g.Tq_[i].x = x;
            dc_g.Tq_[i].text = text;
            break;
        }
    }
    mtx.unlock();
}*/

void Tui::addQue(int y, int x, std::string text)
{
    mtx.lock();
    for (int i = 0; i < 40; i++)
    {
        if (!dc.Tq_[i].update)
        {
            dc.Tq_[i].update = true;
            dc.Tq_[i].y = y;
            dc.Tq_[i].x = x;
            //dc.Tq_[i].text = text.c_str();
            break;
        }
    }
    mtx.unlock();
}

void Tui::ReadAndPrint(int y, int x, std::string buff)
{

    if (ncurse_)
    {
        std::string desc_package_path = ros::package::getPath("dyros_red_controller");
        std::string text_path = desc_package_path + "/ascii/" + buff;
        std::ifstream my_file(text_path.c_str());
        std::string wc, t_temp;

        if (my_file.is_open())
        {
            while (!my_file.eof())
            {
                getline(my_file, t_temp);
                mvprintw(y++, x, t_temp.c_str());
            }
        }
    }
}

void Tui::endTui()
{
    endwin();
}

void tui_start()
{
    initscr();
    nodelay(stdscr, TRUE);
    noecho();
    curs_set(0);
    keypad(stdscr, TRUE);
    start_color();
    //printw(welcome.c_str());
}

void wait_for_keypress()
{
    while (1)
    {
        if (!(getch() == -1))
            break;
    }
}

void wait_for_ms(int ms)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}
