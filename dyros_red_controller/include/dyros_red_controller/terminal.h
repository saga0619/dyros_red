#ifndef TERMINAL_H
#define TERMINAL_H
#include "dyros_red_controller/data_container.h"

#include <stdarg.h>

class Tui
{
public:
  Tui(DataContainer &dc_global);
  DataContainer &dc;
  int que;
  void que_clear();
  bool ncurse_;

  void ReadAndPrint(int y, int x, std::string buff);
  void addQue(int y, int x, std::string text);
  void testThread();
  void tuiThread();
  void endTui();
};

void tui_addQue(DataContainer &dc, int y, int x, const char *text, ...);
void tui_start();
//void tui_end();
void wait_for_keypress();
void wait_for_ms(int ms);

void rprint(DataContainer &dc, int y, int x, const char *str, ...);
void rprint(DataContainer &dc, bool clr_line, int y, int x, const char *str, ...);
void rprint(DataContainer &dc, const char *str, ...);

#endif