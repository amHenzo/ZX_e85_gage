#ifndef DISPLAY_H
#define DISPLAY_H

#include <U8g2lib.h>

void displayInit();
U8G2 &displayDevice();
void displayRenderDashboard();
void displayStartTask();

#endif
