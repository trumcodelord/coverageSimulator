#pragma once

#include "types.h"

#include <string>

void initWindow();
void closeWindow();
void drawFrame(const Robot &rb, bool showPath, int delay, RobotMode mode = NORMAL);
void waitFrame(int delay);

bool saveCurrentFrame(const std::string &filename);

void setHUDState(const std::string &state);
void clearHUDState();

void pushHUDEvent(const std::string &event);
void clearHUDEvents();