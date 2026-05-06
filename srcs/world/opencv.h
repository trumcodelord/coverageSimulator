#pragma once

#include "coverage.h"
#include <string>

void initWindow();
void closeWindow();
void drawFrame(const Robot &rb, bool showPath, int delay);
void waitFrame(int delay);

void setHUDState(const std::string &state);
void clearHUDState();
