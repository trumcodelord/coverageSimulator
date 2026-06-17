#pragma once

#include "coverage_context.h"
#include "types.h"

#include <string>

void initWindow();
void closeWindow();
void drawFrame(const Robot &rb, const CoverageContext &ctx, bool showPath, int delay);
void waitFrame(int delay);

bool saveCurrentFrame(const std::string &filename);

void setHUDState(const std::string &state);
void clearHUDState();

void pushHUDEvent(const std::string &event);
void clearHUDEvents();

// Debug/test-only speed toggle.
// Space cycles between 1x, 5x, and 10x simulation speed.
int testSpeedMultiplier();
bool isTestSpeedEnabled();
