#pragma once

#include "types.h"

#include <opencv2/core.hpp>

#include <string>

void setHUDState(const std::string &state);

void clearHUDState();

void paintHUD(cv::Mat &canvas, const Robot &rb, int delay);
