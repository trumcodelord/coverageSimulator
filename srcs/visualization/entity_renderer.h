#pragma once

#include "types.h"

#include <opencv2/core.hpp>

void paintRobot(cv::Mat &canvas, const Robot &rb);

void paintPath(cv::Mat &canvas, const Robot &rb);

void paintTrail(cv::Mat &canvas, const Robot &rb);

void paintDynamicObstacles(cv::Mat &canvas);
