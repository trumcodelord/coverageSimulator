#pragma once

#include "coverage_context.h"
#include "types.h"

#include <opencv2/core.hpp>

void paintBase(cv::Mat &canvas, const Robot &rb);

void paintRobot(cv::Mat &canvas, const Robot &rb, const CoverageContext &ctx);

void paintPath(cv::Mat &canvas, const Robot &rb);

void paintTrail(cv::Mat &canvas, const Robot &rb);

void paintDynamicObstacles(cv::Mat &canvas);