#pragma once

#include <opencv2/core.hpp>

void paintMapCells(cv::Mat &canvas, bool showLogicalCoverage = true);

void paintGridLines(cv::Mat &canvas);

void paintCoordinateHeaders(cv::Mat &canvas);
