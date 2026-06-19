#pragma once

#include <opencv2/core.hpp>

void paintStaticMapLayer(cv::Mat &canvas);

void paintDynamicMapOverlay(cv::Mat &canvas, bool showLogicalCoverage = true);

void paintMapCells(cv::Mat &canvas, bool showLogicalCoverage = true);

void paintGridLines(cv::Mat &canvas);

void paintCoordinateHeaders(cv::Mat &canvas);
