#pragma once

#include <opencv2/core.hpp>

#include <string>

const std::string& visualWindowName();

void updateVisualScreenSize(int width, int height);

void setupVisualLayout();

int visualCanvasWidth();
int visualCanvasHeight();
int visualCellSize();

cv::Point visualCellTopLeft(int r, int c);
cv::Point visualCellCenter(int r, int c);
cv::Point visualWorldCenter(float x, float y);

cv::Rect visualGridRect();
