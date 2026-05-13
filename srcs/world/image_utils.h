#pragma once
#include <opencv2/opencv.hpp>

void overlayImage(cv::Mat &background, const cv::Mat &icon, cv::Point center, int drawSize);
cv::Mat rotateIconForOverlay(const cv::Mat &src, int drawSize, double angleDeg);
