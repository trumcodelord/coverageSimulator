#pragma once
#include <opencv2/opencv.hpp>

cv::Mat rotateImageKeepSize(const cv::Mat &src, float angle);

cv::Mat rotateImageKeepSize(const cv::Mat &src, float angle);
void overlayImage(cv::Mat &background, const cv::Mat &icon, cv::Point center, int drawSize);
