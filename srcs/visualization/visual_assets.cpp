#include "visual_assets.h"

#include <opencv2/opencv.hpp>

using namespace cv;

namespace
{
    Mat iconGuard;
    Mat iconVehicle;
    Mat iconRandom;
}

void loadVisualizationAssets()
{
    iconGuard = imread("assets/policemen.png", IMREAD_UNCHANGED);
    iconVehicle = imread("assets/jeep.png", IMREAD_UNCHANGED);
    iconRandom = imread("assets/question.png", IMREAD_UNCHANGED);
}

const Mat& guardIcon()
{
    return iconGuard;
}

const Mat& vehicleIcon()
{
    return iconVehicle;
}

const Mat& randomIcon()
{
    return iconRandom;
}
