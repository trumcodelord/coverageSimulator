#include "visual_assets.h"

#include <opencv2/opencv.hpp>

#include <string>
#include <vector>

using namespace cv;
using namespace std;

namespace
{
    Mat iconGuard;
    Mat iconVehicle;

    Mat iconBase;
    Mat iconRobot;

    Mat loadFirstAvailable(const vector<string> &paths)
    {
        for (const string &path : paths)
        {
            Mat img = imread(path, IMREAD_UNCHANGED);

            if (!img.empty())
                return img;
        }

        return Mat();
    }
}

void loadVisualizationAssets()
{
    iconGuard = imread("assets/policemen.png", IMREAD_UNCHANGED);
    iconVehicle = imread("assets/jeep.png", IMREAD_UNCHANGED);

    iconBase = loadFirstAvailable({
        "assets/base.png",
        "assets/base_station.png",
        "assets/base_station_bw.png"
    });

    iconRobot = loadFirstAvailable({
        "assets/robot.png",
        "assets/robot_rover_up.png",
        "assets/robot_rover_up_simple_color.png",
        "assets/robot_rover_up_simple_color_128.png"
    });
}

const Mat& guardIcon()
{
    return iconGuard;
}

const Mat& vehicleIcon()
{
    return iconVehicle;
}

const Mat& baseIcon()
{
    return iconBase;
}

const Mat& robotIcon()
{
    return iconRobot;
}
