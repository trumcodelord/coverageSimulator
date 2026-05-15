#include "entity_renderer.h"

#include "dynamic_obstacle.h"
#include "image_utils.h"
#include "visual_assets.h"
#include "visual_layout.h"

#include <opencv2/opencv.hpp>

#include <algorithm>

using namespace std;
using namespace cv;

namespace
{
    Scalar getTrailColor(int cnt)
    {
        if (cnt <= 1)
            return Scalar(0, 144, 255);

        if (cnt <= 3)
        {
            int g = 144 - (cnt - 1) * 60;

            if (g < 0)
                g = 0;

            return Scalar(0, g, 255);
        }

        int r = 255 - (cnt - 4) * 30;

        if (r < 0)
            r = 0;

        return Scalar(0, 0, r);
    }

    double vehicleRotationAngleDeg(const DynamicObstacle &obs)
    {
        if (obs.dir == 0) return 180.0;   // down
        if (obs.dir == 1) return -90.0;   // right
        if (obs.dir == 2) return 0.0;     // up
        if (obs.dir == 3) return 90.0;    // left

        return 0.0;
    }
}

void paintRobot(Mat &canvas, const Robot &rb)
{
    Point center = visualCellCenter(rb.pos.r, rb.pos.c);

    int cellSize = visualCellSize();
    int radius = max(4, cellSize / 6);
    int coreRadius = max(2, cellSize / 20);
    int thickness = max(1, cellSize / 15);

    circle(canvas, center, radius, Scalar(0, 0, 255), FILLED);
    circle(canvas, center, radius, Scalar(0, 0, 0), 1);
    circle(canvas, center, coreRadius, Scalar(255, 255, 255), FILLED);

    Cell next;

    if (rb.pathID < (int)rb.path.size())
    {
        next = rb.path[rb.pathID];
    }
    else if (rb.trail.size() >= 2)
    {
        Cell prev = rb.trail[rb.trail.size() - 2];

        next = {
            rb.pos.r + (rb.pos.r - prev.r),
            rb.pos.c + (rb.pos.c - prev.c)
        };
    }
    else
    {
        return;
    }

    int dx = next.c - rb.pos.c;
    int dy = next.r - rb.pos.r;

    int arrowLen = radius + max(6, cellSize / 4);
    Point nose = center;

    if (dx == 1) nose.x += arrowLen;
    else if (dx == -1) nose.x -= arrowLen;
    else if (dy == 1) nose.y += arrowLen;
    else if (dy == -1) nose.y -= arrowLen;
    else return;

    Point start = center;

    if (dx == 1) start.x += radius;
    else if (dx == -1) start.x -= radius;
    else if (dy == 1) start.y += radius;
    else if (dy == -1) start.y -= radius;

    arrowedLine(
        canvas,
        start,
        nose,
        Scalar(250, 0, 100),
        thickness,
        LINE_AA,
        0,
        0.4
    );
}

void paintPath(Mat &canvas, const Robot &rb)
{
    if ((int)rb.path.size() < 2)
        return;

    Scalar pathColor(255, 120, 0);

    for (int i = max(1, rb.pathID); i < (int)rb.path.size(); i++)
    {
        Cell a = rb.path[i - 1];
        Cell b = rb.path[i];

        Point p1 = visualCellCenter(a.r, a.c);
        Point p2 = visualCellCenter(b.r, b.c);

        arrowedLine(canvas, p1, p2, pathColor, 2);
    }
}

void paintTrail(Mat &canvas, const Robot &rb)
{
    if (rb.trail.size() < 2)
        return;

    for (int i = 1; i < (int)rb.trail.size(); i++)
    {
        Cell a = rb.trail[i - 1];
        Cell b = rb.trail[i];

        Point p1 = visualCellCenter(a.r, a.c);
        Point p2 = visualCellCenter(b.r, b.c);

        Edge e(a, b);
        int cnt = 1;

        auto it = rb.edgeCount.find(e);

        if (it != rb.edgeCount.end())
            cnt = it->second;

        Scalar color = getTrailColor(cnt);
        int thickness = 3;

        arrowedLine(
            canvas,
            p1,
            p2,
            Scalar(80, 80, 80),
            thickness + 2,
            LINE_AA,
            0,
            0.1
        );

        arrowedLine(
            canvas,
            p1,
            p2,
            color,
            thickness,
            LINE_AA,
            0,
            0.1
        );
    }
}

void paintDynamicObstacles(Mat &canvas)
{
    for (const auto &obs : obstacles)
    {
        Point center = visualWorldCenter(obs.x, obs.y);

        if (obs.type == ObstacleType::GUARD)
        {
            int size = max(18, (int)(visualCellSize() * 0.85));
            overlayImage(canvas, guardIcon(), center, size);
        }
        else if (obs.type == ObstacleType::VEHICLE)
        {
            int size = max(20, (int)(visualCellSize() * 1.10));

            Mat rotatedVehicle = rotateIconForOverlay(
                vehicleIcon(),
                size,
                vehicleRotationAngleDeg(obs)
            );

            overlayImage(canvas, rotatedVehicle, center, size);
        }
        else if (obs.type == ObstacleType::RANDOM)
        {
            int size = max(16, (int)(visualCellSize() * 0.75));
            overlayImage(canvas, randomIcon(), center, size);
        }
    }
}
