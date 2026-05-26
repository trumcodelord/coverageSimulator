#include "entity_renderer.h"

#include "coverage_timing.h"
#include "dynamic_obstacle.h"
#include "image_utils.h"
#include "visual_assets.h"
#include "visual_layout.h"

#include <opencv2/opencv.hpp>

#include <algorithm>
#include <cmath>

using namespace std;
using namespace cv;

namespace
{
    constexpr int ROBOT_MIN_MOVE_FRAMES = 1;
    constexpr int ROBOT_MAX_MOVE_FRAMES = 32;

    struct RobotVisualState
    {
        bool initialized = false;
        Cell targetLogicalPos = {0, 0};
        float x = 0.0f;
        float y = 0.0f;
        float fromX = 0.0f;
        float fromY = 0.0f;
        float toX = 0.0f;
        float toY = 0.0f;
        int frame = 0;
        int totalFrames = 1;
    };

    RobotVisualState &robotVisualState()
    {
        static RobotVisualState state;
        return state;
    }

    int ceilDivPositive(int a, int b)
    {
        return (a + b - 1) / b;
    }

    int moveFramesForMode(RobotMode mode)
    {
        int logicalStepMs = stepTicksForMode(mode) * simTickMs();
        int frames = ceilDivPositive(logicalStepMs, max(1, renderDelayMs()));

        return max(ROBOT_MIN_MOVE_FRAMES, min(ROBOT_MAX_MOVE_FRAMES, frames));
    }

    bool isRobotAnimating()
    {
        const RobotVisualState &state = robotVisualState();
        return state.initialized && state.frame < state.totalFrames;
    }

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
        if (obs.dir == 0) return 180.0;
        if (obs.dir == 1) return -90.0;
        if (obs.dir == 2) return 0.0;
        if (obs.dir == 3) return 90.0;

        return 0.0;
    }

    Cell robotHeadingTarget(const Robot &rb)
    {
        if (rb.pathID < (int)rb.path.size())
            return rb.path[rb.pathID];

        if (rb.trail.size() >= 2)
        {
            Cell prev = rb.trail[rb.trail.size() - 2];

            return {
                rb.pos.r + (rb.pos.r - prev.r),
                rb.pos.c + (rb.pos.c - prev.c)
            };
        }

        return {rb.pos.r - 1, rb.pos.c};
    }

    bool robotHeadingDelta(const Robot &rb, float &dr, float &dc)
    {
        const RobotVisualState &state = robotVisualState();

        if (isRobotAnimating())
        {
            dr = state.toX - state.fromX;
            dc = state.toY - state.fromY;

            if (fabs(dr) > 1e-5f || fabs(dc) > 1e-5f)
                return true;
        }

        Cell next = robotHeadingTarget(rb);
        dr = (float)(next.r - rb.pos.r);
        dc = (float)(next.c - rb.pos.c);

        return fabs(dr) > 1e-5f || fabs(dc) > 1e-5f;
    }

    double angleFromGridDelta(float dr, float dc)
    {
        if (fabs(dc) >= fabs(dr))
        {
            if (dc > 0.0f) return -90.0;
            if (dc < 0.0f) return 90.0;
        }

        if (dr > 0.0f) return 180.0;
        if (dr < 0.0f) return 0.0;

        return 0.0;
    }

    double robotRotationAngleDeg(const Robot &rb)
    {
        float dr = 0.0f;
        float dc = 0.0f;

        if (!robotHeadingDelta(rb, dr, dc))
            return 0.0;

        return angleFromGridDelta(dr, dc);
    }

    Point visualRobotCenter(const Robot &rb, RobotMode mode)
    {
        RobotVisualState &state = robotVisualState();

        if (!state.initialized)
        {
            state.initialized = true;
            state.targetLogicalPos = rb.pos;
            state.x = (float)rb.pos.r;
            state.y = (float)rb.pos.c;
            state.fromX = state.x;
            state.fromY = state.y;
            state.toX = state.x;
            state.toY = state.y;
            state.frame = 1;
            state.totalFrames = 1;
        }

        if (!(rb.pos == state.targetLogicalPos))
        {
            state.targetLogicalPos = rb.pos;
            state.fromX = state.x;
            state.fromY = state.y;
            state.toX = (float)rb.pos.r;
            state.toY = (float)rb.pos.c;
            state.frame = 0;
            state.totalFrames = moveFramesForMode(mode);
        }

        if (state.frame < state.totalFrames)
        {
            state.frame++;
            float t = (float)state.frame / (float)state.totalFrames;
            t = max(0.0f, min(1.0f, t));

            // Linear interpolation preserves the old perceived speed.
            // Ease-in/out would look nicer but makes the robot accelerate/decelerate visually.
            state.x = state.fromX + (state.toX - state.fromX) * t;
            state.y = state.fromY + (state.toY - state.fromY) * t;
        }
        else
        {
            state.x = (float)rb.pos.r;
            state.y = (float)rb.pos.c;
        }

        return visualWorldCenter(state.x, state.y);
    }

    void paintRobotFallback(Mat &canvas, const Robot &rb, Point center)
    {
        int cellSize = visualCellSize();
        int radius = max(4, cellSize / 5);
        int coreRadius = max(2, cellSize / 18);
        int thickness = max(1, cellSize / 15);

        circle(canvas, center, radius, Scalar(245, 153, 43), FILLED);
        circle(canvas, center, radius, Scalar(30, 60, 90), 2);
        circle(canvas, center, coreRadius, Scalar(255, 255, 255), FILLED);

        float dr = 0.0f;
        float dc = 0.0f;

        if (!robotHeadingDelta(rb, dr, dc))
            return;

        int arrowLen = radius + max(6, cellSize / 4);
        Point nose = center;

        if (fabs(dc) >= fabs(dr))
        {
            if (dc > 0.0f) nose.x += arrowLen;
            else if (dc < 0.0f) nose.x -= arrowLen;
        }
        else
        {
            if (dr > 0.0f) nose.y += arrowLen;
            else if (dr < 0.0f) nose.y -= arrowLen;
        }

        arrowedLine(
            canvas,
            center,
            nose,
            Scalar(30, 60, 90),
            thickness,
            LINE_AA,
            0,
            0.4
        );
    }

    void paintBaseCellBackground(Mat &canvas, const Robot &rb)
    {
        Point tl = visualCellTopLeft(rb.base.r, rb.base.c);
        int cellSize = visualCellSize();

        Rect cellRect(tl.x, tl.y, cellSize, cellSize);

        rectangle(canvas, cellRect, Scalar(235, 248, 255), FILLED);
        rectangle(canvas, cellRect, Scalar(255, 170, 40), max(2, cellSize / 14));
    }

    void paintBaseFallback(Mat &canvas, const Robot &rb)
    {
        Point center = visualCellCenter(rb.base.r, rb.base.c);

        int cellSize = visualCellSize();
        int half = max(5, (int)(cellSize * 0.22));

        Rect box(
            center.x - half,
            center.y - half,
            2 * half,
            2 * half
        );

        rectangle(canvas, box, Scalar(255, 255, 255), FILLED);
        rectangle(canvas, box, Scalar(0, 120, 255), max(1, cellSize / 15));

        Point roofA(center.x - half - cellSize / 14, center.y - half);
        Point roofB(center.x, center.y - half - cellSize / 5);
        Point roofC(center.x + half + cellSize / 14, center.y - half);

        line(canvas, roofA, roofB, Scalar(0, 120, 255), max(1, cellSize / 15), LINE_AA);
        line(canvas, roofB, roofC, Scalar(0, 120, 255), max(1, cellSize / 15), LINE_AA);
    }
}

void paintBase(Mat &canvas, const Robot &rb)
{
    paintBaseCellBackground(canvas, rb);

    Point center = visualCellCenter(rb.base.r, rb.base.c);
    int size = max(14, (int)(visualCellSize() * 0.62));

    if (!baseIcon().empty())
    {
        overlayImage(canvas, baseIcon(), center, size);
        return;
    }

    paintBaseFallback(canvas, rb);
}

void paintRobot(Mat &canvas, const Robot &rb, RobotMode mode)
{
    Point center = visualRobotCenter(rb, mode);
    int size = max(18, (int)(visualCellSize() * 0.90));

    if (!robotIcon().empty())
    {
        Mat rotatedRobot = rotateIconForOverlay(
            robotIcon(),
            size,
            robotRotationAngleDeg(rb)
        );

        overlayImage(canvas, rotatedRobot, center, size);
        return;
    }

    paintRobotFallback(canvas, rb, center);
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
    }
}