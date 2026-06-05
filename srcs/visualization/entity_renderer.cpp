#include "entity_renderer.h"

#include "dynamic_obstacle.h"
#include "grid.h"
#include "image_utils.h"
#include "robot_motion.h"
#include "visual_assets.h"
#include "visual_layout.h"

#include <opencv2/opencv.hpp>

#include <algorithm>
#include <cmath>

using namespace std;
using namespace cv;

namespace
{
    Mat trailLayer;
    Mat trailMask;
    int cachedTrailSize = 0;
    int cachedCellSize = 0;
    Size cachedCanvasSize;

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

    void resetTrailCache(const Mat &canvas)
    {
        trailLayer = Mat::zeros(canvas.size(), canvas.type());
        trailMask = Mat::zeros(canvas.size(), CV_8UC1);
        cachedTrailSize = 0;
        cachedCellSize = visualCellSize();
        cachedCanvasSize = canvas.size();
    }

    bool trailCacheNeedsReset(const Mat &canvas, const Robot &rb)
    {
        if (trailLayer.empty() || trailMask.empty())
            return true;

        if (cachedCanvasSize != canvas.size())
            return true;

        if (cachedCellSize != visualCellSize())
            return true;

        if (cachedTrailSize > (int)rb.trail.size())
            return true;

        return false;
    }

    void ensureTrailCache(const Mat &canvas, const Robot &rb)
    {
        if (trailCacheNeedsReset(canvas, rb))
            resetTrailCache(canvas);
    }

    void drawTrailSegmentOnCache(const Robot &rb, int index)
    {
        if (index <= 0 || index >= (int)rb.trail.size())
            return;

        int trailThickness = max(1, visualCellSize() / 18);
        double tipLength = 0.18;

        Cell a = rb.trail[index - 1];
        Cell b = rb.trail[index];

        Point p1 = visualCellCenter(a.r, a.c);
        Point p2 = visualCellCenter(b.r, b.c);

        Edge e(a, b);
        int cnt = 1;

        auto it = rb.edgeCount.find(e);

        if (it != rb.edgeCount.end())
            cnt = it->second;

        Scalar color = getTrailColor(cnt);

        arrowedLine(
            trailLayer,
            p1,
            p2,
            Scalar(80, 80, 80),
            trailThickness + 1,
            LINE_AA,
            0,
            tipLength
        );

        arrowedLine(
            trailMask,
            p1,
            p2,
            Scalar(255),
            trailThickness + 2,
            LINE_AA,
            0,
            tipLength
        );

        arrowedLine(
            trailLayer,
            p1,
            p2,
            color,
            trailThickness,
            LINE_AA,
            0,
            tipLength
        );

        arrowedLine(
            trailMask,
            p1,
            p2,
            Scalar(255),
            trailThickness + 1,
            LINE_AA,
            0,
            tipLength
        );
    }

    void updateTrailCache(const Mat &canvas, const Robot &rb)
    {
        ensureTrailCache(canvas, rb);

        if ((int)rb.trail.size() < 2)
        {
            cachedTrailSize = (int)rb.trail.size();
            return;
        }

        if (cachedTrailSize < 1)
            cachedTrailSize = 1;

        for (int i = cachedTrailSize; i < (int)rb.trail.size(); i++)
            drawTrailSegmentOnCache(rb, i);

        cachedTrailSize = (int)rb.trail.size();
    }

    double vehicleRotationAngleDeg(const DynamicObstacle &obs)
    {
        if (obs.dir == 0) return 180.0;
        if (obs.dir == 1) return -90.0;
        if (obs.dir == 2) return 0.0;
        if (obs.dir == 3) return 90.0;

        return 0.0;
    }

    bool pendingMoveDelta(const CoverageContext &ctx, float &dr, float &dc)
    {
        if (!ctx.pendingMove.active)
            return false;

        dr = (float)(ctx.pendingMove.to.r - ctx.pendingMove.from.r);
        dc = (float)(ctx.pendingMove.to.c - ctx.pendingMove.from.c);
        return fabs(dr) > 1e-5f || fabs(dc) > 1e-5f;
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

    bool robotHeadingDelta(const Robot &rb, const CoverageContext &ctx, float &dr, float &dc)
    {
        if (pendingMoveDelta(ctx, dr, dc))
            return true;

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

    double robotRotationAngleDeg(const Robot &rb, const CoverageContext &ctx)
    {
        float dr = 0.0f;
        float dc = 0.0f;

        if (!robotHeadingDelta(rb, ctx, dr, dc))
            return 0.0;

        return angleFromGridDelta(dr, dc);
    }

    Point robotVisualCenter(const Robot &rb, const CoverageContext &ctx)
    {
        if (!ctx.pendingMove.active)
            return visualCellCenter(rb.pos.r, rb.pos.c);

        float t = pendingRobotMoveProgress(ctx);
        float r = ctx.pendingMove.from.r +
                  (ctx.pendingMove.to.r - ctx.pendingMove.from.r) * t;
        float c = ctx.pendingMove.from.c +
                  (ctx.pendingMove.to.c - ctx.pendingMove.from.c) * t;

        return visualWorldCenter(r, c);
    }

    void paintRobotFallback(Mat &canvas, const Robot &rb, const CoverageContext &ctx, Point center)
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

        if (!robotHeadingDelta(rb, ctx, dr, dc))
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

void paintRobot(Mat &canvas, const Robot &rb, const CoverageContext &ctx)
{
    Point center = robotVisualCenter(rb, ctx);
    int size = max(18, (int)(visualCellSize() * 0.90));

    if (!robotIcon().empty())
    {
        Mat rotatedRobot = rotateIconForOverlay(
            robotIcon(),
            size,
            robotRotationAngleDeg(rb, ctx)
        );

        overlayImage(canvas, rotatedRobot, center, size);
        return;
    }

    paintRobotFallback(canvas, rb, ctx, center);
}

void paintPath(Mat &canvas, const Robot &rb)
{
    if ((int)rb.path.size() < 2)
        return;

    Scalar pathColor(255, 140, 30);

    for (int i = max(1, rb.pathID); i < (int)rb.path.size(); i++)
    {
        Cell a = rb.path[i - 1];
        Cell b = rb.path[i];

        Point p1 = visualCellCenter(a.r, a.c);
        Point p2 = visualCellCenter(b.r, b.c);

        arrowedLine(canvas, p1, p2, pathColor, 2, LINE_AA, 0, 0.12);
    }
}

void paintTrail(Mat &canvas, const Robot &rb)
{
    updateTrailCache(canvas, rb);

    if (trailLayer.empty() || trailMask.empty())
        return;

    trailLayer.copyTo(canvas, trailMask);
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
