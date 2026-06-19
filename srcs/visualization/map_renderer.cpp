#include "map_renderer.h"

#include "dynamic_obstacle.h"
#include "grid.h"
#include "visual_layout.h"

#include <opencv2/opencv.hpp>

#include <algorithm>
#include <string>

using namespace cv;

namespace
{
    Mat coveredLayer;
    Mat coveredMask;
    int cachedCoveredCount = -1;
    int cachedRows = -1;
    int cachedCols = -1;
    int cachedCellSize = -1;
    Size cachedCanvasSize;

    bool isBaseCell(int r, int c)
    {
        return r == start.r && c == start.c;
    }

    Scalar blendColor(Scalar base, Scalar overlay, double alpha)
    {
        alpha = std::max(0.0, std::min(1.0, alpha));

        return Scalar(
            base[0] * (1.0 - alpha) + overlay[0] * alpha,
            base[1] * (1.0 - alpha) + overlay[1] * alpha,
            base[2] * (1.0 - alpha) + overlay[2] * alpha
        );
    }

    int grayForTerrainCost(int cost)
    {
        if (cost <= 1)
            return 245;

        if (cost == 2)
            return 215;

        if (cost == 3)
            return 180;

        if (cost == 4)
            return 155;

        if (cost == 5)
            return 130;

        return std::max(80, 130 - std::min(40, (cost - 5) * 8));
    }

    Scalar terrainColorForCell(int r, int c)
    {
        if (isStaticBlocked(r, c))
            return Scalar(45, 45, 45);

        int gray = grayForTerrainCost(baseTerrainCostAt(r, c));
        return Scalar(gray, gray, gray);
    }

    void paintBaseMarker(Mat &canvas, Point tl, Point br)
    {
        int s = visualCellSize();
        int thickness = std::max(2, s / 12);

        rectangle(
            canvas,
            tl,
            br,
            Scalar(255, 160, 0),
            thickness
        );

        Point center(
            tl.x + s / 2,
            tl.y + s / 2
        );

        int houseW = std::max(6, s / 3);
        int houseH = std::max(5, s / 4);

        Point roof[1][3] = {{
            Point(center.x - houseW / 2, center.y),
            Point(center.x, center.y - houseH),
            Point(center.x + houseW / 2, center.y)
        }};

        const Point* pts[1] = { roof[0] };
        int npts[] = { 3 };

        fillPoly(canvas, pts, npts, 1, Scalar(255, 140, 0));

        rectangle(
            canvas,
            Point(center.x - houseW / 3, center.y),
            Point(center.x + houseW / 3, center.y + houseH),
            Scalar(255, 180, 40),
            FILLED
        );
    }

    void fillCell(Mat &canvas, int r, int c, Scalar color)
    {
        Point tl = visualCellTopLeft(r, c);
        Point br(
            tl.x + visualCellSize(),
            tl.y + visualCellSize()
        );

        rectangle(canvas, tl, br, color, FILLED);
    }

    bool coverageCacheMatches(const Mat &canvas)
    {
        return !coveredLayer.empty() &&
               !coveredMask.empty() &&
               cachedRows == rows &&
               cachedCols == cols &&
               cachedCellSize == visualCellSize() &&
               cachedCanvasSize == canvas.size() &&
               cachedCoveredCount == coveredCellCount;
    }

    void resetCoverageCache(const Mat &canvas)
    {
        coveredLayer = Mat::zeros(canvas.size(), canvas.type());
        coveredMask = Mat::zeros(canvas.size(), CV_8UC1);

        cachedRows = rows;
        cachedCols = cols;
        cachedCellSize = visualCellSize();
        cachedCanvasSize = canvas.size();
        cachedCoveredCount = -1;
    }

    void rebuildCoverageCache(const Mat &canvas)
    {
        if (coveredLayer.empty() ||
            coveredMask.empty() ||
            cachedRows != rows ||
            cachedCols != cols ||
            cachedCellSize != visualCellSize() ||
            cachedCanvasSize != canvas.size())
        {
            resetCoverageCache(canvas);
        }

        coveredLayer.setTo(Scalar(0, 0, 0));
        coveredMask.setTo(Scalar(0));

        for (int r = 1; r <= rows; r++)
        {
            for (int c = 1; c <= cols; c++)
            {
                if (!isCoverageTargetCell(r, c) || !covered[r][c])
                    continue;

                Scalar color = blendColor(
                    terrainColorForCell(r, c),
                    Scalar(80, 210, 80),
                    0.28
                );

                fillCell(coveredLayer, r, c, color);
                fillCell(coveredMask, r, c, Scalar(255));
            }
        }

        cachedCoveredCount = coveredCellCount;
    }

    void ensureCoverageCache(const Mat &canvas)
    {
        if (!coverageCacheMatches(canvas))
            rebuildCoverageCache(canvas);
    }

    void paintDynamicBlockedCell(Mat &canvas, int r, int c)
    {
        if (!isCoverageTargetCell(r, c))
            return;

        Scalar color = blendColor(
            terrainColorForCell(r, c),
            Scalar(180, 105, 255),
            0.65
        );

        fillCell(canvas, r, c, color);
    }

    void paintBaseOverlay(Mat &canvas)
    {
        if (!isCoverageTargetCell(start.r, start.c))
            return;

        Scalar color = blendColor(
            terrainColorForCell(start.r, start.c),
            Scalar(255, 235, 180),
            0.20
        );

        Point tl = visualCellTopLeft(start.r, start.c);
        Point br(
            tl.x + visualCellSize(),
            tl.y + visualCellSize()
        );

        rectangle(canvas, tl, br, color, FILLED);
        paintBaseMarker(canvas, tl, br);
    }
}

void paintStaticMapLayer(Mat &canvas)
{
    // Map terrain and static walls do not change while a test is running.
    // They are cached by opencv.cpp and reused every frame.
    for (int r = 1; r <= rows; r++)
    {
        for (int c = 1; c <= cols; c++)
        {
            fillCell(canvas, r, c, terrainColorForCell(r, c));
        }
    }
}

void paintDynamicMapOverlay(Mat &canvas, bool showLogicalCoverage)
{
    if (showLogicalCoverage)
    {
        // Covered-cell visualization changes only when a new cell is marked
        // covered, not on every animation frame. Keep it in a layer and rebuild
        // only when coveredCellCount/layout changes.
        ensureCoverageCache(canvas);
        coveredLayer.copyTo(canvas, coveredMask);
    }

    // Dynamic obstacles are few; draw their current blocked cells directly
    // instead of scanning the whole map for dynamicBlocked[][] every frame.
    for (const DynamicObstacle &obs : obstacles)
    {
        if (!inBounds(obs.pos.r, obs.pos.c))
            continue;

        if (!isDynamicBlockedCell(obs.pos.r, obs.pos.c))
            continue;

        paintDynamicBlockedCell(canvas, obs.pos.r, obs.pos.c);
    }

    paintBaseOverlay(canvas);
}

void paintMapCells(Mat &canvas, bool showLogicalCoverage)
{
    paintStaticMapLayer(canvas);
    paintDynamicMapOverlay(canvas, showLogicalCoverage);
}

void paintGridLines(Mat &canvas)
{
    Scalar lineColor(100, 100, 100);

    Rect grid = visualGridRect();

    for (int r = 0; r <= rows; r++)
    {
        int y = grid.y + r * visualCellSize();

        line(
            canvas,
            Point(grid.x, y),
            Point(grid.x + grid.width, y),
            lineColor,
            1
        );
    }

    for (int c = 0; c <= cols; c++)
    {
        int x = grid.x + c * visualCellSize();

        line(
            canvas,
            Point(x, grid.y),
            Point(x, grid.y + grid.height),
            lineColor,
            1
        );
    }
}

void paintCoordinateHeaders(Mat &canvas)
{
    Rect grid = visualGridRect();
    int cellSize = visualCellSize();

    if (cellSize < 14)
        return;

    double fontScale = std::max(0.28, std::min(0.48, cellSize / 85.0));
    int thickness = 1;
    Scalar textColor(40, 40, 40);
    Scalar bgColor(245, 245, 245);

    int topBandHeight = std::max(14, cellSize / 3);
    int leftBandWidth = std::max(18, cellSize / 2);

    rectangle(
        canvas,
        Rect(grid.x, std::max(0, grid.y - topBandHeight), grid.width, topBandHeight),
        bgColor,
        FILLED
    );

    rectangle(
        canvas,
        Rect(std::max(0, grid.x - leftBandWidth), grid.y, leftBandWidth, grid.height),
        bgColor,
        FILLED
    );

    for (int c = 1; c <= cols; c++)
    {
        std::string label = std::to_string(c);
        int baseline = 0;
        Size textSize = getTextSize(label, FONT_HERSHEY_SIMPLEX, fontScale, thickness, &baseline);

        int x = grid.x + (c - 1) * cellSize + (cellSize - textSize.width) / 2;
        int y = grid.y - std::max(4, (topBandHeight - textSize.height) / 2);

        putText(canvas, label, Point(x, y), FONT_HERSHEY_SIMPLEX, fontScale, textColor, thickness, LINE_AA);
    }

    for (int r = 1; r <= rows; r++)
    {
        std::string label = std::to_string(r);
        int baseline = 0;
        Size textSize = getTextSize(label, FONT_HERSHEY_SIMPLEX, fontScale, thickness, &baseline);

        int x = grid.x - leftBandWidth + (leftBandWidth - textSize.width) / 2;
        int y = grid.y + (r - 1) * cellSize + (cellSize + textSize.height) / 2;

        putText(canvas, label, Point(x, y), FONT_HERSHEY_SIMPLEX, fontScale, textColor, thickness, LINE_AA);
    }
}
