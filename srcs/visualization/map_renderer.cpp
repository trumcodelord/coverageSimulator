#include "map_renderer.h"

#include "grid.h"
#include "visual_layout.h"

#include <opencv2/opencv.hpp>

#include <algorithm>
#include <string>

using namespace cv;

namespace
{
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
        if (blocked[r][c])
            return Scalar(45, 45, 45);

        int gray = grayForTerrainCost(terrainCostAt(r, c));
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
}

void paintMapCells(Mat &canvas, bool showLogicalCoverage)
{
    for (int r = 1; r <= rows; r++)
    {
        for (int c = 1; c <= cols; c++)
        {
            Scalar color = terrainColorForCell(r, c);

            if (!blocked[r][c] && dynamicBlocked[r][c])
            {
                color = blendColor(color, Scalar(180, 105, 255), 0.65);
            }
            else if (!blocked[r][c] && showLogicalCoverage && covered[r][c])
            {
                // Keep terrain grayscale visible; coverage is a green overlay.
                color = blendColor(color, Scalar(80, 210, 80), 0.28);
            }

            if (!blocked[r][c] && isBaseCell(r, c))
            {
                // Keep the base recognizable without hiding the terrain layer.
                color = blendColor(color, Scalar(255, 235, 180), 0.20);
            }

            Point tl = visualCellTopLeft(r, c);
            Point br(
                tl.x + visualCellSize(),
                tl.y + visualCellSize()
            );

            rectangle(canvas, tl, br, color, FILLED);

            if (isBaseCell(r, c) && !blocked[r][c])
                paintBaseMarker(canvas, tl, br);
        }
    }
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
