#include "map_renderer.h"

#include "grid.h"
#include "visual_layout.h"

#include <opencv2/opencv.hpp>

#include <algorithm>

using namespace cv;

namespace
{
    bool isBaseCell(int r, int c)
    {
        return r == start.r && c == start.c;
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
            Scalar color;

            if (blocked[r][c])
                color = Scalar(160, 160, 160);
            else if (isBaseCell(r, c))
                color = Scalar(255, 235, 180);
            else if (dynamicBlocked[r][c])
                color = Scalar(180, 105, 255);
            else if (showLogicalCoverage && covered[r][c])
                color = Scalar(220, 245, 220);
            else
                color = Scalar(255, 255, 255);

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