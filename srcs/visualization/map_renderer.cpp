#include "map_renderer.h"

#include "grid.h"
#include "visual_layout.h"

#include <opencv2/opencv.hpp>

using namespace cv;

void paintMapCells(Mat &canvas)
{
    for (int r = 1; r <= rows; r++)
    {
        for (int c = 1; c <= cols; c++)
        {
            Scalar color;

            if (blocked[r][c])
                color = Scalar(160, 160, 160);
            else if (dynamicBlocked[r][c])
                color = Scalar(180, 105, 255);
            else if (covered[r][c])
                color = Scalar(220, 245, 220);
            else
                color = Scalar(255, 255, 255);

            Point tl = visualCellTopLeft(r, c);
            Point br(
                tl.x + visualCellSize(),
                tl.y + visualCellSize()
            );

            rectangle(canvas, tl, br, color, FILLED);
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
