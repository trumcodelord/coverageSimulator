#include "visual_layout.h"

#include "grid.h"

#include <algorithm>
#include <cmath>

using namespace std;
using namespace cv;

namespace
{
    int CELL_SIZE = 40;
    constexpr int MARGIN = 20;

    const string WINDOW_NAME = "Coverage Path Planning";

    int SCREEN_W = 800;
    int SCREEN_H = 600;

    int OFFSET_X = MARGIN;
    int OFFSET_Y = MARGIN;
}

const string& visualWindowName()
{
    return WINDOW_NAME;
}

void updateVisualScreenSize(int width, int height)
{
    if (width > 0)
        SCREEN_W = width;

    if (height > 0)
        SCREEN_H = height;
}

void setupVisualLayout()
{
    int usableW = SCREEN_W - 2 * MARGIN;
    int usableH = SCREEN_H - 2 * MARGIN;

    CELL_SIZE = min(usableW / cols, usableH / rows);

    if (CELL_SIZE < 1)
        CELL_SIZE = 1;

    int gridW = cols * CELL_SIZE;
    int gridH = rows * CELL_SIZE;

    OFFSET_X = (SCREEN_W - gridW) / 2;
    OFFSET_Y = (SCREEN_H - gridH) / 2;
}

int visualCanvasWidth()
{
    return SCREEN_W;
}

int visualCanvasHeight()
{
    return SCREEN_H;
}

int visualCellSize()
{
    return CELL_SIZE;
}

Point visualCellTopLeft(int r, int c)
{
    int x = OFFSET_X + (c - 1) * CELL_SIZE;
    int y = OFFSET_Y + (r - 1) * CELL_SIZE;

    return Point(x, y);
}

Point visualCellCenter(int r, int c)
{
    Point p = visualCellTopLeft(r, c);

    return Point(
        p.x + CELL_SIZE / 2,
        p.y + CELL_SIZE / 2
    );
}

Point visualWorldCenter(float x, float y)
{
    int px = OFFSET_X + (int)lround((y - 0.5f) * CELL_SIZE);
    int py = OFFSET_Y + (int)lround((x - 0.5f) * CELL_SIZE);

    return Point(px, py);
}

Rect visualGridRect()
{
    return Rect(
        OFFSET_X,
        OFFSET_Y,
        cols * CELL_SIZE,
        rows * CELL_SIZE
    );
}
