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
    constexpr int HUD_RESERVED_WIDTH = 360;
    constexpr int MIN_SCREEN_W = 1000;
    constexpr int MIN_SCREEN_H = 700;

    const string WINDOW_NAME = "Coverage Path Planning";

    int SCREEN_W = 1200;
    int SCREEN_H = 800;

    int CANVAS_W = 1200;
    int CANVAS_H = 800;

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
    CANVAS_W = max(SCREEN_W, MIN_SCREEN_W);
    CANVAS_H = max(SCREEN_H, MIN_SCREEN_H);

    int usableW = CANVAS_W - HUD_RESERVED_WIDTH - 3 * MARGIN;
    int usableH = CANVAS_H - 2 * MARGIN;

    if (usableW < 1)
        usableW = 1;

    if (usableH < 1)
        usableH = 1;

    CELL_SIZE = min(usableW / cols, usableH / rows);

    if (CELL_SIZE < 1)
        CELL_SIZE = 1;

    int gridW = cols * CELL_SIZE;
    int gridH = rows * CELL_SIZE;

    OFFSET_X = MARGIN;
    OFFSET_Y = max(MARGIN, (CANVAS_H - gridH) / 2);

    int minCanvasW = OFFSET_X + gridW + HUD_RESERVED_WIDTH + 2 * MARGIN;

    if (CANVAS_W < minCanvasW)
        CANVAS_W = minCanvasW;
}

int visualCanvasWidth()
{
    return CANVAS_W;
}

int visualCanvasHeight()
{
    return CANVAS_H;
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
