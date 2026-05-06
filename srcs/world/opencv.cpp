#include "opencv.h"
#include "grid.h"
#include "dynamic_obstacle.h"
#include "image_utils.h"

#include <opencv2/opencv.hpp>
#include <algorithm>
#include <cmath>
#include <string>

using namespace std;
using namespace cv;

static int CELL_SIZE = 40;
static const int MARGIN = 20;
static const string WINDOW_NAME = "Coverage Path Planning";

static int SCREEN_W = 800;
static int SCREEN_H = 600;

static int OFFSET_X = MARGIN;
static int OFFSET_Y = MARGIN;

static Mat iconGuard;
static Mat iconVehicle;
static Mat iconRandom;

static string hudState = "NONE";

void setHUDState(const std::string &state)
{
    hudState = state;
}

void clearHUDState()
{
    hudState = "NONE";
}

static void setupLayout()
{
    int usableW = SCREEN_W - 2 * MARGIN;
    int usableH = SCREEN_H - 2 * MARGIN;

    CELL_SIZE = min(usableW / cols, usableH / rows);
    if (CELL_SIZE < 1) CELL_SIZE = 1;

    int gridW = cols * CELL_SIZE;
    int gridH = rows * CELL_SIZE;

    OFFSET_X = (SCREEN_W - gridW) / 2;
    OFFSET_Y = (SCREEN_H - gridH) / 2;
}

static int canvasWidth()
{
    return SCREEN_W;
}

static int canvasHeight()
{
    return SCREEN_H;
}

static Point cellTopLeft(int r, int c)
{
    int x = OFFSET_X + (c - 1) * CELL_SIZE;
    int y = OFFSET_Y + (r - 1) * CELL_SIZE;
    return Point(x, y);
}

static Point cellCenter(int r, int c)
{
    Point p = cellTopLeft(r, c);
    return Point(p.x + CELL_SIZE / 2, p.y + CELL_SIZE / 2);
}

static Point worldCenter(float x, float y)
{
    int px = OFFSET_X + (int)std::lround((y - 0.5f) * CELL_SIZE);
    int py = OFFSET_Y + (int)std::lround((x - 0.5f) * CELL_SIZE);
    return Point(px, py);
}

void initWindow()
{
    namedWindow(WINDOW_NAME, WINDOW_NORMAL);
    setWindowProperty(WINDOW_NAME, WND_PROP_FULLSCREEN, WINDOW_FULLSCREEN);

    Rect rect = getWindowImageRect(WINDOW_NAME);
    if (rect.width > 0 && rect.height > 0)
    {
        SCREEN_W = rect.width;
        SCREEN_H = rect.height;
    }

    setupLayout();
    iconGuard = imread("assets/policemen.png", IMREAD_UNCHANGED);
    iconVehicle = imread("assets/jeep.png", IMREAD_UNCHANGED);
    iconRandom = imread("assets/question.png", IMREAD_UNCHANGED);
}

void closeWindow()
{
    destroyWindow(WINDOW_NAME);
}

static void paintCells(Mat &canvas)
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

            Point tl = cellTopLeft(r, c);
            Point br(tl.x + CELL_SIZE, tl.y + CELL_SIZE);

            rectangle(canvas, tl, br, color, FILLED);
        }
    }
}

static void paintGridLines(Mat &canvas)
{
    Scalar lineColor(100, 100, 100);

    for (int r = 0; r <= rows; r++)
    {
        int y = OFFSET_Y + r * CELL_SIZE;
        line(canvas,
             Point(OFFSET_X, y),
             Point(OFFSET_X + cols * CELL_SIZE, y),
             lineColor, 1);
    }

    for (int c = 0; c <= cols; c++)
    {
        int x = OFFSET_X + c * CELL_SIZE;
        line(canvas,
             Point(x, OFFSET_Y),
             Point(x, OFFSET_Y + rows * CELL_SIZE),
             lineColor, 1);
    }
}

static void paintRobot(Mat &canvas, const Robot &rb)
{
    Point center = cellCenter(rb.pos.r, rb.pos.c);

    int radius = max(4, CELL_SIZE / 6);
    int coreRadius = max(2, CELL_SIZE / 20);
    int thickness = max(1, CELL_SIZE / 15);

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

    int arrowLen = radius + max(6, CELL_SIZE / 4);
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

    arrowedLine(canvas, start, nose, Scalar(250, 0, 100), thickness, LINE_AA, 0, 0.4);
}

static void paintPath(Mat &canvas, const Robot &rb)
{
    if ((int)rb.path.size() < 2) return;

    Scalar pathColor(255, 120, 0);
    for (int i = max(1, rb.pathID); i < (int)rb.path.size(); i++)
    {
        Cell a = rb.path[i - 1];
        Cell b = rb.path[i];

        Point p1 = cellCenter(a.r, a.c);
        Point p2 = cellCenter(b.r, b.c);

        arrowedLine(canvas, p1, p2, pathColor, 2);
    }
}

static Scalar getTrailColor(int cnt)
{
    if (cnt <= 1)
        return Scalar(0, 144, 255);

    if (cnt <= 3)
    {
        int g = 144 - (cnt - 1) * 60;
        if (g < 0) g = 0;
        return Scalar(0, g, 255);
    }

    int r = 255 - (cnt - 4) * 30;
    if (r < 0) r = 0;
    return Scalar(0, 0, r);
}

static void paintTrail(Mat &canvas, const Robot &rb)
{
    if (rb.trail.size() < 2)
        return;

    for (int i = 1; i < (int)rb.trail.size(); i++)
    {
        Cell a = rb.trail[i - 1];
        Cell b = rb.trail[i];

        Point p1 = cellCenter(a.r, a.c);
        Point p2 = cellCenter(b.r, b.c);

        Edge e(a, b);
        int cnt = 1;
        auto it = rb.edgeCount.find(e);
        if (it != rb.edgeCount.end())
            cnt = it->second;

        Scalar color = getTrailColor(cnt);
        int thickness = 3;

        arrowedLine(canvas, p1, p2, Scalar(80, 80, 80), thickness + 2, LINE_AA, 0, 0.1);
        arrowedLine(canvas, p1, p2, color, thickness, LINE_AA, 0, 0.1);
    }
}

static void paintDynamicObstacles(Mat &canvas)
{
    for (const auto &obs : obstacles)
    {
        Point center = worldCenter(obs.x, obs.y);

        if (obs.type == ObstacleType::GUARD)
        {
            int size = max(18, (int)(CELL_SIZE * 0.85));
            overlayImage(canvas, iconGuard, center, size);
        }
        else if (obs.type == ObstacleType::VEHICLE)
        {
            int size = max(20, (int)(CELL_SIZE * 1.10));
            overlayImage(canvas, iconVehicle, center, size);
        }
        else if (obs.type == ObstacleType::RANDOM)
        {
            int size = max(16, (int)(CELL_SIZE * 0.75));
            overlayImage(canvas, iconRandom, center, size);
        }
    }
}

static Scalar hudStateColor(const string &state)
{
    if (state == "NONE") return Scalar(60, 60, 60);
    if (state == "FIND") return Scalar(0, 140, 255);
    if (state == "RUN!") return Scalar(0, 180, 0);
    if (state == "WAIT") return Scalar(0, 0, 220);
    return Scalar(60, 60, 60);
}

static void paintHUD(Mat &canvas, const Robot &rb, int delay)
{
    bool hasActivePath = (rb.pathID < (int)rb.path.size());
    int currentPathLength = (int)rb.path.size();

    string text1 = "Steps: " + to_string(rb.steps);
    string text2 = "Frame delay: " + to_string(delay) + " ms";
    string text3 = string("Active path: ") + (hasActivePath ? "YES" : "NO");
    string text4 = "Current path length: " + to_string(currentPathLength);
    string text5 = "State: " + hudState;

    putText(canvas, text1, Point(20, 30), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(40, 40, 40), 2, LINE_AA);
    putText(canvas, text2, Point(20, 60), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(40, 40, 40), 2, LINE_AA);
    putText(canvas, text3, Point(20, 90), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(40, 40, 40), 2, LINE_AA);
    putText(canvas, text4, Point(20, 120), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(40, 40, 40), 2, LINE_AA);
    putText(canvas, text5, Point(20, 150), FONT_HERSHEY_SIMPLEX, 0.95, hudStateColor(hudState), 2, LINE_AA);
}

void drawFrame(const Robot &rb, bool showPath, int delay)
{
    Mat canvas(
        canvasHeight(),
        canvasWidth(),
        CV_8UC3,
        Scalar(240, 240, 240)
    );

    paintCells(canvas);
    paintGridLines(canvas);
    paintTrail(canvas, rb);

    if (showPath)
        paintPath(canvas, rb);

    paintDynamicObstacles(canvas);
    paintRobot(canvas, rb);
    paintHUD(canvas, rb, delay);

    imshow(WINDOW_NAME, canvas);
}

void waitFrame(int delay)
{
    if (delay < 0) delay = 0;
    waitKey(delay);
}
