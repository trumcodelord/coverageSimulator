#include "dynamic_obstacle.h"
#include "grid.h"
#include "rng.h"
#include "guard.h"
#include "vehicle.h"
#include "random_walk.h"

#include <thread>
#include <chrono>
#include <cmath>
#include <vector>
#include <atomic>

using namespace std;

static const int SLEEP_MIN_MS = 250;
static const int SLEEP_MAX_MS = 700;

static thread worker;
static atomic<bool> stopRequested(false);

vector<DynamicObstacle> obstacles;
std::mutex simMutex;

static Cell robotAvoidanceCell = {0, 0};
static bool robotAvoidanceEnabled = false;

static const int ROBOT_YIELD_RADIUS = 1;

static int roundToCell(float v)
{
    return (int)std::lround(v);
}

void setRobotAvoidanceCell(Cell pos)
{
    robotAvoidanceCell = pos;
    robotAvoidanceEnabled = inBounds(pos.r, pos.c);
}

static int manhattan(Cell a, Cell b)
{
    return abs(a.r - b.r) + abs(a.c - b.c);
}

static Cell roundedCell(float x, float y)
{
    return {roundToCell(x), roundToCell(y)};
}

static Cell oneStepAheadByVelocity(const DynamicObstacle &obs)
{
    Cell q = obs.pos;

    if (obs.vx > 0.0f) q.r += 1;
    else if (obs.vx < 0.0f) q.r -= 1;
    else if (obs.vy > 0.0f) q.c += 1;
    else if (obs.vy < 0.0f) q.c -= 1;

    return q;
}

static bool wouldThreatenRobot(const DynamicObstacle &obs, float nx, float ny)
{
    if (!robotAvoidanceEnabled)
        return false;

    if (obs.vx == 0.0f && obs.vy == 0.0f)
        return false;

    Cell next = roundedCell(nx, ny);
    Cell front = oneStepAheadByVelocity(obs);

    if (next == robotAvoidanceCell)
        return true;

    if (front == robotAvoidanceCell)
        return true;

    int curDist = manhattan(obs.pos, robotAvoidanceCell);
    int nextDist = manhattan(next, robotAvoidanceCell);

    return nextDist <= ROBOT_YIELD_RADIUS && nextDist < curDist;
}

static bool canPlace(int r, int c)
{
    if (!inBounds(r, c)) return false;
    if (blocked[r][c]) return false;
    if (dynamicBlocked[r][c]) return false;
    return true;
}

void addObstacle(int r, int c, ObstacleType type)
{
    if (!canPlace(r, c)) return;

    DynamicObstacle obs;

    obs.pos = {r, c};
    obs.type = type;

    obs.dir = 0;
    obs.path.clear();
    obs.stateTick = 0;
    obs.waitTick = 0;
    obs.moveTick = 0;

    obs.x = (float)r;
    obs.y = (float)c;
    obs.vx = 0.0f;
    obs.vy = 0.0f;

    if (type == ObstacleType::GUARD)
        obs.state = GUARD_WAIT_CENTER;
    else if (type == ObstacleType::VEHICLE)
        obs.state = VEHICLE_WAIT;
    else
        obs.state = RANDOM_WAIT;

    obstacles.push_back(obs);
}

static bool isLineFree(float x1, float y1, float x2, float y2)
{
    int r = roundToCell(x1);
    int c = roundToCell(y1);
    int r2 = roundToCell(x2);
    int c2 = roundToCell(y2);

    int dr = abs(r2 - r);
    int dc = abs(c2 - c);

    int stepR = (r < r2) ? 1 : -1;
    int stepC = (c < c2) ? 1 : -1;

    int err = dr - dc;

    while (true)
    {
        if (!inBounds(r, c) || blocked[r][c])
            return false;

        if (r == r2 && c == c2)
            break;

        int e2 = 2 * err;

        if (e2 > -dc)
        {
            err -= dc;
            r += stepR;
        }

        if (e2 < dr)
        {
            err += dr;
            c += stepC;
        }
    }

    return true;
}

static void moveStraight(DynamicObstacle &obs)
{
    float nx = obs.x + obs.vx;
    float ny = obs.y + obs.vy;

    if (wouldThreatenRobot(obs, nx, ny))
    {
        obs.vx = 0.0f;
        obs.vy = 0.0f;
        return;
    }

    if (!isLineFree(obs.x, obs.y, nx, ny))
    {
        obs.vx = 0.0f;
        obs.vy = 0.0f;
        return;
    }

    int nr = roundToCell(nx);
    int nc = roundToCell(ny);

    if (!inBounds(nr, nc)) return;
    if (blocked[nr][nc]) return;

    obs.x = nx;
    obs.y = ny;
    obs.pos = {nr, nc};
}

static void syncToGrid()
{
    for (int i = 1; i <= rows; i++)
        for (int j = 1; j <= cols; j++)
            dynamicBlocked[i][j] = false;

    for (auto &obs : obstacles)
    {
        int r = obs.pos.r;
        int c = obs.pos.c;
        dynamicBlocked[r][c] = true;
    }
}

static void updateBehavior(DynamicObstacle &obs)
{
    switch (obs.type)
    {
    case ObstacleType::GUARD:
        updateGuardBehavior(obs);
        break;

    case ObstacleType::VEHICLE:
        updateVehicleBehavior(obs);
        break;

    case ObstacleType::RANDOM:
        updateRandomBehavior(obs);
        break;
    }
}

static void dynamicObstacleLoop()
{
    while (!stopRequested.load())
    {
        this_thread::sleep_for(
            chrono::milliseconds(randInt(SLEEP_MIN_MS, SLEEP_MAX_MS))
        );

        if (stopRequested.load())
            break;

        {
            lock_guard<mutex> lock(simMutex);

            if (allCovered())
                break;

            for (auto &obs : obstacles)
            {
                updateBehavior(obs);
                moveStraight(obs);
            }

            syncToGrid();
        }
    }
}

void initDynamicObstacle()
{
    stopRequested.store(false);

    robotAvoidanceCell = start;
    robotAvoidanceEnabled = inBounds(start.r, start.c);

    for (int i = 1; i <= rows; i++)
        for (int j = 1; j <= cols; j++)
            dynamicBlocked[i][j] = false;

    syncToGrid();
}

void startDynamicObstacle()
{
    stopRequested.store(false);

    if (obstacles.empty())
        return;

    worker = thread(dynamicObstacleLoop);
}

void stopDynamicObstacle()
{
    stopRequested.store(true);
}

void waitDynamicObstacle()
{
    if (worker.joinable())
        worker.join();
}
