#include "guard.h"
#include "grid.h"
#include "dynamic_obstacle.h"

#include <vector>
#include <cmath>
#include <cstdlib>
#include <random>
#include <string>

using namespace std;

// 45 frames ~= 1.5 seconds, 90 frames ~= 3 seconds.
static const int GUARD_WAIT_CENTER_MIN = 45;
static const int GUARD_WAIT_CENTER_MAX = 90;
static const int GUARD_MAX_RADIUS = 2;

// Speed is measured in cell/frame. The dynamic obstacle layer animates the
// obstacle from one cell center to the next using this speed.
static const float GUARD_SPEED = 0.02f;

static const unsigned int DEFAULT_SIM_SEED = 20260621u;
static const unsigned int GUARD_SEED_OFFSET = 101u;

static unsigned int loadSimulationSeed()
{
    const char *raw = std::getenv("SIM_SEED");

    if (raw == nullptr || raw[0] == '\0')
        return DEFAULT_SIM_SEED;

    try
    {
        return (unsigned int)std::stoul(std::string(raw));
    }
    catch (...)
    {
        return DEFAULT_SIM_SEED;
    }
}

static mt19937 guardRng(loadSimulationSeed() + GUARD_SEED_OFFSET);

static int randomIntInclusive(int low, int high)
{
    if (high < low) return low;
    uniform_int_distribution<int> dist(low, high);
    return dist(guardRng);
}

static Cell nextCell(Cell p, int dir)
{
    Cell q = p;

    if (dir == 0) q.r += 1;
    else if (dir == 1) q.c += 1;
    else if (dir == 2) q.r -= 1;
    else if (dir == 3) q.c -= 1;

    return q;
}

static bool canStandCell(int r, int c)
{
    return !isForbiddenDynamicObstacleCell(r, c);
}

static int distManhattan(Cell a, Cell b)
{
    return abs(a.r - b.r) + abs(a.c - b.c);
}

static Cell getHome(DynamicObstacle &obs)
{
    if (obs.path.empty())
        obs.path.push_back(obs.pos);
    return obs.path[0];
}

static int chooseOutDir(DynamicObstacle &obs)
{
    Cell home = getHome(obs);

    vector<int> dirs;

    for (int d = 0; d < 4; d++)
    {
        Cell q = nextCell(obs.pos, d);

        if (!canStandCell(q.r, q.c)) continue;

        if (distManhattan(home, q) <= GUARD_MAX_RADIUS)
            dirs.push_back(d);
    }

    if (dirs.empty()) return -1;

    return dirs[randomIntInclusive(0, (int)dirs.size() - 1)];
}

static void setMoveVelocity(DynamicObstacle &obs, int dir)
{
    obs.vx = 0.0f;
    obs.vy = 0.0f;

    if (dir == 0) obs.vx = GUARD_SPEED;
    else if (dir == 1) obs.vy = GUARD_SPEED;
    else if (dir == 2) obs.vx = -GUARD_SPEED;
    else if (dir == 3) obs.vy = -GUARD_SPEED;
}

void updateGuardBehavior(DynamicObstacle &obs)
{
    Cell home = getHome(obs);

    if (obs.stateTick == 0 && obs.waitTick == 0)
    {
        obs.state = GUARD_WAIT_CENTER;
        obs.waitTick = GUARD_WAIT_CENTER_MIN;
    }

    obs.vx = 0.0f;
    obs.vy = 0.0f;

    if (obs.state == GUARD_WAIT_CENTER)
    {
        obs.x = (float)obs.pos.r;
        obs.y = (float)obs.pos.c;
        obs.stateTick++;

        if (obs.stateTick >= obs.waitTick)
        {
            int dir = chooseOutDir(obs);

            if (dir != -1)
            {
                obs.dir = dir;
                obs.state = GUARD_MOVE_OUT;
            }

            obs.stateTick = 0;
        }

        return;
    }

    if (obs.state == GUARD_MOVE_OUT)
    {
        Cell q = nextCell(obs.pos, obs.dir);

        if (canStandCell(q.r, q.c) &&
            distManhattan(home, q) <= GUARD_MAX_RADIUS)
        {
            setMoveVelocity(obs, obs.dir);
            return;
        }

        obs.state = GUARD_MOVE_BACK;
        obs.stateTick = 0;
        return;
    }

    if (obs.state == GUARD_MOVE_BACK)
    {
        int bestDir = -1;
        int bestDist = 1e9;

        for (int d = 0; d < 4; d++)
        {
            Cell q = nextCell(obs.pos, d);
            if (!canStandCell(q.r, q.c)) continue;

            int dist = distManhattan(q, home);
            if (dist < bestDist)
            {
                bestDist = dist;
                bestDir = d;
            }
        }

        if (bestDir != -1)
        {
            obs.dir = bestDir;
            setMoveVelocity(obs, obs.dir);

            if (bestDist == 0)
            {
                obs.state = GUARD_WAIT_CENTER;
                obs.stateTick = 0;
                obs.waitTick = GUARD_WAIT_CENTER_MIN;
            }

            return;
        }

        obs.state = GUARD_WAIT_CENTER;
        obs.stateTick = 0;
    }
}
