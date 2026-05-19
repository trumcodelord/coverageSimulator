#include "guard.h"
#include "grid.h"
#include "dynamic_obstacle.h"

#include <vector>
#include <cmath>

using namespace std;

static const int GUARD_WAIT_CENTER_MIN = 3;
static const int GUARD_WAIT_CENTER_MAX = 6;
static const int GUARD_MAX_RADIUS = 2;

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

    return dirs[rand() % dirs.size()];
}

static void setMoveVelocity(DynamicObstacle &obs, int dir)
{
    obs.vx = 0.0f;
    obs.vy = 0.0f;

    if (dir == 0) obs.vx = 0.3f;
    else if (dir == 1) obs.vy = 0.3f;
    else if (dir == 2) obs.vx = -0.3f;
    else if (dir == 3) obs.vy = -0.3f;
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
