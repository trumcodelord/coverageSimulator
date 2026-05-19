#include "vehicle.h"
#include "grid.h"
#include "dynamic_obstacle.h"

#include <vector>

using namespace std;

static const float VEHICLE_SPEED = 0.03f;

static const int VEHICLE_WAIT_MIN = 45;
static const int VEHICLE_WAIT_MAX = 90;

static bool canStandCell(int r, int c)
{
    return !isForbiddenDynamicObstacleCell(r, c);
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

static bool canMoveDir(Cell p, int dir)
{
    Cell q = nextCell(p, dir);
    return canStandCell(q.r, q.c);
}

static int turnRight(int dir)
{
    return (dir + 1) % 4;
}

static int turnLeft(int dir)
{
    return (dir + 3) % 4;
}

static int turnBack(int dir)
{
    return (dir + 2) % 4;
}

static void setMoveVelocity(DynamicObstacle &obs, int dir)
{
    obs.vx = 0.0f;
    obs.vy = 0.0f;

    if (dir == 0) obs.vx = VEHICLE_SPEED;
    else if (dir == 1) obs.vy = VEHICLE_SPEED;
    else if (dir == 2) obs.vx = -VEHICLE_SPEED;
    else if (dir == 3) obs.vy = -VEHICLE_SPEED;
}

static int chooseNewDir(Cell p, int curDir)
{
    int rightDir = turnRight(curDir);
    int leftDir  = turnLeft(curDir);
    int backDir  = turnBack(curDir);

    if (canMoveDir(p, rightDir)) return rightDir;
    if (canMoveDir(p, leftDir))  return leftDir;
    if (canMoveDir(p, backDir))  return backDir;

    return -1;
}

void updateVehicleBehavior(DynamicObstacle &obs)
{
    if (obs.stateTick == 0 && obs.waitTick == 0 && obs.vx == 0.0f && obs.vy == 0.0f)
    {
        vector<int> candidates;
        for (int d = 0; d < 4; d++)
        {
            if (canMoveDir(obs.pos, d))
                candidates.push_back(d);
        }

        if (!candidates.empty())
            obs.dir = candidates[0];
        else
            obs.dir = 0;

        obs.state = VEHICLE_WAIT;
        obs.waitTick = VEHICLE_WAIT_MIN;
        obs.vx = 0.0f;
        obs.vy = 0.0f;
    }

    obs.vx = 0.0f;
    obs.vy = 0.0f;

    if (obs.state == VEHICLE_WAIT)
    {
        obs.x = (float)obs.pos.r;
        obs.y = (float)obs.pos.c;

        obs.stateTick++;

        if (obs.stateTick >= obs.waitTick)
        {
            obs.stateTick = 0;
            obs.state = VEHICLE_MOVE;
        }

        return;
    }

    if (canMoveDir(obs.pos, obs.dir))
    {
        setMoveVelocity(obs, obs.dir);
        return;
    }

    int newDir = chooseNewDir(obs.pos, obs.dir);

    if (newDir != -1)
    {
        obs.dir = newDir;
        obs.state = VEHICLE_WAIT;
        obs.stateTick = 0;
        obs.waitTick = VEHICLE_WAIT_MIN;
        obs.vx = 0.0f;
        obs.vy = 0.0f;
        return;
    }

    obs.state = VEHICLE_WAIT;
    obs.stateTick = 0;
    obs.waitTick = VEHICLE_WAIT_MAX;
    obs.vx = 0.0f;
    obs.vy = 0.0f;
}
