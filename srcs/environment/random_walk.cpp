#include "random_walk.h"
#include "grid.h"
#include "rng.h"
#include "dynamic_obstacle.h"

#include <vector>

using namespace std;

static const float RANDOM_SPEED = 0.25f;
static const int RANDOM_WAIT_MIN = 3;
static const int RANDOM_WAIT_MAX = 8;
static const int RANDOM_MOVE_MIN = 4;
static const int RANDOM_MOVE_MAX = 10;

static bool canMoveDir(Cell p, int dir)
{
    int nr = p.r;
    int nc = p.c;

    if (dir == 0) nr += 1;
    else if (dir == 1) nc += 1;
    else if (dir == 2) nr -= 1;
    else if (dir == 3) nc -= 1;

    return !isForbiddenDynamicObstacleCell(nr, nc);
}

void updateRandomBehavior(DynamicObstacle &obs)
{
    if (obs.waitTick == 0 && obs.moveTick == 0 && obs.stateTick == 0)
    {
        obs.state = RANDOM_WAIT;
        obs.waitTick = randInt(RANDOM_WAIT_MIN, RANDOM_WAIT_MAX);
        obs.moveTick = randInt(RANDOM_MOVE_MIN, RANDOM_MOVE_MAX);
        obs.dir = randInt(0, 3);
    }

    obs.vx = 0.0f;
    obs.vy = 0.0f;

    if (obs.state == RANDOM_WAIT)
    {
        obs.stateTick++;

        if (obs.stateTick >= obs.waitTick)
        {
            obs.stateTick = 0;
            obs.waitTick = randInt(RANDOM_WAIT_MIN, RANDOM_WAIT_MAX);
            obs.moveTick = randInt(RANDOM_MOVE_MIN, RANDOM_MOVE_MAX);
            obs.state = RANDOM_PICK_DIRECTION;
        }
    }
    else if (obs.state == RANDOM_PICK_DIRECTION)
    {
        vector<int> candidates;

        for (int d = 0; d < 4; d++)
        {
            if (canMoveDir(obs.pos, d))
                candidates.push_back(d);
        }

        if (candidates.empty())
        {
            obs.state = RANDOM_WAIT;
            obs.stateTick = 0;
            return;
        }

        obs.dir = candidates[randInt(0, (int)candidates.size() - 1)];
        obs.state = RANDOM_MOVE;
        obs.stateTick = 0;
    }
    else if (obs.state == RANDOM_MOVE)
    {
        if (obs.dir == 0)
        {
            obs.vx = RANDOM_SPEED;
            obs.vy = 0.0f;
        }
        else if (obs.dir == 1)
        {
            obs.vx = 0.0f;
            obs.vy = RANDOM_SPEED;
        }
        else if (obs.dir == 2)
        {
            obs.vx = -RANDOM_SPEED;
            obs.vy = 0.0f;
        }
        else if (obs.dir == 3)
        {
            obs.vx = 0.0f;
            obs.vy = -RANDOM_SPEED;
        }

        obs.stateTick++;

        if (obs.stateTick >= obs.moveTick)
        {
            obs.stateTick = 0;
            obs.waitTick = randInt(RANDOM_WAIT_MIN, RANDOM_WAIT_MAX);
            obs.moveTick = randInt(RANDOM_MOVE_MIN, RANDOM_MOVE_MAX);
            obs.state = RANDOM_WAIT;
        }
    }
}
