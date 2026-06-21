#include "vehicle.h"
#include "grid.h"
#include "dynamic_obstacle.h"

#include <vector>
#include <random>
#include <cstdlib>
#include <cmath>
#include <map>

using namespace std;

// Patrol vehicle policy v1.
// The vehicle is a cooperative moving obstacle: it does not chase the robot and
// does not intentionally block it. It tries to stay mobile, may turn into long
// side branches, may occasionally turn back in corridor-like areas, and will
// turn away after being stalled for a few seconds.

static const float VEHICLE_SPEED = 0.03f;

// Dynamic obstacle loop runs at ~30 FPS. 90 frames ~= 3 seconds.
static const int VEHICLE_STALL_TIMEOUT_TICKS = 90;
static const int VEHICLE_SHORT_WAIT_TICKS = 15;

// Local static lookahead. This avoids hard-coding corridor width.
static const int VEHICLE_BRANCH_LOOKAHEAD = 8;
static const int VEHICLE_GOOD_BRANCH_MIN = 5;

// These are measured in grid-cell transitions since the last direction change.
static const int VEHICLE_MIN_SEGMENT_BEFORE_BRANCH = 5;
static const int VEHICLE_MIN_SEGMENT_BEFORE_UTURN = 10;

static const int VEHICLE_BRANCH_TURN_CHANCE_PERCENT = 30;
static const int VEHICLE_CORRIDOR_UTURN_CHANCE_PERCENT = 5;

static const unsigned int DEFAULT_SIM_SEED = 20260621u;
static const unsigned int VEHICLE_SEED_OFFSET = 202u;

struct MotionObserver
{
    bool initialized = false;
    Cell lastCell = {0, 0};
    float lastX = 0.0f;
    float lastY = 0.0f;
    int stalledTicks = 0;
    bool decisionCellInitialized = false;
    Cell lastDecisionCell = {0, 0};
};

static map<const DynamicObstacle*, MotionObserver> motionObservers;

static unsigned int loadSimulationSeed()
{
    const char *raw = std::getenv("SIM_SEED");
    if (raw == NULL || raw[0] == '\0')
        return DEFAULT_SIM_SEED;

    char *endPtr = NULL;
    unsigned long value = std::strtoul(raw, &endPtr, 10);

    if (endPtr == raw || *endPtr != '\0')
        return DEFAULT_SIM_SEED;

    return (unsigned int)value;
}

static mt19937 vehicleRng(loadSimulationSeed() + VEHICLE_SEED_OFFSET);

static int randomIntInclusive(int low, int high)
{
    if (high < low) return low;
    uniform_int_distribution<int> dist(low, high);
    return dist(vehicleRng);
}

static bool randomPercent(int percent)
{
    if (percent <= 0) return false;
    if (percent >= 100) return true;
    return randomIntInclusive(1, 100) <= percent;
}

static int randomChoice(const vector<int> &values)
{
    if (values.empty()) return -1;
    return values[randomIntInclusive(0, (int)values.size() - 1)];
}

static int normalizeDir(int dir)
{
    dir %= 4;
    if (dir < 0) dir += 4;
    return dir;
}

static bool sameCell(Cell a, Cell b)
{
    return a.r == b.r && a.c == b.c;
}

static bool canStandCell(int r, int c)
{
    return !isForbiddenDynamicObstacleCell(r, c);
}

static Cell nextCell(Cell p, int dir)
{
    Cell q = p;
    dir = normalizeDir(dir);

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
    return normalizeDir(dir + 1);
}

static int turnLeft(int dir)
{
    return normalizeDir(dir + 3);
}

static int turnBack(int dir)
{
    return normalizeDir(dir + 2);
}

static void setMoveVelocity(DynamicObstacle &obs, int dir)
{
    dir = normalizeDir(dir);
    obs.vx = 0.0f;
    obs.vy = 0.0f;

    if (dir == 0) obs.vx = VEHICLE_SPEED;
    else if (dir == 1) obs.vy = VEHICLE_SPEED;
    else if (dir == 2) obs.vx = -VEHICLE_SPEED;
    else if (dir == 3) obs.vy = -VEHICLE_SPEED;
}

static bool isNearCellCenter(const DynamicObstacle &obs)
{
    return std::fabs(obs.x - (float)obs.pos.r) < 0.05f &&
           std::fabs(obs.y - (float)obs.pos.c) < 0.05f;
}

static int lookaheadDistance(Cell p, int dir, int limit)
{
    int dist = 0;
    Cell cur = p;

    for (int i = 0; i < limit; i++)
    {
        Cell q = nextCell(cur, dir);
        if (!canStandCell(q.r, q.c))
            break;

        dist++;
        cur = q;
    }

    return dist;
}

static bool isGoodBranch(Cell p, int dir)
{
    return lookaheadDistance(p, dir, VEHICLE_BRANCH_LOOKAHEAD) >= VEHICLE_GOOD_BRANCH_MIN;
}

static int chooseInitialDir(Cell p)
{
    int bestScore = -1;
    vector<int> bestDirs;

    for (int d = 0; d < 4; d++)
    {
        if (!canMoveDir(p, d)) continue;

        int score = lookaheadDistance(p, d, VEHICLE_BRANCH_LOOKAHEAD);
        if (score > bestScore)
        {
            bestScore = score;
            bestDirs.clear();
            bestDirs.push_back(d);
        }
        else if (score == bestScore)
        {
            bestDirs.push_back(d);
        }
    }

    int chosen = randomChoice(bestDirs);
    return chosen == -1 ? 0 : chosen;
}

static int chooseStaticEscapeDir(Cell p, int curDir)
{
    int rightDir = turnRight(curDir);
    int leftDir  = turnLeft(curDir);
    int backDir  = turnBack(curDir);

    vector<int> sideBranches;
    if (canMoveDir(p, rightDir)) sideBranches.push_back(rightDir);
    if (canMoveDir(p, leftDir))  sideBranches.push_back(leftDir);

    if (!sideBranches.empty())
    {
        // Prefer the side that opens to a longer static corridor. Randomize ties.
        int bestScore = -1;
        vector<int> bestDirs;

        for (int dir : sideBranches)
        {
            int score = lookaheadDistance(p, dir, VEHICLE_BRANCH_LOOKAHEAD);
            if (score > bestScore)
            {
                bestScore = score;
                bestDirs.clear();
                bestDirs.push_back(dir);
            }
            else if (score == bestScore)
            {
                bestDirs.push_back(dir);
            }
        }

        int chosen = randomChoice(bestDirs);
        if (chosen != -1) return chosen;
    }

    if (canMoveDir(p, backDir)) return backDir;
    return -1;
}

static int chooseStallEscapeDir(Cell p, int curDir)
{
    int backDir = turnBack(curDir);
    if (canMoveDir(p, backDir)) return backDir;

    vector<int> sideDirs;
    int rightDir = turnRight(curDir);
    int leftDir = turnLeft(curDir);

    if (canMoveDir(p, rightDir)) sideDirs.push_back(rightDir);
    if (canMoveDir(p, leftDir))  sideDirs.push_back(leftDir);

    int chosen = randomChoice(sideDirs);
    if (chosen != -1) return chosen;

    if (canMoveDir(p, curDir)) return curDir;
    return -1;
}

static MotionObserver& observerFor(const DynamicObstacle &obs)
{
    return motionObservers[&obs];
}

static void resetStall(DynamicObstacle &obs)
{
    observerFor(obs).stalledTicks = 0;
}

static void resetSegment(DynamicObstacle &obs)
{
    obs.moveTick = 0;
}

static void changeDirection(DynamicObstacle &obs, int newDir)
{
    if (newDir < 0) return;

    newDir = normalizeDir(newDir);
    if (obs.dir != newDir)
    {
        obs.dir = newDir;
        resetSegment(obs);
    }
}

static void updateMotionObserver(DynamicObstacle &obs)
{
    MotionObserver &mo = observerFor(obs);

    if (!mo.initialized)
    {
        mo.initialized = true;
        mo.lastCell = obs.pos;
        mo.lastX = obs.x;
        mo.lastY = obs.y;
        mo.stalledTicks = 0;
        return;
    }

    if (!sameCell(mo.lastCell, obs.pos))
    {
        obs.moveTick++;
        mo.lastCell = obs.pos;
    }

    bool actualMotion = std::fabs(obs.x - mo.lastX) > 0.0001f ||
                        std::fabs(obs.y - mo.lastY) > 0.0001f;

    if (actualMotion)
    {
        mo.stalledTicks = 0;
    }
    else if (obs.state == VEHICLE_MOVE && canMoveDir(obs.pos, obs.dir))
    {
        // Static map says the front is open, but the object did not move.
        // In practice this usually means yielding to the robot, another obstacle,
        // or a reservation conflict in dynamic_obstacle.cpp.
        mo.stalledTicks++;
    }

    mo.lastX = obs.x;
    mo.lastY = obs.y;
}

static bool canMakeOneDecisionAtCurrentCell(const DynamicObstacle &obs)
{
    if (!isNearCellCenter(obs))
        return false;

    MotionObserver &mo = observerFor(obs);

    if (mo.decisionCellInitialized && sameCell(mo.lastDecisionCell, obs.pos))
        return false;

    mo.decisionCellInitialized = true;
    mo.lastDecisionCell = obs.pos;
    return true;
}

static int choosePatrolDir(DynamicObstacle &obs)
{
    int curDir = normalizeDir(obs.dir);

    if (!canMoveDir(obs.pos, curDir))
        return chooseStaticEscapeDir(obs.pos, curDir);

    if (!canMakeOneDecisionAtCurrentCell(obs))
        return curDir;

    if (obs.moveTick >= VEHICLE_MIN_SEGMENT_BEFORE_BRANCH)
    {
        vector<int> branches;
        int rightDir = turnRight(curDir);
        int leftDir = turnLeft(curDir);

        if (isGoodBranch(obs.pos, rightDir)) branches.push_back(rightDir);
        if (isGoodBranch(obs.pos, leftDir)) branches.push_back(leftDir);

        if (!branches.empty() && randomPercent(VEHICLE_BRANCH_TURN_CHANCE_PERCENT))
        {
            int chosen = randomChoice(branches);
            if (chosen != -1) return chosen;
        }
    }

    if (obs.moveTick >= VEHICLE_MIN_SEGMENT_BEFORE_UTURN)
    {
        int rightDir = turnRight(curDir);
        int leftDir = turnLeft(curDir);
        int backDir = turnBack(curDir);

        bool hasGoodSideBranch = isGoodBranch(obs.pos, rightDir) ||
                                 isGoodBranch(obs.pos, leftDir);

        if (!hasGoodSideBranch && canMoveDir(obs.pos, backDir) &&
            randomPercent(VEHICLE_CORRIDOR_UTURN_CHANCE_PERCENT))
        {
            return backDir;
        }
    }

    return curDir;
}

static void enterShortWait(DynamicObstacle &obs)
{
    obs.state = VEHICLE_WAIT;
    obs.stateTick = 0;
    obs.waitTick = VEHICLE_SHORT_WAIT_TICKS;
    obs.vx = 0.0f;
    obs.vy = 0.0f;
    obs.x = (float)obs.pos.r;
    obs.y = (float)obs.pos.c;
}

void updateVehicleBehavior(DynamicObstacle &obs)
{
    updateMotionObserver(obs);

    obs.vx = 0.0f;
    obs.vy = 0.0f;

    // First automatic activation. Manual release may also leave a positive
    // waitTick; in that case the normal VEHICLE_WAIT block below handles it.
    if (obs.state == VEHICLE_WAIT && obs.stateTick == 0 && obs.waitTick == 0)
    {
        obs.dir = chooseInitialDir(obs.pos);
        obs.state = VEHICLE_MOVE;
        resetSegment(obs);
        resetStall(obs);
    }

    if (obs.state == VEHICLE_WAIT)
    {
        obs.x = (float)obs.pos.r;
        obs.y = (float)obs.pos.c;
        obs.stateTick++;

        if (obs.stateTick >= obs.waitTick)
        {
            obs.stateTick = 0;
            obs.waitTick = 0;
            obs.state = VEHICLE_MOVE;
        }

        return;
    }

    MotionObserver &mo = observerFor(obs);

    if (mo.stalledTicks >= VEHICLE_STALL_TIMEOUT_TICKS)
    {
        int escapeDir = chooseStallEscapeDir(obs.pos, obs.dir);
        if (escapeDir != -1)
        {
            changeDirection(obs, escapeDir);
            resetStall(obs);
        }
        else
        {
            enterShortWait(obs);
            return;
        }
    }

    int chosenDir = choosePatrolDir(obs);
    if (chosenDir != -1)
        changeDirection(obs, chosenDir);

    if (canMoveDir(obs.pos, obs.dir))
    {
        setMoveVelocity(obs, obs.dir);
        return;
    }

    int escapeDir = chooseStaticEscapeDir(obs.pos, obs.dir);
    if (escapeDir != -1)
    {
        changeDirection(obs, escapeDir);
        setMoveVelocity(obs, obs.dir);
        return;
    }

    enterShortWait(obs);
}
