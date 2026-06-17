#include "dynamic_obstacle.h"
#include "grid.h"
#include "guard.h"
#include "vehicle.h"

#include <thread>
#include <chrono>
#include <cmath>
#include <vector>
#include <atomic>
#include <random>

using namespace std;

// Fixed update cadence for smoother obstacle animation.
// Speeds in guard/vehicle modules are interpreted as cell/frame.
static const int OBSTACLE_FRAME_MS = 33;

static thread worker;
static atomic<bool> stopRequested(false);

vector<DynamicObstacle> obstacles;
std::mutex simMutex;

static Cell robotAvoidanceCell = {0, 0};
static bool robotAvoidanceEnabled = false;

static const int ROBOT_YIELD_RADIUS = 1;

static int occupiedCount[1001][1001];
static bool reservedNext[1001][1001];

static bool manualVehicleControlEnabled = false;
static int manualVehicleControlIndex = -1;
static const int MANUAL_RELEASE_WAIT_TICKS = 45;

static std::mt19937 manualVehicleRng(
    (unsigned)std::chrono::steady_clock::now().time_since_epoch().count()
);

static int roundToCell(float v)
{
    return (int)std::lround(v);
}

bool isForbiddenDynamicObstacleCell(int r, int c)
{
    if (!inBounds(r, c))
        return true;

    if (isStaticBlocked(r, c))
        return true;

    // The robot base / command station is a protected operational zone.
    // Dynamic obstacles must not occupy it.
    if (r == start.r && c == start.c)
        return true;

    return false;
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
    if (isForbiddenDynamicObstacleCell(r, c))
        return false;

    if (isDynamicBlockedCell(r, c))
        return false;

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
    else
        obs.state = VEHICLE_WAIT;

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
        if (isForbiddenDynamicObstacleCell(r, c))
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

static void clearReservationGrids()
{
    for (int i = 1; i <= rows; i++)
    {
        for (int j = 1; j <= cols; j++)
        {
            occupiedCount[i][j] = 0;
            reservedNext[i][j] = false;
        }
    }
}

static void buildOccupiedGrid()
{
    clearReservationGrids();

    for (const auto &obs : obstacles)
    {
        if (!inBounds(obs.pos.r, obs.pos.c))
            continue;

        occupiedCount[obs.pos.r][obs.pos.c]++;
    }
}

static bool occupiedByAnotherObstacle(const DynamicObstacle &obs, Cell p)
{
    if (!inBounds(p.r, p.c))
        return true;

    int count = occupiedCount[p.r][p.c];

    if (p == obs.pos)
        count--;

    return count > 0;
}

static void reserveCell(Cell p)
{
    if (inBounds(p.r, p.c))
        reservedNext[p.r][p.c] = true;
}

static bool canReserveCell(Cell p)
{
    if (!inBounds(p.r, p.c))
        return false;

    return !reservedNext[p.r][p.c];
}

static void stopObstacleMovement(DynamicObstacle &obs)
{
    obs.vx = 0.0f;
    obs.vy = 0.0f;
    reserveCell(obs.pos);
}

static void moveStraightWithReservation(DynamicObstacle &obs)
{
    if (obs.vx == 0.0f && obs.vy == 0.0f)
    {
        reserveCell(obs.pos);
        return;
    }

    float nx = obs.x + obs.vx;
    float ny = obs.y + obs.vy;

    Cell next = roundedCell(nx, ny);

    if (wouldThreatenRobot(obs, nx, ny))
    {
        stopObstacleMovement(obs);
        return;
    }

    if (!isLineFree(obs.x, obs.y, nx, ny))
    {
        stopObstacleMovement(obs);
        return;
    }

    if (isForbiddenDynamicObstacleCell(next.r, next.c))
    {
        stopObstacleMovement(obs);
        return;
    }

    if (occupiedByAnotherObstacle(obs, next))
    {
        stopObstacleMovement(obs);
        return;
    }

    if (!canReserveCell(next))
    {
        stopObstacleMovement(obs);
        return;
    }

    obs.x = nx;
    obs.y = ny;
    obs.pos = next;

    reserveCell(obs.pos);
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

        if (inBounds(r, c))
            dynamicBlocked[r][c] = true;
    }
}

static bool validManualVehicleIndexNoLock(int idx)
{
    if (idx < 0 || idx >= (int)obstacles.size())
        return false;

    return obstacles[idx].type == ObstacleType::VEHICLE;
}

static vector<int> collectManualVehicleCandidatesNoLock()
{
    vector<int> candidates;

    for (int i = 0; i < (int)obstacles.size(); i++)
    {
        if (obstacles[i].type == ObstacleType::VEHICLE)
            candidates.push_back(i);
    }

    return candidates;
}

static Cell manualNextCell(Cell p, int dir)
{
    Cell q = p;

    if (dir == 0) q.r += 1;
    else if (dir == 1) q.c += 1;
    else if (dir == 2) q.r -= 1;
    else if (dir == 3) q.c -= 1;

    return q;
}

static bool occupiedByOtherObstacleNoLock(int selfIndex, Cell p)
{
    for (int i = 0; i < (int)obstacles.size(); i++)
    {
        if (i == selfIndex)
            continue;

        if (obstacles[i].pos == p)
            return true;
    }

    return false;
}

static void freezeManualVehicleNoLock(DynamicObstacle &obs)
{
    obs.x = (float)obs.pos.r;
    obs.y = (float)obs.pos.c;

    obs.vx = 0.0f;
    obs.vy = 0.0f;

    obs.state = VEHICLE_WAIT;
    obs.stateTick = 0;
    obs.waitTick = MANUAL_RELEASE_WAIT_TICKS;
}

bool hasManualControllableVehicle()
{
    lock_guard<mutex> lock(simMutex);
    return !collectManualVehicleCandidatesNoLock().empty();
}

bool toggleManualVehicleControl()
{
    lock_guard<mutex> lock(simMutex);

    if (manualVehicleControlEnabled)
    {
        if (validManualVehicleIndexNoLock(manualVehicleControlIndex))
            freezeManualVehicleNoLock(obstacles[manualVehicleControlIndex]);

        manualVehicleControlEnabled = false;
        manualVehicleControlIndex = -1;
        syncToGrid();
        return false;
    }

    vector<int> candidates = collectManualVehicleCandidatesNoLock();

    if (candidates.empty())
    {
        manualVehicleControlEnabled = false;
        manualVehicleControlIndex = -1;
        return false;
    }

    uniform_int_distribution<int> dist(0, (int)candidates.size() - 1);

    manualVehicleControlIndex = candidates[dist(manualVehicleRng)];
    manualVehicleControlEnabled = true;

    freezeManualVehicleNoLock(obstacles[manualVehicleControlIndex]);
    syncToGrid();

    return true;
}

bool isManualVehicleControlEnabled()
{
    lock_guard<mutex> lock(simMutex);

    return manualVehicleControlEnabled &&
           validManualVehicleIndexNoLock(manualVehicleControlIndex);
}

int manualControlledVehicleIndex()
{
    lock_guard<mutex> lock(simMutex);

    if (!manualVehicleControlEnabled ||
        !validManualVehicleIndexNoLock(manualVehicleControlIndex))
        return -1;

    return manualVehicleControlIndex;
}

Cell manualControlledVehicleCell()
{
    lock_guard<mutex> lock(simMutex);

    if (!manualVehicleControlEnabled ||
        !validManualVehicleIndexNoLock(manualVehicleControlIndex))
        return {0, 0};

    return obstacles[manualVehicleControlIndex].pos;
}

bool manualVehicleControlStep(int dir)
{
    lock_guard<mutex> lock(simMutex);

    if (!manualVehicleControlEnabled ||
        !validManualVehicleIndexNoLock(manualVehicleControlIndex))
        return false;

    if (dir < 0 || dir > 3)
        return false;

    DynamicObstacle &obs = obstacles[manualVehicleControlIndex];

    // Rotate even when the move is blocked, so the demo shows the requested
    // direction clearly.
    obs.dir = dir;

    Cell next = manualNextCell(obs.pos, dir);

    if (isForbiddenDynamicObstacleCell(next.r, next.c))
    {
        freezeManualVehicleNoLock(obs);
        syncToGrid();
        return false;
    }

    // The manual vehicle is allowed to create bad tactical situations, but it
    // should not be teleported directly onto the robot cell.
    if (robotAvoidanceEnabled && next == robotAvoidanceCell)
    {
        freezeManualVehicleNoLock(obs);
        syncToGrid();
        return false;
    }

    if (occupiedByOtherObstacleNoLock(manualVehicleControlIndex, next))
    {
        freezeManualVehicleNoLock(obs);
        syncToGrid();
        return false;
    }

    obs.pos = next;
    freezeManualVehicleNoLock(obs);
    syncToGrid();

    return true;
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
    }
}

static void updateAllObstaclesSafely()
{
    if (manualVehicleControlEnabled &&
        !validManualVehicleIndexNoLock(manualVehicleControlIndex))
    {
        manualVehicleControlEnabled = false;
        manualVehicleControlIndex = -1;
    }

    buildOccupiedGrid();

    for (int i = 0; i < (int)obstacles.size(); i++)
    {
        DynamicObstacle &obs = obstacles[i];

        if (manualVehicleControlEnabled && i == manualVehicleControlIndex)
        {
            freezeManualVehicleNoLock(obs);
            stopObstacleMovement(obs);
            continue;
        }

        updateBehavior(obs);
        moveStraightWithReservation(obs);
    }

    syncToGrid();
}

static void dynamicObstacleLoop()
{
    while (!stopRequested.load())
    {
        this_thread::sleep_for(
            chrono::milliseconds(OBSTACLE_FRAME_MS)
        );

        if (stopRequested.load())
            break;

        {
            lock_guard<mutex> lock(simMutex);

            // Do not stop dynamic obstacles just because coverage is complete.
            // The mission is only complete after the robot safely returns to base.
            updateAllObstaclesSafely();
        }
    }
}

void initDynamicObstacle()
{
    stopRequested.store(false);

    manualVehicleControlEnabled = false;
    manualVehicleControlIndex = -1;

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
