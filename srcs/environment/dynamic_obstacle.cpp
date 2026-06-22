#include "dynamic_obstacle.h"
#include "grid.h"
#include "guard.h"
#include "obstacle_trace.h"
#include "vehicle.h"

#include <thread>
#include <chrono>
#include <cmath>
#include <vector>
#include <atomic>
#include <random>
#include <algorithm>
#include <cstdlib>
#include <string>

using namespace std;

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

static const unsigned int DEFAULT_SIM_SEED = 20260621u;
static const unsigned int MANUAL_VEHICLE_SEED_OFFSET = 303u;

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

static std::mt19937 manualVehicleRng(
    loadSimulationSeed() + MANUAL_VEHICLE_SEED_OFFSET
);

static std::vector<Cell> dirtyDynamicCells;
static std::vector<Cell> currentDynamicBlockedCells;
static std::vector<Cell> occupiedReservationCells;
static std::vector<Cell> reservedReservationCells;

static bool containsCell(const std::vector<Cell> &cells, Cell p)
{
    for (const Cell &cell : cells)
        if (cell == p)
            return true;

    return false;
}

static void markDirtyDynamicCellNoLock(Cell p)
{
    if (!inBounds(p.r, p.c))
        return;

    // Keep the list small during fast animation. Duplicate dirty cells do not
    // add information for path invalidation.
    if (containsCell(dirtyDynamicCells, p))
        return;

    dirtyDynamicCells.push_back(p);
}

std::vector<Cell> consumeDirtyDynamicCellsNoLock()
{
    std::vector<Cell> result = dirtyDynamicCells;
    dirtyDynamicCells.clear();
    return result;
}

bool hasDirtyDynamicCellsNoLock()
{
    return !dirtyDynamicCells.empty();
}

void clearDirtyDynamicCellsNoLock()
{
    dirtyDynamicCells.clear();
}

static int roundToCell(float v)
{
    return (int)std::lround(v);
}

static void traceObstacles(const std::string &source)
{
    logObstacleTraceSnapshot(
        source,
        obstacles,
        0,
        robotAvoidanceCell,
        robotAvoidanceEnabled,
        manualVehicleControlEnabled,
        manualVehicleControlIndex
    );
}

bool isForbiddenDynamicObstacleCell(int r, int c)
{
    if (!inBounds(r, c)) return true;
    if (isStaticBlocked(r, c)) return true;
    if (r == start.r && c == start.c) return true;
    return false;
}

void setRobotAvoidanceCell(Cell pos)
{
    robotAvoidanceCell = pos;
    robotAvoidanceEnabled = inBounds(pos.r, pos.c);
    traceObstacles("robot_avoidance_update");
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

static float currentObstacleSpeed(const DynamicObstacle &obs)
{
    return std::max(std::fabs(obs.vx), std::fabs(obs.vy));
}

static int movementTicksForCurrentVelocity(const DynamicObstacle &obs)
{
    float speed = currentObstacleSpeed(obs);
    if (speed <= 0.0f)
        return 0;

    int ticks = (int)std::ceil(1.0f / speed);
    return std::max(1, ticks);
}

static void snapObstacleToLogicalCellCenter(DynamicObstacle &obs)
{
    obs.x = (float)obs.pos.r;
    obs.y = (float)obs.pos.c;
}

static void moveVisualTowardLogicalCellCenter(DynamicObstacle &obs)
{
    float targetX = (float)obs.pos.r;
    float targetY = (float)obs.pos.c;
    float dx = targetX - obs.x;
    float dy = targetY - obs.y;
    float dist = std::sqrt(dx * dx + dy * dy);

    float speed = currentObstacleSpeed(obs);
    if (speed <= 0.0f || dist <= speed || dist < 0.001f)
    {
        obs.x = targetX;
        obs.y = targetY;
        return;
    }

    obs.x += speed * dx / dist;
    obs.y += speed * dy / dist;
}

static bool wouldThreatenRobot(const DynamicObstacle &obs, float nx, float ny)
{
    if (!robotAvoidanceEnabled) return false;
    if (obs.vx == 0.0f && obs.vy == 0.0f) return false;

    Cell next = roundedCell(nx, ny);
    Cell front = oneStepAheadByVelocity(obs);

    if (next == robotAvoidanceCell) return true;
    if (front == robotAvoidanceCell) return true;

    int curDist = manhattan(obs.pos, robotAvoidanceCell);
    int nextDist = manhattan(next, robotAvoidanceCell);
    return nextDist <= ROBOT_YIELD_RADIUS && nextDist < curDist;
}

static bool canPlace(int r, int c)
{
    if (isForbiddenDynamicObstacleCell(r, c)) return false;

    Cell p = {r, c};
    for (const DynamicObstacle &obs : obstacles)
        if (obs.pos == p)
            return false;

    if (isDynamicBlockedCell(r, c)) return false;
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
    obs.state = type == ObstacleType::GUARD ? GUARD_WAIT_CENTER : VEHICLE_WAIT;
    obstacles.push_back(obs);
}

static bool isLineFree(float x1, float y1, float x2, float y2)
{
    int r = roundToCell(x1), c = roundToCell(y1);
    int r2 = roundToCell(x2), c2 = roundToCell(y2);
    int dr = abs(r2 - r), dc = abs(c2 - c);
    int stepR = (r < r2) ? 1 : -1;
    int stepC = (c < c2) ? 1 : -1;
    int err = dr - dc;

    while (true)
    {
        if (isForbiddenDynamicObstacleCell(r, c)) return false;
        if (r == r2 && c == c2) break;
        int e2 = 2 * err;
        if (e2 > -dc) { err -= dc; r += stepR; }
        if (e2 < dr) { err += dr; c += stepC; }
    }
    return true;
}

static void clearReservationGrids()
{
    for (Cell p : occupiedReservationCells)
        if (inBounds(p.r, p.c))
            occupiedCount[p.r][p.c] = 0;

    occupiedReservationCells.clear();

    for (Cell p : reservedReservationCells)
        if (inBounds(p.r, p.c))
            reservedNext[p.r][p.c] = false;

    reservedReservationCells.clear();
}

static void addOccupiedCell(Cell p)
{
    if (!inBounds(p.r, p.c))
        return;

    if (occupiedCount[p.r][p.c] == 0)
        occupiedReservationCells.push_back(p);

    occupiedCount[p.r][p.c]++;
}

static void buildOccupiedGrid()
{
    clearReservationGrids();

    for (const auto &obs : obstacles)
        addOccupiedCell(obs.pos);
}

static bool occupiedByAnotherObstacle(const DynamicObstacle &obs, Cell p)
{
    if (!inBounds(p.r, p.c)) return true;
    int count = occupiedCount[p.r][p.c];
    if (p == obs.pos) count--;
    return count > 0;
}

static void reserveCell(Cell p)
{
    if (!inBounds(p.r, p.c))
        return;

    if (!reservedNext[p.r][p.c])
    {
        reservedNext[p.r][p.c] = true;
        reservedReservationCells.push_back(p);
    }
}

static bool canReserveCell(Cell p)
{
    if (!inBounds(p.r, p.c)) return false;
    return !reservedNext[p.r][p.c];
}

static void stopObstacleMovement(DynamicObstacle &obs)
{
    obs.vx = 0.0f;
    obs.vy = 0.0f;
    obs.moveTick = 0;

    // x/y are render coordinates only.  When an obstacle yields or is blocked,
    // make the rendered body stand exactly at the center of the logical cell.
    snapObstacleToLogicalCellCenter(obs);
    reserveCell(obs.pos);
}

static void advanceActiveGridMove(DynamicObstacle &obs)
{
    if (obs.moveTick <= 0)
        return;

    moveVisualTowardLogicalCellCenter(obs);
    obs.moveTick--;

    if (obs.moveTick <= 0)
    {
        snapObstacleToLogicalCellCenter(obs);
        obs.vx = 0.0f;
        obs.vy = 0.0f;
    }

    reserveCell(obs.pos);
}

static void startCommittedGridMove(DynamicObstacle &obs, Cell next)
{
    Cell from = obs.pos;

    // Start every grid step from the center of the previous logical cell.  The
    // renderer then interpolates x/y toward the new logical cell center.
    obs.x = (float)from.r;
    obs.y = (float)from.c;

    obs.pos = next;
    obs.moveTick = movementTicksForCurrentVelocity(obs);

    if (obs.moveTick <= 0)
    {
        snapObstacleToLogicalCellCenter(obs);
        reserveCell(obs.pos);
        return;
    }

    advanceActiveGridMove(obs);
}

static void moveStraightWithReservation(DynamicObstacle &obs)
{
    if (obs.moveTick > 0)
    {
        advanceActiveGridMove(obs);
        return;
    }

    if (obs.vx == 0.0f && obs.vy == 0.0f)
    {
        snapObstacleToLogicalCellCenter(obs);
        reserveCell(obs.pos);
        return;
    }

    Cell next = oneStepAheadByVelocity(obs);

    if (wouldThreatenRobot(obs, (float)next.r, (float)next.c) ||
        !isLineFree((float)obs.pos.r, (float)obs.pos.c, (float)next.r, (float)next.c) ||
        isForbiddenDynamicObstacleCell(next.r, next.c) ||
        occupiedByAnotherObstacle(obs, next) ||
        !canReserveCell(next))
    {
        stopObstacleMovement(obs);
        return;
    }

    startCommittedGridMove(obs, next);
}

static std::vector<Cell> collectCurrentObstacleCellsNoLock()
{
    std::vector<Cell> cells;

    for (const DynamicObstacle &obs : obstacles)
    {
        Cell p = obs.pos;

        if (!inBounds(p.r, p.c))
            continue;

        if (!containsCell(cells, p))
            cells.push_back(p);
    }

    return cells;
}

static void setDynamicBlockedCellNoLock(Cell p, bool blocked)
{
    if (!inBounds(p.r, p.c))
        return;

    if (dynamicBlocked[p.r][p.c] == blocked)
        return;

    dynamicBlocked[p.r][p.c] = blocked;
    markDirtyDynamicCellNoLock(p);
}

static void syncToGrid()
{
    std::vector<Cell> nextBlockedCells = collectCurrentObstacleCellsNoLock();

    for (Cell oldCell : currentDynamicBlockedCells)
        if (!containsCell(nextBlockedCells, oldCell))
            setDynamicBlockedCellNoLock(oldCell, false);

    for (Cell newCell : nextBlockedCells)
        if (!containsCell(currentDynamicBlockedCells, newCell))
            setDynamicBlockedCellNoLock(newCell, true);

    currentDynamicBlockedCells = nextBlockedCells;
}

static bool validManualVehicleIndexNoLock(int idx)
{
    return idx >= 0 && idx < (int)obstacles.size() && obstacles[idx].type == ObstacleType::VEHICLE;
}

static vector<int> collectManualVehicleCandidatesNoLock()
{
    vector<int> candidates;
    for (int i = 0; i < (int)obstacles.size(); i++)
        if (obstacles[i].type == ObstacleType::VEHICLE)
            candidates.push_back(i);
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
        if (i != selfIndex && obstacles[i].pos == p)
            return true;
    return false;
}

static void freezeManualVehicleNoLock(DynamicObstacle &obs)
{
    snapObstacleToLogicalCellCenter(obs);
    obs.vx = 0.0f;
    obs.vy = 0.0f;
    obs.moveTick = 0;
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
        traceObstacles("manual_toggle_off");
        return false;
    }

    vector<int> candidates = collectManualVehicleCandidatesNoLock();
    if (candidates.empty())
    {
        manualVehicleControlEnabled = false;
        manualVehicleControlIndex = -1;
        traceObstacles("manual_toggle_no_vehicle");
        return false;
    }

    uniform_int_distribution<int> dist(0, (int)candidates.size() - 1);
    manualVehicleControlIndex = candidates[dist(manualVehicleRng)];
    manualVehicleControlEnabled = true;
    freezeManualVehicleNoLock(obstacles[manualVehicleControlIndex]);
    syncToGrid();
    traceObstacles("manual_toggle_on");
    return true;
}

bool isManualVehicleControlEnabled()
{
    lock_guard<mutex> lock(simMutex);
    return manualVehicleControlEnabled && validManualVehicleIndexNoLock(manualVehicleControlIndex);
}

int manualControlledVehicleIndex()
{
    lock_guard<mutex> lock(simMutex);
    if (!manualVehicleControlEnabled || !validManualVehicleIndexNoLock(manualVehicleControlIndex)) return -1;
    return manualVehicleControlIndex;
}

Cell manualControlledVehicleCell()
{
    lock_guard<mutex> lock(simMutex);
    if (!manualVehicleControlEnabled || !validManualVehicleIndexNoLock(manualVehicleControlIndex)) return {0, 0};
    return obstacles[manualVehicleControlIndex].pos;
}

bool manualVehicleControlStep(int dir)
{
    lock_guard<mutex> lock(simMutex);
    if (!manualVehicleControlEnabled || !validManualVehicleIndexNoLock(manualVehicleControlIndex)) return false;
    if (dir < 0 || dir > 3) return false;

    DynamicObstacle &obs = obstacles[manualVehicleControlIndex];
    obs.dir = dir;
    Cell next = manualNextCell(obs.pos, dir);

    if (isForbiddenDynamicObstacleCell(next.r, next.c))
    {
        freezeManualVehicleNoLock(obs); syncToGrid(); traceObstacles("manual_move_blocked_forbidden"); return false;
    }
    if (robotAvoidanceEnabled && next == robotAvoidanceCell)
    {
        freezeManualVehicleNoLock(obs); syncToGrid(); traceObstacles("manual_move_blocked_robot_cell"); return false;
    }
    if (occupiedByOtherObstacleNoLock(manualVehicleControlIndex, next))
    {
        freezeManualVehicleNoLock(obs); syncToGrid(); traceObstacles("manual_move_blocked_other_obstacle"); return false;
    }

    obs.pos = next;
    freezeManualVehicleNoLock(obs);
    syncToGrid();
    traceObstacles("manual_move_success");
    return true;
}

static void updateBehavior(DynamicObstacle &obs)
{
    switch (obs.type)
    {
    case ObstacleType::GUARD: updateGuardBehavior(obs); break;
    case ObstacleType::VEHICLE: updateVehicleBehavior(obs); break;
    }
}

static void updateAllObstaclesSafely()
{
    if (manualVehicleControlEnabled && !validManualVehicleIndexNoLock(manualVehicleControlIndex))
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

        if (obs.moveTick > 0)
        {
            moveStraightWithReservation(obs);
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
        this_thread::sleep_for(chrono::milliseconds(OBSTACLE_FRAME_MS));
        if (stopRequested.load()) break;
        {
            lock_guard<mutex> lock(simMutex);
            updateAllObstaclesSafely();
            traceObstacles("auto_update");
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
    clearReservationGrids();
    currentDynamicBlockedCells.clear();

    for (DynamicObstacle &obs : obstacles)
    {
        obs.vx = 0.0f;
        obs.vy = 0.0f;
        obs.moveTick = 0;
        snapObstacleToLogicalCellCenter(obs);
    }

    for (int i = 1; i <= rows; i++)
        for (int j = 1; j <= cols; j++)
            dynamicBlocked[i][j] = false;
    syncToGrid();
    clearDirtyDynamicCellsNoLock();
    traceObstacles("init");
}

void startDynamicObstacle()
{
    stopRequested.store(false);
    if (obstacles.empty()) return;
    worker = thread(dynamicObstacleLoop);
}

void stopDynamicObstacle()
{
    stopRequested.store(true);
}

void waitDynamicObstacle()
{
    if (worker.joinable()) worker.join();
}
