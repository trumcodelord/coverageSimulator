#pragma once

#include <vector>
#include <mutex>
#include "grid.h"

enum class ObstacleType
{
    GUARD,
    VEHICLE
};

enum ObstacleState
{
    VEHICLE_WAIT,
    VEHICLE_MOVE,

    GUARD_WAIT_CENTER,
    GUARD_MOVE_OUT,
    GUARD_MOVE_BACK
};

struct DynamicObstacle
{
    Cell pos;
    ObstacleType type;
    int dir = 0;
    ObstacleState state = VEHICLE_WAIT;
    std::vector<Cell> path;
    int stateTick = 0;
    int waitTick = 0;
    int moveTick = 0;
    float x = 0.0f, y = 0.0f;
    float vx = 0.0f, vy = 0.0f;
};

extern std::vector<DynamicObstacle> obstacles;
extern std::mutex simMutex;

void initDynamicObstacle();
void startDynamicObstacle();
void stopDynamicObstacle();
void waitDynamicObstacle();
void addObstacle(int r, int c, ObstacleType type);
void setRobotAvoidanceCell(Cell pos);

// Shared safety rule for all dynamic obstacle behavior modules.
// Dynamic obstacles may move through free operational space, but they must not
// occupy static obstacles, out-of-bound cells, or the robot base/start cell.
bool isForbiddenDynamicObstacleCell(int r, int c);
