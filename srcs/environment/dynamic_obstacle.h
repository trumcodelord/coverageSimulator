#pragma once

#include <vector>
#include <mutex>
#include "grid.h"

enum class ObstacleType
{
    GUARD,
    VEHICLE,
    RANDOM
};

enum ObstacleState
{
    VEHICLE_WAIT,
    VEHICLE_MOVE,

    GUARD_WAIT_CENTER,
    GUARD_MOVE_OUT,
    GUARD_WAIT_OUT,
    GUARD_MOVE_BACK,

    RANDOM_WAIT,
    RANDOM_PICK_DIRECTION,
    RANDOM_MOVE
};

struct DynamicObstacle
{
    Cell pos;
    ObstacleType type;
    int dir = 0;
    ObstacleState state = RANDOM_WAIT;
    std::vector<Cell> path;
    int pathIndex = 0;
    int stateTick = 0;
    int waitTick = 0;
    int moveTick = 0;
    int extraTick = 0;
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
