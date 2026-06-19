#pragma once

#include "grid.h"
#include <vector>

enum HeadingDir : unsigned char
{
    DIR_NORTH = 0,
    DIR_EAST = 1,
    DIR_SOUTH = 2,
    DIR_WEST = 3
};

enum class PlannerObstacleMode : unsigned char
{
    RESPECT_DYNAMIC,
    IGNORE_DYNAMIC
};

struct OrientedTraceState
{
    short r = 0;
    short c = 0;
    signed char dir = -1;
};

extern double orientedDist[1001][1001][4];
extern OrientedTraceState orientedTrace[1001][1001][4];

HeadingDir headingDirFromDegrees(double headingDeg);
int quarterTurnsBetween(HeadingDir from, HeadingDir to);

void dijkstraOriented(
    Cell start,
    HeadingDir startDir,
    PlannerObstacleMode obstacleMode = PlannerObstacleMode::RESPECT_DYNAMIC
);

double orientedDistanceTo(Cell goal, HeadingDir goalDir);
double bestOrientedDistanceTo(Cell goal, HeadingDir *bestDir = nullptr);

std::vector<Cell> tracePathOriented(
    Cell start,
    HeadingDir startDir,
    Cell goal,
    HeadingDir goalDir
);

std::vector<Cell> traceBestPathOriented(
    Cell start,
    HeadingDir startDir,
    Cell goal
);
