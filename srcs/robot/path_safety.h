#pragma once

#include "types.h"

#include <vector>

struct PathSafetyConfig
{
    int dynamicDangerRadius = 1;
    int pathLookahead = 8;
};

bool isNearDynamicObstacle(Cell p, int radius = 1);

bool hasImmediateDynamicDanger(
    const Robot &rb,
    const PathSafetyConfig &config = PathSafetyConfig()
);

bool hasBlockedCellAheadOnPath(
    const Robot &rb,
    const PathSafetyConfig &config = PathSafetyConfig()
);

bool isPathNearDynamicObstacle(
    const std::vector<Cell> &path,
    int startIndex = 1,
    int radius = 1
);

bool isNextPathCellFree(const Robot &rb);
