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

// Full future-path check. This is stricter than lookahead and is used when
// a dynamic obstacle, especially a manually controlled vehicle, may have moved
// onto an already planned path outside the short lookahead window.
bool hasBlockedCellAnywhereOnPath(const Robot &rb, int startIndex = -1);

bool isPathNearDynamicObstacle(
    const std::vector<Cell> &path,
    int startIndex = 1,
    int radius = 1
);

bool isPathBlockedByDynamicObstacle(
    const std::vector<Cell> &path,
    int startIndex = 1
);

bool isNextPathCellFree(const Robot &rb);
