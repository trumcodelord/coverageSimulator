#pragma once

#include "types.h"

struct PathBuildResult
{
    bool success = false;
    bool alreadyAtGoal = false;

    // True when uncovered reachable cells exist, but none is safe to visit
    // under the current energy + return-margin policy.
    bool energyInfeasible = false;
};

void clearRobotPath(Robot &rb);

bool isAtBase(const Robot &rb);

PathBuildResult rebuildPathToBase(Robot &rb);

PathBuildResult rebuildSafeDetourPathToBase(Robot &rb);

PathBuildResult rebuildPathToNearestUncoveredTarget(Robot &rb);
