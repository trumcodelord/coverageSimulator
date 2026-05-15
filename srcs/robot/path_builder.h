#pragma once

#include "types.h"

struct PathBuildResult
{
    bool success = false;
    bool alreadyAtGoal = false;
};

void clearRobotPath(Robot &rb);

bool isAtBase(const Robot &rb);

PathBuildResult rebuildPathToBase(Robot &rb);

PathBuildResult rebuildSafeDetourPathToBase(Robot &rb);

PathBuildResult rebuildPathToNearestUncoveredTarget(Robot &rb);
