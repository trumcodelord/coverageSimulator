#pragma once

#include "coverage_context.h"
#include "types.h"

struct PathBuildResult
{
    bool success = false;
    bool alreadyAtGoal = false;

    bool currentEnergyLow = false;
    bool energyInfeasible = false;

    double pathCost = INF;
};

void clearRobotPath(Robot &rb);

bool isAtBase(const Robot &rb);

PathBuildResult rebuildPathToBase(Robot &rb, CoverageContext *ctx = nullptr);

PathBuildResult rebuildSafeDetourPathToBase(Robot &rb, CoverageContext *ctx = nullptr);

PathBuildResult rebuildPathToNearestUncoveredTarget(Robot &rb, CoverageContext *ctx = nullptr);
