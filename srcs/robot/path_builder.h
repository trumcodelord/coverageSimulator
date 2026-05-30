#pragma once

#include "coverage_context.h"
#include "types.h"

struct PathBuildResult
{
    bool success = false;
    bool alreadyAtGoal = false;

    // True when a reachable uncovered cell would be feasible after recharge,
    // but is not feasible with the robot's current remaining energy.
    bool currentEnergyLow = false;

    // True when reachable uncovered cells exist, but even full battery capacity
    // cannot visit any of them and return with the configured safety margin.
    bool energyInfeasible = false;
};

void clearRobotPath(Robot &rb);

bool isAtBase(const Robot &rb);

PathBuildResult rebuildPathToBase(Robot &rb, CoverageContext *ctx = nullptr);

PathBuildResult rebuildSafeDetourPathToBase(Robot &rb, CoverageContext *ctx = nullptr);

PathBuildResult rebuildPathToNearestUncoveredTarget(Robot &rb, CoverageContext *ctx = nullptr);
