#pragma once

#include "coverage_context.h"
#include "types.h"

struct RobotMoveResult
{
    bool moved = false;
    bool blocked = false;
    bool powerLoss = false;
    bool enteredUncoveredCell = false;

    Cell from = {0, 0};
    Cell to = {0, 0};
};

RobotMoveResult moveRobotAlongCurrentPath(
    Robot &rb,
    CoverageContext &ctx,
    int movementEnergyCost,
    int turnQuarterEnergyCost
);

bool hasPendingRobotMove(const CoverageContext &ctx);

RobotMoveResult advancePendingRobotMove(
    Robot &rb,
    CoverageContext &ctx
);

float pendingRobotMoveProgress(const CoverageContext &ctx);
float pendingRobotTurnProgress(const CoverageContext &ctx);
double pendingRobotVisualAngleDeg(const Robot &rb, const CoverageContext &ctx);
