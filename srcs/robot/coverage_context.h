#pragma once

#include "types.h"

struct PendingRobotMove
{
    bool active = false;

    Cell from = {0, 0};
    Cell to = {0, 0};

    int energyCost = 0;
    int elapsedTicks = 0;
    int totalTicks = 1;

    bool enteredUncoveredCell = false;
};

struct CoverageContext
{
    RobotMode mode = NORMAL;
    MissionOutcome outcome = MISSION_RUNNING;

    int retryCount = 0;
    int stableStepCount = 0;
    int alertFailCount = 0;
    int holdTick = 0;
    int holdCycleCount = 0;
    int returnWaitCount = 0;

    int actionCooldownTicks = 0;
    PendingRobotMove pendingMove;

    bool coverageComplete = false;
    bool returnToTerminate = false;
    bool shouldStop = false;
    bool needWaitDraw = false;
};

void beginCoverageTick(CoverageContext &ctx);

void setCoverageCooldown(CoverageContext &ctx, int ticks);