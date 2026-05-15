#pragma once

#include "types.h"

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

    bool coverageComplete = false;
    bool shouldStop = false;
    bool needWaitDraw = false;
};

void beginCoverageTick(CoverageContext &ctx);

void setCoverageCooldown(CoverageContext &ctx, int ticks);
