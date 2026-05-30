#pragma once

#include "types.h"

#include <string>

struct PendingRobotMove
{
    bool active = false;

    Cell from = {0, 0};
    Cell to = {0, 0};

    int energyCost = 0;
    int elapsedTicks = 0;
    int totalTicks = 1;

    bool enteredUncoveredCell = false;

    int pathIndexBefore = 0;
    int pathLength = 0;
    int edgeVisitCountBefore = 0;
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

    // Debug/observability only. These fields do not affect planning.
    int decisionCounter = 0;
    int activeDecisionId = 0;
    Cell activeDecisionTarget = {0, 0};
    std::string activeDecisionPurpose = "coverage";
    std::string activeDecisionReason = "unknown";
    int activeDecisionCandidateCount = 0;
    int activeDecisionCostToTarget = 0;
    int activeDecisionCostTargetToBase = 0;

    bool coverageComplete = false;
    bool returnToTerminate = false;
    bool shouldStop = false;
    bool needWaitDraw = false;
};

void beginCoverageTick(CoverageContext &ctx);

void setCoverageCooldown(CoverageContext &ctx, int ticks);

void beginDecisionTrace(
    CoverageContext &ctx,
    const Robot &rb,
    const std::string &purpose,
    Cell target,
    const std::string &reason,
    int candidateCount = 0,
    int costToTarget = 0,
    int costTargetToBase = 0
);

void logDecisionPath(
    const CoverageContext &ctx,
    const Robot &rb,
    const std::string &event,
    bool success,
    const std::string &details = ""
);
