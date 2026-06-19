#pragma once

#include "types.h"

#include <string>

enum RobotMotionPhase
{
    MOTION_IDLE,
    MOTION_TURNING,
    MOTION_MOVING
};

struct PendingRobotMove
{
    bool active = false;

    Cell from = {0, 0};
    Cell to = {0, 0};

    double movementEnergyCost = 0.0;
    double turnQuarterEnergyCost = 0.0;
    int totalTurnQuarters = 0;
    int turnQuartersConsumed = 0;

    RobotMotionPhase phase = MOTION_IDLE;

    int elapsedTicks = 0;
    int totalTicks = 1;

    int turnTicks = 0;
    int moveTicks = 1;

    double startAngleDeg = 0.0;
    double targetAngleDeg = 0.0;
    double turnDeltaDeg = 0.0;

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

    int recoveryReplanTick = 0;

    int actionCooldownTicks = 0;
    PendingRobotMove pendingMove;

    int decisionCounter = 0;
    int activeDecisionId = 0;
    Cell activeDecisionTarget = {0, 0};
    std::string activeDecisionPurpose = "coverage";
    std::string activeDecisionReason = "unknown";
    int activeDecisionCandidateCount = 0;
    double activeDecisionCostToTarget = 0.0;
    double activeDecisionCostTargetToBase = 0.0;

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
    double costToTarget = 0.0,
    double costTargetToBase = 0.0
);

void logDecisionPath(
    const CoverageContext &ctx,
    const Robot &rb,
    const std::string &event,
    bool success,
    const std::string &details = ""
);
