#include "coverage_tick.h"

#include "behavior_log.h"
#include "coverage_handlers.h"
#include "coverage_timing.h"
#include "dynamic_obstacle.h"
#include "energy_model.h"
#include "mission_policy.h"
#include "mission_state.h"
#include "opencv.h"
#include "path_builder.h"
#include "path_safety.h"
#include "planner.h"
#include "return_to_base.h"
#include "robot_motion.h"

#include <iostream>
#include <vector>

using namespace std;

namespace
{
    int estimateCostToBase(const Robot &rb)
    {
        HeadingDir startDir = headingDirFromDegrees(rb.headingDeg);

        dijkstraOriented(
            rb.pos,
            startDir,
            PlannerObstacleMode::RESPECT_DYNAMIC
        );

        return bestOrientedDistanceTo(rb.base);
    }

    bool shouldRobotReturnForEnergy(const Robot &rb)
    {
        int costToBase = estimateCostToBase(rb);
        return shouldReturnForEnergy(rb, costToBase);
    }

    bool shouldStayInAlert(const Robot &rb)
    {
        return hasImmediateDynamicDanger(rb) ||
               hasBlockedCellAnywhereOnPath(rb);
    }

    bool isNormalModeValidation(const CoverageContext &ctx)
    {
        return ctx.mode == NORMAL || ctx.mode == FINAL_PUSH;
    }

    bool dirtyCellIntersectsPathRange(
        const Robot &rb,
        const std::vector<Cell> &dirtyCells,
        int beginIndex,
        int endIndex
    ) {
        if (beginIndex < 0)
            beginIndex = 0;

        if (endIndex > (int)rb.path.size())
            endIndex = (int)rb.path.size();

        if (beginIndex >= endIndex)
            return false;

        for (const Cell &dirty : dirtyCells)
            for (int i = beginIndex; i < endIndex; i++)
                if (rb.path[i] == dirty)
                    return true;

        return false;
    }

    bool dirtyCellNearRobot(
        const Robot &rb,
        const std::vector<Cell> &dirtyCells,
        int radius
    ) {
        for (const Cell &dirty : dirtyCells)
        {
            int dr = dirty.r - rb.pos.r;
            if (dr < 0) dr = -dr;

            int dc = dirty.c - rb.pos.c;
            if (dc < 0) dc = -dc;

            if (dr + dc <= radius)
                return true;
        }

        return false;
    }

    bool invalidatePathIfAffectedByDirtyCells(
        Robot &rb,
        CoverageContext &ctx
    ) {
        std::vector<Cell> dirtyCells = consumeDirtyDynamicCellsNoLock();

        if (dirtyCells.empty())
            return false;

        if (rb.pathID >= (int)rb.path.size())
            return false;

        int endIndex = (int)rb.path.size();
        std::string scope = "full_path";

        if (isNormalModeValidation(ctx))
        {
            PathSafetyConfig config;
            endIndex = std::min(
                (int)rb.path.size(),
                rb.pathID + config.pathLookahead
            );
            scope = "lookahead";
        }

        bool affected = dirtyCellIntersectsPathRange(
            rb,
            dirtyCells,
            rb.pathID,
            endIndex
        );

        if (!affected && !isNormalModeValidation(ctx))
            affected = dirtyCellNearRobot(rb, dirtyCells, 1);

        if (!affected)
            return false;

        std::string details;
        appendDetail(details, kv("dirty_cells", (int)dirtyCells.size()));
        appendDetail(details, kv("scope", scope));
        appendDetail(details, kv("path_index", rb.pathID));
        appendDetail(details, kv("path_len", (int)rb.path.size()));

        logRobotEvent(
            "WARN",
            "PATH",
            "lazy_dynamic_path_invalidated",
            "Dynamic obstacle changed a cell relevant to the current path; path is cleared lazily.",
            rb,
            ctx.mode,
            details
        );

        clearRobotPath(rb);
        ctx.stableStepCount = 0;

        if (ctx.mode == NORMAL)
            enterAlertMode(ctx);

        setHUDState("REPLAN");
        return true;
    }

    int movementEnergyCostForNextMove(const Robot &rb, RobotMode mode)
    {
        if (rb.pathID >= (int)rb.path.size())
            return 0;

        Cell next = rb.path[rb.pathID];
        return movementEnergyCostForStep(rb, next, mode);
    }

    int turnQuarterEnergyCostForNextMove(const Robot &rb)
    {
        if (rb.pathID >= (int)rb.path.size())
            return 0;

        Cell next = rb.path[rb.pathID];
        return turnQuarterEnergyCostForStep(rb, next);
    }

    void handleMoveResult(
        Robot &rb,
        CoverageContext &ctx,
        const RobotMoveResult &move
    ) {
        if (move.blocked)
        {
            handleBlockedNextCell(rb, ctx);
            return;
        }

        if (!move.moved)
            return;

        if (move.powerLoss)
        {
            clearRobotPath(rb);

            ctx.outcome = powerLossOutcome(ctx.coverageComplete);
            ctx.shouldStop = true;

            setHUDState("POWER_LOSS");
            return;
        }

        ctx.retryCount = 0;
        ctx.alertFailCount = 0;
        ctx.recoveryReplanTick = 0;

        if (ctx.mode != ALERT)
            return;

        setHUDState("ALERT");

        if (shouldStayInAlert(rb))
        {
            ctx.stableStepCount = 0;
            clearRobotPath(rb);
            return;
        }

        if (!move.enteredUncoveredCell)
        {
            ctx.stableStepCount = 0;
            clearRobotPath(rb);
            return;
        }

        ctx.stableStepCount++;

        cout << "[RECOVERY] Safe uncovered step "
             << ctx.stableStepCount
             << "/"
             << recoverySteps()
             << '\n';

        if (ctx.stableStepCount >= recoverySteps())
        {
            switchMissionMode(ctx, NORMAL);
        }
        else
        {
            clearRobotPath(rb);
        }
    }

    void handlePostMoveEnergy(Robot &rb, CoverageContext &ctx)
    {
        if (ctx.shouldStop || ctx.needWaitDraw)
            return;

        if (ctx.mode == FINAL_PUSH)
            return;

        if (ctx.mode == RETURN_TO_BASE || ctx.mode == RECHARGING)
            return;

        if (shouldRobotReturnForEnergy(rb))
            enterReturnToBase(ctx, rb);
    }

    bool advancePendingMoveIfNeeded(Robot &rb, CoverageContext &ctx)
    {
        if (!hasPendingRobotMove(ctx))
            return false;

        RobotMoveResult move = advancePendingRobotMove(rb, ctx);
        handleMoveResult(rb, ctx, move);

        if (move.moved)
            handlePostMoveEnergy(rb, ctx);

        ctx.needWaitDraw = true;
        return true;
    }

    void moveIfPossible(Robot &rb, CoverageContext &ctx)
    {
        if (ctx.shouldStop || ctx.needWaitDraw || ctx.mode == HOLD_SAFE)
            return;

        if (rb.pathID >= (int)rb.path.size())
            return;

        int movementCost =
            movementEnergyCostForNextMove(rb, ctx.mode);

        int turnQuarterCost =
            turnQuarterEnergyCostForNextMove(rb);

        RobotMoveResult move = moveRobotAlongCurrentPath(
            rb,
            ctx,
            movementCost,
            turnQuarterCost
        );

        handleMoveResult(rb, ctx, move);

        if (move.moved)
            handlePostMoveEnergy(rb, ctx);
    }
}

void processCoverageTick(Robot &rb, CoverageContext &ctx)
{
    if (advancePendingMoveIfNeeded(rb, ctx))
        return;

    if (ctx.actionCooldownTicks > 0)
    {
        ctx.actionCooldownTicks--;
        ctx.needWaitDraw = true;
        return;
    }

    if (ctx.mode == RECHARGING)
    {
        handleRecharging(rb, ctx);
        return;
    }

    if (ctx.mode == POWER_SAVE)
    {
        setHUDState("POWER_SAVE");

        ctx.outcome = powerSaveOutcome(ctx.coverageComplete);
        ctx.shouldStop = true;
        return;
    }

    if (ctx.mode == WAIT_FOR_COMMAND)
    {
        handleWaitForCommand(rb, ctx);
        return;
    }

    // Lazy path invalidation: dynamic obstacles may move every frame, but the
    // robot only clears/replans when the changed cells intersect its current
    // path (lookahead in NORMAL, full future path in recovery modes).
    if (invalidatePathIfAffectedByDirtyCells(rb, ctx))
        return;

    if (tryRecoveryReplanToCoverage(rb, ctx))
        return;

    if (ctx.mode == RETURN_TO_BASE)
    {
        handleReturnToBase(rb, ctx);
        return;
    }

    if (ctx.mode == HOLD_SAFE)
    {
        handleHoldSafe(rb, ctx);
        return;
    }

    if (ctx.mode != FINAL_PUSH && shouldRobotReturnForEnergy(rb))
    {
        enterReturnToBase(ctx, rb);
        return;
    }

    if (!ctx.shouldStop &&
        !ctx.needWaitDraw &&
        hasImmediateDynamicDanger(rb))
    {
        enterAlertMode(ctx);
    }

    if (!ctx.shouldStop &&
        !ctx.needWaitDraw &&
        hasBlockedCellAheadOnPath(rb))
    {
        handleActivePathObstructed(rb, ctx);
    }

    if (!ctx.shouldStop)
        planPathIfNeeded(rb, ctx);

    if (!ctx.shouldStop &&
        !ctx.needWaitDraw &&
        hasBlockedCellAheadOnPath(rb))
    {
        handleActivePathObstructed(rb, ctx);
    }

    moveIfPossible(rb, ctx);
}

void handleCoverageCompletion(
    CoverageContext &ctx,
    Robot &rb,
    bool &finished
) {
    if (hasPendingRobotMove(ctx))
        return;

    if (!allCovered())
        return;

    ctx.coverageComplete = true;

    if (isAtBase(rb))
    {
        clearRobotPath(rb);
        ctx.outcome = MISSION_SUCCESS;
        ctx.shouldStop = true;
        finished = true;
        setHUDState("DONE");
        return;
    }

    enterReturnToBase(ctx, rb);
}
