#include "coverage_tick.h"

#include "coverage_handlers.h"
#include "coverage_timing.h"
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

using namespace std;

namespace
{
    int estimateCostToBase(const Robot &rb)
    {
        dijkstra(rb.pos, d, trace);
        return d[rb.base.r][rb.base.c];
    }

    bool shouldRobotReturnForEnergy(const Robot &rb)
    {
        int costToBase = estimateCostToBase(rb);
        return shouldReturnForEnergy(rb, costToBase);
    }

    bool shouldStayInAlert(const Robot &rb)
    {
        return hasImmediateDynamicDanger(rb) ||
               hasBlockedCellAheadOnPath(rb);
    }

    int energyCostForNextMove(const Robot &rb, RobotMode mode)
    {
        if (rb.pathID >= (int)rb.path.size())
            return 0;

        Cell next = rb.path[rb.pathID];
        return computeMoveEnergyCost(rb, next, mode);
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

    void moveIfPossible(Robot &rb, CoverageContext &ctx)
    {
        if (ctx.shouldStop || ctx.needWaitDraw || ctx.mode == HOLD_SAFE)
            return;

        if (rb.pathID >= (int)rb.path.size())
            return;

        int stepsBefore = rb.steps;

        int energyCost = energyCostForNextMove(rb, ctx.mode);

        RobotMoveResult move = moveRobotAlongCurrentPath(
            rb,
            ctx,
            energyCost
        );

        handleMoveResult(rb, ctx, move);

        if (!ctx.shouldStop &&
            !ctx.needWaitDraw &&
            rb.steps > stepsBefore)
        {
            if (ctx.mode != FINAL_PUSH && shouldRobotReturnForEnergy(rb))
            {
                enterReturnToBase(ctx, rb);
                setCoverageCooldown(ctx, stepTicksForMode(ctx.mode));
                return;
            }

            setCoverageCooldown(ctx, stepTicksForMode(ctx.mode));
        }
    }
}

void processCoverageTick(Robot &rb, CoverageContext &ctx)
{
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
    if (!allCovered())
        return;

    ctx.coverageComplete = true;

    if (isAtBase(rb))
    {
        clearRobotPath(rb);

        ctx.outcome = MISSION_SUCCESS;

        setHUDState("DONE");

        finished = true;
        return;
    }

    if (ctx.mode == POWER_SAVE || ctx.mode == WAIT_FOR_COMMAND)
        return;

    if (ctx.mode != RETURN_TO_BASE && ctx.mode != RECHARGING)
    {
        enterReturnToBase(
            ctx,
            rb,
            "[MISSION] Da phu het ban do. Quay ve base de ket thuc nhiem vu."
        );
    }
}