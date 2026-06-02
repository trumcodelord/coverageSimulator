#include "coverage_handlers.h"

#include "behavior_log.h"
#include "coverage_timing.h"
#include "mission_policy.h"
#include "mission_state.h"
#include "opencv.h"
#include "path_builder.h"
#include "path_safety.h"
#include "return_to_base.h"
#include "robot_lifecycle.h"

using namespace std;

namespace
{
    void resetRecoveryCounters(CoverageContext &ctx)
    {
        ctx.holdCycleCount = 0;
        ctx.holdTick = 0;
        ctx.retryCount = 0;
        ctx.alertFailCount = 0;
        ctx.needWaitDraw = false;
        ctx.actionCooldownTicks = 0;
    }

    void enterSafeTerminationReturn(
        CoverageContext &ctx,
        Robot &rb,
        const char *message
    ) {
        resetRecoveryCounters(ctx);

        if (isAtBase(rb))
        {
            logBehavior(message);
            clearRobotPath(rb);

            ctx.returnToTerminate = true;
            ctx.outcome = MISSION_PARTIAL_RETURNED;
            ctx.shouldStop = true;
            setHUDState("PARTIAL_RETURNED");
            return;
        }

        enterReturnToBase(ctx, rb, message);
        ctx.returnToTerminate = true;
    }

    void handleCurrentEnergyLowForCoverage(
        Robot &rb,
        CoverageContext &ctx
    ) {
        resetRecoveryCounters(ctx);

        if (isAtBase(rb))
        {
            logBehavior("[ENERGY] Short on energy. Recharging.");
            clearRobotPath(rb);
            switchMissionMode(ctx, RECHARGING);
            setCoverageCooldown(ctx, rechargeWaitTicks());
            return;
        }

        enterReturnToBase(
            ctx,
            rb,
            "[ENERGY] Current energy is not enough for the next target. Returning to base to recharge."
        );
    }

    void handleEnergyInfeasibleCoverageTarget(
        Robot &rb,
        CoverageContext &ctx
    ) {
        logBehavior("[ENERGY] Even full battery cannot reach any remaining uncovered target and return with margin.");

        enterSafeTerminationReturn(
            ctx,
            rb,
            "[RETURN] Coverage objective is infeasible with current battery capacity. Returning to base safely."
        );
    }
}

void printRetryMessage(const char *msg, int retryCount)
{
    if (retryCount == 1 || retryCount % retryLogInterval() == 0)
    {
        logBehavior(
            string(msg) + " Retry " +
            to_string(retryCount) + "/" +
            to_string(maxRetryCount())
        );
    }
}

void handleWaitForCommand(Robot &rb, CoverageContext &ctx)
{
    setHUDState("WAIT_FOR_COMMAND");

    if (criticalDirective() == PRESERVE)
    {
        logBehavior("[COMMAND] PRESERVE. Switching to POWER_SAVE.");

        clearRobotPath(rb);
        switchMissionMode(ctx, POWER_SAVE);

        ctx.outcome = powerSaveOutcome(ctx.coverageComplete);
        ctx.shouldStop = true;
        ctx.needWaitDraw = false;
        return;
    }

    if (criticalDirective() == HEROIC)
    {
        logBehavior("[COMMAND] HEROIC. Continuing mission until energy is exhausted.");
        enterFinalPushMode(ctx, rb);
        return;
    }
}

void handleActivePathObstructed(Robot &rb, CoverageContext &ctx)
{
    clearRobotPath(rb);
    enterAlertMode(ctx);

    ctx.retryCount++;
    ctx.alertFailCount++;

    printRetryMessage(
        "[ALERT] Dynamic obstacle is on the active path.",
        ctx.retryCount
    );

    if (ctx.alertFailCount >= alertFailToHold())
    {
        logBehavior("[HOLD] Active path was blocked repeatedly. Switching to HOLD_SAFE.");

        enterHoldSafeMode(ctx, rb);
        setCoverageCooldown(ctx, holdWaitTicks());
        return;
    }

    ctx.needWaitDraw = true;
    setCoverageCooldown(ctx, blockedWaitTicks());
}

void handleHoldSafe(Robot &rb, CoverageContext &ctx)
{
    clearRobotPath(rb);

    ctx.holdTick++;
    ctx.needWaitDraw = true;
    setHUDState("HOLD_SAFE");

    if (ctx.holdTick < holdReplanInterval())
    {
        setCoverageCooldown(ctx, holdWaitTicks());
        return;
    }

    ctx.holdTick = 0;
    ctx.holdCycleCount++;

    PathBuildResult recovered = rebuildPathToNearestUncoveredTarget(rb, &ctx);

    if (recovered.success)
    {
        logBehavior("[RECOVER] A usable coverage path is available again. Leaving HOLD_SAFE.");

        switchMissionMode(ctx, ALERT);

        ctx.retryCount = 0;
        ctx.holdCycleCount = 0;
        ctx.actionCooldownTicks = 0;
        ctx.needWaitDraw = false;
        return;
    }

    if (recovered.currentEnergyLow)
    {
        handleCurrentEnergyLowForCoverage(rb, ctx);
        return;
    }

    if (recovered.energyInfeasible)
    {
        handleEnergyInfeasibleCoverageTarget(rb, ctx);
        return;
    }

    if (ctx.holdCycleCount == 1 || ctx.holdCycleCount % 5 == 0)
    {
        logBehavior(
            "[HOLD] No safe path yet. Continuing to wait. Cycle " +
            to_string(ctx.holdCycleCount) + "/" +
            to_string(maxHoldCycles())
        );
    }

    if (ctx.holdCycleCount > maxHoldCycles())
    {
        logBehavior("[RETURN] HOLD_SAFE lasted too long. Cannot continue coverage. Trying to return to base.");

        enterSafeTerminationReturn(
            ctx,
            rb,
            "[RETURN] Coverage path could not recover. Returning to base if possible."
        );
        return;
    }

    setCoverageCooldown(ctx, holdWaitTicks());
}

void handleNoUsablePath(Robot &rb, CoverageContext &ctx)
{
    ctx.retryCount++;

    enterAlertMode(ctx);
    ctx.alertFailCount++;

    printRetryMessage(
        "[WAIT] No usable target/path is available right now.",
        ctx.retryCount
    );

    if (ctx.retryCount > maxRetryCount())
    {
        logBehavior("[STOP] Could not find target/path after many retries.");

        ctx.outcome = stoppedOutcome(ctx.coverageComplete);
        ctx.shouldStop = true;
        setHUDState("STOP");
        return;
    }

    if (ctx.alertFailCount >= alertFailToHold())
    {
        logBehavior("[HOLD] Alert failed repeatedly. Switching to HOLD_SAFE.");

        enterHoldSafeMode(ctx, rb);
        setCoverageCooldown(ctx, holdWaitTicks());
        return;
    }

    ctx.needWaitDraw = true;
    setCoverageCooldown(ctx, noTargetWaitTicks());
    setHUDState("WAIT");
}

void planPathIfNeeded(Robot &rb, CoverageContext &ctx)
{
    if (ctx.needWaitDraw || ctx.mode == HOLD_SAFE)
        return;

    if (rb.pathID < (int)rb.path.size())
        return;

    PathBuildResult built = rebuildPathToNearestUncoveredTarget(rb, &ctx);

    if (!built.success)
    {
        if (built.currentEnergyLow)
        {
            handleCurrentEnergyLowForCoverage(rb, ctx);
            return;
        }

        if (built.energyInfeasible)
        {
            handleEnergyInfeasibleCoverageTarget(rb, ctx);
            return;
        }

        handleNoUsablePath(rb, ctx);
        return;
    }
}

void handleBlockedNextCell(Robot &rb, CoverageContext &ctx)
{
    clearRobotPath(rb);

    ctx.retryCount++;
    enterAlertMode(ctx);
    ctx.alertFailCount++;

    printRetryMessage("[WAIT] The next cell is blocked.", ctx.retryCount);

    if (ctx.retryCount > maxRetryCount())
    {
        logBehavior("[STOP] Path was blocked too many times. Stopping simulation.");

        ctx.outcome = stoppedOutcome(ctx.coverageComplete);
        ctx.shouldStop = true;
        setHUDState("STOP");
        return;
    }

    if (ctx.alertFailCount >= alertFailToHold())
    {
        logBehavior("[HOLD] No useful safe step is available. Switching to HOLD_SAFE.");

        enterHoldSafeMode(ctx, rb);
        setCoverageCooldown(ctx, holdWaitTicks());
        return;
    }

    ctx.needWaitDraw = true;
    setCoverageCooldown(ctx, blockedWaitTicks());
    setHUDState("WAIT");
}

void handleRecharging(Robot &rb, CoverageContext &ctx)
{
    setHUDState("RECHARGING");

    rechargeRobot(rb);

    logBehavior("[RECHARGE] Battery is fully recharged.");

    clearRobotPath(rb);
    switchMissionMode(ctx, NORMAL);
}
