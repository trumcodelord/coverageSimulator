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
            logBehavior("[ENERGY] Nang luong hien tai khong du de tiep tuc coverage. Sac lai pin.");
            clearRobotPath(rb);
            switchMissionMode(ctx, RECHARGING);
            setCoverageCooldown(ctx, rechargeWaitTicks());
            return;
        }

        enterReturnToBase(
            ctx,
            rb,
            "[ENERGY] Nang luong hien tai khong du cho target tiep theo. Quay ve base de sac."
        );
    }

    void handleEnergyInfeasibleCoverageTarget(
        Robot &rb,
        CoverageContext &ctx
    ) {
        logBehavior("[ENERGY] Ngay ca khi sac day, khong con uncovered target nao kha thi voi return margin hien tai.");

        enterSafeTerminationReturn(
            ctx,
            rb,
            "[RETURN] Coverage objective khong kha thi voi dung luong pin hien tai. Quay ve base de ket thuc an toan."
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
        logBehavior("[COMMAND] PRESERVE. Chuyen sang POWER_SAVE.");

        clearRobotPath(rb);
        switchMissionMode(ctx, POWER_SAVE);

        ctx.outcome = powerSaveOutcome(ctx.coverageComplete);
        ctx.shouldStop = true;
        ctx.needWaitDraw = false;
        return;
    }

    if (criticalDirective() == HEROIC)
    {
        logBehavior("[COMMAND] HEROIC. Tiep tuc nhiem vu den khi het nang luong.");
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
        "[ALERT] Dynamic obstacle nam tren active path.",
        ctx.retryCount
    );

    if (ctx.alertFailCount >= alertFailToHold())
    {
        logBehavior("[HOLD] Active path bi chan lien tiep, chuyen sang HOLD_SAFE.");

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

    PathBuildResult recovered = rebuildPathToNearestUncoveredTarget(rb);

    if (recovered.success)
    {
        logBehavior("[RECOVER] Co duong usable tro lai, roi HOLD_SAFE.");

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
            "[HOLD] Chua co duong an toan. Tiep tuc cho. Cycle " +
            to_string(ctx.holdCycleCount) + "/" +
            to_string(maxHoldCycles())
        );
    }

    if (ctx.holdCycleCount > maxHoldCycles())
    {
        logBehavior("[RETURN] HOLD_SAFE qua lau, khong the tiep tuc coverage. Thu quay ve base.");

        enterSafeTerminationReturn(
            ctx,
            rb,
            "[RETURN] Khong recover duoc coverage path. Quay ve base neu con kha nang."
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
        "[WAIT] Chua co target/path usable tam thoi.",
        ctx.retryCount
    );

    if (ctx.retryCount > maxRetryCount())
    {
        logBehavior("[STOP] Khong tim duoc target/path sau nhieu lan thu lai.");

        ctx.outcome = stoppedOutcome(ctx.coverageComplete);
        ctx.shouldStop = true;
        setHUDState("STOP");
        return;
    }

    if (ctx.alertFailCount >= alertFailToHold())
    {
        logBehavior("[HOLD] Alert that bai lien tiep, chuyen sang HOLD_SAFE.");

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

    PathBuildResult built = rebuildPathToNearestUncoveredTarget(rb);

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

    printRetryMessage("[WAIT] O ke tiep dang bi chan.", ctx.retryCount);

    if (ctx.retryCount > maxRetryCount())
    {
        logBehavior("[STOP] Bi chan duong qua nhieu lan, dung mo phong.");

        ctx.outcome = stoppedOutcome(ctx.coverageComplete);
        ctx.shouldStop = true;
        setHUDState("STOP");
        return;
    }

    if (ctx.alertFailCount >= alertFailToHold())
    {
        logBehavior("[HOLD] Khong co buoc an toan huu ich, chuyen sang HOLD_SAFE.");

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

    logBehavior("[RECHARGE] Da sac day pin.");

    clearRobotPath(rb);
    switchMissionMode(ctx, NORMAL);
}
