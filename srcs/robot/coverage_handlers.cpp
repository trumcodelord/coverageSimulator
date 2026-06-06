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
            logBehavior("[ENERGY] Nang luong thap. Dang sac pin.");
            clearRobotPath(rb);
            switchMissionMode(ctx, RECHARGING);
            setCoverageCooldown(ctx, rechargeWaitTicks());
            return;
        }

        enterReturnToBase(
            ctx,
            rb,
            "[ENERGY] Nang luong thap. Quay ve base."
        );
    }

    void handleEnergyInfeasibleCoverageTarget(
        Robot &rb,
        CoverageContext &ctx
    ) {
        logBehavior("[ENERGY] Pin day van khong du cho muc tieu con lai.");

        enterSafeTerminationReturn(
            ctx,
            rb,
            "[RETURN] Nhiem vu khong kha thi. Quay ve base."
        );
    }
}

void printRetryMessage(const char *msg, int retryCount)
{
    if (retryCount == 1 || retryCount % retryLogInterval() == 0)
    {
        logBehavior(
            string(msg) + " Thu lai " +
            to_string(retryCount) + "/" +
            to_string(maxRetryCount()) + "."
        );
    }
}

void handleWaitForCommand(Robot &rb, CoverageContext &ctx)
{
    setHUDState("WAIT_FOR_COMMAND");

    if (criticalDirective() == PRESERVE)
    {
        logBehavior("[COMMAND] Bao toan nang luong. Chuyen sang nghi.");

        clearRobotPath(rb);
        switchMissionMode(ctx, POWER_SAVE);

        ctx.outcome = powerSaveOutcome(ctx.coverageComplete);
        ctx.shouldStop = true;
        ctx.needWaitDraw = false;
        return;
    }

    if (criticalDirective() == HEROIC)
    {
        logBehavior("[COMMAND] Tiep tuc nhiem vu den khi het pin.");
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
        "[ALERT] Vat can dong chan duong.",
        ctx.retryCount
    );

    if (ctx.alertFailCount >= alertFailToHold())
    {
        logBehavior("[HOLD] Duong bi chan nhieu lan. Tam dung.");

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
        logBehavior("[RECOVER] Da tim lai duong an toan.");

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
            "[HOLD] Chua co duong an toan. Chu ky " +
            to_string(ctx.holdCycleCount) + "/" +
            to_string(maxHoldCycles()) + "."
        );
    }

    if (ctx.holdCycleCount > maxHoldCycles())
    {
        logBehavior("[RETURN] Cho qua lau. Thu quay ve base.");

        enterSafeTerminationReturn(
            ctx,
            rb,
            "[RETURN] Khong the tiep tuc. Quay ve base."
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
        "[WAIT] Chua co muc tieu hoac duong di.",
        ctx.retryCount
    );

    if (ctx.retryCount > maxRetryCount())
    {
        logBehavior("[STOP] Khong tim duoc duong sau nhieu lan thu.");

        ctx.outcome = stoppedOutcome(ctx.coverageComplete);
        ctx.shouldStop = true;
        setHUDState("STOP");
        return;
    }

    if (ctx.alertFailCount >= alertFailToHold())
    {
        logBehavior("[HOLD] Thu lai nhieu lan. Tam dung.");

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

    printRetryMessage("[WAIT] O tiep theo dang bi chan.", ctx.retryCount);

    if (ctx.retryCount > maxRetryCount())
    {
        logBehavior("[STOP] Duong bi chan qua nhieu lan.");

        ctx.outcome = stoppedOutcome(ctx.coverageComplete);
        ctx.shouldStop = true;
        setHUDState("STOP");
        return;
    }

    if (ctx.alertFailCount >= alertFailToHold())
    {
        logBehavior("[HOLD] Khong co buoc di an toan. Tam dung.");

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
