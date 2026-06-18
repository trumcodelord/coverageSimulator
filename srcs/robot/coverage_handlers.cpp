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

#include <string>
#include <vector>

using namespace std;

namespace
{
    void resetRecoveryCounters(CoverageContext &ctx)
    {
        ctx.holdCycleCount = 0;
        ctx.holdTick = 0;
        ctx.retryCount = 0;
        ctx.alertFailCount = 0;
        ctx.recoveryReplanTick = 0;
        ctx.needWaitDraw = false;
        ctx.actionCooldownTicks = 0;
    }

    bool isRecoveryMode(RobotMode mode)
    {
        return mode == ALERT ||
               mode == HOLD_SAFE ||
               mode == RETURN_TO_BASE;
    }

    void finishPartialReturnedAtBase(
        CoverageContext &ctx,
        Robot &rb,
        const string &message
    ) {
        logBehavior(message);

        clearRobotPath(rb);

        ctx.returnToTerminate = true;
        ctx.outcome = MISSION_PARTIAL_RETURNED;
        ctx.shouldStop = true;
        ctx.needWaitDraw = false;

        setHUDState("PARTIAL_RETURNED");
    }

    void enterSafeTerminationReturn(
        CoverageContext &ctx,
        Robot &rb,
        const char *message
    ) {
        resetRecoveryCounters(ctx);

        if (isAtBase(rb))
        {
            finishPartialReturnedAtBase(ctx, rb, message);
            return;
        }

        enterReturnToBase(ctx, rb, message);
        ctx.returnToTerminate = true;
    }

    void enterRecoveryReturn(
        CoverageContext &ctx,
        Robot &rb,
        const char *message
    ) {
        resetRecoveryCounters(ctx);
        ctx.returnToTerminate = false;

        if (isAtBase(rb))
        {
            logBehavior(message);
            clearRobotPath(rb);
            switchMissionMode(ctx, RECHARGING);
            setCoverageCooldown(ctx, rechargeWaitTicks());
            setHUDState("RECHARGING");
            return;
        }

        enterReturnToBase(ctx, rb, message);
        ctx.returnToTerminate = false;
    }

    void handleCurrentEnergyLowForCoverage(
        Robot &rb,
        CoverageContext &ctx
    ) {
        resetRecoveryCounters(ctx);

        if (isAtBase(rb))
        {
            logBehavior("[ENERGY] Dang sac pin.");
            clearRobotPath(rb);
            switchMissionMode(ctx, RECHARGING);
            setCoverageCooldown(ctx, rechargeWaitTicks());
            return;
        }

        enterReturnToBase(
            ctx,
            rb,
            "[ENERGY] Can them nang luong. Quay ve base."
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

bool tryRecoveryReplanToCoverage(Robot &rb, CoverageContext &ctx)
{
    if (ctx.shouldStop || ctx.needWaitDraw)
        return false;

    if (!isRecoveryMode(ctx.mode))
        return false;

    if (ctx.coverageComplete || ctx.returnToTerminate)
        return false;

    if (isAtBase(rb))
        return false;

    if (ctx.mode != RETURN_TO_BASE && rb.pathID < (int)rb.path.size())
        return false;

    ctx.recoveryReplanTick++;

    if (ctx.recoveryReplanTick < recoveryReplanInterval())
        return false;

    ctx.recoveryReplanTick = 0;

    vector<Cell> oldPath = rb.path;
    int oldPathID = rb.pathID;

    PathBuildResult recovered = rebuildPathToNearestUncoveredTarget(rb, &ctx);

    if (!recovered.success)
    {
        rb.path = oldPath;
        rb.pathID = oldPathID;
        return false;
    }

    logBehavior("[RECOVER] Moi truong da mo duong. Thu tiep tuc coverage.");

    ctx.retryCount = 0;
    ctx.alertFailCount = 0;
    ctx.holdTick = 0;
    ctx.holdCycleCount = 0;
    ctx.returnWaitCount = 0;
    ctx.stableStepCount = 0;
    ctx.actionCooldownTicks = 0;
    ctx.needWaitDraw = false;
    ctx.returnToTerminate = false;

    switchMissionMode(ctx, ALERT);
    setHUDState("RECOVER");
    return true;
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
        ctx.alertFailCount = 0;
        ctx.stableStepCount = 0;
        ctx.recoveryReplanTick = 0;
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

    int holdCycleLimit = isAtBase(rb)
        ? maxBaseNoPathHoldCycles()
        : maxHoldCyclesBeforeReturn();

    if (ctx.holdCycleCount == 1 || ctx.holdCycleCount % 5 == 0)
    {
        logBehavior(
            "[HOLD] Chua co duong an toan. Chu ky " +
            to_string(ctx.holdCycleCount) + "/" +
            to_string(holdCycleLimit) + "."
        );
    }

    if (ctx.holdCycleCount > holdCycleLimit)
    {
        if (isAtBase(rb))
        {
            finishPartialReturnedAtBase(
                ctx,
                rb,
                "[MISSION] Da ve base nhung khong tim duoc duong tiep tuc sau " +
                to_string(maxBaseNoPathHoldCycles()) +
                " chu ky. Ket thuc nhiem vu mot phan."
            );
            return;
        }

        logBehavior("[RETURN] Cho qua lau. Ve base de replan/recharge.");

        enterRecoveryReturn(
            ctx,
            rb,
            "[RETURN] Bi chan dong qua lau. Quay ve base de phuc hoi."
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
    ctx.returnToTerminate = false;
    ctx.recoveryReplanTick = 0;
    switchMissionMode(ctx, NORMAL);
}
