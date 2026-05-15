#include "coverage_handlers.h"

#include "coverage_timing.h"
#include "mission_policy.h"
#include "mission_state.h"
#include "opencv.h"
#include "path_builder.h"
#include "path_safety.h"
#include "robot_lifecycle.h"

#include <iostream>

using namespace std;

void printRetryMessage(const char *msg, int retryCount)
{
    if (retryCount == 1 || retryCount % retryLogInterval() == 0)
    {
        cout << msg << " Retry " << retryCount
             << "/" << maxRetryCount() << '\n';
    }
}

void handleWaitForCommand(Robot &rb, CoverageContext &ctx)
{
    setHUDState("WAIT_FOR_COMMAND");

    if (criticalDirective() == PRESERVE)
    {
        cout << "[COMMAND] PRESERVE. Chuyen sang POWER_SAVE.\n";

        clearRobotPath(rb);
        switchMissionMode(ctx, POWER_SAVE);

        ctx.outcome = powerSaveOutcome(ctx.coverageComplete);
        ctx.shouldStop = true;
        ctx.needWaitDraw = false;
        return;
    }

    if (criticalDirective() == HEROIC)
    {
        cout << "[COMMAND] HEROIC. Tiep tuc nhiem vu den khi het nang luong.\n";
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
        cout << "[HOLD] Active path bi chan lien tiep, chuyen sang HOLD_SAFE.\n";

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
        cout << "[RECOVER] Co duong usable tro lai, roi HOLD_SAFE.\n";

        switchMissionMode(ctx, ALERT);

        ctx.retryCount = 0;
        ctx.holdCycleCount = 0;
        ctx.actionCooldownTicks = 0;
        ctx.needWaitDraw = false;
        return;
    }

    if (ctx.holdCycleCount == 1 || ctx.holdCycleCount % 5 == 0)
    {
        cout << "[HOLD] Chua co duong an toan. Tiep tuc cho. Cycle "
             << ctx.holdCycleCount << "/" << maxHoldCycles() << '\n';
    }

    if (ctx.holdCycleCount > maxHoldCycles())
    {
        cout << "[STOP] HOLD_SAFE qua lau ma van khong co duong phuc hoi.\n";

        ctx.outcome = stoppedOutcome(ctx.coverageComplete);
        ctx.shouldStop = true;
        setHUDState("STOP");
    }
    else
    {
        setCoverageCooldown(ctx, holdWaitTicks());
    }
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
        cout << "[STOP] Khong tim duoc target/path sau nhieu lan thu lai.\n";

        ctx.outcome = stoppedOutcome(ctx.coverageComplete);
        ctx.shouldStop = true;
        setHUDState("STOP");
        return;
    }

    if (ctx.alertFailCount >= alertFailToHold())
    {
        cout << "[HOLD] Alert that bai lien tiep, chuyen sang HOLD_SAFE.\n";

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
        cout << "[STOP] Bi chan duong qua nhieu lan, dung mo phong.\n";

        ctx.outcome = stoppedOutcome(ctx.coverageComplete);
        ctx.shouldStop = true;
        setHUDState("STOP");
        return;
    }

    if (ctx.alertFailCount >= alertFailToHold())
    {
        cout << "[HOLD] Khong co buoc an toan huu ich, chuyen sang HOLD_SAFE.\n";

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

    cout << "[RECHARGE] Da sac day pin.\n";

    clearRobotPath(rb);
    switchMissionMode(ctx, NORMAL);
}
