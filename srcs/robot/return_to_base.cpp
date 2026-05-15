#include "return_to_base.h"

#include "coverage_timing.h"
#include "energy_model.h"
#include "mission_policy.h"
#include "mission_state.h"
#include "opencv.h"
#include "path_builder.h"
#include "path_safety.h"
#include "planner.h"
#include "robot_motion.h"
#include "tactical_yield.h"

#include <iostream>
#include <vector>

using namespace std;

namespace
{
    int estimateCostToBase(const Robot &rb)
    {
        dijkstra(rb.pos, d, trace);
        return d[rb.base.r][rb.base.c];
    }

    bool isRobotCriticalEnergy(const Robot &rb)
    {
        int costToBase = estimateCostToBase(rb);
        return isCriticalEnergy(rb, costToBase);
    }

    bool tryTacticalYieldMove(Robot &rb, CoverageContext &ctx)
    {
        TacticalYieldResult yield = findTacticalYieldCell(rb);

        if (!yield.found)
            return false;

        dijkstra(rb.pos, d, trace);

        vector<Cell> yieldPath = tracePath(rb.pos, yield.target, trace);

        if ((int)yieldPath.size() <= 1)
            return false;

        rb.path = yieldPath;
        rb.pathID = 1;

        int stepsBefore = rb.steps;

        RobotMoveResult move = moveRobotAlongCurrentPath(
            rb,
            ctx,
            1
        );

        if (!move.moved || move.blocked || ctx.shouldStop)
            return false;

        if (rb.steps <= stepsBefore)
            return false;

        clearRobotPath(rb);
        return true;
    }

    void enterPowerSaveForReturn(
        CoverageContext &ctx,
        Robot &rb,
        const char *message
    ) {
        cout << message << '\n';

        clearRobotPath(rb);
        switchMissionMode(ctx, POWER_SAVE);

        ctx.outcome = powerSaveOutcome(ctx.coverageComplete);
        ctx.shouldStop = true;
        ctx.needWaitDraw = false;

        setHUDState("POWER_SAVE");
    }

    void enterWaitForCommandFromReturn(
        CoverageContext &ctx,
        Robot &rb,
        const char *message
    ) {
        cout << message << '\n';

        enterWaitForCommandMode(ctx, rb);
        setCoverageCooldown(ctx, commandWaitTicks());
    }
}

void waitReturnToBase(
    CoverageContext &ctx,
    Robot &rb,
    const char *message
) {
    cout << message << '\n';

    clearRobotPath(rb);
    ctx.needWaitDraw = true;
    ctx.returnWaitCount++;

    if (ctx.returnWaitCount >= minReturnWaitBeforeYield() &&
        !isRobotCriticalEnergy(rb))
    {
        if (tryTacticalYieldMove(rb, ctx))
        {
            cout << "[YIELD] Robot tam lui de giai phong diem nghen.\n";

            ctx.needWaitDraw = false;
            ctx.returnWaitCount = 0;

            setHUDState("YIELD");
            setCoverageCooldown(ctx, stepTicksForMode(ctx.mode));
            return;
        }
    }

    if (ctx.returnWaitCount >= maxReturnWaitBeforeDetour())
    {
        cout << "[RETURN] Cho qua lau, thu tim duong vong.\n";

        PathBuildResult detour = rebuildSafeDetourPathToBase(rb);

        if (detour.success)
        {
            ctx.needWaitDraw = false;
            ctx.returnWaitCount = 0;
            setHUDState("RETURN_DETOUR");
            return;
        }
    }

    if (isRobotCriticalEnergy(rb) &&
        ctx.returnWaitCount > maxReturnWaitWhenCritical())
    {
        if (ctx.coverageComplete)
        {
            enterPowerSaveForReturn(
                ctx,
                rb,
                "[POWER_SAVE] Da phu xong nhung duong ve khong an toan va nang luong nguy cap."
            );
        }
        else
        {
            enterWaitForCommandFromReturn(
                ctx,
                rb,
                "[COMMAND] Nang luong nguy cap va duong ve bi chan. Xin chi thi."
            );
        }

        return;
    }

    setHUDState("RETURN_WAIT");
    setCoverageCooldown(ctx, blockedWaitTicks());
}

void enterReturnToBase(
    CoverageContext &ctx,
    Robot &rb,
    const char *message
) {
    if (ctx.mode == RETURN_TO_BASE || ctx.mode == RECHARGING)
        return;

    cout << message << '\n';

    rb.returnCount++;
    ctx.returnWaitCount = 0;

    clearRobotPath(rb);
    switchMissionMode(ctx, RETURN_TO_BASE);

    PathBuildResult path = rebuildPathToBase(rb);

    if (!path.success)
    {
        waitReturnToBase(
            ctx,
            rb,
            "[RETURN] Chua tim duoc duong ve base. Cho va thu lai."
        );
    }
}

void handleReturnToBase(Robot &rb, CoverageContext &ctx)
{
    setHUDState("RETURN_TO_BASE");

    if (isAtBase(rb))
    {
        clearRobotPath(rb);

        if (ctx.coverageComplete)
        {
            ctx.outcome = MISSION_SUCCESS;
            ctx.shouldStop = true;
            setHUDState("DONE");
            return;
        }

        switchMissionMode(ctx, RECHARGING);
        setCoverageCooldown(ctx, rechargeWaitTicks());
        return;
    }

    if (hasImmediateDynamicDanger(rb))
    {
        waitReturnToBase(
            ctx,
            rb,
            "[RETURN] Vat can dong qua gan, dung cho an toan."
        );
        return;
    }

    if (rb.pathID >= (int)rb.path.size())
    {
        PathBuildResult path = rebuildPathToBase(rb);

        if (!path.success)
        {
            waitReturnToBase(
                ctx,
                rb,
                "[RETURN] Duong ve base tam thoi bi chan. Thu lai sau."
            );
            return;
        }
    }

    if (hasBlockedCellAheadOnPath(rb))
    {
        waitReturnToBase(
            ctx,
            rb,
            "[RETURN] Active return path bi chan, cho/replan."
        );
        return;
    }

    if (rb.pathID >= (int)rb.path.size())
        return;

    if (!isNextPathCellFree(rb))
    {
        waitReturnToBase(
            ctx,
            rb,
            "[RETURN] O tiep theo khong an toan, dung cho."
        );
        return;
    }

    int stepsBefore = rb.steps;

    RobotMoveResult move = moveRobotAlongCurrentPath(
        rb,
        ctx,
        1
    );

    if (move.powerLoss)
    {
        clearRobotPath(rb);
        ctx.outcome = powerLossOutcome(ctx.coverageComplete);
        ctx.shouldStop = true;
        setHUDState("POWER_LOSS");
        return;
    }

    if (!ctx.shouldStop && !ctx.needWaitDraw && rb.steps > stepsBefore)
        setCoverageCooldown(ctx, stepTicksForMode(ctx.mode));
}
