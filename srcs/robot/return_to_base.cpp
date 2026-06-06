#include "return_to_base.h"

#include "behavior_log.h"
#include "coverage_timing.h"
#include "energy_model.h"
#include "grid.h"
#include "mission_policy.h"
#include "mission_state.h"
#include "opencv.h"
#include "path_builder.h"
#include "path_safety.h"
#include "planner.h"
#include "robot_motion.h"
#include "tactical_yield.h"

#include <queue>
#include <vector>

using namespace std;

namespace
{
    int estimateCostToBase(const Robot &rb)
    {
        dijkstra(rb.pos, d, trace);
        return d[rb.base.r][rb.base.c];
    }

    int estimateStaticCostToBase(const Robot &rb)
    {
        static int staticDist[1001][1001];

        for (int i = 1; i <= rows; i++)
        {
            for (int j = 1; j <= cols; j++)
                staticDist[i][j] = INF;
        }

        queue<Cell> q;
        staticDist[rb.pos.r][rb.pos.c] = 0;
        q.push(rb.pos);

        while (!q.empty())
        {
            Cell u = q.front();
            q.pop();

            for (int k = 1; k <= 4; k++)
            {
                Cell v = {u.r + dr[k], u.c + dc[k]};

                if (!inBounds(v.r, v.c))
                    continue;

                if (isStaticBlocked(v.r, v.c))
                    continue;

                if (staticDist[v.r][v.c] <= staticDist[u.r][u.c] + 1)
                    continue;

                staticDist[v.r][v.c] = staticDist[u.r][u.c] + 1;
                q.push(v);
            }
        }

        return staticDist[rb.base.r][rb.base.c];
    }

    bool canReturnAfterDynamicObstacleClears(const Robot &rb)
    {
        int staticCostToBase = estimateStaticCostToBase(rb);

        if (staticCostToBase >= INF)
            return false;

        return rb.energy >= staticCostToBase;
    }

    bool isRobotCriticalEnergy(const Robot &rb)
    {
        int staticCostToBase = estimateStaticCostToBase(rb);
        return isCriticalEnergy(rb, staticCostToBase);
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

        Cell next = rb.path[rb.pathID];

        int movementCost =
            movementEnergyCostForStep(rb, next, ctx.mode);

        int turnQuarterCost =
            turnQuarterEnergyCostForStep(rb, next);

        RobotMoveResult move = moveRobotAlongCurrentPath(
            rb,
            ctx,
            movementCost,
            turnQuarterCost
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
        logBehavior(message);

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
        logBehavior(message);

        enterWaitForCommandMode(ctx, rb);
        setCoverageCooldown(ctx, commandWaitTicks());
    }
}

void waitReturnToBase(
    CoverageContext &ctx,
    Robot &rb,
    const char *message
) {
    switchMissionMode(ctx, RETURN_TO_BASE);
    setHUDState("RETURN_WAIT");

    logBehavior(message);

    clearRobotPath(rb);
    ctx.needWaitDraw = true;
    ctx.returnWaitCount++;

    if (ctx.returnWaitCount >= minReturnWaitBeforeYield() &&
        canReturnAfterDynamicObstacleClears(rb))
    {
        if (tryTacticalYieldMove(rb, ctx))
        {
            logBehavior("[YIELD] Robot tam lui de nhuong duong.");
            switchMissionMode(ctx, RETURN_TO_BASE);

            ctx.needWaitDraw = false;
            ctx.returnWaitCount = 0;

            setHUDState("YIELD");
            setCoverageCooldown(ctx, stepTicksForMode(ctx.mode));
            return;
        }
    }

    if (ctx.returnWaitCount >= maxReturnWaitBeforeDetour())
    {
        logBehavior("[RETURN] Cho qua lau. Thu tim duong vong.");

        PathBuildResult detour = rebuildSafeDetourPathToBase(rb, &ctx);

        if (detour.success)
        {
            switchMissionMode(ctx, RETURN_TO_BASE);

            ctx.needWaitDraw = false;
            ctx.returnWaitCount = 0;
            setHUDState("RETURN_DETOUR");
            return;
        }
    }

    if (!canReturnAfterDynamicObstacleClears(rb) &&
        isRobotCriticalEnergy(rb) &&
        ctx.returnWaitCount > maxReturnWaitWhenCritical())
    {
        if (ctx.coverageComplete)
        {
            enterPowerSaveForReturn(
                ctx,
                rb,
                "[POWER_SAVE] Da phu xong nhung khong du pin ve base."
            );
        }
        else
        {
            enterWaitForCommandFromReturn(
                ctx,
                rb,
                "[COMMAND] Khong du pin ve base. Cho chi thi."
            );
        }

        return;
    }

    setCoverageCooldown(ctx, blockedWaitTicks());
}

void enterReturnToBase(
    CoverageContext &ctx,
    Robot &rb,
    const char *message
) {
    if (ctx.mode == RETURN_TO_BASE || ctx.mode == RECHARGING)
    {
        setHUDState("RETURN_TO_BASE");
        return;
    }

    logBehavior(message);

    int costToBase = estimateCostToBase(rb);
    logRobotEvent(
        "WARN",
        "ENERGY",
        "return_to_base_requested",
        "Robot switches to return-to-base because continuing coverage is not safe or mission requires return.",
        rb,
        ctx.mode,
        "cost_to_base=" + std::to_string(costToBase) +
        " action=return_to_base"
    );

    rb.returnCount++;
    ctx.returnWaitCount = 0;
    ctx.returnToTerminate = false;

    clearRobotPath(rb);
    switchMissionMode(ctx, RETURN_TO_BASE);
    setHUDState("RETURN_TO_BASE");

    PathBuildResult path = rebuildPathToBase(rb, &ctx);

    if (!path.success)
    {
        waitReturnToBase(
            ctx,
            rb,
            "[RETURN] Chua co duong ve base. Dang cho."
        );
    }
}

void handleReturnToBase(Robot &rb, CoverageContext &ctx)
{
    switchMissionMode(ctx, RETURN_TO_BASE);
    setHUDState("RETURN_TO_BASE");

    if (isAtBase(rb))
    {
        clearRobotPath(rb);

        if (ctx.coverageComplete)
        {
            ctx.outcome = MISSION_SUCCESS;
            ctx.shouldStop = true;
            setHUDState("DONE");
            logBehavior("[MISSION] Hoan thanh. Robot da ve base.");
            return;
        }

        if (ctx.returnToTerminate)
        {
            ctx.outcome = MISSION_PARTIAL_RETURNED;
            ctx.shouldStop = true;
            ctx.needWaitDraw = false;
            setHUDState("PARTIAL_RETURNED");
            logBehavior("[MISSION] Ket thuc mot phan. Robot da ve base.");
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
            "[ALERT] Vat can dong qua gan. Tam dung."
        );
        return;
    }

    if (rb.pathID >= (int)rb.path.size())
    {
        PathBuildResult path = rebuildPathToBase(rb, &ctx);

        if (!path.success)
        {
            waitReturnToBase(
                ctx,
                rb,
                "[RETURN] Duong ve dang bi chan. Dang cho."
            );
            return;
        }
    }

    if (hasBlockedCellAheadOnPath(rb))
    {
        waitReturnToBase(
            ctx,
            rb,
            "[BLOCKED] Duong ve dang bi chan."
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
            "[BLOCKED] O tiep theo khong an toan."
        );
        return;
    }

    int stepsBefore = rb.steps;

    Cell next = rb.path[rb.pathID];

    int movementCost =
        movementEnergyCostForStep(rb, next, ctx.mode);

    int turnQuarterCost =
        turnQuarterEnergyCostForStep(rb, next);

    RobotMoveResult move = moveRobotAlongCurrentPath(
        rb,
        ctx,
        movementCost,
        turnQuarterCost
    );

    if (move.powerLoss)
    {
        clearRobotPath(rb);
        ctx.outcome = powerLossOutcome(ctx.coverageComplete);
        ctx.shouldStop = true;
        setHUDState("POWER_LOSS");
        logBehavior("[MISSION] Mat nguon.");
        return;
    }

    if (!ctx.shouldStop && !ctx.needWaitDraw && rb.steps > stepsBefore)
    {
        ctx.returnWaitCount = 0;
        setCoverageCooldown(ctx, stepTicksForMode(ctx.mode));
    }
}
