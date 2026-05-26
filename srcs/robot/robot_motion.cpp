#include "robot_motion.h"

#include "coverage_timing.h"
#include "dynamic_obstacle.h"
#include "energy_model.h"
#include "grid.h"

namespace
{
    RobotMoveResult commitPendingMove(
        Robot &rb,
        CoverageContext &ctx
    ) {
        RobotMoveResult result;

        PendingRobotMove move = ctx.pendingMove;
        ctx.pendingMove = PendingRobotMove();

        Cell prev = move.from;
        Cell next = move.to;

        result.from = prev;
        result.to = next;
        result.enteredUncoveredCell = move.enteredUncoveredCell;

        rb.pos = next;
        setRobotAvoidanceCell(rb.pos);

        rb.trail.push_back(rb.pos);
        rb.pathID++;

        Edge e(prev, next);
        rb.edgeCount[e]++;

        rb.steps++;
        consumeEnergy(rb, move.energyCost);

        result.moved = true;
        result.powerLoss = (rb.energy <= 0);

        if (result.powerLoss)
        {
            ctx.shouldStop = true;
            return result;
        }

        markCovered(rb.pos.r, rb.pos.c);

        return result;
    }
}

bool hasPendingRobotMove(const CoverageContext &ctx)
{
    return ctx.pendingMove.active;
}

float pendingRobotMoveProgress(const CoverageContext &ctx)
{
    if (!ctx.pendingMove.active)
        return 1.0f;

    if (ctx.pendingMove.totalTicks <= 0)
        return 1.0f;

    float progress = (float)ctx.pendingMove.elapsedTicks /
                     (float)ctx.pendingMove.totalTicks;

    if (progress < 0.0f)
        progress = 0.0f;

    if (progress > 1.0f)
        progress = 1.0f;

    return progress;
}

RobotMoveResult advancePendingRobotMove(
    Robot &rb,
    CoverageContext &ctx
) {
    RobotMoveResult result;

    if (!ctx.pendingMove.active)
        return result;

    ctx.pendingMove.elapsedTicks++;

    if (ctx.pendingMove.elapsedTicks < ctx.pendingMove.totalTicks)
        return result;

    return commitPendingMove(rb, ctx);
}

RobotMoveResult moveRobotAlongCurrentPath(
    Robot &rb,
    CoverageContext &ctx,
    int energyCost
) {
    RobotMoveResult result;

    if (ctx.pendingMove.active)
        return advancePendingRobotMove(rb, ctx);

    if (rb.pathID >= (int)rb.path.size())
        return result;

    Cell prev = rb.pos;
    Cell next = rb.path[rb.pathID];

    result.from = prev;
    result.to = next;
    result.enteredUncoveredCell = !isCovered(next.r, next.c);

    if (!isFree(next.r, next.c))
    {
        result.blocked = true;
        return result;
    }

    ctx.pendingMove.active = true;
    ctx.pendingMove.from = prev;
    ctx.pendingMove.to = next;
    ctx.pendingMove.energyCost = energyCost;
    ctx.pendingMove.elapsedTicks = 0;
    ctx.pendingMove.totalTicks = stepTicksForMode(ctx.mode);
    ctx.pendingMove.enteredUncoveredCell = result.enteredUncoveredCell;

    return result;
}