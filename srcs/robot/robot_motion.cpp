#include "robot_motion.h"

#include "behavior_log.h"
#include "coverage_timing.h"
#include "dynamic_obstacle.h"
#include "energy_model.h"
#include "grid.h"

namespace
{
    int countCoveredCells()
    {
        int count = 0;

        for (int r = 1; r <= rows; r++)
            for (int c = 1; c <= cols; c++)
                if (covered[r][c])
                    count++;

        return count;
    }

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
            logRobotEvent(
                "ERROR",
                "MOVE",
                "commit_power_loss",
                "Robot reached the new cell but lost power after this step.",
                rb,
                ctx.mode,
                "from=" + cellText(prev) +
                " to=" + cellText(next) +
                " move_cost=" + std::to_string(move.energyCost)
            );

            ctx.shouldStop = true;
            return result;
        }

        markCovered(rb.pos.r, rb.pos.c);

        int coveredCount = countCoveredCells();
        int rewardNewCell = move.enteredUncoveredCell ? 1 : 0;
        int penaltyRevisit = move.enteredUncoveredCell ? 0 : 1;

        logRobotEvent(
            "INFO",
            "MOVE",
            "commit",
            "Robot reached the center of the new cell; the step is committed now.",
            rb,
            ctx.mode,
            "from=" + cellText(prev) +
            " to=" + cellText(next) +
            " move_cost=" + std::to_string(move.energyCost) +
            " covered=" + std::to_string(coveredCount) + "/" + std::to_string(initialFreeCells) +
            " reward_new_cell=" + std::to_string(rewardNewCell) +
            " penalty_revisit=" + std::to_string(penaltyRevisit)
        );

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

    logRobotEvent(
        "INFO",
        "MOVE",
        "start",
        "Robot starts moving to the next cell; the step is not committed yet.",
        rb,
        ctx.mode,
        "from=" + cellText(prev) +
        " to=" + cellText(next) +
        " energy_before=" + energyText(rb) +
        " move_cost=" + std::to_string(energyCost) +
        " total_ticks=" + std::to_string(ctx.pendingMove.totalTicks)
    );

    return result;
}
