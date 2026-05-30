#include "robot_motion.h"

#include "behavior_log.h"
#include "coverage_timing.h"
#include "dynamic_obstacle.h"
#include "energy_model.h"
#include "grid.h"

#include <string>

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

    std::string cellStatusText(bool enteredUncoveredCell)
    {
        return enteredUncoveredCell ? "new" : "revisit";
    }

    std::string pathGoalText(const CoverageContext &ctx, const Robot &rb)
    {
        if (ctx.activeDecisionId > 0)
            return cellText(ctx.activeDecisionTarget);

        if (!rb.path.empty())
            return cellText(rb.path.back());

        return "none";
    }

    std::string purposeForMove(const CoverageContext &ctx)
    {
        if (ctx.activeDecisionId > 0 && ctx.activeDecisionPurpose != "unknown")
            return ctx.activeDecisionPurpose;

        if (ctx.mode == RETURN_TO_BASE)
            return "return";
        if (ctx.mode == ALERT)
            return "alert";
        if (ctx.mode == FINAL_PUSH)
            return "final_push";
        if (ctx.mode == HOLD_SAFE)
            return "hold_safe";
        if (ctx.mode == WAIT_FOR_COMMAND)
            return "wait_for_command";
        if (ctx.mode == RECHARGING)
            return "recharging";
        if (ctx.mode == POWER_SAVE)
            return "power_save";

        return "coverage";
    }

    std::string reasonForMove(const CoverageContext &ctx)
    {
        if (ctx.activeDecisionId > 0 && ctx.activeDecisionReason != "unknown")
            return ctx.activeDecisionReason;

        if (ctx.mode == RETURN_TO_BASE)
            return "return_to_base";

        return "follow_current_path";
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

        int edgeVisitCountAfter = rb.edgeCount[e];

        rb.steps++;
        consumeEnergy(rb, move.energyCost);

        result.moved = true;
        result.powerLoss = (rb.energy <= 0);

        std::string commonDetails =
            "decision_id=" + std::to_string(ctx.activeDecisionId) +
            " purpose=" + purposeForMove(ctx) +
            " reason=" + reasonForMove(ctx) +
            " from=" + cellText(prev) +
            " to=" + cellText(next) +
            " path_index_before=" + std::to_string(move.pathIndexBefore) +
            " path_index_after=" + std::to_string(rb.pathID) +
            " path_len=" + std::to_string(move.pathLength) +
            " path_goal=" + pathGoalText(ctx, rb) +
            " cell_status=" + cellStatusText(move.enteredUncoveredCell) +
            " edge_visit_count=" + std::to_string(edgeVisitCountAfter) +
            " move_cost=" + std::to_string(move.energyCost);

        if (result.powerLoss)
        {
            logRobotEvent(
                "ERROR",
                "MOVE",
                "commit_power_loss",
                "Robot reached the new cell but lost power after this step.",
                rb,
                ctx.mode,
                commonDetails
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
            "Move committed at cell center.",
            rb,
            ctx.mode,
            commonDetails +
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

        logRobotEvent(
            "WARN",
            "MOVE",
            "blocked_before_start",
            "Next path cell is not free, so the move cannot start.",
            rb,
            ctx.mode,
            "decision_id=" + std::to_string(ctx.activeDecisionId) +
            " purpose=" + purposeForMove(ctx) +
            " reason=" + reasonForMove(ctx) +
            " from=" + cellText(prev) +
            " to=" + cellText(next) +
            " path_index=" + std::to_string(rb.pathID) +
            " path_len=" + std::to_string((int)rb.path.size()) +
            " path_goal=" + pathGoalText(ctx, rb)
        );

        return result;
    }

    Edge e(prev, next);
    int edgeVisitCountBefore = 0;

    auto it = rb.edgeCount.find(e);
    if (it != rb.edgeCount.end())
        edgeVisitCountBefore = it->second;

    ctx.pendingMove.active = true;
    ctx.pendingMove.from = prev;
    ctx.pendingMove.to = next;
    ctx.pendingMove.energyCost = energyCost;
    ctx.pendingMove.elapsedTicks = 0;
    ctx.pendingMove.totalTicks = stepTicksForMode(ctx.mode);
    ctx.pendingMove.enteredUncoveredCell = result.enteredUncoveredCell;
    ctx.pendingMove.pathIndexBefore = rb.pathID;
    ctx.pendingMove.pathLength = (int)rb.path.size();
    ctx.pendingMove.edgeVisitCountBefore = edgeVisitCountBefore;

    logRobotEvent(
        "INFO",
        "MOVE",
        "start",
        "Move started along current path; the step is not committed yet.",
        rb,
        ctx.mode,
        "decision_id=" + std::to_string(ctx.activeDecisionId) +
        " purpose=" + ctx.activeDecisionPurpose +
        " reason=" + ctx.activeDecisionReason +
        " from=" + cellText(prev) +
        " to=" + cellText(next) +
        " path_index=" + std::to_string(rb.pathID) +
        " path_len=" + std::to_string((int)rb.path.size()) +
        " path_goal=" + pathGoalText(ctx, rb) +
        " next_cell_status=" + cellStatusText(result.enteredUncoveredCell) +
        " edge_visit_count_before=" + std::to_string(edgeVisitCountBefore) +
        " energy_before=" + energyText(rb) +
        " move_cost=" + std::to_string(energyCost) +
        " total_ticks=" + std::to_string(ctx.pendingMove.totalTicks)
    );

    return result;
}
