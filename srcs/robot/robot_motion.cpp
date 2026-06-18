#include "robot_motion.h"

#include "behavior_log.h"
#include "coverage_timing.h"
#include "dynamic_obstacle.h"
#include "energy_model.h"
#include "grid.h"
#include "opencv.h"

#include <algorithm>
#include <cmath>
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

    bool shouldOpportunisticallyRecharge(const Robot &rb, const CoverageContext &ctx)
    {
        return ctx.mode == NORMAL &&
               rb.pos == rb.base &&
               rb.energy < rb.maxEnergy;
    }

    void opportunisticRechargeAtBase(Robot &rb, CoverageContext &ctx)
    {
        int energyBefore = rb.energy;
        rb.energy = rb.maxEnergy;

        logRobotEvent(
            "INFO",
            "ENERGY",
            "opportunistic_recharge",
            "Robot happened to pass through base during coverage, so it conveniently recharges.",
            rb,
            ctx.mode,
            "decision_id=" + std::to_string(ctx.activeDecisionId) +
            " purpose=" + purposeForMove(ctx) +
            " reason=incidental_base_pass_through" +
            " energy_before=" + std::to_string(energyBefore) +
            " energy_after=" + std::to_string(rb.energy) +
            " base=" + cellText(rb.base)
        );

        pushHUDEvent("[RECHARGE] Tien the di ngang base, sac pin.");
    }

    double angleForMove(Cell from, Cell to)
    {
        int dr = to.r - from.r;
        int dc = to.c - from.c;

        if (dc > 0) return -90.0;
        if (dc < 0) return 90.0;
        if (dr > 0) return 180.0;
        return 0.0;
    }

    double normalizeAngle(double angle)
    {
        while (angle <= -180.0) angle += 360.0;
        while (angle > 180.0) angle -= 360.0;
        return angle;
    }

    double shortestTurnDelta(double fromDeg, double toDeg)
    {
        return normalizeAngle(toDeg - fromDeg);
    }

    int turnQuarterCount(double deltaDeg)
    {
        double amount = std::fabs(deltaDeg);
        if (amount < 1e-6)
            return 0;

        return amount > 135.0 ? 2 : 1;
    }

    void consumeTurnQuarterEnergy(Robot &rb, CoverageContext &ctx)
    {
        if (ctx.pendingMove.turnQuartersConsumed >=
            ctx.pendingMove.totalTurnQuarters)
        {
            return;
        }

        int before = rb.energy;
        consumeEnergy(rb, ctx.pendingMove.turnQuarterEnergyCost);

        ctx.pendingMove.turnQuartersConsumed++;

        logRobotEvent(
            rb.energy <= 0 ? "ERROR" : "INFO",
            "ENERGY",
            "turn_energy_consumed",
            "Robot consumes one quarter-turn energy unit while rotating in place.",
            rb,
            ctx.mode,
            "decision_id=" + std::to_string(ctx.activeDecisionId) +
            " purpose=" + purposeForMove(ctx) +
            " reason=" + reasonForMove(ctx) +
            " turn_quarter_cost=" + std::to_string(ctx.pendingMove.turnQuarterEnergyCost) +
            " turn_quarters_consumed=" + std::to_string(ctx.pendingMove.turnQuartersConsumed) +
            " total_turn_quarters=" + std::to_string(ctx.pendingMove.totalTurnQuarters) +
            " energy_before=" + std::to_string(before) +
            " energy_after=" + std::to_string(rb.energy) +
            " pos=" + cellText(rb.pos)
        );

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

        int energyBeforeMove = rb.energy;

        if (!isFree(next.r, next.c))
        {
            result.blocked = true;

            logRobotEvent(
                "WARN",
                "MOVE",
                "blocked_before_commit",
                "Target cell became unsafe while the robot was executing a pending move; commit is cancelled.",
                rb,
                ctx.mode,
                "decision_id=" + std::to_string(ctx.activeDecisionId) +
                " purpose=" + purposeForMove(ctx) +
                " reason=" + reasonForMove(ctx) +
                " from=" + cellText(prev) +
                " to=" + cellText(next) +
                " path_index_before=" + std::to_string(move.pathIndexBefore) +
                " path_index_current=" + std::to_string(rb.pathID) +
                " path_len=" + std::to_string(move.pathLength) +
                " path_goal=" + pathGoalText(ctx, rb) +
                " movement_cost_not_consumed=" + std::to_string(move.movementEnergyCost) +
                " energy=" + std::to_string(rb.energy) +
                " turn_quarters_consumed=" + std::to_string(move.turnQuartersConsumed) +
                " total_turn_quarters=" + std::to_string(move.totalTurnQuarters)
            );

            return result;
        }

        rb.pos = next;
        rb.headingDeg = move.targetAngleDeg;
        setRobotAvoidanceCell(rb.pos);

        rb.trail.push_back(rb.pos);
        rb.pathID++;

        Edge e(prev, next);
        rb.edgeCount[e]++;

        int edgeVisitCountAfter = rb.edgeCount[e];

        rb.steps++;
        consumeEnergy(rb, move.movementEnergyCost);

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
            " movement_cost=" + std::to_string(move.movementEnergyCost) +
            " energy_before_move=" + std::to_string(energyBeforeMove) +
            " energy_after_move=" + std::to_string(rb.energy) +
            " turn_delta_deg=" + std::to_string((int)move.turnDeltaDeg) +
            " turn_ticks=" + std::to_string(move.turnTicks) +
            " move_ticks=" + std::to_string(move.moveTicks);

        if (result.powerLoss)
        {
            logRobotEvent(
                "ERROR",
                "MOVE",
                "commit_power_loss",
                "Robot reached the new cell but lost power after movement energy was consumed.",
                rb,
                ctx.mode,
                commonDetails
            );

            ctx.shouldStop = true;
            return result;
        }

        markCovered(rb.pos.r, rb.pos.c);

        if (shouldOpportunisticallyRecharge(rb, ctx))
            opportunisticRechargeAtBase(rb, ctx);

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

float pendingRobotTurnProgress(const CoverageContext &ctx)
{
    if (!ctx.pendingMove.active || ctx.pendingMove.turnTicks <= 0)
        return 1.0f;

    if (ctx.pendingMove.phase != MOTION_TURNING)
        return 1.0f;

    float progress = (float)ctx.pendingMove.elapsedTicks /
                     (float)ctx.pendingMove.turnTicks;

    return std::max(0.0f, std::min(1.0f, progress));
}

float pendingRobotMoveProgress(const CoverageContext &ctx)
{
    if (!ctx.pendingMove.active)
        return 1.0f;

    if (ctx.pendingMove.phase == MOTION_TURNING)
        return 0.0f;

    if (ctx.pendingMove.moveTicks <= 0)
        return 1.0f;

    float progress = (float)ctx.pendingMove.elapsedTicks /
                     (float)ctx.pendingMove.moveTicks;

    return std::max(0.0f, std::min(1.0f, progress));
}

double pendingRobotVisualAngleDeg(const Robot &rb, const CoverageContext &ctx)
{
    if (!ctx.pendingMove.active)
        return rb.headingDeg;

    if (ctx.pendingMove.phase == MOTION_TURNING)
    {
        double progress = pendingRobotTurnProgress(ctx);

        return normalizeAngle(
            ctx.pendingMove.startAngleDeg +
            ctx.pendingMove.turnDeltaDeg * progress
        );
    }

    return ctx.pendingMove.targetAngleDeg;
}

RobotMoveResult advancePendingRobotMove(
    Robot &rb,
    CoverageContext &ctx
) {
    RobotMoveResult result;

    if (!ctx.pendingMove.active)
        return result;

    if (ctx.pendingMove.phase == MOTION_TURNING)
    {
        ctx.pendingMove.elapsedTicks++;

        int completedQuarters = 0;

        if (ctx.pendingMove.turnTicks > 0 &&
            ctx.pendingMove.totalTurnQuarters > 0)
        {
            completedQuarters =
                (ctx.pendingMove.elapsedTicks *
                 ctx.pendingMove.totalTurnQuarters) /
                ctx.pendingMove.turnTicks;
        }

        while (ctx.pendingMove.turnQuartersConsumed <
                   completedQuarters &&
               ctx.pendingMove.turnQuartersConsumed <
                   ctx.pendingMove.totalTurnQuarters)
        {
            consumeTurnQuarterEnergy(rb, ctx);

            if (rb.energy <= 0)
            {
                result.powerLoss = true;
                ctx.pendingMove = PendingRobotMove();
                ctx.shouldStop = true;
                return result;
            }
        }

        if (ctx.pendingMove.elapsedTicks < ctx.pendingMove.turnTicks)
            return result;

        rb.headingDeg = ctx.pendingMove.targetAngleDeg;
        ctx.pendingMove.phase = MOTION_MOVING;
        ctx.pendingMove.elapsedTicks = 0;
        return result;
    }

    ctx.pendingMove.elapsedTicks++;

    if (ctx.pendingMove.elapsedTicks < ctx.pendingMove.moveTicks)
        return result;

    return commitPendingMove(rb, ctx);
}

RobotMoveResult moveRobotAlongCurrentPath(
    Robot &rb,
    CoverageContext &ctx,
    int movementEnergyCost,
    int turnQuarterEnergyCost
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

    int stepTicks = stepTicksForMode(ctx.mode);
    double targetAngle = angleForMove(prev, next);
    double turnDelta = shortestTurnDelta(rb.headingDeg, targetAngle);
    int quarterTurns = turnQuarterCount(turnDelta);
    int turnTicks = quarterTurns * stepTicks;

    ctx.pendingMove.active = true;
    ctx.pendingMove.from = prev;
    ctx.pendingMove.to = next;
    ctx.pendingMove.movementEnergyCost = movementEnergyCost;
    ctx.pendingMove.turnQuarterEnergyCost = turnQuarterEnergyCost;
    ctx.pendingMove.totalTurnQuarters = quarterTurns;
    ctx.pendingMove.turnQuartersConsumed = 0;
    ctx.pendingMove.phase = turnTicks > 0 ? MOTION_TURNING : MOTION_MOVING;
    ctx.pendingMove.elapsedTicks = 0;
    ctx.pendingMove.totalTicks = turnTicks + stepTicks;
    ctx.pendingMove.turnTicks = turnTicks;
    ctx.pendingMove.moveTicks = stepTicks;
    ctx.pendingMove.startAngleDeg = rb.headingDeg;
    ctx.pendingMove.targetAngleDeg = targetAngle;
    ctx.pendingMove.turnDeltaDeg = turnDelta;
    ctx.pendingMove.enteredUncoveredCell = result.enteredUncoveredCell;
    ctx.pendingMove.pathIndexBefore = rb.pathID;
    ctx.pendingMove.pathLength = (int)rb.path.size();
    ctx.pendingMove.edgeVisitCountBefore = edgeVisitCountBefore;

    return advancePendingRobotMove(rb, ctx);
}
