#include "coverage_context.h"

#include "behavior_log.h"
#include "energy_model.h"
#include "grid.h"

#include <string>

void beginCoverageTick(CoverageContext &ctx)
{
    ctx.needWaitDraw = false;
}

void setCoverageCooldown(CoverageContext &ctx, int ticks)
{
    ctx.actionCooldownTicks = ticks;
}

void beginDecisionTrace(
    CoverageContext &ctx,
    const Robot &rb,
    const std::string &purpose,
    Cell target,
    const std::string &reason,
    int candidateCount,
    double costToTarget,
    double costTargetToBase
) {
    ctx.decisionCounter++;
    ctx.activeDecisionId = ctx.decisionCounter;
    ctx.activeDecisionTarget = target;
    ctx.activeDecisionPurpose = purpose;
    ctx.activeDecisionReason = reason;
    ctx.activeDecisionCandidateCount = candidateCount;
    ctx.activeDecisionCostToTarget = costToTarget;
    ctx.activeDecisionCostTargetToBase = costTargetToBase;

    logRobotEvent(
        "INFO",
        "TARGET",
        "selected",
        "Target selected for current decision.",
        rb,
        ctx.mode,
        "decision_id=" + std::to_string(ctx.activeDecisionId) +
        " purpose=" + purpose +
        " target=" + cellText(target) +
        " reason=" + reason +
        " candidates=" + std::to_string(candidateCount) +
        " cost_to_target=" + formatEnergy(costToTarget) +
        " cost_target_to_base=" + formatEnergy(costTargetToBase) +
        " target_covered=" + boolText(isCovered(target.r, target.c))
    );
}

void logDecisionPath(
    const CoverageContext &ctx,
    const Robot &rb,
    const std::string &event,
    bool success,
    const std::string &details
) {
    std::string pathDetails =
        "decision_id=" + std::to_string(ctx.activeDecisionId) +
        " purpose=" + ctx.activeDecisionPurpose +
        " target=" + cellText(ctx.activeDecisionTarget) +
        " reason=" + ctx.activeDecisionReason +
        " candidates=" + std::to_string(ctx.activeDecisionCandidateCount) +
        " cost_to_target=" + formatEnergy(ctx.activeDecisionCostToTarget) +
        " cost_target_to_base=" + formatEnergy(ctx.activeDecisionCostTargetToBase) +
        " path_len=" + std::to_string((int)rb.path.size()) +
        " path_index=" + std::to_string(rb.pathID) +
        " result=" + std::string(success ? "success" : "fail");

    if (!rb.path.empty())
    {
        pathDetails +=
            " first=" + cellText(rb.path.front()) +
            " last=" + cellText(rb.path.back());
    }

    if (rb.pathID < (int)rb.path.size())
        pathDetails += " next=" + cellText(rb.path[rb.pathID]);

    if (!details.empty())
        pathDetails += " " + details;

    logRobotEvent(
        success ? "INFO" : "WARN",
        "PATH",
        event,
        success ? "Path built for current decision." : "Path build failed for current decision.",
        rb,
        ctx.mode,
        pathDetails
    );
}
