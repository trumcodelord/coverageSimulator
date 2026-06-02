#include "path_builder.h"
#include "behavior_log.h"
#include "coverage_context.h"
#include "energy_model.h"
#include "planner.h"
#include "path_safety.h"

#include <algorithm>
#include <string>
#include <vector>

using namespace std;

namespace
{
    struct CoverageCandidate
    {
        int costFromRobot = INF;
        Cell target = {0, 0};
    };

    struct PathValueStats
    {
        int pathCost = 0;
        int newCells = 0;
        int revisits = 0;
        int blockedCells = 0;
    };

    bool compareCandidateByDistance(
        const CoverageCandidate &a,
        const CoverageCandidate &b
    ) {
        if (a.costFromRobot != b.costFromRobot)
            return a.costFromRobot < b.costFromRobot;

        if (a.target.r != b.target.r)
            return a.target.r < b.target.r;

        return a.target.c < b.target.c;
    }

    vector<CoverageCandidate> collectReachableUncoveredCandidates(
        const Robot &rb
    ) {
        vector<CoverageCandidate> candidates;

        dijkstra(rb.pos, d, trace);

        for (int i = 1; i <= rows; i++)
        {
            for (int j = 1; j <= cols; j++)
            {
                if (blocked[i][j] || covered[i][j])
                    continue;

                if (d[i][j] >= INF)
                    continue;

                candidates.push_back({d[i][j], {i, j}});
            }
        }

        sort(
            candidates.begin(),
            candidates.end(),
            compareCandidateByDistance
        );

        return candidates;
    }

    int estimateCostFromTargetToBase(Cell target, const Robot &rb)
    {
        dijkstra(target, d, trace);
        return d[rb.base.r][rb.base.c];
    }

    bool canFullBatteryVisitTargetAndReturn(
        const Robot &rb,
        int costToTarget,
        int costTargetToBase
    ) {
        Robot fullBatteryRobot = rb;
        fullBatteryRobot.energy = fullBatteryRobot.maxEnergy;

        return canVisitTargetAndReturn(
            fullBatteryRobot,
            costToTarget,
            costTargetToBase
        );
    }

    long long requiredEnergyLowerBound(int costToTarget, int costToBase)
    {
        if (costToTarget >= INF || costToBase >= INF)
            return INF;

        return (long long)costToTarget + (long long)costToBase;
    }

    long long requiredEnergyWithMargin(int costToTarget, int costToBase)
    {
        if (costToTarget >= INF || costToBase >= INF)
            return INF;

        int margin = returnMarginForCost(costToBase);
        if (margin >= INF)
            return INF;

        return (long long)costToTarget + (long long)costToBase + (long long)margin;
    }

    long long energySlack(const Robot &rb, int costToTarget, int costToBase)
    {
        long long required = requiredEnergyWithMargin(costToTarget, costToBase);

        if (required >= INF)
            return -INF;

        return (long long)rb.energy - required;
    }

    int pathCost(const vector<Cell> &path)
    {
        if (path.size() <= 1)
            return 0;

        int total = 0;

        for (int i = 1; i < (int)path.size(); i++)
            total += terrainCostAt(path[i].r, path[i].c);

        return total;
    }

    PathValueStats evaluatePathValue(const vector<Cell> &path)
    {
        PathValueStats stats;
        stats.pathCost = pathCost(path);

        for (int i = 1; i < (int)path.size(); i++)
        {
            Cell p = path[i];

            if (blocked[p.r][p.c])
                stats.blockedCells++;

            if (covered[p.r][p.c])
                stats.revisits++;
            else
                stats.newCells++;
        }

        return stats;
    }

    const int CELL_REVISIT_PENALTY = 2;

    int revisitPenaltyForStats(const PathValueStats &stats)
    {
        return stats.revisits * CELL_REVISIT_PENALTY;
    }

    int targetScoreForStats(
        const CoverageCandidate &candidate,
        const PathValueStats &stats
    ) {
        return candidate.costFromRobot + revisitPenaltyForStats(stats);
    }

    bool isBetterTargetScore(
        int score,
        const CoverageCandidate &candidate,
        int bestScore,
        const CoverageCandidate &bestCandidate,
        bool hasBest
    ) {
        if (!hasBest)
            return true;

        if (score != bestScore)
            return score < bestScore;

        if (candidate.costFromRobot != bestCandidate.costFromRobot)
            return candidate.costFromRobot < bestCandidate.costFromRobot;

        if (candidate.target.r != bestCandidate.target.r)
            return candidate.target.r < bestCandidate.target.r;

        return candidate.target.c < bestCandidate.target.c;
    }

    string compactPathText(const vector<Cell> &path, int limit = 20)
    {
        if (path.empty())
            return "empty";

        string text;
        int n = (int)path.size();

        for (int i = 0; i < n && i < limit; i++)
        {
            if (!text.empty())
                text += "->";

            text += cellText(path[i]);
        }

        if (n > limit)
            text += "->...->" + cellText(path.back());

        return text;
    }

    void logCandidateCheck(
        const Robot &rb,
        int decisionId,
        int rank,
        const CoverageCandidate &candidate,
        int costToBase,
        bool fullBatteryFeasible,
        bool currentEnergyFeasible,
        const string &result,
        const string &failedConstraint,
        const string &note,
        const PathValueStats *stats = nullptr,
        int revisitPenalty = 0,
        int targetScore = INF
    ) {
        string details;
        appendDetail(details, kv("decision_id", decisionId));
        appendDetail(details, kv("rank", rank));
        appendDetail(details, kvCell("candidate", candidate.target));
        appendDetail(details, kv("covered", covered[candidate.target.r][candidate.target.c]));
        appendDetail(details, kv("reachable", candidate.costFromRobot < INF));
        appendDetail(details, kv("cost_to_target", candidate.costFromRobot));
        appendDetail(details, kv("cost_to_base", costToBase));
        appendDetail(details, kv("required_lower_bound", requiredEnergyLowerBound(candidate.costFromRobot, costToBase)));
        appendDetail(details, kv("return_margin", returnMarginForCost(costToBase)));
        appendDetail(details, kv("required_with_margin", requiredEnergyWithMargin(candidate.costFromRobot, costToBase)));
        appendDetail(details, kv("energy_slack", energySlack(rb, candidate.costFromRobot, costToBase)));

        if (stats != nullptr)
        {
            appendDetail(details, kv("path_cost", stats->pathCost));
            appendDetail(details, kv("new_cells", stats->newCells));
            appendDetail(details, kv("revisits", stats->revisits));
            appendDetail(details, kv("revisit_penalty", revisitPenalty));
            appendDetail(details, kv("target_score", targetScore));
        }

        appendDetail(details, kv("full_battery_feasible", fullBatteryFeasible));
        appendDetail(details, kv("current_energy_feasible", currentEnergyFeasible));
        appendDetail(details, kv("result", result));
        appendDetail(details, kv("failed_constraint", failedConstraint));
        appendDetail(details, kv("note", note));

        logEvent(
            "DEBUG",
            "CHECK",
            "candidate_check",
            rb,
            modeName(NORMAL),
            details
        );
    }

    void logCandidateSummary(
        const Robot &rb,
        int decisionId,
        int candidateCount,
        int selectedRank,
        int loggedCandidates,
        int feasibleCandidates,
        int rejectedCurrentEnergyLow,
        int rejectedMaxEnergyInfeasible,
        const CoverageCandidate &selected,
        int selectedCostToBase,
        bool selectedFullBatteryFeasible,
        bool selectedCurrentEnergyFeasible,
        const PathValueStats &selectedStats,
        int selectedRevisitPenalty,
        int selectedScore
    ) {
        string details;
        appendDetail(details, kv("decision_id", decisionId));
        appendDetail(details, kv("candidates", candidateCount));
        appendDetail(details, kv("selected_rank", selectedRank));
        appendDetail(details, kv("logged_top", loggedCandidates));
        appendDetail(details, kv("feasible_candidates", feasibleCandidates));
        appendDetail(details, kv("rejected_current_energy_low", rejectedCurrentEnergyLow));
        appendDetail(details, kv("rejected_max_energy_infeasible", rejectedMaxEnergyInfeasible));
        appendDetail(details, kvCell("selected", selected.target));
        appendDetail(details, kv("selected_cost_to_target", selected.costFromRobot));
        appendDetail(details, kv("selected_cost_to_base", selectedCostToBase));
        appendDetail(details, kv("selected_required_lower_bound", requiredEnergyLowerBound(selected.costFromRobot, selectedCostToBase)));
        appendDetail(details, kv("selected_return_margin", returnMarginForCost(selectedCostToBase)));
        appendDetail(details, kv("selected_required_with_margin", requiredEnergyWithMargin(selected.costFromRobot, selectedCostToBase)));
        appendDetail(details, kv("selected_energy_slack", energySlack(rb, selected.costFromRobot, selectedCostToBase)));
        appendDetail(details, kv("selected_path_cost", selectedStats.pathCost));
        appendDetail(details, kv("selected_new_cells", selectedStats.newCells));
        appendDetail(details, kv("selected_revisits", selectedStats.revisits));
        appendDetail(details, kv("selected_revisit_penalty", selectedRevisitPenalty));
        appendDetail(details, kv("selected_score", selectedScore));
        appendDetail(details, kv("selected_full_battery_feasible", selectedFullBatteryFeasible));
        appendDetail(details, kv("selected_current_energy_feasible", selectedCurrentEnergyFeasible));

        logEvent(
            "INFO",
            "TARGET",
            "candidate_summary",
            rb,
            modeName(NORMAL),
            details
        );
    }

    void logCoveragePathValue(
        CoverageContext *ctx,
        const Robot &rb,
        const CoverageCandidate &candidate
    ) {
        PathValueStats stats = evaluatePathValue(rb.path);

        string details;
        if (ctx != nullptr)
            appendDetail(details, kv("decision_id", ctx->activeDecisionId));

        appendDetail(details, kvCell("target", candidate.target));
        appendDetail(details, kv("path_len", (int)rb.path.size()));
        appendDetail(details, kv("path_cost", stats.pathCost));
        appendDetail(details, kv("new_cells", stats.newCells));
        appendDetail(details, kv("revisits", stats.revisits));
        appendDetail(details, kv("blocked_cells", stats.blockedCells));
        appendDetail(details, kv("revisit_penalty", revisitPenaltyForStats(stats)));
        appendDetail(details, kv("target_score", targetScoreForStats(candidate, stats)));

        if (!rb.path.empty())
        {
            appendDetail(details, kvCell("first", rb.path.front()));
            appendDetail(details, kvCell("last", rb.path.back()));
        }

        if (rb.pathID < (int)rb.path.size())
            appendDetail(details, kvCell("next", rb.path[rb.pathID]));

        appendDetail(details, kv("path", compactPathText(rb.path)));

        logEvent(
            "INFO",
            "PATH",
            "coverage_path_value",
            rb,
            modeName(NORMAL),
            details
        );

        string valueDetails;
        if (ctx != nullptr)
            appendDetail(valueDetails, kv("decision_id", ctx->activeDecisionId));

        appendDetail(valueDetails, kv("reward_new_cells", stats.newCells));
        appendDetail(valueDetails, kv("penalty_revisits", stats.revisits));
        appendDetail(valueDetails, kv("penalty_revisit_score", revisitPenaltyForStats(stats)));
        appendDetail(valueDetails, kv("penalty_energy", stats.pathCost));
        appendDetail(valueDetails, kv("target_score", targetScoreForStats(candidate, stats)));
        appendDetail(valueDetails, kv("penalty_blocked_cells", stats.blockedCells));
        appendDetail(valueDetails, kv("result", "accepted"));

        logEvent(
            "INFO",
            "OUTCOME",
            "decision_value",
            rb,
            modeName(NORMAL),
            valueDetails
        );
    }
}

void clearRobotPath(Robot &rb)
{
    rb.path.clear();
    rb.pathID = 0;
}

bool isAtBase(const Robot &rb)
{
    return rb.pos == rb.base;
}

static PathBuildResult makeAlreadyAtGoalResult(Robot &rb)
{
    clearRobotPath(rb);

    PathBuildResult result;
    result.success = true;
    result.alreadyAtGoal = true;
    return result;
}

PathBuildResult rebuildPathToBase(Robot &rb, CoverageContext *ctx)
{
    if (isAtBase(rb))
        return makeAlreadyAtGoalResult(rb);

    dijkstra(rb.pos, d, trace);

    if (d[rb.base.r][rb.base.c] >= INF)
    {
        clearRobotPath(rb);
        return {};
    }

    rb.path = tracePath(rb.pos, rb.base, trace);

    if ((int)rb.path.size() <= 1)
    {
        clearRobotPath(rb);
        return {isAtBase(rb), isAtBase(rb), false, false};
    }

    rb.pathID = 1;

    if (ctx != nullptr)
    {
        beginDecisionTrace(
            *ctx,
            rb,
            "return",
            rb.base,
            "return_to_base",
            1,
            d[rb.base.r][rb.base.c],
            0
        );

        logDecisionPath(
            *ctx,
            rb,
            "return_path_built",
            true,
            "path_cost=" + std::to_string(pathCost(rb.path)) +
            " first_step=" + cellText(rb.path[rb.pathID]) +
            " path=" + compactPathText(rb.path)
        );
    }

    if (!isNextPathCellFree(rb))
    {
        clearRobotPath(rb);
        return {};
    }

    return {true, false, false, false};
}

PathBuildResult rebuildSafeDetourPathToBase(Robot &rb, CoverageContext *ctx)
{
    if (isAtBase(rb))
        return makeAlreadyAtGoalResult(rb);

    dijkstra(rb.pos, d, trace);

    if (d[rb.base.r][rb.base.c] >= INF)
    {
        clearRobotPath(rb);
        return {};
    }

    vector<Cell> candidate = tracePath(rb.pos, rb.base, trace);

    if ((int)candidate.size() <= 1)
    {
        clearRobotPath(rb);
        return {isAtBase(rb), isAtBase(rb), false, false};
    }

    if (isPathNearDynamicObstacle(candidate, 1, 1))
    {
        clearRobotPath(rb);
        return {};
    }

    rb.path = candidate;
    rb.pathID = 1;

    if (ctx != nullptr)
    {
        beginDecisionTrace(
            *ctx,
            rb,
            "return_detour",
            rb.base,
            "dynamic_obstacle_blocked_return",
            1,
            d[rb.base.r][rb.base.c],
            0
        );

        logDecisionPath(
            *ctx,
            rb,
            "return_detour_path_built",
            true,
            "path_cost=" + std::to_string(pathCost(rb.path)) +
            " first_step=" + cellText(rb.path[rb.pathID]) +
            " path=" + compactPathText(rb.path)
        );
    }

    return {true, false, false, false};
}

PathBuildResult rebuildPathToNearestUncoveredTarget(Robot &rb, CoverageContext *ctx)
{
    vector<CoverageCandidate> candidates =
        collectReachableUncoveredCandidates(rb);

    if (candidates.empty())
    {
        clearRobotPath(rb);
        return {};
    }

    bool foundCurrentEnergyLowTarget = false;
    bool foundMaxEnergyInfeasibleTarget = false;

    const int DEBUG_CANDIDATE_LIMIT = 15;
    int rank = 0;
    int loggedCandidates = 0;
    int rejectedCurrentEnergyLow = 0;
    int rejectedMaxEnergyInfeasible = 0;
    int feasibleCandidates = 0;
    int decisionId = ctx == nullptr ? 0 : ctx->activeDecisionId;

    bool hasBestCandidate = false;
    int bestRank = 0;
    CoverageCandidate bestCandidate;
    int bestCostToBase = INF;
    bool bestFullBatteryFeasible = false;
    bool bestCurrentEnergyFeasible = false;
    vector<Cell> bestPath;
    PathValueStats bestStats;
    int bestRevisitPenalty = 0;
    int bestScore = INF;

    // collectReachableUncoveredCandidates() leaves d/trace rooted at rb.pos.
    // estimateCostFromTargetToBase() runs Dijkstra from each candidate, so we
    // restore the robot-rooted Dijkstra before tracing a feasible candidate path.
    for (const CoverageCandidate &candidate : candidates)
    {
        rank++;

        int costToBase =
            estimateCostFromTargetToBase(candidate.target, rb);

        bool fullBatteryFeasible = canFullBatteryVisitTargetAndReturn(
            rb,
            candidate.costFromRobot,
            costToBase
        );

        bool currentEnergyFeasible = canVisitTargetAndReturn(
            rb,
            candidate.costFromRobot,
            costToBase
        );

        if (!fullBatteryFeasible)
        {
            foundMaxEnergyInfeasibleTarget = true;
            rejectedMaxEnergyInfeasible++;

            if (loggedCandidates < DEBUG_CANDIDATE_LIMIT)
            {
                logCandidateCheck(
                    rb,
                    decisionId,
                    rank,
                    candidate,
                    costToBase,
                    fullBatteryFeasible,
                    currentEnergyFeasible,
                    "rejected_max_energy_infeasible",
                    "full_battery_feasible",
                    "full_battery_cannot_visit_and_return"
                );
                loggedCandidates++;
            }

            continue;
        }

        if (!currentEnergyFeasible)
        {
            foundCurrentEnergyLowTarget = true;
            rejectedCurrentEnergyLow++;

            if (loggedCandidates < DEBUG_CANDIDATE_LIMIT)
            {
                logCandidateCheck(
                    rb,
                    decisionId,
                    rank,
                    candidate,
                    costToBase,
                    fullBatteryFeasible,
                    currentEnergyFeasible,
                    "rejected_current_energy_low",
                    "current_energy_feasible",
                    "current_battery_cannot_visit_and_return"
                );
                loggedCandidates++;
            }

            continue;
        }

        dijkstra(rb.pos, d, trace);
        vector<Cell> candidatePath = tracePath(rb.pos, candidate.target, trace);

        if (candidatePath.empty())
        {
            if (loggedCandidates < DEBUG_CANDIDATE_LIMIT)
            {
                logCandidateCheck(
                    rb,
                    decisionId,
                    rank,
                    candidate,
                    costToBase,
                    fullBatteryFeasible,
                    currentEnergyFeasible,
                    "rejected_empty_path",
                    "path_trace",
                    "candidate_path_empty_after_feasibility"
                );
                loggedCandidates++;
            }

            continue;
        }

        if ((int)candidatePath.size() <= 1)
        {
            markCovered(candidate.target.r, candidate.target.c);
            clearRobotPath(rb);
            return rebuildPathToNearestUncoveredTarget(rb, ctx);
        }

        PathValueStats stats = evaluatePathValue(candidatePath);
        int revisitPenalty = revisitPenaltyForStats(stats);
        int targetScore = targetScoreForStats(candidate, stats);
        feasibleCandidates++;

        if (loggedCandidates < DEBUG_CANDIDATE_LIMIT)
        {
            logCandidateCheck(
                rb,
                decisionId,
                rank,
                candidate,
                costToBase,
                fullBatteryFeasible,
                currentEnergyFeasible,
                "feasible_scored",
                "none",
                "candidate_feasible_score_evaluated",
                &stats,
                revisitPenalty,
                targetScore
            );
            loggedCandidates++;
        }

        if (isBetterTargetScore(
                targetScore,
                candidate,
                bestScore,
                bestCandidate,
                hasBestCandidate
            ))
        {
            hasBestCandidate = true;
            bestRank = rank;
            bestCandidate = candidate;
            bestCostToBase = costToBase;
            bestFullBatteryFeasible = fullBatteryFeasible;
            bestCurrentEnergyFeasible = currentEnergyFeasible;
            bestPath = candidatePath;
            bestStats = stats;
            bestRevisitPenalty = revisitPenalty;
            bestScore = targetScore;
        }
    }

    if (hasBestCandidate)
    {
        if (ctx != nullptr)
        {
            beginDecisionTrace(
                *ctx,
                rb,
                "coverage",
                bestCandidate.target,
                "lowest_revisit_penalty_energy_feasible",
                (int)candidates.size(),
                bestCandidate.costFromRobot,
                bestCostToBase
            );

            decisionId = ctx->activeDecisionId;
        }

        logCandidateCheck(
            rb,
            decisionId,
            bestRank,
            bestCandidate,
            bestCostToBase,
            bestFullBatteryFeasible,
            bestCurrentEnergyFeasible,
            "accepted_selected",
            "none",
            "lowest_score_feasible_uncovered_candidate",
            &bestStats,
            bestRevisitPenalty,
            bestScore
        );

        logCandidateSummary(
            rb,
            decisionId,
            (int)candidates.size(),
            bestRank,
            loggedCandidates,
            feasibleCandidates,
            rejectedCurrentEnergyLow,
            rejectedMaxEnergyInfeasible,
            bestCandidate,
            bestCostToBase,
            bestFullBatteryFeasible,
            bestCurrentEnergyFeasible,
            bestStats,
            bestRevisitPenalty,
            bestScore
        );

        rb.path = bestPath;
        rb.pathID = 1;

        if (!isNextPathCellFree(rb))
        {
            clearRobotPath(rb);
            return {};
        }

        if (ctx != nullptr)
        {
            PathValueStats stats = evaluatePathValue(rb.path);

            logDecisionPath(
                *ctx,
                rb,
                "coverage_path_built",
                true,
                "path_cost=" + std::to_string(stats.pathCost) +
                " new_cells=" + std::to_string(stats.newCells) +
                " revisits=" + std::to_string(stats.revisits) +
                " blocked_cells=" + std::to_string(stats.blockedCells) +
                " revisit_penalty=" + std::to_string(revisitPenaltyForStats(stats)) +
                " target_score=" + std::to_string(targetScoreForStats(bestCandidate, stats)) +
                " first_step=" + cellText(rb.path[rb.pathID]) +
                " path=" + compactPathText(rb.path)
            );
        }

        logCoveragePathValue(ctx, rb, bestCandidate);

        return {true, false, false, false};
    }

    clearRobotPath(rb);

    PathBuildResult result;
    result.success = false;
    result.alreadyAtGoal = false;
    result.currentEnergyLow = foundCurrentEnergyLowTarget;
    result.energyInfeasible = !foundCurrentEnergyLowTarget && foundMaxEnergyInfeasibleTarget;

    if (ctx != nullptr)
    {
        Cell target = candidates.empty() ? rb.pos : candidates.front().target;
        beginDecisionTrace(
            *ctx,
            rb,
            "coverage",
            target,
            result.currentEnergyLow ? "current_energy_low_for_candidates" : "max_energy_infeasible_for_candidates",
            (int)candidates.size(),
            candidates.empty() ? 0 : candidates.front().costFromRobot,
            0
        );

        logDecisionPath(
            *ctx,
            rb,
            "coverage_path_failed",
            false,
            "current_energy_low=" + boolText(result.currentEnergyLow) +
            " energy_infeasible=" + boolText(result.energyInfeasible) +
            " rejected_current_energy_low=" + std::to_string(rejectedCurrentEnergyLow) +
            " rejected_max_energy_infeasible=" + std::to_string(rejectedMaxEnergyInfeasible) +
            " feasible_candidates=" + std::to_string(feasibleCandidates)
        );
    }

    return result;
}
