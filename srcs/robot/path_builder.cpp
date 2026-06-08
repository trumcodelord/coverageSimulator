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
        HeadingDir arrivalDir = DIR_NORTH;
    };

    struct PathValueStats
    {
        int pathCost = 0;
        int newCells = 0;
        int revisits = 0;
        int blockedCells = 0;
    };

    HeadingDir currentHeadingDir(const Robot &rb)
    {
        return headingDirFromDegrees(rb.headingDeg);
    }

    bool directionForStep(Cell from, Cell to, HeadingDir &dir)
    {
        int dr = to.r - from.r;
        int dc = to.c - from.c;

        if (dr == -1 && dc == 0)
        {
            dir = DIR_NORTH;
            return true;
        }

        if (dr == 0 && dc == 1)
        {
            dir = DIR_EAST;
            return true;
        }

        if (dr == 1 && dc == 0)
        {
            dir = DIR_SOUTH;
            return true;
        }

        if (dr == 0 && dc == -1)
        {
            dir = DIR_WEST;
            return true;
        }

        return false;
    }

    int movementCostForPathCell(
        Cell p,
        PlannerObstacleMode obstacleMode
    ) {
        if (obstacleMode == PlannerObstacleMode::IGNORE_DYNAMIC)
            return baseTerrainCostAt(p.r, p.c);

        return effectiveTerrainCostAt(p.r, p.c);
    }

    int pathEnergyCost(
        const vector<Cell> &path,
        HeadingDir startDir,
        PlannerObstacleMode obstacleMode = PlannerObstacleMode::RESPECT_DYNAMIC
    ) {
        if (path.size() <= 1)
            return 0;

        long long total = 0;
        HeadingDir currentDir = startDir;

        for (int i = 1; i < (int)path.size(); i++)
        {
            Cell current = path[i - 1];
            Cell next = path[i];
            HeadingDir nextDir;

            if (!directionForStep(current, next, nextDir))
                return INF;

            int currentTerrainCost = baseTerrainCostAt(current.r, current.c);
            int movementCost = movementCostForPathCell(next, obstacleMode);

            if (currentTerrainCost >= INF || movementCost >= INF)
                return INF;

            total += movementCost;
            total += (long long)quarterTurnsBetween(currentDir, nextDir) *
                     currentTerrainCost;

            if (total >= INF)
                return INF;

            currentDir = nextDir;
        }

        return (int)total;
    }

    bool compareCandidateByDistance(
        const CoverageCandidate &a,
        const CoverageCandidate &b
    ) {
        if (a.costFromRobot != b.costFromRobot)
            return a.costFromRobot < b.costFromRobot;

        if (a.target.r != b.target.r)
            return a.target.r < b.target.r;

        if (a.target.c != b.target.c)
            return a.target.c < b.target.c;

        return (int)a.arrivalDir < (int)b.arrivalDir;
    }

    vector<CoverageCandidate> collectReachableUncoveredCandidates(
        const Robot &rb
    ) {
        vector<CoverageCandidate> candidates;

        HeadingDir startDir = currentHeadingDir(rb);
        dijkstraOriented(
            rb.pos,
            startDir,
            PlannerObstacleMode::RESPECT_DYNAMIC
        );

        for (int i = 1; i <= rows; i++)
        {
            for (int j = 1; j <= cols; j++)
            {
                if (!isCoverageTargetCell(i, j) || covered[i][j])
                    continue;

                Cell target = {i, j};
                HeadingDir arrivalDir = DIR_NORTH;
                int cost = bestOrientedDistanceTo(target, &arrivalDir);

                if (cost >= INF)
                    continue;

                candidates.push_back({cost, target, arrivalDir});
            }
        }

        sort(
            candidates.begin(),
            candidates.end(),
            compareCandidateByDistance
        );

        return candidates;
    }

    int estimateCostFromTargetToBase(
        const CoverageCandidate &candidate,
        const Robot &rb
    ) {
        dijkstraOriented(
            candidate.target,
            candidate.arrivalDir,
            PlannerObstacleMode::RESPECT_DYNAMIC
        );

        return bestOrientedDistanceTo(rb.base);
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

    PathValueStats evaluatePathValue(
        const vector<Cell> &path,
        HeadingDir startDir,
        PlannerObstacleMode obstacleMode = PlannerObstacleMode::RESPECT_DYNAMIC
    ) {
        PathValueStats stats;
        stats.pathCost = pathEnergyCost(path, startDir, obstacleMode);

        for (int i = 1; i < (int)path.size(); i++)
        {
            Cell p = path[i];

            if (isStaticBlocked(p.r, p.c))
                stats.blockedCells++;

            if (covered[p.r][p.c])
                stats.revisits++;
            else
                stats.newCells++;
        }

        return stats;
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
        const string &note
    ) {
        string details;
        appendDetail(details, kv("decision_id", decisionId));
        appendDetail(details, kv("rank", rank));
        appendDetail(details, kvCell("candidate", candidate.target));
        appendDetail(details, kv("covered", covered[candidate.target.r][candidate.target.c]));
        appendDetail(details, kv("reachable", candidate.costFromRobot < INF));
        appendDetail(details, kv("cost_to_target", candidate.costFromRobot));
        appendDetail(details, kv("arrival_dir", (int)candidate.arrivalDir));
        appendDetail(details, kv("cost_to_base", costToBase));
        appendDetail(details, kv("required_lower_bound", requiredEnergyLowerBound(candidate.costFromRobot, costToBase)));
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
        int rejectedCurrentEnergyLow,
        int rejectedMaxEnergyInfeasible,
        const CoverageCandidate &selected,
        int selectedCostToBase,
        bool selectedFullBatteryFeasible,
        bool selectedCurrentEnergyFeasible
    ) {
        string details;
        appendDetail(details, kv("decision_id", decisionId));
        appendDetail(details, kv("candidates", candidateCount));
        appendDetail(details, kv("selected_rank", selectedRank));
        appendDetail(details, kv("logged_top", loggedCandidates));
        appendDetail(details, kv("rejected_current_energy_low", rejectedCurrentEnergyLow));
        appendDetail(details, kv("rejected_max_energy_infeasible", rejectedMaxEnergyInfeasible));
        appendDetail(details, kvCell("selected", selected.target));
        appendDetail(details, kv("selected_cost_to_target", selected.costFromRobot));
        appendDetail(details, kv("selected_arrival_dir", (int)selected.arrivalDir));
        appendDetail(details, kv("selected_cost_to_base", selectedCostToBase));
        appendDetail(details, kv("selected_required_lower_bound", requiredEnergyLowerBound(selected.costFromRobot, selectedCostToBase)));
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
        PathValueStats stats = evaluatePathValue(
            rb.path,
            currentHeadingDir(rb)
        );

        string details;
        if (ctx != nullptr)
            appendDetail(details, kv("decision_id", ctx->activeDecisionId));

        appendDetail(details, kvCell("target", candidate.target));
        appendDetail(details, kv("target_arrival_dir", (int)candidate.arrivalDir));
        appendDetail(details, kv("path_len", (int)rb.path.size()));
        appendDetail(details, kv("path_cost", stats.pathCost));
        appendDetail(details, kv("new_cells", stats.newCells));
        appendDetail(details, kv("revisits", stats.revisits));
        appendDetail(details, kv("blocked_cells", stats.blockedCells));

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
        appendDetail(valueDetails, kv("penalty_energy", stats.pathCost));
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

    HeadingDir startDir = currentHeadingDir(rb);
    dijkstraOriented(
        rb.pos,
        startDir,
        PlannerObstacleMode::RESPECT_DYNAMIC
    );

    HeadingDir baseDir = DIR_NORTH;
    int costToBase = bestOrientedDistanceTo(rb.base, &baseDir);

    if (costToBase >= INF)
    {
        clearRobotPath(rb);
        return {};
    }

    rb.path = tracePathOriented(rb.pos, startDir, rb.base, baseDir);

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
            costToBase,
            0
        );

        logDecisionPath(
            *ctx,
            rb,
            "return_path_built",
            true,
            "path_cost=" + std::to_string(costToBase) +
            " arrival_dir=" + std::to_string((int)baseDir) +
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

    HeadingDir startDir = currentHeadingDir(rb);
    dijkstraOriented(
        rb.pos,
        startDir,
        PlannerObstacleMode::RESPECT_DYNAMIC
    );

    HeadingDir baseDir = DIR_NORTH;
    int costToBase = bestOrientedDistanceTo(rb.base, &baseDir);

    if (costToBase >= INF)
    {
        clearRobotPath(rb);
        return {};
    }

    vector<Cell> candidate =
        tracePathOriented(rb.pos, startDir, rb.base, baseDir);

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
            costToBase,
            0
        );

        logDecisionPath(
            *ctx,
            rb,
            "return_detour_path_built",
            true,
            "path_cost=" + std::to_string(costToBase) +
            " arrival_dir=" + std::to_string((int)baseDir) +
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
    int decisionId = ctx == nullptr ? 0 : ctx->activeDecisionId;

    for (const CoverageCandidate &candidate : candidates)
    {
        rank++;

        int costToBase = estimateCostFromTargetToBase(candidate, rb);

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

        if (ctx != nullptr)
        {
            beginDecisionTrace(
                *ctx,
                rb,
                "coverage",
                candidate.target,
                "nearest_uncovered_energy_feasible",
                (int)candidates.size(),
                candidate.costFromRobot,
                costToBase
            );

            decisionId = ctx->activeDecisionId;
        }

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
                "accepted_selected",
                "none",
                "nearest_feasible_uncovered_candidate"
            );
            loggedCandidates++;
        }

        logCandidateSummary(
            rb,
            decisionId,
            (int)candidates.size(),
            rank,
            loggedCandidates,
            rejectedCurrentEnergyLow,
            rejectedMaxEnergyInfeasible,
            candidate,
            costToBase,
            fullBatteryFeasible,
            currentEnergyFeasible
        );

        HeadingDir startDir = currentHeadingDir(rb);
        dijkstraOriented(
            rb.pos,
            startDir,
            PlannerObstacleMode::RESPECT_DYNAMIC
        );

        rb.path = tracePathOriented(
            rb.pos,
            startDir,
            candidate.target,
            candidate.arrivalDir
        );

        if (rb.path.empty())
        {
            clearRobotPath(rb);
            continue;
        }

        if ((int)rb.path.size() <= 1)
        {
            markCovered(candidate.target.r, candidate.target.c);
            clearRobotPath(rb);
            return rebuildPathToNearestUncoveredTarget(rb, ctx);
        }

        rb.pathID = 1;

        if (!isNextPathCellFree(rb))
        {
            clearRobotPath(rb);
            return {};
        }

        if (ctx != nullptr)
        {
            PathValueStats stats = evaluatePathValue(rb.path, startDir);

            logDecisionPath(
                *ctx,
                rb,
                "coverage_path_built",
                true,
                "path_cost=" + std::to_string(stats.pathCost) +
                " arrival_dir=" + std::to_string((int)candidate.arrivalDir) +
                " new_cells=" + std::to_string(stats.newCells) +
                " revisits=" + std::to_string(stats.revisits) +
                " blocked_cells=" + std::to_string(stats.blockedCells) +
                " first_step=" + cellText(rb.path[rb.pathID]) +
                " path=" + compactPathText(rb.path)
            );
        }

        logCoveragePathValue(ctx, rb, candidate);

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
            " rejected_max_energy_infeasible=" + std::to_string(rejectedMaxEnergyInfeasible)
        );
    }

    return result;
}
