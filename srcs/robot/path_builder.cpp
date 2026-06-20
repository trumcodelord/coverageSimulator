#include "path_builder.h"
#include "behavior_log.h"
#include "energy_model.h"
#include "grid.h"
#include "motion_geometry.h"
#include "path_safety.h"
#include "planner.h"
#include "uncovered_island.h"

#include <algorithm>
#include <string>
#include <vector>

using namespace std;

namespace
{
    struct CoverageCandidate
    {
        double costFromRobot = INF;
        double costToBase = INF;
        Cell target = {0, 0};
        HeadingDir arrivalDir = DIR_NORTH;
        HeadingDir returnArrivalDir = DIR_NORTH;
        int islandPriority = uncovered_island::NO_ISLAND_PRIORITY;
        int revisitCountOnPath = 0;
        int frontierScore = 0;
        vector<Cell> path;
    };

    int movementCostForPathCell(Cell p, PlannerObstacleMode mode)
    {
        if (mode == PlannerObstacleMode::IGNORE_DYNAMIC)
            return baseTerrainCostAt(p.r, p.c);

        return effectiveTerrainCostAt(p.r, p.c);
    }

    double pathEnergyCost(
        const vector<Cell> &path,
        HeadingDir startDir,
        PlannerObstacleMode mode = PlannerObstacleMode::RESPECT_DYNAMIC
    ) {
        if (path.size() <= 1)
            return 0;

        double total = 0.0;
        HeadingDir curDir = startDir;

        for (int i = 1; i < (int)path.size(); i++)
        {
            HeadingDir nextDir;
            if (!directionForStep(path[i - 1], path[i], nextDir))
                return INF;

            double turnCostTerrain = turnQuarterEnergyCostAtCell(path[i - 1]);
            double moveCost = movementCostForPathCell(path[i], mode);

            if (turnCostTerrain >= INF || moveCost >= INF)
                return INF;

            total += moveCost;
            total = quantizeEnergy(total + (double)quarterTurnsBetween(curDir, nextDir) * turnCostTerrain);

            if (total >= INF)
                return INF;

            curDir = nextDir;
        }

        return quantizeEnergy(total);
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

    int frontierScore(Cell target)
    {
        int score = 0;

        for (int k = 1; k <= 4; k++)
        {
            Cell n = {target.r + dr[k], target.c + dc[k]};

            if (uncovered_island::isUncoveredTarget(n))
                score++;
        }

        return score;
    }

    int revisitCountOnPath(const vector<Cell> &path)
    {
        int count = 0;

        for (int i = 1; i < (int)path.size(); i++)
        {
            Cell p = path[i];

            if (isCoverageTargetCell(p.r, p.c) && covered[p.r][p.c])
                count++;
        }

        return count;
    }

    bool compareCandidateLexicographic(
        const CoverageCandidate &a,
        const CoverageCandidate &b
    ) {
        if (a.target.r != b.target.r)
            return a.target.r < b.target.r;
        if (a.target.c != b.target.c)
            return a.target.c < b.target.c;
        return (int)a.arrivalDir < (int)b.arrivalDir;
    }

    bool samePrimaryCoverageRank(
        const CoverageCandidate &a,
        const CoverageCandidate &b
    ) {
        return a.costFromRobot == b.costFromRobot &&
               a.islandPriority == b.islandPriority &&
               a.revisitCountOnPath == b.revisitCountOnPath &&
               a.frontierScore == b.frontierScore;
    }

    bool compareCandidateByCoveragePolicy(
        const CoverageCandidate &a,
        const CoverageCandidate &b
    ) {
        if (a.costFromRobot != b.costFromRobot)
            return a.costFromRobot < b.costFromRobot;

        if (a.islandPriority != b.islandPriority)
            return a.islandPriority < b.islandPriority;

        if (a.revisitCountOnPath != b.revisitCountOnPath)
            return a.revisitCountOnPath < b.revisitCountOnPath;

        if (a.frontierScore != b.frontierScore)
            return a.frontierScore > b.frontierScore;

        // costToBase is intentionally late: feasibility already guarantees that
        // the robot can return home; this only preserves a better safety margin
        // when coverage-oriented metrics are otherwise equal.
        if (a.costToBase != b.costToBase)
            return a.costToBase < b.costToBase;

        // Deterministic fallback only; this is not a navigation policy.
        return compareCandidateLexicographic(a, b);
    }

    bool compareCandidateByCheapPolicy(
        const CoverageCandidate &a,
        const CoverageCandidate &b
    ) {
        if (a.costFromRobot != b.costFromRobot)
            return a.costFromRobot < b.costFromRobot;

        if (a.islandPriority != b.islandPriority)
            return a.islandPriority < b.islandPriority;

        if (a.revisitCountOnPath != b.revisitCountOnPath)
            return a.revisitCountOnPath < b.revisitCountOnPath;

        if (a.frontierScore != b.frontierScore)
            return a.frontierScore > b.frontierScore;

        return compareCandidateLexicographic(a, b);
    }

    vector<CoverageCandidate> collectCandidates(
        const Robot &rb,
        PlannerObstacleMode mode
    ) {
        vector<CoverageCandidate> candidates;

        HeadingDir startDir = currentHeadingDir(rb);
        dijkstraOriented(rb.pos, startDir, mode);

        for (int r = 1; r <= rows; r++)
        {
            for (int c = 1; c <= cols; c++)
            {
                if (!isCoverageTargetCell(r, c) || covered[r][c])
                    continue;

                Cell target = {r, c};
                HeadingDir arrivalDir = DIR_NORTH;
                double cost = bestOrientedDistanceTo(target, &arrivalDir);

                if (cost >= INF)
                    continue;

                CoverageCandidate candidate;
                candidate.costFromRobot = cost;
                candidate.target = target;
                candidate.arrivalDir = arrivalDir;
                candidate.islandPriority = uncovered_island::cleanupPriorityForTarget(target);
                candidate.frontierScore = frontierScore(target);
                candidate.path = tracePathOriented(
                    rb.pos,
                    startDir,
                    candidate.target,
                    candidate.arrivalDir
                );
                candidate.revisitCountOnPath = revisitCountOnPath(candidate.path);

                candidates.push_back(candidate);
            }
        }

        sort(candidates.begin(), candidates.end(), compareCandidateByCheapPolicy);
        return candidates;
    }

    double estimateTargetToBase(
        const CoverageCandidate &candidate,
        const Robot &rb,
        PlannerObstacleMode mode,
        HeadingDir *baseArrivalDir = nullptr
    ) {
        // Use the temp oriented tables so candidate->base feasibility checks do
        // not destroy the main robot->target trace built by collectCandidates().
        dijkstraOrientedTemp(candidate.target, candidate.arrivalDir, mode, rb.base);
        return bestTempOrientedDistanceTo(rb.base, baseArrivalDir);
    }

    bool canFullBatteryVisitAndReturn(
        const Robot &rb,
        double costToTarget,
        double costToBase
    ) {
        Robot full = rb;
        full.energy = full.maxEnergy;
        return canVisitTargetAndReturn(full, costToTarget, costToBase);
    }

    bool hasStaticFullBatteryFeasibleTarget(const Robot &rb)
    {
        vector<CoverageCandidate> staticCandidates =
            collectCandidates(rb, PlannerObstacleMode::IGNORE_DYNAMIC);

        for (const CoverageCandidate &candidate : staticCandidates)
        {
            double costToBase = estimateTargetToBase(
                candidate,
                rb,
                PlannerObstacleMode::IGNORE_DYNAMIC
            );

            if (canFullBatteryVisitAndReturn(
                    rb,
                    candidate.costFromRobot,
                    costToBase
                ))
            {
                return true;
            }
        }

        return false;
    }

    PathBuildResult alreadyAtGoalResult(Robot &rb)
    {
        clearRobotPath(rb);
        PathBuildResult result;
        result.success = true;
        result.alreadyAtGoal = true;
        result.pathCost = 0;
        return result;
    }

    void logPathRejectedForEnergy(
        CoverageContext *ctx,
        const Robot &rb,
        const string &event,
        double pathCost
    ) {
        if (ctx == nullptr)
            return;

        logDecisionPath(
            *ctx,
            rb,
            event,
            false,
            "path_cost=" + formatEnergy(pathCost) +
            " energy=" + formatEnergy(rb.energy) +
            " failed_constraint=current_energy_for_path"
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

PathBuildResult rebuildPathToBase(Robot &rb, CoverageContext *ctx)
{
    if (isAtBase(rb))
        return alreadyAtGoalResult(rb);

    HeadingDir startDir = currentHeadingDir(rb);
    dijkstraOriented(rb.pos, startDir, PlannerObstacleMode::RESPECT_DYNAMIC, rb.base);

    HeadingDir baseDir = DIR_NORTH;
    double costToBase = bestOrientedDistanceTo(rb.base, &baseDir);

    if (costToBase >= INF)
    {
        clearRobotPath(rb);
        return {};
    }

    vector<Cell> candidate = tracePathOriented(rb.pos, startDir, rb.base, baseDir);

    if ((int)candidate.size() <= 1)
        return alreadyAtGoalResult(rb);

    rb.path = candidate;
    rb.pathID = 1;

    if (ctx != nullptr)
    {
        beginDecisionTrace(*ctx, rb, "return", rb.base, "return_to_base", 1, costToBase, 0);
        logDecisionPath(
            *ctx,
            rb,
            "return_path_built",
            true,
            "path_cost=" + formatEnergy(costToBase) +
            " arrival_dir=" + to_string((int)baseDir) +
            " first_step=" + cellText(rb.path[rb.pathID]) +
            " path=" + compactPathText(rb.path)
        );
    }

    // Arriving with exactly zero energy is still power loss in robot_motion.cpp,
    // so require strictly more energy than the full path cost.
    if (costToBase >= rb.energy)
    {
        logPathRejectedForEnergy(ctx, rb, "return_path_energy_rejected", costToBase);
        clearRobotPath(rb);

        PathBuildResult result;
        result.currentEnergyLow = true;
        result.pathCost = costToBase;
        return result;
    }

    if (hasBlockedCellAnywhereOnPath(rb) || !isNextPathCellFree(rb))
    {
        clearRobotPath(rb);
        return {};
    }

    return {true, false, false, false, costToBase};
}

PathBuildResult rebuildSafeDetourPathToBase(Robot &rb, CoverageContext *ctx)
{
    if (isAtBase(rb))
        return alreadyAtGoalResult(rb);

    HeadingDir startDir = currentHeadingDir(rb);
    dijkstraOriented(rb.pos, startDir, PlannerObstacleMode::RESPECT_DYNAMIC, rb.base);

    HeadingDir baseDir = DIR_NORTH;
    double costToBase = bestOrientedDistanceTo(rb.base, &baseDir);

    if (costToBase >= INF)
    {
        clearRobotPath(rb);
        return {};
    }

    vector<Cell> candidate = tracePathOriented(rb.pos, startDir, rb.base, baseDir);

    if ((int)candidate.size() <= 1)
        return alreadyAtGoalResult(rb);

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
    }

    if (costToBase >= rb.energy)
    {
        logPathRejectedForEnergy(ctx, rb, "return_detour_energy_rejected", costToBase);
        clearRobotPath(rb);

        PathBuildResult result;
        result.currentEnergyLow = true;
        result.pathCost = costToBase;
        return result;
    }

    if (hasBlockedCellAnywhereOnPath(rb) || isPathNearDynamicObstacle(candidate, 1, 1))
    {
        clearRobotPath(rb);
        return {};
    }

    if (ctx != nullptr)
    {
        logDecisionPath(
            *ctx,
            rb,
            "return_detour_path_built",
            true,
            "path_cost=" + formatEnergy(costToBase) +
            " arrival_dir=" + to_string((int)baseDir) +
            " first_step=" + cellText(rb.path[rb.pathID]) +
            " path=" + compactPathText(rb.path)
        );
    }

    return {true, false, false, false, costToBase};
}

PathBuildResult rebuildPathToNearestUncoveredTarget(Robot &rb, CoverageContext *ctx)
{
    vector<CoverageCandidate> candidates =
        collectCandidates(rb, PlannerObstacleMode::RESPECT_DYNAMIC);

    bool foundCurrentEnergyLow = false;
    bool foundMaxEnergyInfeasible = false;
    HeadingDir startDir = currentHeadingDir(rb);

    for (int groupStart = 0; groupStart < (int)candidates.size(); )
    {
        int groupEnd = groupStart + 1;

        while (groupEnd < (int)candidates.size() &&
               samePrimaryCoverageRank(candidates[groupStart], candidates[groupEnd]))
        {
            groupEnd++;
        }

        vector<CoverageCandidate> feasibleGroup;

        for (int i = groupStart; i < groupEnd; i++)
        {
            CoverageCandidate candidate = candidates[i];
            candidate.costToBase = estimateTargetToBase(
                candidate,
                rb,
                PlannerObstacleMode::RESPECT_DYNAMIC,
                &candidate.returnArrivalDir
            );

            bool fullFeasible = canFullBatteryVisitAndReturn(
                rb,
                candidate.costFromRobot,
                candidate.costToBase
            );

            bool currentFeasible = canVisitTargetAndReturn(
                rb,
                candidate.costFromRobot,
                candidate.costToBase
            );

            if (!fullFeasible)
            {
                foundMaxEnergyInfeasible = true;
                continue;
            }

            if (!currentFeasible)
            {
                foundCurrentEnergyLow = true;
                continue;
            }

            feasibleGroup.push_back(candidate);
        }

        sort(
            feasibleGroup.begin(),
            feasibleGroup.end(),
            compareCandidateByCoveragePolicy
        );

        for (int rankInGroup = 0; rankInGroup < (int)feasibleGroup.size(); rankInGroup++)
        {
            const CoverageCandidate &candidate = feasibleGroup[rankInGroup];

            if (ctx != nullptr)
            {
                beginDecisionTrace(
                    *ctx,
                    rb,
                    "coverage",
                    candidate.target,
                    "nearest_uncovered_island_aware_feasible",
                    (int)candidates.size(),
                    candidate.costFromRobot,
                    candidate.costToBase
                );
            }

            // collectCandidates() already ran dijkstraOriented() from rb.pos with
            // this same startDir. candidate->base checks use temp tables, so the
            // main orientedTrace still describes robot->target and can be reused.
            rb.path = candidate.path;

            if ((int)rb.path.size() <= 1)
            {
                markCovered(candidate.target.r, candidate.target.c);
                clearRobotPath(rb);
                return rebuildPathToNearestUncoveredTarget(rb, ctx);
            }

            rb.pathID = 1;

            if (hasBlockedCellAnywhereOnPath(rb) || !isNextPathCellFree(rb))
            {
                // A dynamic obstacle may have occupied the selected candidate path
                // after candidate collection. Do not fail the whole decision; try
                // the next reachable uncovered candidate instead.
                clearRobotPath(rb);
                continue;
            }

            double builtPathCost = pathEnergyCost(rb.path, startDir);

            if (ctx != nullptr)
            {
                vector<Cell> returnPreview = traceTempPathOriented(
                    candidate.target,
                    candidate.arrivalDir,
                    rb.base,
                    candidate.returnArrivalDir
                );

                logDecisionPath(
                    *ctx,
                    rb,
                    "coverage_path_built",
                    true,
                    "rank=" + to_string(groupStart + rankInGroup + 1) +
                    " path_cost=" + formatEnergy(builtPathCost) +
                    " arrival_dir=" + to_string((int)candidate.arrivalDir) +
                    " cost_to_base=" + formatEnergy(candidate.costToBase) +
                    " return_arrival_dir=" + to_string((int)candidate.returnArrivalDir) +
                    " island_priority=" + to_string(candidate.islandPriority) +
                    " pending_island_cells=" + to_string(uncovered_island::pendingCleanupCellCount()) +
                    " revisit_count_on_path=" + to_string(candidate.revisitCountOnPath) +
                    " frontier_score=" + to_string(candidate.frontierScore) +
                    " first_step=" + cellText(rb.path[rb.pathID]) +
                    " path=" + compactPathText(rb.path) +
                    " return_path_preview=" + compactPathText(returnPreview)
                );
            }

            return {true, false, false, false, builtPathCost};
        }

        groupStart = groupEnd;
    }

    clearRobotPath(rb);

    bool staticFeasible = hasStaticFullBatteryFeasibleTarget(rb);

    PathBuildResult result;
    result.success = false;
    result.currentEnergyLow = foundCurrentEnergyLow;

    // If a full-battery static route exists, do not classify the mission as
    // terminally infeasible. The blocker is dynamic/temporary, so recovery logic
    // should keep listening instead of ending the mission.
    result.energyInfeasible =
        !staticFeasible && !foundCurrentEnergyLow && foundMaxEnergyInfeasible;

    if (ctx != nullptr)
    {
        Cell target = candidates.empty() ? rb.pos : candidates.front().target;

        beginDecisionTrace(
            *ctx,
            rb,
            "coverage",
            target,
            result.currentEnergyLow
                ? "current_energy_low_for_candidates"
                : result.energyInfeasible
                    ? "max_energy_infeasible_for_candidates"
                    : "temporarily_blocked_or_no_safe_candidate",
            (int)candidates.size(),
            candidates.empty() ? 0.0 : candidates.front().costFromRobot,
            0
        );

        logDecisionPath(
            *ctx,
            rb,
            "coverage_path_failed",
            false,
            "current_energy_low=" + boolText(result.currentEnergyLow) +
            " energy_infeasible=" + boolText(result.energyInfeasible) +
            " static_full_battery_feasible=" + boolText(staticFeasible)
        );
    }

    return result;
}
