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
            total = quantizeEnergy(
                total + (double)quarterTurnsBetween(curDir, nextDir) * turnCostTerrain
            );

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

    int absInt(int value)
    {
        return value < 0 ? -value : value;
    }

    int manhattanDistance(Cell a, Cell b)
    {
        return absInt(a.r - b.r) + absInt(a.c - b.c);
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
               a.revisitCountOnPath == b.revisitCountOnPath &&
               a.frontierScore == b.frontierScore;
    }

    // Island/pocket cleanup is handled by tryBuildCleanupPath().  The normal
    // comparator intentionally stays simple so a noisy detector cannot pollute
    // target selection for the whole map.
    bool compareCandidateByCoveragePolicy(
        const CoverageCandidate &a,
        const CoverageCandidate &b
    ) {
        if (a.costFromRobot != b.costFromRobot)
            return a.costFromRobot < b.costFromRobot;

        if (a.revisitCountOnPath != b.revisitCountOnPath)
            return a.revisitCountOnPath < b.revisitCountOnPath;

        if (a.frontierScore != b.frontierScore)
            return a.frontierScore > b.frontierScore;

        if (a.costToBase != b.costToBase)
            return a.costToBase < b.costToBase;

        return compareCandidateLexicographic(a, b);
    }

    bool compareCandidateByCheapPolicy(
        const CoverageCandidate &a,
        const CoverageCandidate &b
    ) {
        if (a.costFromRobot != b.costFromRobot)
            return a.costFromRobot < b.costFromRobot;

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

    double bestReachableNormalCost(
        const vector<CoverageCandidate> &candidates
    ) {
        if (candidates.empty())
            return INF;

        // collectCandidates() already sorted by cheap coverage policy and reused
        // the single robot->all oriented Dijkstra for this decision.  Do not run
        // target->base Dijkstra here; this value is only a detour gate baseline.
        return candidates.front().costFromRobot;
    }

    bool cleanupDetourAllowed(
        double cleanupCost,
        double bestNormalCost,
        bool continuingActiveComponent,
        uncovered_island::CleanupSource source
    ) {
        if (bestNormalCost >= INF)
            return true;

        double ratio = continuingActiveComponent ? 1.60 : 1.35;
        double absoluteAllowance = continuingActiveComponent ? 6.0 : 4.0;

        if (source == uncovered_island::CleanupSource::SPLIT)
            absoluteAllowance += 2.0;
        else if (source == uncovered_island::CleanupSource::LOCAL_POCKET)
            ratio = min(ratio, 1.25);

        return cleanupCost <= bestNormalCost * ratio + absoluteAllowance;
    }

    bool isBetterCleanupChoice(
        const CoverageCandidate &a,
        const CoverageCandidate &b
    ) {
        if (a.costFromRobot != b.costFromRobot)
            return a.costFromRobot < b.costFromRobot;

        if (a.revisitCountOnPath != b.revisitCountOnPath)
            return a.revisitCountOnPath < b.revisitCountOnPath;

        if (a.frontierScore != b.frontierScore)
            return a.frontierScore > b.frontierScore;

        if (a.costToBase != b.costToBase)
            return a.costToBase < b.costToBase;

        return compareCandidateLexicographic(a, b);
    }

    struct CleanupProbe
    {
        CoverageCandidate candidate;
        int componentId = -1;
        int componentSize = 0;
        uncovered_island::CleanupSource source = uncovered_island::CleanupSource::LOCAL_POCKET;
        bool continuingActiveComponent = false;
    };

    bool isBetterCleanupProbe(
        const CleanupProbe &a,
        const CleanupProbe &b
    ) {
        if (a.continuingActiveComponent != b.continuingActiveComponent)
            return a.continuingActiveComponent;

        if (a.candidate.costFromRobot != b.candidate.costFromRobot)
            return a.candidate.costFromRobot < b.candidate.costFromRobot;

        if (a.componentSize != b.componentSize)
            return a.componentSize < b.componentSize;

        if (a.candidate.revisitCountOnPath != b.candidate.revisitCountOnPath)
            return a.candidate.revisitCountOnPath < b.candidate.revisitCountOnPath;

        if (a.candidate.frontierScore != b.candidate.frontierScore)
            return a.candidate.frontierScore > b.candidate.frontierScore;

        return compareCandidateLexicographic(a.candidate, b.candidate);
    }

    bool isBetterCleanupComponent(
        const uncovered_island::CleanupComponentView &a,
        const uncovered_island::CleanupComponentView &b,
        Cell robotPos,
        int activeId
    ) {
        bool aActive = activeId >= 0 && a.id == activeId;
        bool bActive = activeId >= 0 && b.id == activeId;

        if (aActive != bActive)
            return aActive;

        int da = manhattanDistance(a.sourceCell, robotPos);
        int db = manhattanDistance(b.sourceCell, robotPos);

        if (da != db)
            return da < db;

        if (a.size != b.size)
            return a.size < b.size;

        return a.id < b.id;
    }

    bool tryBuildCleanupPath(
        Robot &rb,
        CoverageContext *ctx,
        const vector<CoverageCandidate> &normalCandidates,
        PathBuildResult &result
    ) {
        constexpr int MAX_CLEANUP_COMPONENTS_CONSIDERED = 8;
        constexpr int MAX_CLEANUP_CELLS_CONSIDERED = 24;
        constexpr int MAX_RETURN_CHECKS_PER_DECISION = 3;

        uncovered_island::pruneStaleComponents(rb.steps);

        vector<uncovered_island::CleanupComponentView> components =
            uncovered_island::cleanupComponents();

        if (components.empty())
            return false;

        int activeId = uncovered_island::activeComponentId();
        double bestNormalCost = bestReachableNormalCost(normalCandidates);
        HeadingDir startDir = currentHeadingDir(rb);

        sort(
            components.begin(),
            components.end(),
            [&](const uncovered_island::CleanupComponentView &a,
                const uncovered_island::CleanupComponentView &b)
            {
                return isBetterCleanupComponent(a, b, rb.pos, activeId);
            }
        );

        vector<CleanupProbe> probes;
        int componentsSeen = 0;
        int cellsSeen = 0;

        for (const uncovered_island::CleanupComponentView &component : components)
        {
            bool continuingActiveComponent = activeId >= 0 && component.id == activeId;

            if (activeId >= 0 && !continuingActiveComponent)
                continue;

            if (!continuingActiveComponent && componentsSeen >= MAX_CLEANUP_COMPONENTS_CONSIDERED)
                break;

            componentsSeen++;

            for (Cell target : component.cells)
            {
                if (cellsSeen >= MAX_CLEANUP_CELLS_CONSIDERED)
                    break;

                if (!uncovered_island::isUncoveredTarget(target))
                    continue;

                HeadingDir arrivalDir = DIR_NORTH;
                double cost = bestOrientedDistanceTo(target, &arrivalDir);

                if (cost >= INF)
                    continue;

                if (!cleanupDetourAllowed(
                        cost,
                        bestNormalCost,
                        continuingActiveComponent,
                        component.source
                    ))
                {
                    continue;
                }

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

                CleanupProbe probe;
                probe.candidate = candidate;
                probe.componentId = component.id;
                probe.componentSize = component.size;
                probe.source = component.source;
                probe.continuingActiveComponent = continuingActiveComponent;
                probes.push_back(probe);

                cellsSeen++;
            }
        }

        if (probes.empty())
        {
            if (activeId >= 0)
                uncovered_island::releaseActiveComponent();

            return false;
        }

        sort(probes.begin(), probes.end(), isBetterCleanupProbe);

        bool hasChoice = false;
        CleanupProbe bestProbe;
        int returnChecks = 0;

        for (CleanupProbe probe : probes)
        {
            if (returnChecks >= MAX_RETURN_CHECKS_PER_DECISION)
                break;

            returnChecks++;

            probe.candidate.costToBase = estimateTargetToBase(
                probe.candidate,
                rb,
                PlannerObstacleMode::RESPECT_DYNAMIC,
                &probe.candidate.returnArrivalDir
            );

            if (!canFullBatteryVisitAndReturn(
                    rb,
                    probe.candidate.costFromRobot,
                    probe.candidate.costToBase
                ))
            {
                continue;
            }

            if (!canVisitTargetAndReturn(
                    rb,
                    probe.candidate.costFromRobot,
                    probe.candidate.costToBase
                ))
            {
                continue;
            }

            if (!hasChoice || isBetterCleanupChoice(probe.candidate, bestProbe.candidate))
            {
                hasChoice = true;
                bestProbe = probe;
            }
        }

        if (!hasChoice)
        {
            if (activeId >= 0)
                uncovered_island::releaseActiveComponent();

            return false;
        }

        CoverageCandidate bestChoice = bestProbe.candidate;
        rb.path = bestChoice.path;

        if ((int)rb.path.size() <= 1)
        {
            markCovered(bestChoice.target.r, bestChoice.target.c);
            clearRobotPath(rb);
            result = alreadyAtGoalResult(rb);
            return true;
        }

        rb.pathID = 1;

        if (hasBlockedCellAnywhereOnPath(rb) || !isNextPathCellFree(rb))
        {
            clearRobotPath(rb);
            uncovered_island::releaseActiveComponent();
            return false;
        }

        double builtPathCost = pathEnergyCost(rb.path, startDir);
        uncovered_island::markComponentSelected(bestProbe.componentId);

        if (ctx != nullptr)
        {
            beginDecisionTrace(
                *ctx,
                rb,
                "coverage",
                bestChoice.target,
                "local_cleanup_component_fast",
                (int)normalCandidates.size(),
                bestChoice.costFromRobot,
                bestChoice.costToBase
            );

            vector<Cell> returnPreview = traceTempPathOriented(
                bestChoice.target,
                bestChoice.arrivalDir,
                rb.base,
                bestChoice.returnArrivalDir
            );

            logDecisionPath(
                *ctx,
                rb,
                "cleanup_path_built",
                true,
                "path_cost=" + formatEnergy(builtPathCost) +
                " arrival_dir=" + to_string((int)bestChoice.arrivalDir) +
                " cost_to_base=" + formatEnergy(bestChoice.costToBase) +
                " return_arrival_dir=" + to_string((int)bestChoice.returnArrivalDir) +
                " cleanup_component=" + to_string(bestProbe.componentId) +
                " cleanup_source=" + uncovered_island::cleanupSourceName(bestProbe.source) +
                " cleanup_component_size=" + to_string(bestProbe.componentSize) +
                " continuing_active_component=" + boolText(bestProbe.continuingActiveComponent) +
                " best_normal_cost=" + formatEnergy(bestNormalCost) +
                " cleanup_probes=" + to_string((int)probes.size()) +
                " cleanup_return_checks=" + to_string(returnChecks) +
                " pending_cleanup_components=" + to_string(uncovered_island::pendingCleanupComponentCount()) +
                " pending_cleanup_cells=" + to_string(uncovered_island::pendingCleanupCellCount()) +
                " revisit_count_on_path=" + to_string(bestChoice.revisitCountOnPath) +
                " frontier_score=" + to_string(bestChoice.frontierScore) +
                " first_step=" + cellText(rb.path[rb.pathID]) +
                " path=" + compactPathText(rb.path) +
                " return_path_preview=" + compactPathText(returnPreview)
            );
        }

        result = {true, false, false, false, builtPathCost};
        return true;
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

    PathBuildResult cleanupResult;

    if (tryBuildCleanupPath(rb, ctx, candidates, cleanupResult))
        return cleanupResult;

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

        sort(feasibleGroup.begin(), feasibleGroup.end(), compareCandidateByCoveragePolicy);

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
                    "nearest_uncovered_feasible",
                    (int)candidates.size(),
                    candidate.costFromRobot,
                    candidate.costToBase
                );
            }

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
                    " pending_cleanup_components=" + to_string(uncovered_island::pendingCleanupComponentCount()) +
                    " pending_cleanup_cells=" + to_string(uncovered_island::pendingCleanupCellCount()) +
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
