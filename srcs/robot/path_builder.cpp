#include "path_builder.h"
#include "behavior_log.h"
#include "coverage_context.h"
#include "energy_model.h"
#include "planner.h"
#include "path_safety.h"

#include <algorithm>
#include <vector>

using namespace std;

namespace
{
    struct CoverageCandidate
    {
        int costFromRobot = INF;
        Cell target = {0, 0};
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

    int pathCost(const vector<Cell> &path)
    {
        if (path.size() <= 1)
            return 0;

        int total = 0;

        for (int i = 1; i < (int)path.size(); i++)
            total += terrainCostAt(path[i].r, path[i].c);

        return total;
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
            " first_step=" + cellText(rb.path[rb.pathID])
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
            " first_step=" + cellText(rb.path[rb.pathID])
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

    for (const CoverageCandidate &candidate : candidates)
    {
        int costToBase =
            estimateCostFromTargetToBase(candidate.target, rb);

        if (!canFullBatteryVisitTargetAndReturn(
                rb,
                candidate.costFromRobot,
                costToBase
            ))
        {
            foundMaxEnergyInfeasibleTarget = true;
            continue;
        }

        if (!canVisitTargetAndReturn(
                rb,
                candidate.costFromRobot,
                costToBase
            ))
        {
            foundCurrentEnergyLowTarget = true;
            continue;
        }

        dijkstra(rb.pos, d, trace);

        rb.path = tracePath(rb.pos, candidate.target, trace);

        if (rb.path.empty())
        {
            clearRobotPath(rb);
            continue;
        }

        if ((int)rb.path.size() <= 1)
        {
            markCovered(candidate.target.r, candidate.target.c);
            clearRobotPath(rb);
            return rebuildPathToNearestUncoveredTarget(rb);
        }

        rb.pathID = 1;

        if (!isNextPathCellFree(rb))
        {
            clearRobotPath(rb);
            return {};
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

            logDecisionPath(
                *ctx,
                rb,
                "coverage_path_built",
                true,
                "path_cost=" + std::to_string(pathCost(rb.path)) +
                " first_step=" + cellText(rb.path[rb.pathID])
            );
        }

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
            " energy_infeasible=" + boolText(result.energyInfeasible)
        );
    }

    return result;
}
