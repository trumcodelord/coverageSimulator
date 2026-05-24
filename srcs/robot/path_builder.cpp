#include "path_builder.h"

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

PathBuildResult rebuildPathToBase(Robot &rb)
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

    if (!isNextPathCellFree(rb))
    {
        clearRobotPath(rb);
        return {};
    }

    return {true, false, false, false};
}

PathBuildResult rebuildSafeDetourPathToBase(Robot &rb)
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

    return {true, false, false, false};
}

PathBuildResult rebuildPathToNearestUncoveredTarget(Robot &rb)
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

        return {true, false, false, false};
    }

    clearRobotPath(rb);

    PathBuildResult result;
    result.success = false;
    result.alreadyAtGoal = false;
    result.currentEnergyLow = foundCurrentEnergyLowTarget;
    result.energyInfeasible = !foundCurrentEnergyLowTarget && foundMaxEnergyInfeasibleTarget;
    return result;
}
