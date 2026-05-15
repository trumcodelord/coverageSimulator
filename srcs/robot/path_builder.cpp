#include "path_builder.h"
#include "planner.h"
#include "path_safety.h"

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
        return {isAtBase(rb), isAtBase(rb)};
    }

    rb.pathID = 1;

    if (!isNextPathCellFree(rb))
    {
        clearRobotPath(rb);
        return {};
    }

    return {true, false};
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

    std::vector<Cell> candidate = tracePath(rb.pos, rb.base, trace);

    if ((int)candidate.size() <= 1)
    {
        clearRobotPath(rb);
        return {isAtBase(rb), isAtBase(rb)};
    }

    if (isPathNearDynamicObstacle(candidate, 1, 1))
    {
        clearRobotPath(rb);
        return {};
    }

    rb.path = candidate;
    rb.pathID = 1;

    return {true, false};
}

PathBuildResult rebuildPathToNearestUncoveredTarget(Robot &rb)
{
    while (true)
    {
        Cell target = findNearestUncovered(rb.pos);

        if (target == Cell{0, 0})
        {
            clearRobotPath(rb);
            return {};
        }

        rb.path = tracePath(rb.pos, target, trace);

        if (rb.path.empty())
        {
            clearRobotPath(rb);
            return {};
        }

        if ((int)rb.path.size() <= 1)
        {
            markCovered(target.r, target.c);
            clearRobotPath(rb);
            continue;
        }

        rb.pathID = 1;

        if (!isNextPathCellFree(rb))
        {
            clearRobotPath(rb);
            return {};
        }

        return {true, false};
    }
}
