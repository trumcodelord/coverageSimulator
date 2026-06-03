#include "tactical_yield.h"
#include "planner.h"

static bool isNearDynamicObstacle(Cell p)
{
    for (int dr = -1; dr <= 1; dr++)
    {
        for (int dc = -1; dc <= 1; dc++)
        {
            int r = p.r + dr;
            int c = p.c + dc;

            if (!inBounds(r, c))
                continue;

            if (isDynamicBlockedCell(r, c))
                return true;
        }
    }

    return false;
}

static bool isUsefulYieldCell(const Robot &rb, Cell p)
{
    if (!inBounds(p.r, p.c))
        return false;

    if (!isFree(p.r, p.c))
        return false;

    if (!isCovered(p.r, p.c))
        return false;

    if (isNearDynamicObstacle(p))
        return false;

    if (p == rb.pos)
        return false;

    return true;
}

TacticalYieldResult findTacticalYieldCell(
    const Robot &rb,
    const TacticalYieldConfig &config
) {
    TacticalYieldResult result;

    dijkstra(rb.pos, d, trace);

    for (int r = 1; r <= rows; r++)
    {
        for (int c = 1; c <= cols; c++)
        {
            Cell candidate = {r, c};

            if (!isUsefulYieldCell(rb, candidate))
                continue;

            int costFromRobot = d[r][c];

            if (costFromRobot <= 0 || costFromRobot > config.maxCandidateCost)
                continue;

            dijkstra(candidate, d, trace);
            int costToBase = d[rb.base.r][rb.base.c];

            if (costToBase >= INF)
            {
                dijkstra(rb.pos, d, trace);
                continue;
            }

            int score = costFromRobot + costToBase;

            if (!result.found || score < result.score)
            {
                result.found = true;
                result.target = candidate;
                result.costFromRobot = costFromRobot;
                result.costToBase = costToBase;
                result.score = score;
            }

            dijkstra(rb.pos, d, trace);
        }
    }

    return result;
}
