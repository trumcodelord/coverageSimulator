#include "tactical_yield.h"
#include "energy_model.h"
#include "planner.h"

#include <vector>

namespace
{
    struct OrientedYieldCandidate
    {
        Cell target = {0, 0};
        double costFromRobot = INF;
        HeadingDir arrivalDir = DIR_NORTH;
    };

    bool isNearDynamicObstacle(Cell p)
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

    bool isUsefulYieldCell(const Robot &rb, Cell p)
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
}

TacticalYieldResult findTacticalYieldCell(
    const Robot &rb,
    const TacticalYieldConfig &config
) {
    TacticalYieldResult result;
    std::vector<OrientedYieldCandidate> candidates;

    HeadingDir startDir = headingDirFromDegrees(rb.headingDeg);

    dijkstraOriented(
        rb.pos,
        startDir,
        PlannerObstacleMode::RESPECT_DYNAMIC
    );

    for (int r = 1; r <= rows; r++)
    {
        for (int c = 1; c <= cols; c++)
        {
            Cell candidate = {r, c};

            if (!isUsefulYieldCell(rb, candidate))
                continue;

            HeadingDir arrivalDir = DIR_NORTH;
            double costFromRobot =
                bestOrientedDistanceTo(candidate, &arrivalDir);

            if (costFromRobot <= 0.0 ||
                costFromRobot > config.maxCandidateCost)
            {
                continue;
            }

            candidates.push_back({
                candidate,
                costFromRobot,
                arrivalDir
            });
        }
    }

    for (const OrientedYieldCandidate &candidate : candidates)
    {
        dijkstraOriented(
            candidate.target,
            candidate.arrivalDir,
            PlannerObstacleMode::RESPECT_DYNAMIC
        );

        double costToBase = bestOrientedDistanceTo(rb.base);

        if (costToBase >= INF)
            continue;

        double score = quantizeEnergy(candidate.costFromRobot + costToBase);

        if (score >= INF)
            continue;

        if (!result.found || score < result.score)
        {
            result.found = true;
            result.target = candidate.target;
            result.costFromRobot = candidate.costFromRobot;
            result.costToBase = costToBase;
            result.score = score;
        }
    }

    return result;
}
