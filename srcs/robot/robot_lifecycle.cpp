#include "robot_lifecycle.h"

#include "behavior_log.h"
#include "dynamic_obstacle.h"
#include "energy_model.h"
#include "grid.h"
#include "motion_geometry.h"
#include "planner.h"

#include <algorithm>
#include <array>
#include <limits>
#include <string>
#include <vector>

using namespace std;

namespace
{
    constexpr int INITIAL_HEADING_TARGET_SAMPLE = 50;

    struct InitialHeadingScore
    {
        HeadingDir dir = DIR_NORTH;
        double sampledCost = numeric_limits<double>::max();
        int sampledTargets = 0;
        int straightFreeCells = 0;
    };

    const char *headingName(HeadingDir dir)
    {
        if (dir == DIR_NORTH) return "NORTH";
        if (dir == DIR_EAST) return "EAST";
        if (dir == DIR_SOUTH) return "SOUTH";
        if (dir == DIR_WEST) return "WEST";
        return "UNKNOWN";
    }

    const char *headingKey(HeadingDir dir)
    {
        if (dir == DIR_NORTH) return "north";
        if (dir == DIR_EAST) return "east";
        if (dir == DIR_SOUTH) return "south";
        if (dir == DIR_WEST) return "west";
        return "unknown";
    }

    Cell nextCellInDirection(Cell p, HeadingDir dir)
    {
        if (dir == DIR_NORTH) p.r--;
        else if (dir == DIR_EAST) p.c++;
        else if (dir == DIR_SOUTH) p.r++;
        else p.c--;

        return p;
    }

    int countStraightFreeCells(Cell start, HeadingDir dir)
    {
        int count = 0;
        Cell cur = start;

        while (true)
        {
            cur = nextCellInDirection(cur, dir);

            if (!inBounds(cur.r, cur.c) || isStaticBlocked(cur.r, cur.c))
                break;

            count++;
        }

        return count;
    }

    InitialHeadingScore evaluateInitialHeading(Cell start, HeadingDir dir)
    {
        InitialHeadingScore score;
        score.dir = dir;
        score.straightFreeCells = countStraightFreeCells(start, dir);

        dijkstraOriented(
            start,
            dir,
            PlannerObstacleMode::IGNORE_DYNAMIC
        );

        vector<double> targetCosts;
        targetCosts.reserve(rows * cols);

        for (int r = 1; r <= rows; r++)
        {
            for (int c = 1; c <= cols; c++)
            {
                Cell target = {r, c};

                if (target == start || !isCoverageTargetCell(r, c))
                    continue;

                double cost = bestOrientedDistanceTo(target);

                if (cost < INF)
                    targetCosts.push_back(cost);
            }
        }

        sort(targetCosts.begin(), targetCosts.end());

        score.sampledTargets = min(
            INITIAL_HEADING_TARGET_SAMPLE,
            (int)targetCosts.size()
        );

        score.sampledCost = 0.0;

        for (int i = 0; i < score.sampledTargets; i++)
            score.sampledCost = quantizeEnergy(score.sampledCost + targetCosts[i]);

        return score;
    }

    bool isBetterInitialHeading(
        const InitialHeadingScore &candidate,
        const InitialHeadingScore &best
    ) {
        if (candidate.sampledTargets != best.sampledTargets)
            return candidate.sampledTargets > best.sampledTargets;

        if (candidate.sampledCost != best.sampledCost)
            return candidate.sampledCost < best.sampledCost;

        if (candidate.straightFreeCells != best.straightFreeCells)
            return candidate.straightFreeCells > best.straightFreeCells;

        return (int)candidate.dir < (int)best.dir;
    }

    HeadingDir chooseInitialHeading(Cell start)
    {
        const array<HeadingDir, 4> directions = {
            DIR_NORTH,
            DIR_EAST,
            DIR_SOUTH,
            DIR_WEST
        };

        array<InitialHeadingScore, 4> scores;

        for (int i = 0; i < 4; i++)
            scores[i] = evaluateInitialHeading(start, directions[i]);

        InitialHeadingScore best = scores[0];

        for (int i = 1; i < 4; i++)
        {
            if (isBetterInitialHeading(scores[i], best))
                best = scores[i];
        }

        string details;
        appendDetail(details, kv("selected", headingName(best.dir)));
        appendDetail(details, kv("setup_turn_energy", 0.0));
        appendDetail(details, kv("sample_limit", INITIAL_HEADING_TARGET_SAMPLE));

        for (const InitialHeadingScore &score : scores)
        {
            string suffix = headingKey(score.dir);

            appendDetail(details, kv("score_" + suffix, score.sampledCost));
            appendDetail(details, kv("targets_" + suffix, score.sampledTargets));
            appendDetail(details, kv("straight_" + suffix, score.straightFreeCells));
        }

        logReadableEvent(
            "INFO",
            "PLANNER",
            "initial_heading_selected",
            "Initial robot heading selected before mission.",
            details
        );

        return best.dir;
    }
}

void initializeCoverageRobot(Robot &rb, double maxEnergy)
{
    rb.steps = 0;
    rb.pathID = 0;
    rb.path.clear();
    rb.trail.clear();
    rb.edgeCount.clear();

    rb.base = rb.pos;
    rb.maxEnergy = quantizeEnergy(maxEnergy);
    rb.energy = rb.maxEnergy;
    rb.totalEnergyUsed = 0.0;
    rb.movementEnergyUsed = 0.0;
    rb.turnEnergyUsed = 0.0;
    rb.returnCount = 0;
    rb.rechargeCount = 0;
    rb.missionOutcome = MISSION_RUNNING;

    HeadingDir initialDir = chooseInitialHeading(rb.pos);
    rb.headingDeg = angleForDirection(initialDir);

    rb.trail.push_back(rb.pos);
    markCovered(rb.pos.r, rb.pos.c);
    setRobotAvoidanceCell(rb.pos);
}

void rechargeRobot(Robot &rb)
{
    rb.energy = rb.maxEnergy;
    rb.rechargeCount++;
}
