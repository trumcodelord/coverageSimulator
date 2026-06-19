#include "planner.h"

#include "energy_model.h"

#include <algorithm>
#include <cmath>
#include <queue>
#include <tuple>

using namespace std;

double orientedDist[1001][1001][4];
OrientedTraceState orientedTrace[1001][1001][4];

double tempOrientedDist[1001][1001][4];
OrientedTraceState tempOrientedTrace[1001][1001][4];

namespace
{
    const int ORIENTED_DR[4] = {-1, 0, 1, 0};
    const int ORIENTED_DC[4] = {0, 1, 0, -1};

    bool isValidDirection(int dir)
    {
        return 0 <= dir && dir < 4;
    }

    double normalizeAngleForPlanner(double angle)
    {
        while (angle <= -180.0)
            angle += 360.0;

        while (angle > 180.0)
            angle -= 360.0;

        return angle;
    }

    double angularDistance(double a, double b)
    {
        return fabs(normalizeAngleForPlanner(a - b));
    }

    int movementTerrainCostAt(
        int r,
        int c,
        PlannerObstacleMode obstacleMode
    ) {
        if (obstacleMode == PlannerObstacleMode::IGNORE_DYNAMIC)
            return baseTerrainCostAt(r, c);

        return effectiveTerrainCostAt(r, c);
    }

    void resetOrientedTables(
        double dist[1001][1001][4],
        OrientedTraceState traceTable[1001][1001][4]
    ) {
        for (int r = 1; r <= rows; r++)
        {
            for (int c = 1; c <= cols; c++)
            {
                for (int dir = 0; dir < 4; dir++)
                {
                    dist[r][c][dir] = (double)INF;
                    traceTable[r][c][dir] = {0, 0, -1};
                }
            }
        }
    }

    void runDijkstraOriented(
        Cell start,
        HeadingDir startDir,
        PlannerObstacleMode obstacleMode,
        double dist[1001][1001][4],
        OrientedTraceState traceTable[1001][1001][4],
        Cell stopGoal
    ) {
        resetOrientedTables(dist, traceTable);

        if (!inBounds(start.r, start.c) ||
            isStaticBlocked(start.r, start.c) ||
            !isValidDirection((int)startDir))
        {
            return;
        }

        const bool hasStopGoal = inBounds(stopGoal.r, stopGoal.c);

        using QueueNode = tuple<double, int, int, int>;
        priority_queue<QueueNode, vector<QueueNode>, greater<QueueNode>> pq;

        dist[start.r][start.c][startDir] = 0.0;
        pq.push({0.0, start.r, start.c, (int)startDir});

        while (!pq.empty())
        {
            auto [du, ur, uc, currentDirValue] = pq.top();
            pq.pop();

            if (du != dist[ur][uc][currentDirValue])
                continue;

            // Dijkstra pops states in nondecreasing cost order. The first
            // popped state at the requested goal cell is therefore the best
            // arrival direction for that goal, so callers that only need one
            // target can stop without exploring the rest of the map.
            if (hasStopGoal && ur == stopGoal.r && uc == stopGoal.c)
                break;

            HeadingDir currentDir = static_cast<HeadingDir>(currentDirValue);
            double turnQuarterCost = turnQuarterEnergyCostAtCell({ur, uc});

            if (turnQuarterCost >= INF)
                continue;

            for (int nextDirValue = 0; nextDirValue < 4; nextDirValue++)
            {
                int vr = ur + ORIENTED_DR[nextDirValue];
                int vc = uc + ORIENTED_DC[nextDirValue];

                int movementCost = movementTerrainCostAt(
                    vr,
                    vc,
                    obstacleMode
                );

                if (movementCost >= INF)
                    continue;

                HeadingDir nextDir = static_cast<HeadingDir>(nextDirValue);
                int turns = quarterTurnsBetween(currentDir, nextDir);

                double candidateCost = quantizeEnergy(
                    du +
                    (double)movementCost +
                    (double)turns * turnQuarterCost
                );

                if (candidateCost >= INF)
                    continue;

                if (candidateCost >= dist[vr][vc][nextDirValue])
                    continue;

                dist[vr][vc][nextDirValue] = candidateCost;
                traceTable[vr][vc][nextDirValue] = {
                    (short)ur,
                    (short)uc,
                    (signed char)currentDirValue
                };

                pq.push({
                    candidateCost,
                    vr,
                    vc,
                    nextDirValue
                });
            }
        }
    }

    double orientedDistanceFromTable(
        double dist[1001][1001][4],
        Cell goal,
        HeadingDir goalDir
    ) {
        if (!inBounds(goal.r, goal.c) || !isValidDirection((int)goalDir))
            return (double)INF;

        return dist[goal.r][goal.c][goalDir];
    }

    double bestOrientedDistanceFromTable(
        double dist[1001][1001][4],
        Cell goal,
        HeadingDir *bestDir
    ) {
        if (bestDir != nullptr)
            *bestDir = DIR_NORTH;

        if (!inBounds(goal.r, goal.c))
            return (double)INF;

        double bestCost = (double)INF;
        int bestDirValue = DIR_NORTH;

        for (int dir = 0; dir < 4; dir++)
        {
            if (dist[goal.r][goal.c][dir] < bestCost)
            {
                bestCost = dist[goal.r][goal.c][dir];
                bestDirValue = dir;
            }
        }

        if (bestDir != nullptr)
            *bestDir = static_cast<HeadingDir>(bestDirValue);

        return bestCost;
    }

    vector<Cell> tracePathFromTable(
        double dist[1001][1001][4],
        OrientedTraceState traceTable[1001][1001][4],
        Cell start,
        HeadingDir startDir,
        Cell goal,
        HeadingDir goalDir
    ) {
        if (!inBounds(start.r, start.c) ||
            !inBounds(goal.r, goal.c) ||
            !isValidDirection((int)startDir) ||
            !isValidDirection((int)goalDir) ||
            orientedDistanceFromTable(dist, goal, goalDir) >= INF)
        {
            return {};
        }

        vector<Cell> path;
        Cell cur = goal;
        int curDir = (int)goalDir;

        while (!(cur == start && curDir == (int)startDir))
        {
            path.push_back(cur);

            OrientedTraceState previous =
                traceTable[cur.r][cur.c][curDir];

            if (previous.dir < 0 ||
                !inBounds(previous.r, previous.c) ||
                !isValidDirection(previous.dir))
            {
                return {};
            }

            cur = {previous.r, previous.c};
            curDir = previous.dir;
        }

        path.push_back(start);
        reverse(path.begin(), path.end());
        return path;
    }
}

HeadingDir headingDirFromDegrees(double headingDeg)
{
    const double CARDINAL_ANGLES[4] = {
        0.0,
        -90.0,
        180.0,
        90.0
    };

    int bestDir = DIR_NORTH;
    double bestDelta = angularDistance(headingDeg, CARDINAL_ANGLES[bestDir]);

    for (int dir = 1; dir < 4; dir++)
    {
        double delta = angularDistance(headingDeg, CARDINAL_ANGLES[dir]);

        if (delta < bestDelta)
        {
            bestDelta = delta;
            bestDir = dir;
        }
    }

    return static_cast<HeadingDir>(bestDir);
}

int quarterTurnsBetween(HeadingDir from, HeadingDir to)
{
    int delta = abs((int)from - (int)to);
    return min(delta, 4 - delta);
}

void dijkstraOriented(
    Cell start,
    HeadingDir startDir,
    PlannerObstacleMode obstacleMode,
    Cell stopGoal
) {
    runDijkstraOriented(
        start,
        startDir,
        obstacleMode,
        orientedDist,
        orientedTrace,
        stopGoal
    );
}

void dijkstraOrientedTemp(
    Cell start,
    HeadingDir startDir,
    PlannerObstacleMode obstacleMode,
    Cell stopGoal
) {
    runDijkstraOriented(
        start,
        startDir,
        obstacleMode,
        tempOrientedDist,
        tempOrientedTrace,
        stopGoal
    );
}

double orientedDistanceTo(Cell goal, HeadingDir goalDir)
{
    return orientedDistanceFromTable(orientedDist, goal, goalDir);
}

double bestOrientedDistanceTo(Cell goal, HeadingDir *bestDir)
{
    return bestOrientedDistanceFromTable(orientedDist, goal, bestDir);
}

double tempOrientedDistanceTo(Cell goal, HeadingDir goalDir)
{
    return orientedDistanceFromTable(tempOrientedDist, goal, goalDir);
}

double bestTempOrientedDistanceTo(Cell goal, HeadingDir *bestDir)
{
    return bestOrientedDistanceFromTable(tempOrientedDist, goal, bestDir);
}

vector<Cell> tracePathOriented(
    Cell start,
    HeadingDir startDir,
    Cell goal,
    HeadingDir goalDir
) {
    return tracePathFromTable(
        orientedDist,
        orientedTrace,
        start,
        startDir,
        goal,
        goalDir
    );
}

vector<Cell> traceBestPathOriented(
    Cell start,
    HeadingDir startDir,
    Cell goal
) {
    HeadingDir bestDir;

    if (bestOrientedDistanceTo(goal, &bestDir) >= INF)
        return {};

    return tracePathOriented(start, startDir, goal, bestDir);
}

vector<Cell> traceTempPathOriented(
    Cell start,
    HeadingDir startDir,
    Cell goal,
    HeadingDir goalDir
) {
    return tracePathFromTable(
        tempOrientedDist,
        tempOrientedTrace,
        start,
        startDir,
        goal,
        goalDir
    );
}
