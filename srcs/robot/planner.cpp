#include "planner.h"

#include "energy_model.h"

#include <algorithm>
#include <cmath>
#include <queue>
#include <tuple>

using namespace std;

int d[1001][1001];
Cell trace[1001][1001];

double orientedDist[1001][1001][4];
OrientedTraceState orientedTrace[1001][1001][4];

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
}

void dijkstra(Cell start, int d[1001][1001], Cell trace[1001][1001])
{
    for (int i = 1; i <= rows; i++)
        for (int j = 1; j <= cols; j++)
        {
            d[i][j] = INF;
            trace[i][j] = {0, 0};
        }

    priority_queue<pair<int, Cell>, vector<pair<int, Cell>>, greater<pair<int, Cell>>> pq;

    d[start.r][start.c] = 0;
    trace[start.r][start.c] = {0, 0};
    pq.push({0, start});

    while (!pq.empty())
    {
        Cell u = pq.top().second;
        int du = pq.top().first;
        pq.pop();

        if (du != d[u.r][u.c])
            continue;

        for (Cell v : getNeighbors(u))
        {
            int stepCost = effectiveTerrainCostAt(v.r, v.c);

            if (stepCost >= INF)
                continue;

            int uv = du + stepCost;

            if (uv < d[v.r][v.c])
            {
                d[v.r][v.c] = uv;
                trace[v.r][v.c] = u;
                pq.push({uv, v});
            }
        }
    }
}

vector<Cell> tracePath(Cell start, Cell goal, Cell trace[1001][1001])
{
    if (goal == Cell{0, 0})
        return {};

    if (start == goal)
        return {start};

    vector<Cell> path;
    Cell cur = goal;

    while (!(cur == Cell{0, 0}) && !(cur == start))
    {
        path.push_back(cur);
        cur = trace[cur.r][cur.c];
    }

    if (!(cur == start))
        return {};

    path.push_back(start);
    reverse(path.begin(), path.end());

    return path;
}

Cell findNearestUncovered(Cell start)
{
    dijkstra(start, d, trace);

    int best = INF;
    Cell target = {0, 0};

    for (int i = 1; i <= rows; i++)
    {
        for (int j = 1; j <= cols; j++)
        {
            if (isCoverageTargetCell(i, j) && !covered[i][j] && d[i][j] < best)
            {
                best = d[i][j];
                target = {i, j};
            }
        }
    }

    return target;
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
    PlannerObstacleMode obstacleMode
) {
    for (int r = 1; r <= rows; r++)
    {
        for (int c = 1; c <= cols; c++)
        {
            for (int dir = 0; dir < 4; dir++)
            {
                orientedDist[r][c][dir] = (double)INF;
                orientedTrace[r][c][dir] = {0, 0, -1};
            }
        }
    }

    if (!inBounds(start.r, start.c) ||
        isStaticBlocked(start.r, start.c) ||
        !isValidDirection((int)startDir))
    {
        return;
    }

    using QueueNode = tuple<double, int, int, int>;
    priority_queue<QueueNode, vector<QueueNode>, greater<QueueNode>> pq;

    orientedDist[start.r][start.c][startDir] = 0.0;
    pq.push({0.0, start.r, start.c, (int)startDir});

    while (!pq.empty())
    {
        auto [du, ur, uc, currentDirValue] = pq.top();
        pq.pop();

        if (du != orientedDist[ur][uc][currentDirValue])
            continue;

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

            if (candidateCost >= orientedDist[vr][vc][nextDirValue])
                continue;

            orientedDist[vr][vc][nextDirValue] = candidateCost;
            orientedTrace[vr][vc][nextDirValue] = {
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

double orientedDistanceTo(Cell goal, HeadingDir goalDir)
{
    if (!inBounds(goal.r, goal.c) || !isValidDirection((int)goalDir))
        return (double)INF;

    return orientedDist[goal.r][goal.c][goalDir];
}

double bestOrientedDistanceTo(Cell goal, HeadingDir *bestDir)
{
    if (bestDir != nullptr)
        *bestDir = DIR_NORTH;

    if (!inBounds(goal.r, goal.c))
        return (double)INF;

    double bestCost = (double)INF;
    int bestDirValue = DIR_NORTH;

    for (int dir = 0; dir < 4; dir++)
    {
        if (orientedDist[goal.r][goal.c][dir] < bestCost)
        {
            bestCost = orientedDist[goal.r][goal.c][dir];
            bestDirValue = dir;
        }
    }

    if (bestDir != nullptr)
        *bestDir = static_cast<HeadingDir>(bestDirValue);

    return bestCost;
}

vector<Cell> tracePathOriented(
    Cell start,
    HeadingDir startDir,
    Cell goal,
    HeadingDir goalDir
) {
    if (!inBounds(start.r, start.c) ||
        !inBounds(goal.r, goal.c) ||
        !isValidDirection((int)startDir) ||
        !isValidDirection((int)goalDir) ||
        orientedDistanceTo(goal, goalDir) >= INF)
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
            orientedTrace[cur.r][cur.c][curDir];

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
