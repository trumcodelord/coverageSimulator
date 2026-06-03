#include "planner.h"
#include <queue>
#include <algorithm>

using namespace std;

int d[1001][1001];
Cell trace[1001][1001];

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
            int stepCost = terrainCostAt(v.r, v.c);

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
            if (!blocked[i][j] && !covered[i][j] && d[i][j] < best)
            {
                best = d[i][j];
                target = {i, j};
            }
        }
    }

    return target;
}
