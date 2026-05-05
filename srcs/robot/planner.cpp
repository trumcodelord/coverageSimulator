#include "planner.h"
#include <queue>

using namespace std;

int d[1001][1001];
Cell trace[1001][1001];

void dijkstra(Cell start, int d[1001][1001], Cell trace[1001][1001])
{
    for (int i = 1; i <= rows; i++)
        for (int j = 1; j <= cols; j++)
            d[i][j] = INF;

    priority_queue<pair<int, Cell>, vector<pair<int, Cell>>, greater<pair<int, Cell>>> pq;

    d[start.r][start.c] = 0;
    trace[start.r][start.c] = {0, 0};
    pq.push({0, start});

    while (!pq.empty())
    {
        Cell u = pq.top().second;
        int du = pq.top().first;
        pq.pop();

        if (du != d[u.r][u.c]) continue;

        for (Cell v : getNeighbors(u))
        {
            int uv = du + 1;
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
    if (goal == Cell{0, 0}) return {};
    if (start == goal) return {start};

    Cell prev = trace[goal.r][goal.c];
    if (prev == Cell{0, 0}) return {};

    vector<Cell> path = tracePath(start, prev, trace);
    path.push_back(goal);
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
