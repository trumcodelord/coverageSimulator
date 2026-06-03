#include "grid.h"

using namespace std;

int rows, cols;
Cell start;
bool blocked[1001][1001], covered[1001][1001], dynamicBlocked[1001][1001];
int terrainCost[1001][1001];
const int dr[5] = {0, 1, 0, -1, 0};
const int dc[5] = {0, 0, 1, 0, -1};
int initialFreeCells = 0;

bool inBounds(int r, int c)
{
    return 1 <= r && r <= rows && 1 <= c && c <= cols;
}

bool isStaticBlocked(int r, int c)
{
    if (!inBounds(r, c))
        return true;

    return blocked[r][c] || terrainCost[r][c] >= INF;
}

bool isDynamicBlockedCell(int r, int c)
{
    return inBounds(r, c) && dynamicBlocked[r][c];
}

bool isBlockedCell(int r, int c)
{
    return !inBounds(r, c) ||
           isStaticBlocked(r, c) ||
           isDynamicBlockedCell(r, c);
}

int baseTerrainCostAt(int r, int c)
{
    if (!inBounds(r, c))
        return INF;

    if (isStaticBlocked(r, c))
        return INF;

    return terrainCost[r][c];
}

int effectiveTerrainCostAt(int r, int c)
{
    if (isBlockedCell(r, c))
        return INF;

    return terrainCost[r][c];
}

bool isFree(int r, int c)
{
    return effectiveTerrainCostAt(r, c) < INF;
}

bool isCovered(int r, int c)
{
    return inBounds(r, c) && covered[r][c];
}

bool isCoverageTargetCell(int r, int c)
{
    return inBounds(r, c) && !isStaticBlocked(r, c);
}

int terrainCostAt(int r, int c)
{
    return baseTerrainCostAt(r, c);
}

void markCovered(int r, int c)
{
    if (isFree(r, c))
        covered[r][c] = 1;
}

bool allCovered()
{
    for (int i = 1; i <= rows; i++)
        for (int j = 1; j <= cols; j++)
            if (isCoverageTargetCell(i, j) && !covered[i][j])
                return false;

    return true;
}

vector<Cell> getNeighbors(Cell p)
{
    vector<Cell> neighbors;

    for (int k = 1; k <= 4; k++)
    {
        int nr = p.r + dr[k];
        int nc = p.c + dc[k];

        if (isFree(nr, nc))
            neighbors.push_back({nr, nc});
    }

    return neighbors;
}
