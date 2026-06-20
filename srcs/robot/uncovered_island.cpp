#include "uncovered_island.h"

#include "grid.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <queue>
#include <vector>

namespace uncovered_island
{
    namespace
    {
        constexpr int LOCAL_POCKET_RADIUS = 2;

        struct PendingCleanupCell
        {
            Cell cell = {0, 0};
            int priority = NO_ISLAND_PRIORITY;
        };

        std::vector<PendingCleanupCell> pendingCleanupCells;
        int visitedStamp[1001][1001];
        int activeStamp = 1;

        bool sameCell(Cell a, Cell b)
        {
            return a.r == b.r && a.c == b.c;
        }

        int manhattan(Cell a, Cell b)
        {
            return std::abs(a.r - b.r) + std::abs(a.c - b.c);
        }

        void nextStamp()
        {
            activeStamp++;

            if (activeStamp < 2000000000)
                return;

            std::memset(visitedStamp, 0, sizeof(visitedStamp));
            activeStamp = 1;
        }

        std::vector<Cell> uncoveredNeighbors4(Cell p)
        {
            std::vector<Cell> result;

            for (int k = 1; k <= 4; k++)
            {
                Cell n = {p.r + dr[k], p.c + dc[k]};

                if (isUncoveredTarget(n))
                    result.push_back(n);
            }

            return result;
        }

        int uncoveredDegree(Cell p)
        {
            return (int)uncoveredNeighbors4(p).size();
        }

        int closedBoundaryCount(Cell p)
        {
            int count = 0;

            for (int k = 1; k <= 4; k++)
            {
                Cell n = {p.r + dr[k], p.c + dc[k]};

                if (!isUncoveredTarget(n))
                    count++;
            }

            return count;
        }

        bool isConstrainedPocketCell(Cell p)
        {
            if (!isUncoveredTarget(p))
                return false;

            int degree = uncoveredDegree(p);
            int closed = closedBoundaryCount(p);

            // A local pocket/corridor leftover is usually bounded by covered cells,
            // static obstacles, or the map border on at least two sides. This catches
            // dead-end/corridor cells that are not necessarily a separate global
            // uncovered component yet.
            return degree <= 2 && closed >= 2;
        }

        void pruneStaleCleanupCells()
        {
            pendingCleanupCells.erase(
                std::remove_if(
                    pendingCleanupCells.begin(),
                    pendingCleanupCells.end(),
                    [](const PendingCleanupCell &entry)
                    {
                        return !isUncoveredTarget(entry.cell);
                    }
                ),
                pendingCleanupCells.end()
            );
        }

        int addCleanupCell(Cell p, int priority)
        {
            if (!isUncoveredTarget(p))
                return 0;

            priority = std::max(1, std::min(priority, NO_ISLAND_PRIORITY - 1));

            for (PendingCleanupCell &entry : pendingCleanupCells)
            {
                if (sameCell(entry.cell, p))
                {
                    entry.priority = std::min(entry.priority, priority);
                    return 0;
                }
            }

            pendingCleanupCells.push_back({p, priority});
            return 1;
        }

        int addCleanupComponent(const std::vector<Cell> &component, int priority)
        {
            int added = 0;

            if (component.empty() || (int)component.size() > SMALL_ISLAND_LIMIT)
                return 0;

            priority = std::min(priority, (int)component.size());

            for (Cell p : component)
                added += addCleanupCell(p, priority);

            return added;
        }

        std::vector<Cell> collectUncoveredComponentLimited(Cell seed, int limit)
        {
            std::vector<Cell> component;

            if (!isUncoveredTarget(seed))
                return component;

            std::queue<Cell> q;
            visitedStamp[seed.r][seed.c] = activeStamp;
            q.push(seed);

            while (!q.empty())
            {
                Cell cur = q.front();
                q.pop();
                component.push_back(cur);

                if ((int)component.size() > limit)
                    return component;

                for (int k = 1; k <= 4; k++)
                {
                    Cell nxt = {cur.r + dr[k], cur.c + dc[k]};

                    if (!isUncoveredTarget(nxt))
                        continue;

                    if (visitedStamp[nxt.r][nxt.c] == activeStamp)
                        continue;

                    visitedStamp[nxt.r][nxt.c] = activeStamp;
                    q.push(nxt);
                }
            }

            return component;
        }

        int detectSplitOrDeadEndComponents(Cell coveredCell)
        {
            std::vector<Cell> neighbors = uncoveredNeighbors4(coveredCell);

            if (neighbors.empty())
                return 0;

            nextStamp();

            int added = 0;

            for (Cell seed : neighbors)
            {
                if (visitedStamp[seed.r][seed.c] == activeStamp)
                    continue;

                std::vector<Cell> component = collectUncoveredComponentLimited(
                    seed,
                    SMALL_ISLAND_LIMIT + 1
                );

                if ((int)component.size() <= SMALL_ISLAND_LIMIT)
                    added += addCleanupComponent(component, (int)component.size());
            }

            return added;
        }

        std::vector<Cell> collectLocalPocketCluster(Cell seed, Cell center)
        {
            std::vector<Cell> cluster;

            if (!isConstrainedPocketCell(seed))
                return cluster;

            std::queue<Cell> q;
            visitedStamp[seed.r][seed.c] = activeStamp;
            q.push(seed);

            while (!q.empty())
            {
                Cell cur = q.front();
                q.pop();
                cluster.push_back(cur);

                if ((int)cluster.size() > SMALL_ISLAND_LIMIT)
                    return cluster;

                for (int k = 1; k <= 4; k++)
                {
                    Cell nxt = {cur.r + dr[k], cur.c + dc[k]};

                    if (!isConstrainedPocketCell(nxt))
                        continue;

                    if (manhattan(nxt, center) > LOCAL_POCKET_RADIUS)
                        continue;

                    if (visitedStamp[nxt.r][nxt.c] == activeStamp)
                        continue;

                    visitedStamp[nxt.r][nxt.c] = activeStamp;
                    q.push(nxt);
                }
            }

            return cluster;
        }

        int scanLocalPocketsNear(Cell center)
        {
            nextStamp();

            int added = 0;

            for (int r = std::max(1, center.r - LOCAL_POCKET_RADIUS);
                 r <= std::min(rows, center.r + LOCAL_POCKET_RADIUS);
                 r++)
            {
                for (int c = std::max(1, center.c - LOCAL_POCKET_RADIUS);
                     c <= std::min(cols, center.c + LOCAL_POCKET_RADIUS);
                     c++)
                {
                    Cell seed = {r, c};

                    if (manhattan(seed, center) > LOCAL_POCKET_RADIUS)
                        continue;

                    if (visitedStamp[r][c] == activeStamp)
                        continue;

                    if (!isConstrainedPocketCell(seed))
                        continue;

                    std::vector<Cell> cluster = collectLocalPocketCluster(seed, center);

                    if (cluster.empty() || (int)cluster.size() > SMALL_ISLAND_LIMIT)
                        continue;

                    int bestDegree = 4;

                    for (Cell p : cluster)
                        bestDegree = std::min(bestDegree, uncoveredDegree(p));

                    // Stronger priority for dead ends; weaker but still useful
                    // priority for short corridor/pocket leftovers.
                    int priority = std::max(1, bestDegree + 1);
                    added += addCleanupComponent(cluster, priority);
                }
            }

            return added;
        }
    }

    bool isUncoveredTarget(Cell p)
    {
        return inBounds(p.r, p.c) &&
               isCoverageTargetCell(p.r, p.c) &&
               !covered[p.r][p.c];
    }

    int cleanupPriorityForTarget(Cell target)
    {
        pruneStaleCleanupCells();

        int best = NO_ISLAND_PRIORITY;

        for (const PendingCleanupCell &entry : pendingCleanupCells)
        {
            if (sameCell(entry.cell, target))
                best = std::min(best, entry.priority);
        }

        return best;
    }

    int pendingCleanupCellCount()
    {
        pruneStaleCleanupCells();
        return (int)pendingCleanupCells.size();
    }

    int notifyCoveredCell(Cell coveredCell)
    {
        pruneStaleCleanupCells();

        int added = 0;
        added += detectSplitOrDeadEndComponents(coveredCell);
        added += scanLocalPocketsNear(coveredCell);

        return added;
    }

    void clearPendingCleanup()
    {
        pendingCleanupCells.clear();
    }
}
