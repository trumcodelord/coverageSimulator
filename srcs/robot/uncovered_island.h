#pragma once

#include "grid.h"
#include "types.h"

#include <algorithm>
#include <cstring>
#include <queue>
#include <vector>

namespace uncovered_island
{
    constexpr int SMALL_ISLAND_LIMIT = 5;
    constexpr int NO_ISLAND_PRIORITY = SMALL_ISLAND_LIMIT + 1;

    struct PendingCleanupCell
    {
        Cell cell = {0, 0};
        int componentSize = NO_ISLAND_PRIORITY;
    };

    inline std::vector<PendingCleanupCell> pendingCleanupCells;
    inline int visitedStamp[1001][1001];
    inline int activeStamp = 1;

    inline bool isUncoveredTarget(Cell p)
    {
        return inBounds(p.r, p.c) &&
               isCoverageTargetCell(p.r, p.c) &&
               !covered[p.r][p.c];
    }

    inline bool sameCell(Cell a, Cell b)
    {
        return a.r == b.r && a.c == b.c;
    }

    inline void pruneStaleCleanupCells()
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

    inline int cleanupPriorityForTarget(Cell target)
    {
        pruneStaleCleanupCells();

        int best = NO_ISLAND_PRIORITY;

        for (const PendingCleanupCell &entry : pendingCleanupCells)
        {
            if (sameCell(entry.cell, target))
                best = std::min(best, entry.componentSize);
        }

        return best;
    }

    inline int pendingCleanupCellCount()
    {
        pruneStaleCleanupCells();
        return (int)pendingCleanupCells.size();
    }

    inline void nextStamp()
    {
        activeStamp++;

        if (activeStamp < 2000000000)
            return;

        std::memset(visitedStamp, 0, sizeof(visitedStamp));
        activeStamp = 1;
    }

    inline void addCleanupComponent(const std::vector<Cell> &component)
    {
        int componentSize = (int)component.size();

        if (componentSize <= 0 || componentSize > SMALL_ISLAND_LIMIT)
            return;

        for (Cell p : component)
        {
            bool exists = false;

            for (const PendingCleanupCell &entry : pendingCleanupCells)
            {
                if (sameCell(entry.cell, p))
                {
                    exists = true;
                    break;
                }
            }

            if (!exists)
                pendingCleanupCells.push_back({p, componentSize});
        }
    }

    inline std::vector<Cell> uncoveredNeighbors4(Cell p)
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

    inline std::vector<Cell> collectUncoveredComponent(Cell seed)
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

    inline int notifyCoveredCell(Cell coveredCell)
    {
        pruneStaleCleanupCells();

        std::vector<Cell> neighbors = uncoveredNeighbors4(coveredCell);

        // Removing a cell with zero or one uncovered neighbor cannot split the
        // remaining uncovered graph into multiple local islands.
        if ((int)neighbors.size() <= 1)
            return 0;

        nextStamp();

        int newCleanupCells = 0;

        for (Cell seed : neighbors)
        {
            if (visitedStamp[seed.r][seed.c] == activeStamp)
                continue;

            std::vector<Cell> component = collectUncoveredComponent(seed);

            if ((int)component.size() <= SMALL_ISLAND_LIMIT)
            {
                int before = (int)pendingCleanupCells.size();
                addCleanupComponent(component);
                newCleanupCells += (int)pendingCleanupCells.size() - before;
            }
        }

        return newCleanupCells;
    }
}
