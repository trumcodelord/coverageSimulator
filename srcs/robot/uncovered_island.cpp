#include "uncovered_island.h"

#include "grid.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <queue>
#include <string>
#include <vector>

namespace uncovered_island
{
    namespace
    {
        constexpr int LOCAL_POCKET_RADIUS = 2;
        constexpr int INACTIVE_COMPONENT_TTL_STEPS = 250;
        constexpr int ACTIVE_COMPONENT_TTL_STEPS = 500;

        struct CleanupComponent
        {
            int id = -1;
            CleanupSource source = CleanupSource::LOCAL_POCKET;
            std::vector<Cell> cells;
            Cell sourceCell = {0, 0};
            int createdStep = 0;
            bool active = true;
        };

        std::vector<CleanupComponent> cleanupComponentList;
        int nextComponentId = 1;
        int activeCleanupComponentId = -1;

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

        int sourcePriority(CleanupSource source)
        {
            if (source == CleanupSource::SPLIT)
                return 1;
            if (source == CleanupSource::DEAD_END)
                return 2;
            return 3;
        }

        bool cellInComponent(const CleanupComponent &component, Cell p)
        {
            for (Cell cell : component.cells)
                if (sameCell(cell, p))
                    return true;
            return false;
        }

        bool componentStillHasUncoveredCell(const CleanupComponent &component)
        {
            for (Cell p : component.cells)
                if (isUncoveredTarget(p))
                    return true;
            return false;
        }

        int uncoveredCellCountInComponent(const CleanupComponent &component)
        {
            int count = 0;

            for (Cell p : component.cells)
                if (isUncoveredTarget(p))
                    count++;

            return count;
        }

        int hardBoundaryCount(Cell p)
        {
            int count = 0;

            for (int k = 1; k <= 4; k++)
            {
                Cell n = {p.r + dr[k], p.c + dc[k]};

                if (!inBounds(n.r, n.c))
                {
                    count++;
                    continue;
                }

                if (isStaticBlocked(n.r, n.c) || !isCoverageTargetCell(n.r, n.c))
                    count++;
            }

            return count;
        }

        int softBoundaryCount(Cell p)
        {
            int count = 0;

            for (int k = 1; k <= 4; k++)
            {
                Cell n = {p.r + dr[k], p.c + dc[k]};

                if (!inBounds(n.r, n.c))
                    continue;

                if (isCoverageTargetCell(n.r, n.c) && covered[n.r][n.c])
                    count++;
            }

            return count;
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

        bool isReliableDeadEndCell(Cell p)
        {
            if (!isUncoveredTarget(p))
                return false;

            return uncoveredDegree(p) <= 1 &&
                   hardBoundaryCount(p) >= 1;
        }

        bool isReliableLocalPocketCell(Cell p)
        {
            if (!isUncoveredTarget(p))
                return false;

            int degree = uncoveredDegree(p);
            int hard = hardBoundaryCount(p);
            int soft = softBoundaryCount(p);

            // A covered cell is only a soft signal. Require at least one hard
            // boundary (wall/static obstacle/non-target/border) to avoid turning
            // every newly-swept corridor into a fake pocket.
            return degree <= 2 &&
                   hard >= 1 &&
                   soft >= 1;
        }

        bool componentOverlapsExisting(const std::vector<Cell> &cells)
        {
            for (const CleanupComponent &component : cleanupComponentList)
            {
                if (!component.active)
                    continue;

                for (Cell p : cells)
                    if (cellInComponent(component, p))
                        return true;
            }

            return false;
        }

        int addCleanupComponent(
            const std::vector<Cell> &cells,
            CleanupSource source,
            Cell sourceCell,
            int currentStep
        ) {
            if (cells.empty() || (int)cells.size() > SMALL_ISLAND_LIMIT)
                return 0;

            if (componentOverlapsExisting(cells))
                return 0;

            CleanupComponent component;
            component.id = nextComponentId++;
            component.source = source;
            component.cells = cells;
            component.sourceCell = sourceCell;
            component.createdStep = currentStep;
            component.active = true;

            cleanupComponentList.push_back(component);
            return 1;
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

        CleanupSource sourceForSmallComponent(const std::vector<Cell> &component)
        {
            bool hasDeadEnd = false;

            for (Cell p : component)
            {
                if (isReliableDeadEndCell(p))
                {
                    hasDeadEnd = true;
                    break;
                }
            }

            if (component.size() == 1 || hasDeadEnd)
                return CleanupSource::DEAD_END;

            return CleanupSource::SPLIT;
        }

        int detectSplitOrDeadEndComponents(Cell coveredCell, int currentStep)
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

                std::vector<Cell> component =
                    collectUncoveredComponentLimited(seed, SMALL_ISLAND_LIMIT + 1);

                if (component.empty() || (int)component.size() > SMALL_ISLAND_LIMIT)
                    continue;

                CleanupSource source = sourceForSmallComponent(component);
                added += addCleanupComponent(component, source, coveredCell, currentStep);
            }

            return added;
        }

        std::vector<Cell> collectLocalPocketCluster(Cell seed, Cell center)
        {
            std::vector<Cell> cluster;

            if (!isReliableLocalPocketCell(seed) && !isReliableDeadEndCell(seed))
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

                    if (manhattan(nxt, center) > LOCAL_POCKET_RADIUS)
                        continue;

                    if (!isReliableLocalPocketCell(nxt) && !isReliableDeadEndCell(nxt))
                        continue;

                    if (visitedStamp[nxt.r][nxt.c] == activeStamp)
                        continue;

                    visitedStamp[nxt.r][nxt.c] = activeStamp;
                    q.push(nxt);
                }
            }

            return cluster;
        }

        int scanLocalPocketsNear(Cell center, int currentStep)
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

                    if (!isReliableLocalPocketCell(seed) && !isReliableDeadEndCell(seed))
                        continue;

                    std::vector<Cell> cluster = collectLocalPocketCluster(seed, center);

                    if (cluster.empty() || (int)cluster.size() > SMALL_ISLAND_LIMIT)
                        continue;

                    CleanupSource source = CleanupSource::LOCAL_POCKET;

                    for (Cell p : cluster)
                    {
                        if (isReliableDeadEndCell(p))
                        {
                            source = CleanupSource::DEAD_END;
                            break;
                        }
                    }

                    added += addCleanupComponent(cluster, source, center, currentStep);
                }
            }

            return added;
        }

        void compactInactiveComponents()
        {
            cleanupComponentList.erase(
                std::remove_if(
                    cleanupComponentList.begin(),
                    cleanupComponentList.end(),
                    [](const CleanupComponent &component)
                    {
                        return !component.active;
                    }
                ),
                cleanupComponentList.end()
            );
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
        pruneStaleComponents(0);

        int best = NO_ISLAND_PRIORITY;

        for (const CleanupComponent &component : cleanupComponentList)
        {
            if (!component.active)
                continue;

            if (!cellInComponent(component, target))
                continue;

            best = std::min(best, sourcePriority(component.source));
        }

        return best;
    }

    int pendingCleanupCellCount()
    {
        pruneStaleComponents(0);

        int count = 0;

        for (const CleanupComponent &component : cleanupComponentList)
        {
            if (!component.active)
                continue;

            count += uncoveredCellCountInComponent(component);
        }

        return count;
    }

    int pendingCleanupComponentCount()
    {
        pruneStaleComponents(0);

        int count = 0;

        for (const CleanupComponent &component : cleanupComponentList)
            if (component.active)
                count++;

        return count;
    }

    std::vector<CleanupComponentView> cleanupComponents()
    {
        pruneStaleComponents(0);

        std::vector<CleanupComponentView> result;

        for (const CleanupComponent &component : cleanupComponentList)
        {
            if (!component.active)
                continue;

            CleanupComponentView view;
            view.id = component.id;
            view.source = component.source;
            view.size = uncoveredCellCountInComponent(component);
            view.sourceCell = component.sourceCell;
            view.createdStep = component.createdStep;
            view.active = component.id == activeCleanupComponentId;

            for (Cell p : component.cells)
                if (isUncoveredTarget(p))
                    view.cells.push_back(p);

            if (!view.cells.empty())
                result.push_back(view);
        }

        return result;
    }

    int activeComponentId()
    {
        return activeCleanupComponentId;
    }

    void markComponentSelected(int componentId)
    {
        activeCleanupComponentId = componentId;
    }

    void releaseActiveComponent()
    {
        activeCleanupComponentId = -1;
    }

    std::string cleanupSourceName(CleanupSource source)
    {
        if (source == CleanupSource::SPLIT)
            return "split";
        if (source == CleanupSource::DEAD_END)
            return "dead_end";
        return "local_pocket";
    }

    int notifyCoveredCell(Cell coveredCell, int currentStep)
    {
        pruneStaleComponents(currentStep);

        int added = 0;
        added += detectSplitOrDeadEndComponents(coveredCell, currentStep);
        added += scanLocalPocketsNear(coveredCell, currentStep);

        return added;
    }

    void pruneStaleComponents(int currentStep)
    {
        for (CleanupComponent &component : cleanupComponentList)
        {
            if (!component.active)
                continue;

            if (!componentStillHasUncoveredCell(component))
            {
                component.active = false;

                if (component.id == activeCleanupComponentId)
                    activeCleanupComponentId = -1;

                continue;
            }

            if (currentStep <= 0)
                continue;

            int age = currentStep - component.createdStep;
            int ttl = component.id == activeCleanupComponentId
                ? ACTIVE_COMPONENT_TTL_STEPS
                : INACTIVE_COMPONENT_TTL_STEPS;

            if (age > ttl)
            {
                component.active = false;

                if (component.id == activeCleanupComponentId)
                    activeCleanupComponentId = -1;
            }
        }

        compactInactiveComponents();
    }

    void clearPendingCleanup()
    {
        cleanupComponentList.clear();
        activeCleanupComponentId = -1;
        nextComponentId = 1;
    }
}
