#pragma once

#include "types.h"

#include <string>
#include <vector>

namespace uncovered_island
{
    constexpr int SMALL_ISLAND_LIMIT = 5;
    constexpr int NO_ISLAND_PRIORITY = SMALL_ISLAND_LIMIT + 1;

    enum class CleanupSource
    {
        SPLIT,
        DEAD_END,
        LOCAL_POCKET
    };

    struct CleanupComponentView
    {
        int id = -1;
        CleanupSource source = CleanupSource::LOCAL_POCKET;
        int size = 0;
        Cell sourceCell = {0, 0};
        int createdStep = 0;
        bool active = false;
        std::vector<Cell> cells;
    };

    bool isUncoveredTarget(Cell p);

    // Legacy/debug helper. Do not use this as the primary global target policy.
    // Local cleanup is selected through component candidates instead.
    int cleanupPriorityForTarget(Cell target);

    int pendingCleanupCellCount();
    int pendingCleanupComponentCount();

    std::vector<CleanupComponentView> cleanupComponents();

    int activeComponentId();
    void markComponentSelected(int componentId);
    void releaseActiveComponent();

    std::string cleanupSourceName(CleanupSource source);

    // Called after a real newly-covered cell is committed.
    // Returns the number of newly-created cleanup components, not cells.
    int notifyCoveredCell(Cell coveredCell, int currentStep);

    void pruneStaleComponents(int currentStep);
    void clearPendingCleanup();
}
