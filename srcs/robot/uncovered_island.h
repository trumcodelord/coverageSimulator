#pragma once

#include "types.h"

namespace uncovered_island
{
    constexpr int SMALL_ISLAND_LIMIT = 5;
    constexpr int NO_ISLAND_PRIORITY = SMALL_ISLAND_LIMIT + 1;

    bool isUncoveredTarget(Cell p);

    int cleanupPriorityForTarget(Cell target);
    int pendingCleanupCellCount();

    // Called after a real newly-covered cell is committed.
    // It detects both:
    // 1) split/dead-end uncovered components caused by covering that cell;
    // 2) small local pockets/corridor leftovers near the robot, especially along
    //    static obstacles or map borders.
    // Returns the number of newly-added pending cleanup cells.
    int notifyCoveredCell(Cell coveredCell);

    void clearPendingCleanup();
}
