#include "path_safety.h"
#include "grid.h"

#include <algorithm>
#include <cstdlib>

using namespace std;

bool isNearDynamicObstacle(Cell p, int radius)
{
    for (int dr = -radius; dr <= radius; dr++)
    {
        for (int dc = -radius; dc <= radius; dc++)
        {
            int r = p.r + dr;
            int c = p.c + dc;

            if (!inBounds(r, c))
                continue;

            int manhattan = abs(dr) + abs(dc);
            if (manhattan > radius)
                continue;

            if (dynamicBlocked[r][c])
                return true;
        }
    }

    return false;
}

bool hasImmediateDynamicDanger(
    const Robot &rb,
    const PathSafetyConfig &config
) {
    return isNearDynamicObstacle(rb.pos, config.dynamicDangerRadius);
}

bool hasBlockedCellAheadOnPath(
    const Robot &rb,
    const PathSafetyConfig &config
) {
    if (rb.pathID >= (int)rb.path.size())
        return false;

    int last = min(
        (int)rb.path.size(),
        rb.pathID + config.pathLookahead
    );

    for (int i = rb.pathID; i < last; i++)
    {
        Cell p = rb.path[i];

        if (!isFree(p.r, p.c))
            return true;
    }

    return false;
}

bool isPathNearDynamicObstacle(
    const vector<Cell> &path,
    int startIndex,
    int radius
) {
    if (startIndex < 0)
        startIndex = 0;

    for (int i = startIndex; i < (int)path.size(); i++)
    {
        if (isNearDynamicObstacle(path[i], radius))
            return true;
    }

    return false;
}

bool isNextPathCellFree(const Robot &rb)
{
    if (rb.pathID >= (int)rb.path.size())
        return false;

    Cell next = rb.path[rb.pathID];
    return isFree(next.r, next.c);
}
