
#pragma once

#include "types.h"

struct TacticalYieldConfig
{
    int maxCandidateCost = 3;
};

struct TacticalYieldResult
{
    bool found = false;
    Cell target = {0, 0};
    int costFromRobot = INF;
    int costToBase = INF;
    int score = INF;
};

TacticalYieldResult findTacticalYieldCell(
    const Robot &rb,
    const TacticalYieldConfig &config = TacticalYieldConfig()
);
