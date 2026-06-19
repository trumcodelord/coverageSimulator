#pragma once

#include "types.h"

struct TacticalYieldConfig
{
    double maxCandidateCost = 3.0;
};

struct TacticalYieldResult
{
    bool found = false;
    Cell target = {0, 0};
    double costFromRobot = INF;
    double costToBase = INF;
    double score = INF;
};

TacticalYieldResult findTacticalYieldCell(
    const Robot &rb,
    const TacticalYieldConfig &config = TacticalYieldConfig()
);
