#pragma once

#include "energy_reserve_policy.h"
#include "types.h"

struct EnergyCostConfig
{
    int baseMoveCost = 1;
    int alertPenalty = 0;
    int returnPenalty = 0;
    int finalPushPenalty = 0;
};

int movementEnergyCostForStep(
    const Robot &rb,
    Cell next,
    RobotMode mode,
    const EnergyCostConfig &config = EnergyCostConfig()
);

int turnQuarterEnergyCostForStep(
    const Robot &rb,
    Cell next
);

int computeMoveEnergyCost(
    const Robot &rb,
    Cell next,
    RobotMode mode,
    const EnergyCostConfig &config = EnergyCostConfig()
);

void consumeEnergy(Robot &rb, int amount);
