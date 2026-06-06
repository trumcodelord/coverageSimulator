#pragma once

#include "types.h"

struct EnergyCostConfig
{
    int baseMoveCost = 1;
    int alertPenalty = 0;
    int returnPenalty = 0;
    int finalPushPenalty = 0;
};

struct ReturnEnergyPolicy
{
    int minReturnMargin = 10;
    int returnMarginDivisor = 3;
    int minEmergencyEnergy = 3;
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

int returnMarginForCost(
    int costToBase,
    const ReturnEnergyPolicy &policy = ReturnEnergyPolicy()
);

bool shouldReturnForEnergy(
    const Robot &rb,
    int costToBase,
    const ReturnEnergyPolicy &policy = ReturnEnergyPolicy()
);

bool isCriticalEnergy(
    const Robot &rb,
    int costToBase,
    const ReturnEnergyPolicy &policy = ReturnEnergyPolicy()
);

bool canVisitTargetAndReturn(
    const Robot &rb,
    int costToTarget,
    int costTargetToBase,
    const ReturnEnergyPolicy &policy = ReturnEnergyPolicy()
);
