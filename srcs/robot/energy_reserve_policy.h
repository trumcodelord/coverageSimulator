#pragma once

#include "types.h"

struct ReturnEnergyPolicy
{
    // Minimum reserve for an unexpected turn/replan near the base.
    int minReturnMargin = 15;

    // Long return paths receive an additional proportional reserve:
    // ceil(costToBase / returnMarginDivisor).
    int returnMarginDivisor = 4;

    // Absolute emergency threshold used when the robot is nearly depleted.
    int minEmergencyEnergy = 3;
};

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
