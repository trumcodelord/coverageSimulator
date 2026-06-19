#pragma once

#include "types.h"

struct ReturnEnergyPolicy
{
    // Minimum reserve for an unexpected turn/replan near the base.
    double minReturnMargin = 15.0;

    // Long return paths receive an additional proportional reserve:
    // ceil(costToBase / returnMarginDivisor) to the nearest 0.5 energy unit.
    double returnMarginDivisor = 4.0;

    // Absolute emergency threshold used when the robot is nearly depleted.
    double minEmergencyEnergy = 3.0;
};

double returnMarginForCost(
    double costToBase,
    const ReturnEnergyPolicy &policy = ReturnEnergyPolicy()
);

bool shouldReturnForEnergy(
    const Robot &rb,
    double costToBase,
    const ReturnEnergyPolicy &policy = ReturnEnergyPolicy()
);

bool isCriticalEnergy(
    const Robot &rb,
    double costToBase,
    const ReturnEnergyPolicy &policy = ReturnEnergyPolicy()
);

bool canVisitTargetAndReturn(
    const Robot &rb,
    double costToTarget,
    double costTargetToBase,
    const ReturnEnergyPolicy &policy = ReturnEnergyPolicy()
);
