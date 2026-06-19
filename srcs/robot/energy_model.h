#pragma once

#include "energy_reserve_policy.h"
#include "types.h"

#include <string>

struct EnergyCostConfig
{
    double baseMoveCost = 1.0;
    double alertPenalty = 0.0;
    double returnPenalty = 0.0;
    double finalPushPenalty = 0.0;
};

enum EnergyUseCategory
{
    ENERGY_USE_GENERIC,
    ENERGY_USE_MOVEMENT,
    ENERGY_USE_TURN
};

double quantizeEnergy(double value);
std::string formatEnergy(double value);

double movementEnergyCostForStep(
    const Robot &rb,
    Cell next,
    RobotMode mode,
    const EnergyCostConfig &config = EnergyCostConfig()
);

double turnQuarterEnergyCostAtCell(Cell cell);

double turnQuarterEnergyCostForStep(
    const Robot &rb,
    Cell next
);

double computeMoveEnergyCost(
    const Robot &rb,
    Cell next,
    RobotMode mode,
    const EnergyCostConfig &config = EnergyCostConfig()
);

void consumeEnergy(
    Robot &rb,
    double amount,
    EnergyUseCategory category = ENERGY_USE_GENERIC
);
