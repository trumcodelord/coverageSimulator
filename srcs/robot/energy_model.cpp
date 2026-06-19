#include "energy_model.h"

#include "grid.h"
#include "motion_geometry.h"

#include <algorithm>

using namespace std;

int movementEnergyCostForStep(
    const Robot &,
    Cell next,
    RobotMode mode,
    const EnergyCostConfig &config
) {
    int terrainEntryCost = effectiveTerrainCostAt(next.r, next.c);

    if (terrainEntryCost >= INF)
        return INF;

    int cost = max(config.baseMoveCost, terrainEntryCost);

    if (mode == ALERT)
        cost += config.alertPenalty;

    if (mode == RETURN_TO_BASE)
        cost += config.returnPenalty;

    if (mode == FINAL_PUSH)
        cost += config.finalPushPenalty;

    return max(0, cost);
}

int turnQuarterEnergyCostForStep(
    const Robot &rb,
    Cell next
) {
    if (quarterTurnsForMove(rb, next) <= 0)
        return 0;

    int currentTerrainCost = baseTerrainCostAt(rb.pos.r, rb.pos.c);

    if (currentTerrainCost >= INF)
        return INF;

    return currentTerrainCost;
}

int computeMoveEnergyCost(
    const Robot &rb,
    Cell next,
    RobotMode mode,
    const EnergyCostConfig &config
) {
    int movementCost =
        movementEnergyCostForStep(rb, next, mode, config);

    int turnQuarterCost =
        turnQuarterEnergyCostForStep(rb, next);

    if (movementCost >= INF || turnQuarterCost >= INF)
        return INF;

    return movementCost +
           quarterTurnsForMove(rb, next) * turnQuarterCost;
}

void consumeEnergy(Robot &rb, int amount)
{
    if (amount <= 0)
        return;

    rb.energy = max(0, rb.energy - amount);
    rb.totalEnergyUsed += amount;
}
