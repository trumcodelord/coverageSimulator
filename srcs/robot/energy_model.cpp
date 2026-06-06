#include "energy_model.h"

#include "grid.h"

#include <algorithm>
#include <cmath>

using namespace std;

namespace
{
    bool isAtBase(const Robot &rb)
    {
        return rb.pos == rb.base;
    }

    double angleForMove(Cell from, Cell to)
    {
        int dr = to.r - from.r;
        int dc = to.c - from.c;

        if (dc > 0) return -90.0;
        if (dc < 0) return 90.0;
        if (dr > 0) return 180.0;
        return 0.0;
    }

    double normalizeAngle(double angle)
    {
        while (angle <= -180.0) angle += 360.0;
        while (angle > 180.0) angle -= 360.0;
        return angle;
    }

    int quarterTurnsForMove(const Robot &rb, Cell next)
    {
        double targetAngle = angleForMove(rb.pos, next);
        double delta = fabs(normalizeAngle(targetAngle - rb.headingDeg));

        if (delta < 1e-6)
            return 0;

        return delta > 135.0 ? 2 : 1;
    }
}

int movementEnergyCostForStep(
    const Robot &rb,
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

int returnMarginForCost(
    int costToBase,
    const ReturnEnergyPolicy &policy
) {
    if (costToBase >= INF)
        return INF;

    return max(
        policy.minReturnMargin,
        costToBase / policy.returnMarginDivisor
    );
}

bool shouldReturnForEnergy(
    const Robot &rb,
    int costToBase,
    const ReturnEnergyPolicy &policy
) {
    if (isAtBase(rb))
        return false;

    if (costToBase >= INF)
        return false;

    return rb.energy <= costToBase + returnMarginForCost(costToBase, policy);
}

bool isCriticalEnergy(
    const Robot &rb,
    int costToBase,
    const ReturnEnergyPolicy &policy
) {
    if (isAtBase(rb))
        return false;

    if (costToBase >= INF)
        return rb.energy <= policy.minReturnMargin;

    return rb.energy <= costToBase ||
           rb.energy <= policy.minEmergencyEnergy;
}

bool canVisitTargetAndReturn(
    const Robot &rb,
    int costToTarget,
    int costTargetToBase,
    const ReturnEnergyPolicy &policy
) {
    if (costToTarget < 0 || costTargetToBase < 0)
        return false;

    if (costToTarget >= INF || costTargetToBase >= INF)
        return false;

    long long requiredEnergy =
        (long long)costToTarget +
        (long long)costTargetToBase +
        (long long)returnMarginForCost(costTargetToBase, policy);

    return rb.energy >= requiredEnergy;
}
