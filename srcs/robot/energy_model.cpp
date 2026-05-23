#include "energy_model.h"

#include <algorithm>
#include <cstdlib>

using namespace std;

static bool isAtBase(const Robot &rb)
{
    return rb.pos == rb.base;
}

static Cell currentDirection(const Robot &rb)
{
    if (rb.trail.size() < 2)
        return {0, 0};

    Cell prev = rb.trail[rb.trail.size() - 2];
    Cell curr = rb.trail.back();

    return {
        curr.r - prev.r,
        curr.c - prev.c
    };
}

static Cell nextDirection(const Robot &rb, Cell next)
{
    return {
        next.r - rb.pos.r,
        next.c - rb.pos.c
    };
}

static bool directionChanged(const Robot &rb, Cell next)
{
    Cell a = currentDirection(rb);
    Cell b = nextDirection(rb, next);

    if (a == Cell{0, 0})
        return false;

    return !(a == b);
}

int computeMoveEnergyCost(
    const Robot &rb,
    Cell next,
    RobotMode mode,
    const EnergyCostConfig &config
) {
    int cost = config.baseMoveCost;

    if (directionChanged(rb, next))
        cost += config.turnCost;

    if (mode == ALERT)
        cost += config.alertPenalty;

    if (mode == RETURN_TO_BASE)
        cost += config.returnPenalty;

    if (mode == FINAL_PUSH)
        cost += config.finalPushPenalty;

    return max(0, cost);
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
