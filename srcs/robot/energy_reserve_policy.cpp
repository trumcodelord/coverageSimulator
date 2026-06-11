#include "energy_reserve_policy.h"

#include <algorithm>

using namespace std;

namespace
{
    bool isAtBase(const Robot &rb)
    {
        return rb.pos == rb.base;
    }

    int proportionalReserve(
        int costToBase,
        const ReturnEnergyPolicy &policy
    ) {
        if (policy.returnMarginDivisor <= 0)
            return policy.minReturnMargin;

        // Integer ceiling division keeps the reserve from being rounded down.
        long long numerator =
            (long long)costToBase + policy.returnMarginDivisor - 1;

        return (int)(numerator / policy.returnMarginDivisor);
    }
}

int returnMarginForCost(
    int costToBase,
    const ReturnEnergyPolicy &policy
) {
    if (costToBase < 0 || costToBase >= INF)
        return INF;

    return max(
        policy.minReturnMargin,
        proportionalReserve(costToBase, policy)
    );
}

bool shouldReturnForEnergy(
    const Robot &rb,
    int costToBase,
    const ReturnEnergyPolicy &policy
) {
    if (isAtBase(rb))
        return false;

    if (costToBase < 0 || costToBase >= INF)
        return false;

    int reserve = returnMarginForCost(costToBase, policy);

    if (reserve >= INF)
        return false;

    long long requiredEnergy =
        (long long)costToBase + (long long)reserve;

    return (long long)rb.energy <= requiredEnergy;
}

bool isCriticalEnergy(
    const Robot &rb,
    int costToBase,
    const ReturnEnergyPolicy &policy
) {
    if (isAtBase(rb))
        return false;

    if (costToBase < 0 || costToBase >= INF)
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

    int reserve = returnMarginForCost(costTargetToBase, policy);

    if (reserve >= INF)
        return false;

    long long requiredEnergy =
        (long long)costToTarget +
        (long long)costTargetToBase +
        (long long)reserve;

    return (long long)rb.energy >= requiredEnergy;
}
