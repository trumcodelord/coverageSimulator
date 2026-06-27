#include "energy_reserve_policy.h"

#include "dynamic_obstacle.h"
#include "energy_model.h"

#include <algorithm>
#include <cmath>

using namespace std;

namespace
{
    bool isAtBase(const Robot &rb)
    {
        return rb.pos == rb.base;
    }

    bool hasDynamicReturnRisk()
    {
        return !obstacles.empty();
    }

    double proportionalReserve(
        double costToBase,
        const ReturnEnergyPolicy &policy
    ) {
        if (policy.returnMarginDivisor <= 0.0)
            return policy.minReturnMargin;

        return quantizeEnergy(
            std::ceil((costToBase / policy.returnMarginDivisor) * 2.0) / 2.0
        );
    }
}

double returnMarginForCost(
    double costToBase,
    const ReturnEnergyPolicy &policy
) {
    if (costToBase < 0.0 || costToBase >= INF)
        return (double)INF;

    // The 25%/minimum reserve is a risk buffer for dynamic obstacles.
    // When the scenario has no dynamic obstacle, the return path is evaluated
    // on the known static map, so the extra reserve is not applied.
    if (!hasDynamicReturnRisk())
        return 0.0;

    return quantizeEnergy(max(
        policy.minReturnMargin,
        proportionalReserve(costToBase, policy)
    ));
}

bool shouldReturnForEnergy(
    const Robot &rb,
    double costToBase,
    const ReturnEnergyPolicy &policy
) {
    if (isAtBase(rb))
        return false;

    if (costToBase < 0.0 || costToBase >= INF)
        return false;

    double reserve = returnMarginForCost(costToBase, policy);

    if (reserve >= INF)
        return false;

    double requiredEnergy = quantizeEnergy(costToBase + reserve);
    return rb.energy <= requiredEnergy;
}

bool isCriticalEnergy(
    const Robot &rb,
    double costToBase,
    const ReturnEnergyPolicy &policy
) {
    if (isAtBase(rb))
        return false;

    if (costToBase < 0.0 || costToBase >= INF)
        return rb.energy <= policy.minReturnMargin;

    return rb.energy <= costToBase ||
           rb.energy <= policy.minEmergencyEnergy;
}

bool canVisitTargetAndReturn(
    const Robot &rb,
    double costToTarget,
    double costTargetToBase,
    const ReturnEnergyPolicy &policy
) {
    if (costToTarget < 0.0 || costTargetToBase < 0.0)
        return false;

    if (costToTarget >= INF || costTargetToBase >= INF)
        return false;

    double reserve = returnMarginForCost(costTargetToBase, policy);

    if (reserve >= INF)
        return false;

    double requiredEnergy = quantizeEnergy(
        costToTarget + costTargetToBase + reserve
    );

    return rb.energy >= requiredEnergy;
}
