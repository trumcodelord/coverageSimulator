#include "energy_model.h"

#include "grid.h"
#include "motion_geometry.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

using namespace std;

namespace
{
    constexpr double ENERGY_QUANTUM = 0.5;
    constexpr double ENERGY_EPS = 1e-9;

    TurnCostModel activeTurnCostModel = TurnCostModel::NORMAL_HALF_MOVE;
}

void setTurnCostModel(TurnCostModel model)
{
    activeTurnCostModel = model;
}

TurnCostModel currentTurnCostModel()
{
    return activeTurnCostModel;
}

const char *turnCostModelName(TurnCostModel model)
{
    if (model == TurnCostModel::METRIC_H_NEGLIGIBLE)
        return "metric_h";

    return "normal_half_move";
}

const char *currentTurnCostModelName()
{
    return turnCostModelName(currentTurnCostModel());
}

bool isMetricHTurnCostModel()
{
    return currentTurnCostModel() == TurnCostModel::METRIC_H_NEGLIGIBLE;
}

double quantizeEnergy(double value)
{
    if (value <= 0.0)
        return 0.0;

    if (value >= INF)
        return (double)INF;

    return std::round(value / ENERGY_QUANTUM) * ENERGY_QUANTUM;
}

std::string formatEnergy(double value)
{
    std::ostringstream out;
    out << std::fixed << std::setprecision(1) << quantizeEnergy(value);
    return out.str();
}

double movementEnergyCostForStep(
    const Robot &,
    Cell next,
    RobotMode mode,
    const EnergyCostConfig &config
) {
    int terrainEntryCost = effectiveTerrainCostAt(next.r, next.c);

    if (terrainEntryCost >= INF)
        return (double)INF;

    double cost = max(config.baseMoveCost, (double)terrainEntryCost);

    if (mode == ALERT)
        cost += config.alertPenalty;

    if (mode == RETURN_TO_BASE)
        cost += config.returnPenalty;

    if (mode == FINAL_PUSH)
        cost += config.finalPushPenalty;

    return quantizeEnergy(max(0.0, cost));
}

double turnQuarterEnergyCostAtCell(Cell cell)
{
    if (isMetricHTurnCostModel())
        return 0.0;

    int currentTerrainCost = baseTerrainCostAt(cell.r, cell.c);

    if (currentTerrainCost >= INF)
        return (double)INF;

    // A 90-degree in-place turn is modeled as half of one movement-energy unit
    // on the terrain where the robot is currently standing.
    return quantizeEnergy(0.5 * (double)currentTerrainCost);
}

double turnQuarterEnergyCostForStep(
    const Robot &rb,
    Cell next
) {
    if (quarterTurnsForMove(rb, next) <= 0)
        return 0.0;

    return turnQuarterEnergyCostAtCell(rb.pos);
}

double computeMoveEnergyCost(
    const Robot &rb,
    Cell next,
    RobotMode mode,
    const EnergyCostConfig &config
) {
    double movementCost =
        movementEnergyCostForStep(rb, next, mode, config);

    double turnQuarterCost =
        turnQuarterEnergyCostForStep(rb, next);

    if (movementCost >= INF || turnQuarterCost >= INF)
        return (double)INF;

    return quantizeEnergy(
        movementCost +
        quarterTurnsForMove(rb, next) * turnQuarterCost
    );
}

void consumeEnergy(Robot &rb, double amount, EnergyUseCategory category)
{
    double cost = quantizeEnergy(amount);

    if (cost <= ENERGY_EPS)
        return;

    rb.energy = quantizeEnergy(max(0.0, rb.energy - cost));
    rb.totalEnergyUsed = quantizeEnergy(rb.totalEnergyUsed + cost);

    if (category == ENERGY_USE_MOVEMENT)
        rb.movementEnergyUsed = quantizeEnergy(rb.movementEnergyUsed + cost);
    else if (category == ENERGY_USE_TURN)
        rb.turnEnergyUsed = quantizeEnergy(rb.turnEnergyUsed + cost);
}
