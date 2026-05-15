
#pragma once

#include "types.h"

#include <string>

struct MissionSummary
{
    MissionOutcome outcome = MISSION_RUNNING;

    bool coverageComplete = false;
    bool returnedToBase = false;
    bool powerPreserved = false;

    int totalSteps = 0;
    int maxEnergy = 0;
    int energyRemaining = 0;
    int totalEnergyUsed = 0;

    int returnCount = 0;
    int rechargeCount = 0;

    int initialFreeCells = 0;
    int coveredCells = 0;
    double coverageRate = 0.0;
};

MissionSummary collectMissionSummary(const Robot &rb);

void printMissionSummary(const MissionSummary &summary);

void logMissionSummary(
    const MissionSummary &summary,
    const std::string &filename
);
