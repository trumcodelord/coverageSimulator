#pragma once

#include "types.h"

#include <string>

struct CoverageStats
{
    int rows = 0;
    int cols = 0;
    int totalCells = 0;

    int initialFreeCells = 0;
    int obstacleCells = 0;
    double obstacleDensity = 0.0;

    int coveredCells = 0;
    double coverageRate = 0.0;

    int totalSteps = 0;
    double energyUsed = 0.0;
    double movementEnergyUsed = 0.0;
    double turnEnergyUsed = 0.0;
    double remainingEnergy = 0.0;
    double energyPerCoveredCell = 0.0;
    int returnCount = 0;
    int rechargeCount = 0;

    std::string turnCostModel = "normal_half_move";

    MissionOutcome missionOutcome = MISSION_RUNNING;
    bool finalAtBase = false;

    int dynamicBlockedCells = 0;
};

CoverageStats collectStats(const Robot& rb);

void printStats(const CoverageStats& s);

void logStats(const CoverageStats& s, const std::string& filename);

void appendBenchmarkCsv(
    const CoverageStats& s,
    const std::string& csvFile,
    const std::string& mapName,
    const std::string& screenshotPath
);
