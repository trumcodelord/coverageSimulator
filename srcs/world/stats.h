#pragma once
#include "types.h"
#include <string>

struct CoverageStats
{
    int totalSteps = 0;
    int totalEdges = 0;
    int overlapEdges = 0;

    int initialFreeCells = 0;
    int coveredCells = 0;
    int dynamicBlockedCells = 0;

    double coverageRate = 0.0;
};

CoverageStats collectStats(const Robot& rb);

void printStats(const CoverageStats& s);

void logStats(const CoverageStats& s, const std::string& filename);
