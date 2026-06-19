#pragma once

#include "types.h"
#include <vector>

extern int rows, cols;
extern Cell start;
extern bool covered[1001][1001];
extern bool dynamicBlocked[1001][1001];
extern int terrainCost[1001][1001];
extern const int dr[5], dc[5];
extern int initialFreeCells;
extern int coveredCellCount;

bool inBounds(int r, int c);

bool isStaticBlocked(int r, int c);
bool isDynamicBlockedCell(int r, int c);
bool isBlockedCell(int r, int c);

bool isFree(int r, int c);
bool isCovered(int r, int c);
bool isCoverageTargetCell(int r, int c);

int terrainCostAt(int r, int c);
int baseTerrainCostAt(int r, int c);
int effectiveTerrainCostAt(int r, int c);

void markCovered(int r, int c);
bool allCovered();
std::vector<Cell> getNeighbors(Cell p);
