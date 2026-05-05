#pragma once

#include "grid.h"
#include <vector>

extern int d[1001][1001];
extern Cell trace[1001][1001];

void dijkstra(Cell start, int d[1001][1001], Cell trace[1001][1001]);
std::vector<Cell> tracePath(Cell start, Cell goal, Cell trace[1001][1001]);
Cell findNearestUncovered(Cell start);
