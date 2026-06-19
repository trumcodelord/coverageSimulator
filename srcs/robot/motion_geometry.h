#pragma once

#include "planner.h"
#include "types.h"

bool directionForStep(Cell from, Cell to, HeadingDir &dir);
HeadingDir currentHeadingDir(const Robot &rb);

double angleForDirection(HeadingDir dir);
double angleForMove(Cell from, Cell to);
double normalizeAngle(double angle);
double shortestTurnDelta(double fromDeg, double toDeg);

int turnQuarterCountFromDelta(double deltaDeg);
int quarterTurnsForMove(const Robot &rb, Cell next);
