#include "motion_geometry.h"

#include <cmath>

bool directionForStep(Cell from, Cell to, HeadingDir &dir)
{
    int dr = to.r - from.r;
    int dc = to.c - from.c;

    if (dr == -1 && dc == 0) { dir = DIR_NORTH; return true; }
    if (dr == 0 && dc == 1)  { dir = DIR_EAST;  return true; }
    if (dr == 1 && dc == 0)  { dir = DIR_SOUTH; return true; }
    if (dr == 0 && dc == -1) { dir = DIR_WEST;  return true; }

    return false;
}

HeadingDir currentHeadingDir(const Robot &rb)
{
    return headingDirFromDegrees(rb.headingDeg);
}

double angleForDirection(HeadingDir dir)
{
    if (dir == DIR_EAST) return -90.0;
    if (dir == DIR_WEST) return 90.0;
    if (dir == DIR_SOUTH) return 180.0;
    return 0.0;
}

double angleForMove(Cell from, Cell to)
{
    HeadingDir dir = DIR_NORTH;

    if (!directionForStep(from, to, dir))
        return 0.0;

    return angleForDirection(dir);
}

double normalizeAngle(double angle)
{
    while (angle <= -180.0) angle += 360.0;
    while (angle > 180.0) angle -= 360.0;
    return angle;
}

double shortestTurnDelta(double fromDeg, double toDeg)
{
    return normalizeAngle(toDeg - fromDeg);
}

int turnQuarterCountFromDelta(double deltaDeg)
{
    double amount = std::fabs(deltaDeg);

    if (amount < 1e-6)
        return 0;

    return amount > 135.0 ? 2 : 1;
}

int quarterTurnsForMove(const Robot &rb, Cell next)
{
    double targetAngle = angleForMove(rb.pos, next);
    double delta = shortestTurnDelta(rb.headingDeg, targetAngle);
    return turnQuarterCountFromDelta(delta);
}
