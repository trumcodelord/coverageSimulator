#include "robot_lifecycle.h"

#include "dynamic_obstacle.h"
#include "grid.h"

void initializeCoverageRobot(Robot &rb, int maxEnergy)
{
    rb.steps = 0;
    rb.pathID = 0;
    rb.path.clear();
    rb.trail.clear();
    rb.edgeCount.clear();

    rb.base = rb.pos;
    rb.maxEnergy = maxEnergy;
    rb.energy = rb.maxEnergy;
    rb.totalEnergyUsed = 0;
    rb.returnCount = 0;
    rb.rechargeCount = 0;
    rb.missionOutcome = MISSION_RUNNING;

    rb.trail.push_back(rb.pos);
    markCovered(rb.pos.r, rb.pos.c);
    setRobotAvoidanceCell(rb.pos);
}

void rechargeRobot(Robot &rb)
{
    rb.energy = rb.maxEnergy;
    rb.rechargeCount++;
}
