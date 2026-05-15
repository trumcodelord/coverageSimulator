#include "robot_motion.h"

#include "dynamic_obstacle.h"
#include "energy_model.h"
#include "grid.h"

RobotMoveResult moveRobotAlongCurrentPath(
    Robot &rb,
    CoverageContext &ctx,
    int energyCost
) {
    RobotMoveResult result;

    if (rb.pathID >= (int)rb.path.size())
        return result;

    Cell prev = rb.pos;
    Cell next = rb.path[rb.pathID];

    result.from = prev;
    result.to = next;
    result.enteredUncoveredCell = !isCovered(next.r, next.c);

    if (!isFree(next.r, next.c))
    {
        result.blocked = true;
        return result;
    }

    rb.pos = next;
    setRobotAvoidanceCell(rb.pos);

    rb.trail.push_back(rb.pos);
    rb.pathID++;

    Edge e(prev, next);
    rb.edgeCount[e]++;

    rb.steps++;
    consumeEnergy(rb, energyCost);

    result.moved = true;
    result.powerLoss = (rb.energy <= 0);

    if (result.powerLoss)
    {
        ctx.shouldStop = true;
        return result;
    }

    markCovered(rb.pos.r, rb.pos.c);

    return result;
}
