#include "mission_state.h"

#include "behavior_log.h"
#include "opencv.h"
#include "path_builder.h"

using namespace std;

void switchMissionMode(CoverageContext &ctx, RobotMode next)
{
    if (ctx.mode == next)
        return;

    ctx.mode = next;

    logBehavior("[MODE] -> " + robotModeName(next));
    setHUDState(robotModeName(next));
}

void enterAlertMode(CoverageContext &ctx)
{
    switchMissionMode(ctx, ALERT);
}

void enterHoldSafeMode(CoverageContext &ctx, Robot &rb)
{
    clearRobotPath(rb);
    ctx.holdTick = 0;
    ctx.holdCycleCount = 0;
    ctx.needWaitDraw = false;

    switchMissionMode(ctx, HOLD_SAFE);
}

void enterWaitForCommandMode(CoverageContext &ctx, Robot &rb)
{
    clearRobotPath(rb);
    ctx.needWaitDraw = true;

    switchMissionMode(ctx, WAIT_FOR_COMMAND);
}

void enterFinalPushMode(CoverageContext &ctx, Robot &rb)
{
    clearRobotPath(rb);
    ctx.needWaitDraw = false;

    switchMissionMode(ctx, FINAL_PUSH);
}
