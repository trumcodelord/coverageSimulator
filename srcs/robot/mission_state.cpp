#include "mission_state.h"

#include "opencv.h"
#include "path_builder.h"

#include <iostream>

using namespace std;

void switchMissionMode(CoverageContext &ctx, RobotMode newMode)
{
    if (ctx.mode == newMode)
        return;

    ctx.mode = newMode;
    ctx.stableStepCount = 0;
    ctx.alertFailCount = 0;
    ctx.holdTick = 0;

    cout << "[MODE] -> " << modeName(ctx.mode) << '\n';
    setHUDState(modeName(ctx.mode));
}

void enterAlertMode(CoverageContext &ctx)
{
    if (ctx.mode == NORMAL)
        switchMissionMode(ctx, ALERT);
    else
        setHUDState("ALERT");
}

void enterHoldSafeMode(CoverageContext &ctx, Robot &rb)
{
    switchMissionMode(ctx, HOLD_SAFE);
    ctx.holdCycleCount = 0;
    clearRobotPath(rb);
    ctx.needWaitDraw = true;
}

void enterFinalPushMode(CoverageContext &ctx, Robot &rb)
{
    clearRobotPath(rb);
    ctx.returnWaitCount = 0;
    ctx.needWaitDraw = false;
    ctx.actionCooldownTicks = 0;
    switchMissionMode(ctx, FINAL_PUSH);
}

void enterWaitForCommandMode(CoverageContext &ctx, Robot &rb)
{
    clearRobotPath(rb);
    ctx.needWaitDraw = true;
    switchMissionMode(ctx, WAIT_FOR_COMMAND);
}
