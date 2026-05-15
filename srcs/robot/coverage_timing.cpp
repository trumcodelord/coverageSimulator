#include "coverage_timing.h"

namespace
{
    constexpr int SIM_TICK_MS = 20;
    constexpr int RENDER_DELAY_MS = 30;
    constexpr int MAX_CATCHUP_TICKS_PER_RENDER = 20;

    constexpr int NORMAL_STEP_MS = 500;
    constexpr int ALERT_STEP_MS  = 80;

    constexpr int BLOCKED_WAIT_MS = 120;
    constexpr int NO_TARGET_WAIT_MS = 150;
    constexpr int HOLD_WAIT_MS = 180;

    constexpr int RECHARGE_WAIT_TICKS = 10;
    constexpr int COMMAND_WAIT_TICKS = 8;

    constexpr int ceilDiv(int a, int b)
    {
        return (a + b - 1) / b;
    }
}

int simTickMs()
{
    return SIM_TICK_MS;
}

int renderDelayMs()
{
    return RENDER_DELAY_MS;
}

int maxCatchupTicksPerRender()
{
    return MAX_CATCHUP_TICKS_PER_RENDER;
}

int normalStepTicks()
{
    return ceilDiv(NORMAL_STEP_MS, SIM_TICK_MS);
}

int alertStepTicks()
{
    return ceilDiv(ALERT_STEP_MS, SIM_TICK_MS);
}

int stepTicksForMode(RobotMode mode)
{
    return mode == ALERT ? alertStepTicks() : normalStepTicks();
}

int blockedWaitTicks()
{
    return ceilDiv(BLOCKED_WAIT_MS, SIM_TICK_MS);
}

int noTargetWaitTicks()
{
    return ceilDiv(NO_TARGET_WAIT_MS, SIM_TICK_MS);
}

int holdWaitTicks()
{
    return ceilDiv(HOLD_WAIT_MS, SIM_TICK_MS);
}

int rechargeWaitTicks()
{
    return RECHARGE_WAIT_TICKS;
}

int commandWaitTicks()
{
    return COMMAND_WAIT_TICKS;
}
