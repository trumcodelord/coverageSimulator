#include "coverage_context.h"

#include <algorithm>

void beginCoverageTick(CoverageContext &ctx)
{
    ctx.shouldStop = false;
    ctx.needWaitDraw = false;
}

void setCoverageCooldown(CoverageContext &ctx, int ticks)
{
    ctx.actionCooldownTicks = std::max(ctx.actionCooldownTicks, ticks);
}
