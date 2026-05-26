#include "coverage.h"

#include "coverage_context.h"
#include "coverage_render.h"
#include "coverage_tick.h"
#include "coverage_timing.h"
#include "dynamic_obstacle.h"
#include "input.h"
#include "opencv.h"
#include "robot_lifecycle.h"

#include <algorithm>
#include <chrono>
#include <mutex>

using namespace std;

namespace
{
    void renderFrame(const Robot &rb, const CoverageContext &ctx)
    {
        renderCoverageFrame(rb, ctx, true, renderDelayMs());
        waitFrame(renderDelayMs());
    }
}

void executeCoverage(Robot &rb)
{
    CoverageContext ctx;

    {
        lock_guard<mutex> lock(simMutex);
        initializeCoverageRobot(rb, configuredMaxEnergy());
    }

    initWindow();
    setHUDState("NORMAL");
    renderFrame(rb, ctx);

    using Clock = chrono::steady_clock;

    auto lastTime = Clock::now();
    long long accumulatedMs = simTickMs();

    bool finished = false;

    while (true)
    {
        auto now = Clock::now();

        accumulatedMs += chrono::duration_cast<chrono::milliseconds>(
            now - lastTime
        ).count();

        lastTime = now;

        int processedTicks = 0;

        while (accumulatedMs >= simTickMs() &&
               processedTicks < maxCatchupTicksPerRender())
        {
            {
                lock_guard<mutex> lock(simMutex);

                beginCoverageTick(ctx);

                handleCoverageCompletion(ctx, rb, finished);

                if (finished)
                    break;

                if (!ctx.needWaitDraw)
                    processCoverageTick(rb, ctx);

                handleCoverageCompletion(ctx, rb, finished);
            }

            accumulatedMs -= simTickMs();
            processedTicks++;

            if (ctx.shouldStop || finished)
                break;
        }

        if (processedTicks == maxCatchupTicksPerRender())
        {
            accumulatedMs = min(
                accumulatedMs,
                1LL * simTickMs() * maxCatchupTicksPerRender()
            );
        }

        renderFrame(rb, ctx);

        if (ctx.shouldStop || finished)
            break;
    }

    rb.missionOutcome = ctx.outcome;
}