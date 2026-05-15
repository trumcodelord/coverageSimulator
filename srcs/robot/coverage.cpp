#include "coverage.h"

#include "coverage_context.h"
#include "coverage_render.h"
#include "coverage_tick.h"
#include "coverage_timing.h"
#include "dynamic_obstacle.h"
#include "opencv.h"
#include "robot_lifecycle.h"

#include <algorithm>
#include <chrono>
#include <mutex>

using namespace std;

namespace
{
    constexpr int DEFAULT_MAX_ENERGY = 120;

    void renderFrame(const Robot &rb)
    {
        renderCoverageFrame(rb, true, renderDelayMs());
        waitFrame(renderDelayMs());
    }
}

void executeCoverage(Robot &rb)
{
    CoverageContext ctx;

    {
        lock_guard<mutex> lock(simMutex);
        initializeCoverageRobot(rb, DEFAULT_MAX_ENERGY);
    }

    initWindow();
    setHUDState("NORMAL");
    renderFrame(rb);

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

        renderFrame(rb);

        if (ctx.shouldStop || finished)
            break;
    }

    rb.missionOutcome = ctx.outcome;
}
