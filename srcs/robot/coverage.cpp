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
        int multiplier = testSpeedMultiplier();
        int delay = renderDelayMs();

        if (multiplier > 1)
            delay = max(1, delay / multiplier);

        renderCoverageFrame(rb, ctx, true, delay);
        waitFrame(delay);
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

        int multiplier = testSpeedMultiplier();
        long long elapsedMs = chrono::duration_cast<chrono::milliseconds>(
            now - lastTime
        ).count();

        accumulatedMs += elapsedMs * multiplier;

        lastTime = now;

        int processedTicks = 0;
        int maxTicksThisRender = maxCatchupTicksPerRender() * multiplier;

        while (accumulatedMs >= simTickMs() &&
               processedTicks < maxTicksThisRender)
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

        if (processedTicks == maxTicksThisRender)
        {
            accumulatedMs = min(
                accumulatedMs,
                1LL * simTickMs() * maxTicksThisRender
            );
        }

        renderFrame(rb, ctx);

        if (ctx.shouldStop || finished)
            break;
    }

    rb.missionOutcome = ctx.outcome;
}
